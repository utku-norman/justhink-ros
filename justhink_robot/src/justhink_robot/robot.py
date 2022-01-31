import sys

import rospy
import rosservice

# QTrobot services.
import qt_robot_interface.srv
import qt_gesture_controller.srv
import qt_motors_controller.srv

# Standard ROS messages
import std_msgs.msg
# Custom ROS messages and services
import justhink_msgs.msg
import justhink_msgs.srv

from justhink_agent.tools import make_service_proxy

from justhink_scenario.logging import log_publish, log_heard, \
    log_service_response, log_service_call


gestures = {
    'point_human': 'epfl/justhink/point_front_head_forward',
    'point_activity': 'QT/show_tablet',
    'point_self': 'epfl/justhink/show_QT_head_down',
    'head_scratch': 'epfl/old_QT/head_scratch',
    'clap': 'QT/clapping',
    'curious': 'epfl/old_QT/curious',
    'left_arm_up_pull_down': 'epfl/old_QT/strong',
    'challenge': 'QT/challenge',
    'hard_nod': 'QT/neutral',
    'yes': 'epfl/old_QT/yes',
    'no': 'epfl/old_QT/no',
    'right_arm_up_down': 'QT/one-arm-up',
    'arms_open_close': 'QT/emotions/calm',
    'send_kiss': 'QT/send_kiss',
    'wave': 'epfl/justhink/hi_head',
    'sad': 'QT/emotions/sad',
    'happy': 'QT/emotions/happy',
    'hi': 'QT/hi',
    'swipe_right': 'QT/swipe_right',
    'swipe_left': 'QT/swipe_left',
    'bored': 'QT/bored',
    'protect': 'epfl/old_QT/protect',
    'so': 'epfl/old_QT/so',
    'rappel': 'epfl/old_QT/rappel',
    'shy': 'QT/emotions/shy',
    'afraid': 'QT/emotions/afraid',
    'yawn': 'epfl/old_QT/yawn',
    'hoora': 'QT/emotions/hoora',   # epfl/old_QT/thanks
}

emotions = {
    'smile': 'QT/showing_smile',
    'sad': 'QT/sad',
    'happy': 'QT/happy',
    'kiss': 'QT/kiss',
    'happy_blinking': 'QT/happy_blinking',
}


def get_robot_gesture_name(name):
    try:
        new_name = gestures[name]
        return new_name
    except KeyError:
        rospy.logwarn('Gesture {} not found. Ignoring.'.format(name))
        return None


def get_robot_emotion_name(name):
    try:
        new_name = emotions[name]
        return new_name
    except KeyError:
        rospy.logwarn('Emotion {} not found. Ignoring.'.format(name))
        return None


class Robot(object):
    """A class to represent an the list of available robots.

    Attributes:
        TODO
    """
    QTROBOT = 'QTrobot'
    REACHY = 'Reachy'


class PhysicalRobot(object):
    """docstring for PhysicalRobot"""

    def __init__(self, robot=Robot.QTROBOT):
        self.robot = robot
        self.is_robot_connected = False

        self.init_ros()

        rospy.on_shutdown(self.call_stop_say)
        rospy.on_shutdown(self.call_stop_emote)
        rospy.on_shutdown(self.call_stop_express)
        rospy.on_shutdown(self.home_all)
        rospy.on_shutdown(self.on_close)

        rospy.loginfo(
            '{} embodiment node is ready!'.format(self.robot))
        print()

    def init_ros(self):
        # Initialise the subscribers.
        self.say_sub = rospy.Subscriber(
            'embodiment/say', std_msgs.msg.String,
            self.say_callback)
        self.express_sub = rospy.Subscriber(
            'embodiment/express', std_msgs.msg.String,
            self.express_callback)
        self.emote_sub = rospy.Subscriber(
            'embodiment/emote', std_msgs.msg.String,
            self.emote_callback)

        # For QTrobot.
        print()
        rospy.logwarn('Intialising for robot "{}"'.format(self.robot))
        print()
        if self.robot == Robot.QTROBOT:
            self.is_robot_connected = self.init_qtrobot_ros()
        else:
            raise NotImplementedError

        # Initialise the services to offer.
        self.say_service = rospy.Service(
            'embodiment/say', justhink_msgs.srv.Say,
            self.reply_say_request)
        self.express_service = rospy.Service(
            'embodiment/express', justhink_msgs.srv.Express,
            self.reply_express_request)
        self.emote_service = rospy.Service(
            'embodiment/emote', justhink_msgs.srv.Emote,
            self.reply_emote_request)
        self.home_service = rospy.Service(
            'embodiment/home', justhink_msgs.srv.Home,
            self.reply_home_request)
        self.configure_speech_service = rospy.Service(
            'embodiment/configure_speech', justhink_msgs.srv.ConfigureSpeech,
            self.reply_configure_speech_request)
        self.stop_say_service = rospy.Service(
            'embodiment/stop_say', justhink_msgs.srv.StopSay,
            self.reply_stop_say_request)
        self.stop_emote_service = rospy.Service(
            'embodiment/stop_emote', justhink_msgs.srv.StopEmote,
            self.reply_stop_emote_request)
        self.stop_express_service = rospy.Service(
            'embodiment/stop_express', justhink_msgs.srv.StopExpress,
            self.reply_stop_express_request)

        rospy.loginfo('Node initialised.')

    def on_close(self):
        rospy.loginfo("Closing robot embodiment node.")
        rospy.signal_shutdown('Robot embodiment node is closed.')

    def init_qtrobot_ros(self):
        # Initialise the publishers.
        opts = {'data_class': std_msgs.msg.String, 'queue_size': 10}
        self.say_pub = rospy.Publisher('/qt_robot/behavior/talkText', **opts)
        self.express_pub = rospy.Publisher('/qt_robot/gesture/play', **opts)
        self.emote_pub = rospy.Publisher('/qt_robot/emotion/show', **opts)
        self.head_pos_pub = rospy.Publisher(
            '/qt_robot/head_position/command',
            std_msgs.msg.Float64MultiArray, queue_size=10)

        # Initialise the services.
        try:
            self.call_say = make_service_proxy(
                '/qt_robot/behavior/talkText',
                qt_robot_interface.srv.behavior_talk_text,
                wait=True, timeout=5)
            self.call_express = make_service_proxy(
                '/qt_robot/gesture/play',
                qt_gesture_controller.srv.gesture_play,
                wait=False)
            self.call_emote = make_service_proxy(
                '/qt_robot/emotion/show',
                qt_robot_interface.srv.emotion_show,
                wait=False)
            self.call_home = make_service_proxy(
                '/qt_robot/motors/home',
                qt_motors_controller.srv.home,
                wait=False)
            self.call_configure_speech = make_service_proxy(
                '/qt_robot/speech/config',
                qt_robot_interface.srv.speech_config,
                wait=False)
            self.call_stop_say = make_service_proxy(
                '/qt_robot/speech/stop',
                qt_robot_interface.srv.speech_stop,
                wait=False)
            self.call_stop_emote = make_service_proxy(
                '/qt_robot/emotion/stop',
                qt_robot_interface.srv.emotion_stop,
                wait=False)
            self.call_stop_express = make_service_proxy(
                '/qt_robot/gesture/stop',
                qt_gesture_controller.srv.gesture_stop,
                wait=False)

        except (rospy.ROSInterruptException, rospy.exceptions.ROSException) \
                as e:
            sys.exit(e)

        service_name = '/qt_robot/behavior/talkText'
        is_service_found = service_name in rosservice.get_service_list()
        return is_service_found

    # Callbacks.

    def say_callback(self, data):
        log_heard(self.say_sub, data)
        log_publish(self.say_pub, data)
        self.say_pub.publish(data)

    def express_callback(self, data):
        log_heard(self.express_sub, data)
        new_data = get_robot_gesture_name(data.data)
        if new_data is not None:
            log_publish(self.express_pub, new_data)
            self.express_pub.publish(new_data)

    def emote_callback(self, data):
        log_heard(self.emote_sub, data)
        new_data = get_robot_emotion_name(data.data)
        if new_data is not None:
            log_publish(self.emote_pub, data)
            self.emote_pub.publish(data)

    def home_all(self):
        rospy.loginfo('Homing all...')
        self.call_home(['HeadPitch', 'left_arm', 'right_arm'])

    # Service request replies.

    def reply_say_request(self, data):
        # rospy.loginfo('Replying say request...')
        log_service_response(self.say_service, data)
        resp = self.call_say(data.message)
        log_service_call(self.call_say, data, resp)
        return resp.status

    def reply_express_request(self, data):
        # rospy.loginfo('Replying express request...')
        data.name = get_robot_gesture_name(data.name)
        if data.name is not None:
            log_service_response(self.express_service, data)
            resp = self.call_express(data.name, data.speed)
            log_service_call(self.call_express, data, resp)
            return resp.status
        else:
            return False

    def reply_emote_request(self, data):
        # rospy.loginfo('Replying emote request...')
        data.name = get_robot_gesture_name(data.name)
        if data.name is not None:
            log_service_response(self.emote_service, data)
            resp = self.call_emote(data.name)
            log_service_call(self.call_emote, data, resp)
            return resp.status
        else:
            return False
            
    def reply_home_request(self, data):
        rospy.loginfo('Replying home request: {}'.format(data))
        # resp = self.call_home(data.parts)
        if 'HeadPitch' in data.parts:
            data = [0, 10]
            self.head_pos_pub.publish(data=data)
            log_publish(self.head_pos_pub, data)
            rospy.sleep(1)
        resp = True
        log_service_response(self.home_service, data, resp)
        # return resp.status
        return resp

    def reply_configure_speech_request(self, data):
        rospy.loginfo('Replying configure speech request...')
        resp = self.call_configure_speech(
            speed=data.speed, pitch=data.pitch, language=data.language)
        log_service_response(self.configure_speech_service, data, resp)
        return resp.status

    def reply_stop_say_request(self, data):
        rospy.loginfo('Replying stop say request...')
        try:
            resp = self.call_stop_say()
            log_service_response(self.stop_say_service, data, resp)
            resp = resp.status
        except Exception as e:
            rospy.logerr(e)
            resp = False
        return resp

    def reply_stop_emote_request(self, data):
        rospy.loginfo('Replying stop emote request...')
        try:
            resp = self.call_stop_emote()
            log_service_response(self.stop_emote_service, data, resp)
            resp = resp.status
        except Exception as e:
            rospy.logerr(e)
            resp = False
        return resp

    def reply_stop_express_request(self, data):
        rospy.loginfo('Replying stop express request...')
        try:
            resp = self.call_stop_express()
            log_service_response(self.stop_express_service, data, resp)
            resp = resp.status
        except Exception as e:
            rospy.logerr(e)
            resp = False
        return resp
