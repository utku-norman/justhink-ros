import importlib_resources

import pyglet
from pyglet.window import mouse, key

import rospy
import rosservice

import justhink_msgs.msg
import justhink_msgs.srv

from justhink_world import create_all_worlds
from justhink_world.visual import IntroWorldScene, TutorialWorldScene
from justhink_world.domain.action import ResetAction
# from justhink_world.domain.observation import Observation
from justhink_world.tools.graphics import ButtonWidget, Graphics, BLACK, WHITEA
from justhink_world.tools.read import load_image_from_reference

from .scenes import CoverScene, WelcomeScene, TestScene, CollaborativeScene
from .messages import decode_action_message, make_state_message, \
    make_state_transition_message, make_activity_transition_message, \
    make_button_message, make_drawing_message, ROBOT_SPEAKING
from .logging import log_publish, log_heard, log_service_response


def show_scenario():
    """TODO"""
    AppWindow()

    # Enter main event loop.
    while not rospy.is_shutdown():
        pyglet.app.run()


class AppWindow(pyglet.window.Window):
    def __init__(self, width=1920, height=1080, screen_index=-1):

        image_container = importlib_resources.files(
            'justhink_scenario.resources.images')
        self._init_graphics(width, height, image_container)

        # Initialise the worlds.
        worlds = create_all_worlds()
        self._worlds = worlds

        # Make the list of activities as triples (name, scene class, world).
        activities = []
        activities += [('cover', CoverScene, None)]
        activities += [('welcome', WelcomeScene, None)]
        # activities += [('profile', ProfileScene, None)]
        activities += [('intro', IntroWorldScene, worlds['intro'])]
        activities += [('tutorial', TutorialWorldScene, worlds['tutorial'])]
        # Add the pretests.
        for i in range(1, 6):
            name = 'pretest-{}'.format(i)
            activities += [(name, TestScene, worlds[name])]
        # Add the collaborative activities.
        for i in range(1, 3):
            name = 'collaboration-{}'.format(i)
            activities += [(name, CollaborativeScene, worlds[name])]
        # Add the posttests.
        for i in range(1, 6):
            name = 'posttest-{}'.format(i)
            activities += [(name, TestScene, worlds[name])]
        # Add a final bye scene.
        name = 'bye'
        activities += [(name, CollaborativeScene, worlds[name])]

        # Order the scenes (i.e. activities) and set to the scene.
        self._scene_order = list([name for name, _, _ in activities])

        # Create the scenes for the activities.
        scenes = dict()
        for name, scene_type, world in activities:
            rospy.loginfo('Initialising {} activity...'.format(name))
            scenes[name] = scene_type(
                name=name, width=width, height=height, world=world)
        self._scenes = scenes
        for scene in scenes.values():
            scene.win = self

        # Set a variable to check if the SHIFT key is pressed.
        self._is_shift = False
        self._is_alt = False
        self._is_a = False

        # # Set Name of the current and hence initial activity.
        # self._cur_scene_name = 'cover'
        # # self._cur_scene_name = 'welcome'
        # # self._cur_scene_name = 'tutorial'
        # # self._cur_scene_name = 'pretest-1'
        # # self._cur_scene_name = 'pretest-5'
        # self._cur_scene_name = 'collaboration-1'
        # # self._cur_scene_name = 'collaboration-2'

        # Robot mode or condition for experimentation.
        param_name = '~entry'
        entry = rospy.get_param(param_name, 'cover')
        if rospy.has_param(param_name):
            rospy.delete_param(param_name)
        assert entry in self._scenes
        rospy.logwarn('Entry activity is "{}".'.format(entry))
        self._cur_scene_name = entry
        print()

        caption = self._make_caption()

        self._init_ros()
        rospy.on_shutdown(self.on_close)

        # Initialising pyglet window.
        rospy.loginfo('Creating the window...')
        style = pyglet.window.Window.WINDOW_STYLE_BORDERLESS
        super().__init__(width, height, caption, style=style)

        # scenario_16x16
        # scenario_32x32
        icon1 = load_image_from_reference(
            image_container.joinpath('scenario_128x128.png'))
        icon2 = load_image_from_reference(
            image_container.joinpath('scenario_64x64.png'))
        self.set_icon(icon1, icon2)

        display = pyglet.canvas.get_display()
        screens = display.get_screens()
        active_screen = screens[screen_index]
        self.set_location(active_screen.x, active_screen.y)

        self._is_animating = False
        self._is_paused = False

        param_name = '~robot_text'
        self.with_robot_text = rospy.get_param(param_name, False)
        if rospy.has_param(param_name):
            rospy.delete_param(param_name)
        print()
        if self.with_robot_text:
            rospy.logwarn('Application will show the robot utterance.')
        else:
            rospy.logwarn('Application will NOT show the robot utterance.')
        print()

        if not self.with_robot_text:
            label = self.graphics.robot_text_label
            label.font_size = 32
            label.bold = True

        rospy.loginfo('Updating the window...')
        self.update()

        self.set_scene(self.cur_scene_name)
        ## set scene already publishes
        # self.publish_activity_transition(
        #     self.cur_scene_name, self.cur_scene_name)

        rospy.loginfo('Human application window is ready!')
        print()

    @property
    def cur_world(self):
        """The world being rendered."""
        if self._cur_scene_name in self._worlds:
            return self._worlds[self._cur_scene_name]
        else:
            return None

    @property
    def scene_no(self):
        """Index of the current scene being rendered."""
        return self._scene_order.index(self._cur_scene_name)

    @property
    def cur_scene_name(self):
        """Name of the current scene being rendered."""
        return self._cur_scene_name

    @property
    def cur_scene(self):
        """Current scene being rendered."""
        return self._scenes[self._cur_scene_name]

    @property
    def num_scenes(self):
        """Number of scenes i.e. activities in the application."""
        return len(self._scene_order)

    @scene_no.setter
    def scene_no(self, value):
        if value < 0:
            value = 0
        elif value > self.num_scenes - 1:
            value = self.num_scenes - 1
        else:
            try:
                # Do not animate if the key A is pressed.
                self.set_scene(
                    self._scene_order[value], animated=not self._is_a)
            except Exception as e:
                print('Trying to set scene to {}/{}'.format(
                    value, self.num_scenes))
                print(e)

    def animate_transition(self, dt, name, rate=10):
        """Animate scene transitions with fade-out and fade-in."""
        self.graphics.trans_rect.opacity += rate
        if self.graphics.trans_rect.opacity >= 250:
            self.set_scene(name, animated=False)
            pyglet.clock.unschedule(self.animate_transition)
            pyglet.clock.schedule(
                self.animate_transition, name=name, rate=-rate)
        elif rate < 0 and self.graphics.trans_rect.opacity <= 10:
            pyglet.clock.unschedule(self.animate_transition)
            self.graphics.trans_rect.opacity = 0
            self._is_animating = False

    def set_scene(self, name, animated=True):
        pyglet.clock.unschedule(self.animate_transition)

        if animated and not self._is_animating:
            self._is_animating = True
            # pyglet.clock.unschedule(self.animate_transition)
            pyglet.clock.schedule(self.animate_transition, name=name)
            # self.graphics.help_button.set_state(ButtonWidget.DISABLED)
        else:
            # Publish the activity change if shift is not pressed.
            if not self._is_shift:
                self.publish_activity_transition(self.cur_scene_name, name)
                self.update_robot_text('')

            self._cur_scene_name = name
            self.update()

            self.set_caption(self._make_caption())

        return True

    # GUI methods.

    def on_draw(self):
        self.clear()
        self.cur_scene.on_draw()
        self.graphics.batch.draw()

    def on_mouse_motion(self, x, y, dx, dy):
        self.publish_mouse_motion(x, y, dx, dy)

    def on_mouse_press(self, x, y, button, modifiers):
        self.publish_mouse_press(x, y, button)

        for name, button in self.cur_scene.graphics.buttons.items():
            if button.check_hit(x, y):
                self.publish_button_press(name=name, state=button.state)

        for name, button in self.graphics.buttons.items():
            if button.check_hit(x, y):
                self.publish_button_press(name=name, state=button.state)

        if not self._is_paused and not self._is_animating:
            self.cur_scene.on_mouse_press(x, y, button, modifiers, win=self)

            # Process repeat robot speech.
            if self.graphics.repeat_button.state == ButtonWidget.ENABLED:
                service_name = '/agent/cognition/repeat_speech'
                if self.graphics.repeat_button.check_hit(x, y):
                    if service_name in rosservice.get_service_list():
                        self.graphics.repeat_button.set_state(
                            ButtonWidget.SELECTED)
                        pyglet.clock.schedule_once(
                            self._robot_repeat_callback, 1)
                    else:
                        rospy.loginfo('Service {} not found'.format(
                            service_name))
                        rospy.loginfo('Ignoring repeat speech press.')

            # # Process help button.
            # if self.graphics.help_button.state == ButtonWidget.ENABLED:
            #     service_name = '/agent/cognition/request_help'
            #     if self.graphics.help_button.check_hit(x, y):
            #         if service_name in rosservice.get_service_list():
            #             self.graphics.help_button.set_state(
            #                 ButtonWidget.SELECTED)
            #             pyglet.clock.schedule_once(
            #                 self._robot_help_callback, 1)
            #         else:
            #             rospy.loginfo('Service {} not found'.format(
            #                 service_name))
            #             rospy.loginfo('Ignoring help button press.')

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        self.publish_mouse_drag(x, y, buttons, dx=dx, dy=dy)

        if not self._is_paused and not self._is_animating:
            self.cur_scene.on_mouse_drag(
                x, y, dx, dy, buttons, modifiers, win=self)

    def on_mouse_release(self, x, y, button, modifiers):
        self.publish_mouse_release(x, y, button)

        if not self._is_paused and not self._is_animating:
            self.cur_scene.on_mouse_release(x, y, button, modifiers, win=self)

    def on_key_press(self, symbol, modifiers):
        self.publish_key_press(symbol, modifiers)

        if symbol == key.LSHIFT or symbol == key.RSHIFT:
            self._is_shift = True
        if symbol == key.LALT or symbol == key.RALT:
            self._is_alt = True
        if symbol == key.A:
            self._is_a = True

        # Execute of the current scene.
        self.cur_scene.on_key_press(symbol, modifiers, self)

        # Execute other things.
        if symbol == key.ESCAPE and modifiers & key.MOD_CTRL:
            rospy.signal_shutdown("Closing by CTRL+ESCAPE")
            self.close()

        # Navigate the scenes with CTRL left and right.
        elif symbol == key.LEFT and modifiers & key.MOD_CTRL:
            if not self._is_animating:
                self.scene_no = self.scene_no - 1

        elif symbol == key.RIGHT and modifiers & key.MOD_CTRL:
            if not self._is_animating:
                self.scene_no = self.scene_no + 1

        elif symbol == key.TAB and modifiers & key.MOD_CTRL:
            self.cur_scene.toggle_role()
            self.update()

        # publish the current scene if CTRL+SHIFT+P.
        elif symbol == key.P and modifiers & key.MOD_CTRL:
            if modifiers & key.MOD_SHIFT:
                self.publish_activity_transition(
                    self.cur_scene_name, self.cur_scene_name)
            else:
                self.set_paused(not self._is_paused)

        elif symbol == key.R and modifiers & key.MOD_CTRL:
            if modifiers & key.MOD_SHIFT:
                self.execute_action(ResetAction())

    def on_key_release(self, symbol, modifiers):
        self.publish_key_release(symbol, modifiers)

        if symbol == key.LSHIFT or symbol == key.RSHIFT:
            self._is_shift = False
        if symbol == key.LALT or symbol == key.RALT:
            self._is_alt = False
        if symbol == key.A:
            self._is_a = False

    def on_text(self, text):
        self.cur_scene.on_text(text)

    def on_text_motion(self, motion):
        self.cur_scene.on_text_motion(motion)

    def on_text_motion_select(self, motion):
        self.cur_scene.on_text_motion_select(motion)

    def on_close(self):
        # Unregister the provided ROS services.
        self.act_service.shutdown()
        self.observe_activity_service.shutdown()
        self.observe_state_service.shutdown()

        rospy.loginfo("Closing human application node & its window.")
        rospy.signal_shutdown("Human application node 6 window is closed.")

        pyglet.app.exit()

    # Custom public methods.

    def update(self):
        pyglet.clock.schedule_once(self.update_callback, 0)

    def update_callback(self, dt):
        self.on_update()

    def on_update(self):
        self.cur_scene.on_update()

    def execute_action(self, action):

        state = self.cur_world.cur_state
        success = self.cur_world.act(action)
        next_state = self.cur_world.cur_state

        # rospy.loginfo('')
        print()
        rospy.loginfo('New event (on action execution):')
        rospy.loginfo('State: {}'.format(state))
        rospy.loginfo('Action: {}'.format(action))
        rospy.loginfo('Next state: {}'.format(next_state))
        print()
        # rospy.loginfo('')

        if success:
            next_state = self.cur_world.cur_state
            self.cur_scene.state = next_state
            self.update()
            self.publish_state_transition(state, action, next_state)

    def set_paused(self, is_paused):
        self._is_paused = is_paused
        self.graphics.paused_rect.visible = is_paused

    # Private methods.

    def _make_caption(self):
        """Construct the caption of the window."""
        return 'JUSThink {} (Scene #{})'.format(
            self.cur_scene_name, self.scene_no+1)

    def _init_graphics(self, width, height, image_container, batch=None):
        graphics = Graphics(width, height, batch=batch)
        batch = graphics.batch

        # Create drawing groups, higher the index the more foreground.
        groups = [pyglet.graphics.OrderedGroup(i) for i in range(10)]

        rect = pyglet.shapes.Rectangle(
            0, 0, width, height-100, color=BLACK, batch=batch, group=groups[5])
        rect.opacity = 80  # 100
        rect.visible = False
        graphics.paused_rect = rect

        rect = pyglet.shapes.Rectangle(
            0, 0, width, height, color=(192, 192, 192), batch=batch)
        rect.opacity = 0
        rect.visible = True
        graphics.trans_rect = rect

        # Create a robot text label on top.
        pad = 100
        label = pyglet.text.Label(
            '', x=pad, y=height-20, width=width-pad-20, height=20,
            color=WHITEA, anchor_y='center', font_name='monospace',
            font_size=24, multiline=True, align='center',
            batch=batch, group=groups[5])
        graphics.robot_text_label = label

        # Create background rectangle for robot text.
        h = 100
        rect = pyglet.shapes.BorderedRectangle(
            0, height-h, width, h, color=(90, 90, 90), border=15,
            border_color=BLACK, batch=batch, group=groups[4])
        graphics.robot_text_rect = rect

        # Create the robot logo.
        ref = image_container.joinpath('QT.png')
        image = load_image_from_reference(ref)
        s = pyglet.sprite.Sprite(
            image, x=20, y=height-90, batch=batch, group=groups[5])
        s.scale = 0.25
        graphics.robot_logo_sprite = s

        # Create the repeat button.
        c = image_container
        paths = {
            ButtonWidget.ENABLED: c.joinpath('repeat_enabled.png'),
            ButtonWidget.DISABLED: c.joinpath('repeat_disabled.png'),
            ButtonWidget.SELECTED: c.joinpath('repeat_selected.png'),
        }
        button = ButtonWidget(
            x=width-50, y=height-50, paths=paths, state=ButtonWidget.DISABLED,
            scale=0.12, batch=batch, group=groups[8])
        graphics.repeat_button = button
        graphics.buttons['repeat'] = button

        # # Create the help button.
        # button_pads, scale = (100, 180), 0.2
        # paths = {
        #     ButtonWidget.ENABLED: c.joinpath('help_enabled.png'),
        #     ButtonWidget.DISABLED: c.joinpath('help_disabled.png'),
        #     ButtonWidget.SELECTED: c.joinpath('help_selected.png'),
        # }
        # button = ButtonWidget(
        #     x=button_pads[0], y=height-button_pads[1], paths=paths,
        #     state=ButtonWidget.DISABLED, scale=scale, batch=batch,
        #     group=groups[1])
        # graphics.help_button = button
        # graphics.buttons['help'] = button

        self.graphics = graphics

    # ROS methods.

    def _init_ros(self):
        # Initialise the services.
        self.act_service = rospy.Service(
            '~act', justhink_msgs.srv.Act,
            self.reply_act_request)

        self.observe_activity_service = rospy.Service(
            '~observe_activity', justhink_msgs.srv.ObserveActivity,
            self.reply_observe_activity_request)

        self.observe_state_service = rospy.Service(
            '~observe_state', justhink_msgs.srv.ObserveState,
            self.reply_observe_state_request)

        self.pause_service = rospy.Service(
            '~pause', justhink_msgs.srv.Pause,
            self.reply_pause_request)

        self.set_activity_service = rospy.Service(
            '~set_activity', justhink_msgs.srv.SetActivity,
            self.reply_set_activity_request)

        self.set_robot_text_service = rospy.Service(
            '~set_robot_text', justhink_msgs.srv.SetRobotText,
            self.reply_set_robot_text_request)

        # Initialize robot behaviour services.
        self.repeat_service_name = '/agent/cognition/repeat_speech'
        self.repeat_speech_service = rospy.ServiceProxy(
            self.repeat_service_name, justhink_msgs.srv.RepeatSpeech)

        self.help_service_name = '/agent/cognition/request_help'
        self.request_help_service = rospy.ServiceProxy(
            self.help_service_name, justhink_msgs.srv.RequestHelp)

        # Initialize the publishers.
        self.state_transition_pub = rospy.Publisher(
            '~event', justhink_msgs.msg.StateTransition,
            queue_size=10)
        self.activity_transition_pub = rospy.Publisher(
            '~outset', justhink_msgs.msg.ActivityTransition,
            queue_size=10)
        self.button_press_pub = rospy.Publisher(
            '~button_press', justhink_msgs.msg.Button, queue_size=10)
        self.drawing_change_pub = rospy.Publisher(
            '~drawing_change', justhink_msgs.msg.Drawing, queue_size=10)
        self.mouse_press_pub = rospy.Publisher(
            '~mouse_press', justhink_msgs.msg.Mouse, queue_size=10)
        self.mouse_drag_pub = rospy.Publisher(
            '~mouse_drag', justhink_msgs.msg.Mouse, queue_size=10)
        self.mouse_release_pub = rospy.Publisher(
            '~mouse_release', justhink_msgs.msg.Mouse, queue_size=10)
        self.mouse_motion_pub = rospy.Publisher(
            '~mouse_motion', justhink_msgs.msg.Mouse, queue_size=10)
        self.key_press_pub = rospy.Publisher(
            '~key_press', justhink_msgs.msg.Key, queue_size=10)
        self.key_release_pub = rospy.Publisher(
            '~key_release', justhink_msgs.msg.Key, queue_size=10)

        # Initialise the subscribers.
        self.act_sub = rospy.Subscriber(
            '~act', justhink_msgs.msg.Action, self.act_callback)

        # Initialise messages.
        self.mouse_message = justhink_msgs.msg.Mouse()
        self.key_message = justhink_msgs.msg.Key()

        rospy.loginfo('Human application node is ready!')
        print()

    def reply_set_activity_request(self, data):
        resp = self.set_scene(name=data.name)
        log_service_response(self.set_activity_service, data, resp)
        return resp

    def reply_observe_activity_request(self, data):
        try:
            resp = self.cur_scene_name
            log_service_response(self.observe_activity_service, data, resp)
        except Exception as e:
            rospy.logerr(e)
            resp = False
        return resp

    def reply_observe_state_request(self, data):
        try:
            state = self.cur_world.env.state
            resp = make_state_message(state)
            log_service_response(self.observe_state_service, data, resp)
        except Exception as e:
            rospy.logerr(e)
            resp = False

        return resp

    def reply_pause_request(self, data):
        """TODO"""
        resp = True
        try:
            self.set_paused(data.is_paused)
            log_service_response(self.pause_service, data, resp)
        except Exception as e:
            rospy.logerr(e)
            resp = False

        return resp

    def reply_act_request(self, data):
        """TODO"""
        resp = True
        try:
            action = decode_action_message(data.action)
            self.execute_action(action)
            log_service_response(self.act_service, data, resp)
        except Exception as e:
            rospy.logerr(e)
            resp = False

        return resp

    def act_callback(self, data):
        """TODO"""
        log_heard(self.act_sub, data)
        action = decode_action_message(data)
        self.execute_action(action)

    def reply_set_robot_text_request(self, data):
        """TODO"""
        try:
            self.update_robot_text(data.text)
            self.graphics.repeat_button.set_state(ButtonWidget.ENABLED)
            # self.graphics.help_button.set_state(ButtonWidget.ENABLED)
            resp = True
        except Exception as e:
            rospy.logerr(e)
            resp = False
        log_service_response(self.set_robot_text_service, data, resp)
        return resp

    def update_robot_text(self, text):
        pyglet.clock.schedule_once(self.robot_text_callback, 0, text=text)

    def robot_text_callback(self, dt, text):
        is_robot_speaking_text = text == ROBOT_SPEAKING
        if text is not None:
            if len(text) > 0:
                if not is_robot_speaking_text:
                    text = '"{}"'.format(text)
                button_state = ButtonWidget.ENABLED
            else:
                button_state = ButtonWidget.DISABLED
            self.graphics.repeat_button.set_state(button_state)

            if not self.with_robot_text and not is_robot_speaking_text:
                text = ''
            self.graphics.robot_text_label.text = text

    def _robot_repeat_callback(self, dt):
        pyglet.clock.unschedule(self._robot_repeat_callback)
        # log_service_call(self.repeat_speech_service)
        resp = self.repeat_speech_service()
        log_service_response(self.repeat_speech_service, None, resp)

        self.graphics.repeat_button.set_state(ButtonWidget.ENABLED)

    def _robot_help_callback(self, dt):
        pyglet.clock.unschedule(self._robot_help_callback)
        resp = self.request_help_service()
        log_service_response(self.request_help_service, None, resp)

        # self.graphics.help_button.set_state(ButtonWidget.ENABLED)

    def make_mouse_message(self, x, y, buttons=mouse.LEFT, dx=0, dy=0):
        self.mouse_message.header.frame_id = self.cur_scene_name
        self.mouse_message.header.stamp = rospy.Time.now()
        self.mouse_message.header.seq += 1
        self.mouse_message.x = x
        self.mouse_message.y = y
        self.mouse_message.dx = dx
        self.mouse_message.dy = dy
        self.mouse_message.left = bool(buttons & mouse.LEFT)
        self.mouse_message.right = bool(buttons & mouse.RIGHT)

    def make_key_message(self, symbol, modifiers):
        self.key_message.header.frame_id = self.cur_scene_name
        self.key_message.header.stamp = rospy.Time.now()
        self.key_message.header.seq += 1
        self.key_message.symbol = str(symbol)
        self.key_message.modifiers = str(modifiers)

    def publish_button_press(self, name, state):
        message = make_button_message(self.cur_scene_name, name, state)
        self.button_press_pub.publish(message)
        log_publish(self.button_press_pub, message)

    def publish_mouse_motion(self, x, y, dx, dy):
        self.make_mouse_message(x, y, dx=dx, dy=dy)
        self.mouse_motion_pub.publish(self.mouse_message)

    def publish_mouse_press(self, x, y, button):
        self.make_mouse_message(x, y, button)
        self.mouse_press_pub.publish(self.mouse_message)

    def publish_mouse_release(self, x, y, button):
        self.make_mouse_message(x, y, button)
        self.mouse_release_pub.publish(self.mouse_message)

    def publish_key_press(self, symbol, modifiers):
        self.make_key_message(symbol, modifiers)
        self.key_press_pub.publish(self.key_message)

    def publish_key_release(self, symbol, modifiers):
        self.make_key_message(symbol, modifiers)
        self.key_release_pub.publish(self.key_message)

    def publish_drawing_change(self, from_data, to_data):
        message = make_drawing_message(
            self.cur_scene_name, from_data, to_data)
        self.drawing_change_pub.publish(message)
        log_publish(self.drawing_change_pub, message)

    def publish_mouse_drag(self, x, y, buttons, dx, dy):
        self.make_mouse_message(x, y, buttons, dx=dx, dy=dy)
        self.mouse_drag_pub.publish(self.mouse_message)

    def publish_state_transition(self, state, action, next_state):
        message = make_state_transition_message(
            state, action, next_state, current=self.cur_scene_name)
        self.state_transition_pub.publish(message)
        log_publish(self.state_transition_pub, message)

    def publish_activity_transition(self, cur_scene, scene):
        message = make_activity_transition_message(cur_scene, scene)
        self.activity_transition_pub.publish(message)
        log_publish(self.activity_transition_pub, message)
