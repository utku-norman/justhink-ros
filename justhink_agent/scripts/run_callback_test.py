#!/usr/bin/env python
# Run:
# rosrun justhink_agent run_test.py
# rostopic pub -1 /chatter std_msgs/String "data: 'Hi'"

import rospy
import sys
from std_msgs.msg import String


class simple_class:

    def __init__(self):
        self.sub = rospy.Subscriber("chatter", String, self.callback)

        self.received = []

        self.is_callback_locked = False

    def callback(self, data):
        rospy.loginfo('Message heard in callback: {}'.format(data.data))
        self.received = self.received + [data.data]

        # rospy.Timer(0, self.intervene, oneshot=True)
        # rospy.Timer(rospy.Duration(0), self.intervene, oneshot=True)
        # rospy.Timer(rospy.Duration(-1), self.intervene, oneshot=True)
        self.intervene()

    def intervene(self, event=None):

        if not self.is_callback_locked:

            self.is_callback_locked = True

            rospy.loginfo('Prev list: {}'.format(self.received))

            rospy.sleep(10)

            rospy.loginfo('New list after sleep: {}'.format(self.received))

            self.is_callback_locked = False


def main(args):
    obc = simple_class()
    rospy.init_node('simple_class', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
