#!/usr/bin/env python3

# Runs the JUSThink robot cognition node.

import rospy
import pyglet

from justhink_agent.agent import RoboticAgent


def run():
    rospy.init_node('cognition', anonymous=False)

    rospy.loginfo("Starting the robot cognition node and its window...")
    print()

    try:
        # Initialize the robot, that also initializes the window.
        robot = RoboticAgent()
        # Enter main event loop.
        pyglet.app.run()
    except rospy.ROSInterruptException as e:
        rospy.logwarn(e)


if __name__ == '__main__':
    run()
