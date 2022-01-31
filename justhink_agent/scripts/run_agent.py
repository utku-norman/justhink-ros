#!/usr/bin/env python3

# Runs the JUSThink robot cognition node.
#
#   rosrun justhink_agent run_agent.py
#

import rospy
import pyglet

from justhink_agent.agent import RoboticAgent


def run():
    rospy.init_node('cognition', anonymous=False)

    rospy.loginfo("Starting the robot cognition node and its window...")
    print()

    try:
        # Initialise the robot, that also initialises the window.
        robot = RoboticAgent()
        # Enter main event loop.
        pyglet.app.run()

        # rospy.loginfo("Stopping robot activity...")
    except rospy.ROSInterruptException as e:
        rospy.logwarn(e)


if __name__ == '__main__':
    run()
