#!/usr/bin/env python3

# Runs the JUSThink human application.
#   rosrun justhink_scenario run_scenario.py
#

import rospy

from justhink_scenario import show_scenario


def run():
    # Initialise the scenario's ROS node.
    rospy.init_node('situation', anonymous=False)

    rospy.loginfo("Starting the human application node and its window...")
    print()

    # Show the human/app window (blocking).
    show_scenario()


if __name__ == '__main__':
    run()
