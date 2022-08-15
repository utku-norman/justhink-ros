#!/usr/bin/env python3

# Runs the JUSThink robot embodiment node.

import rospy

from justhink_robot.robot import PhysicalRobot


def run():
    # Initialize the agent's ROS node.
    rospy.init_node('embodiment', anonymous=False)

    rospy.loginfo("Starting a body node...")

    try:
        # Initialize the robot, that also initializes the window.
        PhysicalRobot()

        # Enter main event loop.
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logwarn(e)
    except Exception as e:
        rospy.logerr(e)


if __name__ == '__main__':
    run()
