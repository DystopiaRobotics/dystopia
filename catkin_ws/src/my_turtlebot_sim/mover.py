#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


def move():
    """Continuously move the robot by sending velocity commands
    """
    rospy.init_node('robot_mover', log_level=rospy.DEBUG)

    # We want to publish velocity message to the /cmd_vel topic
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Create the velocity message
    vel_msg = Twist()
    vel_msg.linear.x = 1.0

    # Send the message at 10Hz (10 times a second)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Publish the velocity message each tick
        # , so that our robot is continuously moving
        velocity_publisher.publish(vel_msg)
        r.sleep()


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
