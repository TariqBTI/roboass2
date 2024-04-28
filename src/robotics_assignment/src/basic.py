#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move_robot(steps, speed, time_per_step):
    # Initialize the ROS Node
    rospy.init_node('move_robot', anonymous=True)

    # Publisher to the robot's velocity command topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Set the rate of the loop
    rate = rospy.Rate(10) # 10 Hz

    # Define a Twist message to send velocity commands
    move_cmd = Twist()

    # Move forward
    move_cmd.linear.x = speed
    move_cmd.angular.z = 0.0
    rospy.loginfo("Moving forward")
    for _ in range(steps):
        pub.publish(move_cmd)
        rate.sleep()

    # Stop for a bit before moving backwards
    move_cmd.linear.x = 0
    pub.publish(move_cmd)
    rospy.sleep(2)

    # Move backward
    move_cmd.linear.x = -speed
    rospy.loginfo("Moving backward")
    for _ in range(steps):
        pub.publish(move_cmd)
        rate.sleep()

    # Stop the robot after the loop
    move_cmd.linear.x = 0
    pub.publish(move_cmd)
    rospy.loginfo("Stopped moving")

if __name__ == '__main__':
    try:
        # Define the number of steps, speed, and time per step
        num_steps = 30
        move_speed = 0.1 # meters per second
        time_per_step = 0.1 # seconds per 'step'

        move_robot(num_steps, move_speed, time_per_step)
    except rospy.ROSInterruptException:
        pass

