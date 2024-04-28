#!/usr/bin/env python
import rospy
import actionlib
from math import sin, cos
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

def set_initial_pose(x, y, theta):
    """
    Set the initial pose of the TurtleBot3.
    """
    rospy.loginfo("Setting the initial pose of the robot.")
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.pose.pose.position.x = x
    initial_pose.pose.pose.position.y = y
    initial_pose.pose.pose.orientation.z = sin(theta/2.0)
    initial_pose.pose.pose.orientation.w = cos(theta/2.0)
    pose_pub.publish(initial_pose)

def move_to_goal(x, y, theta):
    """
    Move the robot to the specified coordinates.
    """
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = sin(theta/2.0)
    goal.target_pose.pose.orientation.w = cos(theta/2.0)

    rospy.loginfo("Sending goal location to the robot...")
    client.send_goal(goal)
    client.wait_for_result()
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("The robot reached the destination")
        return True
    else:
        rospy.loginfo("The robot failed to reach the destination")
        return False

if __name__ == '__main__':
    rospy.init_node('turtlebot3_navigation_script')
    pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
    rospy.sleep(1)  # Wait for the publisher to establish a connection to the ROS master

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Set initial pose
    set_initial_pose(-0.7, -0.0, 0)

    # Define the sequence of target locations
    locations = [(-2, 1, 0), (-1.5, -2, 0), (2, 1, 0), (0.5, -2, 0)]

    # Visit each location
    for x, y, theta in locations:
        success = move_to_goal(x, y, theta)
        if not success:
            rospy.loginfo("The robot failed to complete the mission.")
            break
