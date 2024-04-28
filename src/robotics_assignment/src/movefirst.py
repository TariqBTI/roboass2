#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

def go_to_goal(x, y, theta):
    # Define the action client (to send move base goals)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Define the goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    q_angle = quaternion_from_euler(0, 0, theta)
    quaternion = q_angle

    # Orientation of the goal
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    # Position of the goal
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # Send the goal to the action server and wait for it to be reached
    client.send_goal(goal)
    wait = client.wait_for_result()

    # Check if the robot reached its goal
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        result = go_to_goal(-2, 1, 0)  # x, y, theta
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
