#!/usr/bin/env python
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import sys
from math import tau
import tf2_ros

# Here i have to remap the joint states as moveit can only access /joint_states
joint_state_topic = ['joint_states:=/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node('move_arm')
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("ur5e_arm")
moveit_commander.roscpp_initialize(sys.argv)

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
rate = rospy.Rate(10.0)
transform = None
group.set_pose_reference_frame('ur5e_base_link')
while transform is  None:
    try:
       # Perform the transform lookup from 'your_frame' to 'base_link'
       transform = tf_buffer.lookup_transform('ur5e_base_link', 'eef_frame', rospy.Time(0))
       # 'your_frame' should be replaced with the frame you want to look up
       # Print the transform details
       rospy.loginfo("Transform from 'base_link' to 'your_frame':")
       rospy.loginfo("Translation: x={}, y={}, z={}".format(
           transform.transform.translation.x,
           transform.transform.translation.y,
           transform.transform.translation.z
       ))
       rospy.loginfo("Rotation: x={}, y={}, z={}, w={}".format(
           transform.transform.rotation.x,
           transform.transform.rotation.y,
           transform.transform.rotation.z,
           transform.transform.rotation.w
       ))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Exception occurred: {}".format(e))
    rate.sleep()
        
pose_goal = Pose()
pose_goal.orientation.w =transform.transform.rotation.w
pose_goal.orientation.z =transform.transform.rotation.z
pose_goal.orientation.y =transform.transform.rotation.y
pose_goal.orientation.x =transform.transform.rotation.x

pose_goal.position.x = transform.transform.translation.x
pose_goal.position.y = transform.transform.translation.y
pose_goal.position.z = transform.transform.translation.z
group.set_pose_target(pose_goal)


plan = group.go(wait=True)

group.stop()

group.clear_pose_targets()