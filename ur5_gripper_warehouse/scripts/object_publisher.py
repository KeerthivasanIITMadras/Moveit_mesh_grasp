import rospy
import tf2_ros
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import TransformStamped

def link_states_callback(msg):
    # Assuming the object you want to track is at index 0 in the LinkStates message
    link_names = msg.name
    desired_link_name = "cuboid_zero_mass::link"
    if desired_link_name in link_names:
        index = link_names.index(desired_link_name)
        # Access the pose of the desired link using the index
        desired_link_pose = msg.pose[index]
    # Create a TransformStamped message
    tf_msg = TransformStamped()
    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.header.frame_id = "world"  # Replace with the parent frame ID
    tf_msg.child_frame_id = "Object_center"  # Replace with the object frame ID
    tf_msg.transform.translation.x = desired_link_pose.position.x
    tf_msg.transform.translation.y = desired_link_pose.position.y
    tf_msg.transform.translation.z = desired_link_pose.position.z
    tf_msg.transform.rotation = desired_link_pose.orientation
    rospy.loginfo("Publishing tansforms")
    # Publish the TF message
    tf_broadcaster.sendTransform(tf_msg)
    
    
if __name__ == "__main__":
    rospy.init_node("gazebo_tf_publisher")
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Subscribe to Gazebo link states topic
    rospy.Subscriber("/gazebo/link_states", LinkStates, link_states_callback)

    rospy.spin()
