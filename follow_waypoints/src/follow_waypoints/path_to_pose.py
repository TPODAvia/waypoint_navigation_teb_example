#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import time

class PathToPoseConverter:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('path_to_pose_converter', anonymous=True)

        # Subscribe to the nav_msgs/Path topic and path_reset topic
        self.path_subscriber = rospy.Subscriber('/add_wp_list', Path, self.path_callback)
        self.path_reset_subscriber = rospy.Subscriber('/path_reset', Empty, self.path_reset_callback)

        # Publisher to publish geometry_msgs/PoseWithCovarianceStamped
        self.pose_publisher = rospy.Publisher('/add_wp_pose', PoseWithCovarianceStamped, queue_size=10)
        self.path_reset_publisher = rospy.Publisher('/path_reset', Empty, queue_size=10)

        # Create a tf broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Store the last received path for comparison and last path_reset time
        self.last_path = None
        self.last_path_reset_time = rospy.Time.now()  # Initialize with the current time
        self.last_path_reset_trigger = False

    def path_reset_callback(self, msg):
        # Update the last path_reset time when a path_reset message is received
        self.last_path_reset_time = rospy.Time.now()
        self.last_path_reset_trigger = True

    def path_callback(self, path_msg):
        # Check if the new path is the same as the last path
        if not self.last_path_reset_trigger:
            if self.last_path is not None and self.compare_paths(self.last_path, path_msg):
                rospy.loginfo("Received path is identical to the previous one. Ignoring.")
                return

        # Check if more than 5 seconds have passed since the last path_reset
        # time_since_last_reset = (rospy.Time.now() - self.last_path_reset_time).to_sec()
        # if time_since_last_reset <= 5:
        #     rospy.loginfo("Path reset was sent less than 5 seconds ago. Ignoring path processing.")
        #     return

        # Publish the path_reset message since this is a new path
        self.path_reset_publisher.publish(Empty())
        rospy.loginfo("New path received. Publishing path_reset and processing path.")

        # Store the current path as the last path
        self.last_path = path_msg
        self.last_path_reset_time = rospy.Time.now()  # Update the reset time

        # Iterate over the poses in the path and convert to PoseWithCovarianceStamped
        for idx, pose_stamped in enumerate(path_msg.poses):
            pose_with_covariance = PoseWithCovarianceStamped()

            # Copy the pose data from PoseStamped to PoseWithCovarianceStamped
            pose_with_covariance.header = pose_stamped.header
            pose_with_covariance.pose.pose = pose_stamped.pose

            # Name the frame_id with the index (1, 2, 3, etc.)
            pose_with_covariance.header.frame_id = "map"

            # Set a dummy covariance (for example, identity matrix scaled)
            pose_with_covariance.pose.covariance = [
                0.25, 0, 0, 0, 0, 0,
                0, 0.25, 0, 0, 0, 0,
                0, 0, 0.0, 0, 0, 0,
                0, 0, 0, 0.0, 0, 0,
                0, 0, 0, 0, 0.0, 0,
                0, 0, 0, 0, 0, 0.06853892326654787
            ]

            # Publish the converted pose
            self.pose_publisher.publish(pose_with_covariance)

            # Create a transform from map to frame_id (frame_1, frame_2, etc.)
            transform = TransformStamped()
            transform.header.frame_id = "map"  # map as the base frame
            transform.child_frame_id = f"frame_{idx + 1}"

            # Set the transform translation and rotation from the pose
            transform.transform.translation.x = pose_stamped.pose.position.x
            transform.transform.translation.y = pose_stamped.pose.position.y
            transform.transform.translation.z = pose_stamped.pose.position.z
            transform.transform.rotation = pose_stamped.pose.orientation

            # Publish the transform multiple times with unique timestamps
            for i in range(5):
                # Update timestamp for each iteration
                transform.header.stamp = rospy.Time.now()
                self.tf_broadcaster.sendTransform(transform)
                time.sleep(0.1)
                
        self.last_path_reset_trigger = False

    def compare_paths(self, path1, path2):
        """Compare two paths to determine if they are identical."""
        if len(path1.poses) != len(path2.poses):
            return False
        for pose1, pose2 in zip(path1.poses, path2.poses):
            if pose1.pose.position != pose2.pose.position or pose1.pose.orientation != pose2.pose.orientation:
                return False
        return True

if __name__ == '__main__':
    try:
        # Create an instance of the converter and spin
        converter = PathToPoseConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
