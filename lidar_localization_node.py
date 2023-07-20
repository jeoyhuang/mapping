#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from cartographer_ros_msgs.msg import TrajectoryStates

class LidarLocalizationNode(Node):
    def __init__(self):
        super().__init__("lidar_localization_node")
        self.publisher = self.create_publisher(PoseStamped, "localized_pose", 10)
        self.subscription = self.create_subscription(
            TrajectoryStates, "/trajectory_states", self.trajectory_callback, 10
        )
        self.is_slam_initialized = False

    def trajectory_callback(self, msg):
        if not self.is_slam_initialized:
            self.is_slam_initialized = True

    def publish_localized_pose(self, pose):
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_msg.header.frame_id = "map"
        pose_stamped_msg.pose = pose
        self.publisher.publish(pose_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_localization_node = LidarLocalizationNode()
    rclpy.spin(lidar_localization_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()