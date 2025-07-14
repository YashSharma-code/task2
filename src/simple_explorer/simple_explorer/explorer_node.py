import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import numpy as np
import random
import math


class SimpleExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.map = None
        self.resolution = None
        self.origin = None

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.blacklist = set()
        self.successful_poses = []
        self.navigating = False
        self.failed_attempts = 0
        self.failure_threshold = 3

    def map_callback(self, msg):
        self.map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin

    def timer_callback(self):
        if self.map is None or self.navigating:
            return

        frontier = self.find_best_frontier()
        if frontier:
            goal_pose = self.to_pose(frontier)
            self.send_goal(goal_pose)
        else:
            self.get_logger().info("🎉 Map fully explored or no reachable frontiers.")

    def find_best_frontier(self):
        frontier_points = []
        for y in range(1, self.map.shape[0] - 1):
            for x in range(1, self.map.shape[1] - 1):
                if self.map[y, x] == 0:
                    neighbors = self.map[y - 1:y + 2, x - 1:x + 2]
                    if -1 in neighbors:
                        frontier_points.append((x, y))

        # Remove blacklisted frontiers
        frontier_points = [pt for pt in frontier_points if pt not in self.blacklist]

        # Sort by distance to current robot position (optional: get from TF)
        if not frontier_points:
            return None

        return random.choice(frontier_points)  # Can replace with distance sort

    def to_pose(self, cell):
        x, y = cell
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x * self.resolution + self.origin.position.x
        pose.pose.position.y = y * self.resolution + self.origin.position.y
        pose.pose.orientation.w = 1.0
        return pose

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.current_goal = pose
        self.navigating = True

        self.get_logger().info(f"🚀 Sending goal to: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Goal rejected')
            self.navigating = False
            self.blacklist_frontier()
            return

        self.get_logger().info('✅ Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        if status == 4:  # FAILED
            self.get_logger().warn('❌ Goal FAILED')
            self.blacklist_frontier()
            self.failed_attempts += 1
        else:
            self.get_logger().info('🎯 Goal reached!')
            self.successful_poses.append(self.current_goal)
            self.failed_attempts = 0

        if self.failed_attempts >= self.failure_threshold:
            self.recover()

        self.navigating = False

    def blacklist_frontier(self):
        # Convert goal back to cell
        x = int((self.current_goal.pose.position.x - self.origin.position.x) / self.resolution)
        y = int((self.current_goal.pose.position.y - self.origin.position.y) / self.resolution)
        self.blacklist.add((x, y))

    def recover(self):
        self.get_logger().warn("🛑 Too many failed goals. Trying recovery...")

        if self.successful_poses:
            last_good = self.successful_poses[-1]
            self.get_logger().info("🔄 Backtracking to last good position.")
            self.send_goal(last_good)
        else:
            self.get_logger().warn("⚠️ No previous good poses. Stopping exploration.")



def main(args=None):
    rclpy.init(args=args)
    node = SimpleExplorer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
