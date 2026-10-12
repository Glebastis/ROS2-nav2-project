#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.declare_parameter('waypoints', [0.0, 0.0, 2.5, -1.5, 5.0, 1.5, 10.0, 0.0])
        self.wps = self.get_parameter('waypoints').value
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('navigate_to_pose action server not available after 30s')
            raise SystemExit(1)
        self.send_next(0)

    def send_next(self, i):
        if i * 2 >= len(self.wps):
            self.get_logger().info('All waypoints reached')
            rclpy.shutdown()
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.wps[i * 2])
        goal.pose.pose.position.y = float(self.wps[i * 2 + 1])
        goal.pose.pose.orientation.w = 1.0
        self.get_logger().info(
            f'Sending waypoint {i}: ({self.wps[i * 2]}, {self.wps[i * 2 + 1]})')
        fut = self.client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self.on_accepted(f, i))

    def on_accepted(self, fut, i):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn(f'Waypoint {i} not accepted, retry in 1s')
            self.create_timer(1.0, lambda: self.send_next(i))
            return
        gh.get_result_async().add_done_callback(lambda f: self.on_result(f, i))

    def on_result(self, fut, i):
        status = fut.result().status
        # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if status == 4:
            self.get_logger().info(f'Waypoint {i} SUCCEEDED')
        elif status == 6:
            self.get_logger().warn(f'Waypoint {i} ABORTED, skipping')
        else:
            self.get_logger().info(f'Waypoint {i} finished with status {status}')
        self.send_next(i + 1)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
