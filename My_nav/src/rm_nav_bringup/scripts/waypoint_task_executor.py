#!/usr/bin/env python3
import argparse
import math
import os
import yaml
import json

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


def load_yaml(path: str):
    if not os.path.exists(path):
        return {"points": {}, "tasks": {}}

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if "points" not in data:
        data["points"] = {}
    if "tasks" not in data:
        data["tasks"] = {}

    return data


class WaypointTaskExecutor(Node):
    def __init__(
        self,
        waypoint_file: str,
        command_topic: str,
        odom_topic: str,
        linear_threshold: float,
        angular_threshold: float,
        stable_duration: float,
    ):
        super().__init__("waypoint_task_executor")

        self.waypoint_file = os.path.expanduser(waypoint_file)
        self.command_topic = command_topic
        self.odom_topic = odom_topic

        self.linear_threshold = linear_threshold
        self.angular_threshold = angular_threshold
        self.stable_duration = stable_duration

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.cmd_sub = self.create_subscription(
            String,
            self.command_topic,
            self.on_command,
            10,
        )

        self.result_pub = self.create_publisher(
            String,
            "/waypoint_task_result",
            10,
        )

        self.explain_text_pub = self.create_publisher(
            String,
            "/xiaozhi_explain_text",
            10,
        )


        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.on_odom,
            20,
        )

        self.timer = self.create_timer(0.1, self.check_stable)

        self.state = "idle"
        self.current_waypoint_name = None
        self.current_goal_handle = None

        self.last_linear_speed = None
        self.last_angular_speed = None
        self.last_odom_time = None
        self.stable_since = None

        self.get_logger().info("Waypoint task executor started.")
        self.get_logger().info(f"Waypoint file: {self.waypoint_file}")
        self.get_logger().info(f"Command topic: {self.command_topic}")
        self.get_logger().info(f"Odom topic: {self.odom_topic}")

    def on_odom(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        wz = msg.twist.twist.angular.z

        self.last_linear_speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        self.last_angular_speed = abs(wz)
        self.last_odom_time = self.get_clock().now()

    def on_command(self, msg: String):
        waypoint_name = msg.data.strip()

        if not waypoint_name:
            self.get_logger().warn("Received empty waypoint command.")
            return

        if self.state != "idle":
            self.get_logger().warn(
                f"Robot is busy. Current state: {self.state}, "
                f"current waypoint: {self.current_waypoint_name}"
            )
            return

        data = load_yaml(self.waypoint_file)
        points = data.get("points", {})

        if waypoint_name not in points:
            self.get_logger().error(f"Waypoint '{waypoint_name}' not found.")
            self.get_logger().info(f"Available waypoints: {list(points.keys())}")
            return

        wp = points[waypoint_name]

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = wp.get("frame_id", "map")
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(wp["position"]["x"])
        goal.pose.pose.position.y = float(wp["position"]["y"])
        goal.pose.pose.position.z = float(wp["position"].get("z", 0.0))

        goal.pose.pose.orientation.x = float(wp["orientation"].get("x", 0.0))
        goal.pose.pose.orientation.y = float(wp["orientation"].get("y", 0.0))
        goal.pose.pose.orientation.z = float(wp["orientation"].get("z", 0.0))
        goal.pose.pose.orientation.w = float(wp["orientation"].get("w", 1.0))

        self.get_logger().info(f"Received command: go to '{waypoint_name}'")
        self.get_logger().info("Waiting for /navigate_to_pose action server...")

        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/navigate_to_pose action server is not available.")
            return

        self.state = "navigating"
        self.current_waypoint_name = waypoint_name
        self.stable_since = None

        self.get_logger().info(
            f"Sending navigation goal: {waypoint_name}, "
            f"x={goal.pose.pose.position.x:.3f}, "
            f"y={goal.pose.pose.position.y:.3f}"
        )

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal was rejected.")
            self.reset_state()
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info("Navigation goal accepted.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_navigation_result)

    def on_navigation_result(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"Navigation to '{self.current_waypoint_name}' succeeded. "
                "Waiting for robot to become stable..."
            )
            self.state = "waiting_stable"
            self.stable_since = None
        else:
            self.get_logger().warn(
                f"Navigation to '{self.current_waypoint_name}' did not succeed. "
                f"Status: {status}"
            )
            self.reset_state()

    def check_stable(self):
        if self.state != "waiting_stable":
            return

        now = self.get_clock().now()

        if self.last_odom_time is None:
            self.get_logger().warn("No odometry received yet. Cannot check stability.")
            return

        odom_age = (now - self.last_odom_time).nanoseconds / 1e9
        if odom_age > 1.0:
            self.get_logger().warn(
                f"Odometry is too old: {odom_age:.2f}s. Waiting..."
            )
            self.stable_since = None
            return

        linear_ok = self.last_linear_speed is not None and self.last_linear_speed < self.linear_threshold
        angular_ok = self.last_angular_speed is not None and self.last_angular_speed < self.angular_threshold

        if linear_ok and angular_ok:
            if self.stable_since is None:
                self.stable_since = now
                self.get_logger().info("Robot speed is low. Start stable timer...")

            stable_time = (now - self.stable_since).nanoseconds / 1e9

            if stable_time >= self.stable_duration:
                self.execute_task()
        else:
            self.stable_since = None

    def execute_task(self):
        waypoint_name = self.current_waypoint_name

        data = load_yaml(self.waypoint_file)
        tasks = data.get("tasks", {})

        default_message = f"已到达 {waypoint_name}，现在开始执行讲解任务。"
        task_info = tasks.get(waypoint_name, {})
        message = task_info.get("message", default_message)

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"到达导航点：{waypoint_name}")
        self.get_logger().info("开始执行任务：讲解")
        self.get_logger().info("-" * 60)

        for line in str(message).splitlines():
            self.get_logger().info(f"[讲解] {line}")

        self.get_logger().info("-" * 60)
        self.get_logger().info("讲解任务执行完成。")
        self.get_logger().info("=" * 60)
        
        result_msg = String()
        result_msg.data = json.dumps(
            {
                "event": "arrived",
                "waypoint": waypoint_name,
                "task": "explain",
                "message": str(message),
            },
            ensure_ascii=False,
        )

        self.result_pub.publish(result_msg)
        
        explain_msg = String()
        explain_msg.data = str(message)
        self.explain_text_pub.publish(explain_msg)        

        self.get_logger().info(
            f"已发布 /waypoint_task_result: waypoint={waypoint_name}"
        )
        self.get_logger().info(
            f"已发布 /xiaozhi_explain_text: {explain_msg.data}"
        )

        self.reset_state()

    def reset_state(self):
        self.state = "idle"
        self.current_waypoint_name = None
        self.current_goal_handle = None
        self.stable_since = None


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--file",
        default="~/Desktop/My_nav/src/rm_nav_bringup/config/waypoints/test_map.yaml",
        help="Waypoint YAML file.",
    )

    parser.add_argument(
        "--command-topic",
        default="/waypoint_task_cmd",
        help="Topic used to receive waypoint commands.",
    )

    parser.add_argument(
        "--odom-topic",
        default="/Odometry",
        help="Odometry topic used to check whether robot is stable.",
    )

    parser.add_argument(
        "--linear-threshold",
        type=float,
        default=0.03,
        help="Linear speed threshold for stable check, m/s.",
    )

    parser.add_argument(
        "--angular-threshold",
        type=float,
        default=0.05,
        help="Angular speed threshold for stable check, rad/s.",
    )

    parser.add_argument(
        "--stable-duration",
        type=float,
        default=1.5,
        help="Required stable duration after navigation succeeded, seconds.",
    )

    args = parser.parse_args()

    rclpy.init()

    node = WaypointTaskExecutor(
        waypoint_file=args.file,
        command_topic=args.command_topic,
        odom_topic=args.odom_topic,
        linear_threshold=args.linear_threshold,
        angular_threshold=args.angular_threshold,
        stable_duration=args.stable_duration,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
