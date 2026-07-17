#!/usr/bin/env python3
import argparse
import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw: float):
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw / 2.0),
        "w": math.cos(yaw / 2.0),
    }


def load_yaml(path: str):
    if not os.path.exists(path):
        return {"points": {}}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if "points" not in data:
        data["points"] = {}
    return data


def save_yaml(path: str, data: dict):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)


class WaypointRecorder(Node):
    def __init__(self, yaml_file: str, name: str, yaw: float):
        super().__init__("waypoint_recorder")
        self.yaml_file = yaml_file
        self.name = name
        self.yaw = yaw
        self.done = False

        self.sub = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.callback,
            10,
        )

        self.get_logger().info(
            f"Waiting for one /clicked_point to save as '{self.name}'. "
            "Use RViz Publish Point tool."
        )

    def callback(self, msg: PointStamped):
        data = load_yaml(self.yaml_file)

        frame_id = msg.header.frame_id or "map"
        if frame_id != "map":
            self.get_logger().warn(
                f"Clicked point frame is '{frame_id}', not 'map'. "
                "For persistent navigation points, RViz Fixed Frame should be map."
            )

        data["points"][self.name] = {
            "frame_id": frame_id,
            "position": {
                "x": float(msg.point.x),
                "y": float(msg.point.y),
                "z": 0.0,
            },
            "orientation": yaw_to_quaternion(self.yaw),
        }

        save_yaml(self.yaml_file, data)

        self.get_logger().info(
            f"Saved waypoint '{self.name}' to {self.yaml_file}: "
            f"x={msg.point.x:.3f}, y={msg.point.y:.3f}, yaw={self.yaw:.3f}"
        )
        self.done = True


class WaypointNavigator(Node):
    def __init__(self, yaml_file: str, name: str):
        super().__init__("waypoint_navigator")
        self.yaml_file = yaml_file
        self.name = name
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def go(self):
        data = load_yaml(self.yaml_file)
        points = data.get("points", {})

        if self.name not in points:
            self.get_logger().error(f"Waypoint '{self.name}' not found in {self.yaml_file}")
            self.get_logger().info(f"Available points: {list(points.keys())}")
            return False

        wp = points[self.name]

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

        self.get_logger().info("Waiting for /navigate_to_pose action server...")
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("NavigateToPose action server not available.")
            return False

        self.get_logger().info(
            f"Sending robot to '{self.name}': "
            f"x={goal.pose.pose.position.x:.3f}, y={goal.pose.pose.position.y:.3f}"
        )

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected.")
            return False

        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        self.get_logger().info(f"Navigation finished with status: {result.status}")
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--file",
        default=os.path.expanduser(
            "~/Desktop/My_nav/src/rm_nav_bringup/config/waypoints/test_map.yaml"
        ),
        help="Waypoint YAML file path.",
    )

    sub = parser.add_subparsers(dest="cmd", required=True)

    record_parser = sub.add_parser("record")
    record_parser.add_argument("name")
    record_parser.add_argument(
        "--yaw",
        type=float,
        default=0.0,
        help="Target yaw in radians. 0 means facing x direction in map frame.",
    )

    goto_parser = sub.add_parser("goto")
    goto_parser.add_argument("name")

    list_parser = sub.add_parser("list")

    args = parser.parse_args()

    if args.cmd == "list":
        data = load_yaml(args.file)
        points = data.get("points", {})
        print(f"Waypoint file: {args.file}")
        for name, wp in points.items():
            pos = wp["position"]
            print(f"- {name}: frame={wp.get('frame_id', 'map')}, x={pos['x']:.3f}, y={pos['y']:.3f}")
        return

    rclpy.init()

    if args.cmd == "record":
        node = WaypointRecorder(args.file, args.name, args.yaw)
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
        node.destroy_node()

    elif args.cmd == "goto":
        node = WaypointNavigator(args.file, args.name)
        node.go()
        node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
