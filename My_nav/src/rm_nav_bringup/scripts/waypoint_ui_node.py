#!/usr/bin/env python3
import argparse
import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose


def load_yaml(path):
    if not os.path.exists(path):
        return {"points": {}}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if "points" not in data:
        data["points"] = {}
    return data


class WaypointUiNode(Node):
    def __init__(self, yaml_file, click_to_go, click_threshold):
        super().__init__("waypoint_ui_node")

        self.yaml_file = os.path.expanduser(yaml_file)
        self.click_to_go = click_to_go
        self.click_threshold = click_threshold

        self.points = {}
        self.last_mtime = None
        self.need_delete_all = True

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/waypoint_markers",
            10,
        )

        self.click_sub = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.on_clicked_point,
            10,
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

        self.timer = self.create_timer(1.0, self.on_timer)

        self.get_logger().info(f"Waypoint file: {self.yaml_file}")
        self.get_logger().info("Publishing markers on /waypoint_markers")

        if self.click_to_go:
            self.get_logger().info(
                f"Click-to-go enabled. Click near a waypoint within {self.click_threshold:.2f} m."
            )
        else:
            self.get_logger().info("Click-to-go disabled. Markers will only be displayed.")

    def reload_if_needed(self):
        if not os.path.exists(self.yaml_file):
            if self.points:
                self.points = {}
                self.need_delete_all = True
            return

        mtime = os.path.getmtime(self.yaml_file)
        if self.last_mtime == mtime:
            return

        data = load_yaml(self.yaml_file)
        self.points = data.get("points", {})
        self.last_mtime = mtime
        self.need_delete_all = True

        self.get_logger().info(f"Loaded {len(self.points)} waypoint(s).")

    def on_timer(self):
        self.reload_if_needed()
        self.publish_markers()

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        if self.need_delete_all:
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
            self.need_delete_all = False

        marker_id = 0

        for name, wp in sorted(self.points.items()):
            frame_id = wp.get("frame_id", "map")
            pos = wp.get("position", {})

            x = float(pos.get("x", 0.0))
            y = float(pos.get("y", 0.0))
            z = float(pos.get("z", 0.0))

            sphere = Marker()
            sphere.header.frame_id = frame_id
            sphere.header.stamp = now
            sphere.ns = "waypoint_spheres"
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = z + 0.05
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.28
            sphere.scale.y = 0.28
            sphere.scale.z = 0.28
            sphere.color.r = 0.0
            sphere.color.g = 1.0
            sphere.color.b = 0.2
            sphere.color.a = 0.9
            marker_array.markers.append(sphere)

            text = Marker()
            text.header.frame_id = frame_id
            text.header.stamp = now
            text.ns = "waypoint_text"
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = z + 0.45
            text.pose.orientation.w = 1.0
            text.scale.z = 0.28
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = name
            marker_array.markers.append(text)

            marker_id += 1

        self.marker_pub.publish(marker_array)

    def on_clicked_point(self, msg):
        if not self.click_to_go:
            self.get_logger().info("Clicked point received, but click-to-go is disabled.")
            return

        if not self.points:
            self.get_logger().warn("No waypoints loaded.")
            return

        click_frame = msg.header.frame_id or "map"
        cx = msg.point.x
        cy = msg.point.y

        best_name = None
        best_wp = None
        best_dist = None

        for name, wp in self.points.items():
            wp_frame = wp.get("frame_id", "map")

            if wp_frame != click_frame:
                continue

            pos = wp.get("position", {})
            wx = float(pos.get("x", 0.0))
            wy = float(pos.get("y", 0.0))

            dist = math.hypot(cx - wx, cy - wy)

            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_name = name
                best_wp = wp

        if best_name is None:
            self.get_logger().warn(
                f"No waypoint in clicked frame '{click_frame}'. "
                "Set RViz Fixed Frame to map."
            )
            return

        if best_dist > self.click_threshold:
            self.get_logger().warn(
                f"Clicked point is {best_dist:.2f} m away from nearest waypoint '{best_name}', "
                f"threshold is {self.click_threshold:.2f} m. Ignored."
            )
            return

        self.get_logger().info(
            f"Selected waypoint '{best_name}' by click, distance={best_dist:.2f} m."
        )
        self.send_goal(best_name, best_wp)

    def send_goal(self, name, wp):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("/navigate_to_pose action server is not available.")
            return

        pos = wp.get("position", {})
        ori = wp.get("orientation", {})

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = wp.get("frame_id", "map")
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(pos.get("x", 0.0))
        goal.pose.pose.position.y = float(pos.get("y", 0.0))
        goal.pose.pose.position.z = float(pos.get("z", 0.0))

        goal.pose.pose.orientation.x = float(ori.get("x", 0.0))
        goal.pose.pose.orientation.y = float(ori.get("y", 0.0))
        goal.pose.pose.orientation.z = float(ori.get("z", 0.0))
        goal.pose.pose.orientation.w = float(ori.get("w", 1.0))

        self.get_logger().info(
            f"Sending NavigateToPose goal: {name}, "
            f"x={goal.pose.pose.position.x:.3f}, y={goal.pose.pose.position.y:.3f}"
        )

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected.")
            return

        self.get_logger().info("Navigation goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future):
        result = future.result()
        self.get_logger().info(f"Navigation finished. Status: {result.status}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--file",
        default="~/Desktop/My_nav/src/rm_nav_bringup/config/waypoints/test_map.yaml",
        help="Waypoint YAML file.",
    )
    parser.add_argument(
        "--click-to-go",
        action="store_true",
        help="Enable RViz Publish Point click-to-go.",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.6,
        help="Click selection distance threshold in meters.",
    )

    args = parser.parse_args()

    rclpy.init()
    node = WaypointUiNode(args.file, args.click_to_go, args.threshold)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
