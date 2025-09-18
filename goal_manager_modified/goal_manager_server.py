#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from goal_manager_msgs.srv import GetNextGoal
import json
import paho.mqtt.client as mqtt
import math
import random

# MQTT settings
BROKER = "broker.hivemq.com"
PORT = 1883
TOPIC = "colosseum/update"

# Reference home GPS for NED conversion
HOME_LAT = 42.33894284868896
HOME_LON = -71.08613491058351


class GoalManagerServer(Node):
    def __init__(self):
        super().__init__('goal_manager_server')
        self.latest_gps = None  # Store latest GPS

        # MQTT client
        self.client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(BROKER, PORT, 60)
        self.client.loop_start()

        # ROS2 service
        self.srv = self.create_service(
            GetNextGoal, 'get_next_goal', self.handle_get_next_goal
        )
        self.get_logger().info("Goal Manager Server started. Waiting for GPS data...")

    # MQTT callbacks
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f"Connected to MQTT broker, subscribing to {TOPIC}")
            client.subscribe(TOPIC)
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker, code {rc}")

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            lat = float(data.get("lat"))
            lon = float(data.get("lon"))
            self.latest_gps = (lat, lon)
            self.get_logger().info(f"Received GPS -> lat={lat}, lon={lon}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse MQTT message: {e}")

    # Convert GPS to local meters relative to home
    def gps_to_local(self, lat, lon):
        dlat = lat - HOME_LAT
        dlon = lon - HOME_LON
        x = dlat * 111000  # meters
        y = dlon * 111000 * math.cos(math.radians(HOME_LAT))
        return x, y

    # ROS2 service handler
    def handle_get_next_goal(self, request, response):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        if self.latest_gps is not None:
            lat, lon = self.latest_gps
            x, y = self.gps_to_local(lat, lon)

            # Add small random offset for navigation
            pose.pose.position.x = x + random.uniform(-2, 2)
            pose.pose.position.y = y + random.uniform(-2, 2)
            self.get_logger().info(
                f"Goal from GPS -> lat={lat}, lon={lon}, "
                f"x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}"
            )
        else:
            # fallback random goal
            pose.pose.position.x = random.uniform(-10.0, 10.0)
            pose.pose.position.y = random.uniform(-10.0, 10.0)
            self.get_logger().warn("No GPS received, returning random goal")

        pose.pose.position.z = -2.0
        pose.pose.orientation.w = 1.0

        response.goal_pose = pose
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GoalManagerServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
