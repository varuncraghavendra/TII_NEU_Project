#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

BROKER = "broker.hivemq.com"
PORT = 1883
SUBSCRIBE_TOPIC = "colosseum/update"

class MQTTtoROSBridge(Node):
    def __init__(self):
        super().__init__('mqtt_to_ros_bridge')
        self.publisher_ = self.create_publisher(String, '/drone/gps', 10)

        self.client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(BROKER, PORT, 60)
        self.client.loop_start()

        self.get_logger().info("MQTT to ROS Bridge started, waiting for GPS...")

    def on_connect(self, client, userdata, flags, rc):
        print(f"[MQTT] Connected with result code {rc}")
        client.subscribe(SUBSCRIBE_TOPIC)
        print(f"[MQTT] Subscribed to topic: {SUBSCRIBE_TOPIC}")

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            lat = payload.get("lat")
            lon = payload.get("lon")

            ros_msg = String()
            ros_msg.data = json.dumps({"lat": lat, "lon": lon})
            self.publisher_.publish(ros_msg)
            self.get_logger().info(f"GPS forwarded to ROS -> lat={lat}, lon={lon}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse MQTT: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MQTTtoROSBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
