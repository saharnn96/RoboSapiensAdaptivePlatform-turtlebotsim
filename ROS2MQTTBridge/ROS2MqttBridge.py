import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from spin_interfaces.msg import SpinPeriodicCommands
import json
import time

class ROS2MQTTBridge(Node):
    def __init__(self):
        super().__init__('ros2_mqtt_bridge')

        # ROS2 Subscriber to /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            5
        )

        # ROS2 Publisher to /cmd_vel topic
        self.publisher = self.create_publisher(SpinPeriodicCommands,
                                               '/spin_config', 5)

        self.get_logger().info(
            'ROS2 Node started and ready to bridge messages.')

    def lidar_callback(self, msg):
        # Convert LaserScan message to JSON
        lidar_data = {
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'scan_time': msg.scan_time,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'ranges': list(msg.ranges),
            # 'intensities': list(msg.intensities)
        }
        json_data = json.dumps(lidar_data)
        # Publish JSON data to MQTT
        mqtt_client.publish(mqtt_lidar_topic, json_data)
        self.get_logger().info('Published LiDAR data to MQTT.')

    def publish_spin_config(self, spin_msg):
        # Publish spin configuration to MQTT
        spin_config = SpinPeriodicCommands()
        spin_config.spin_period = spin_msg['spin_period']
        spin_config.commands = spin_msg['commands']
        self.publisher.publish(spin_msg)
        self.get_logger().info('Published Spin configuration to ROS2.')

# MQTT Callback for incoming messages
def on_message(client, userdata, msg):
    try:
        spin_msg = json.loads(msg.payload.decode())
        bridge.publish_spin(spin_msg)
        print(f"Received Twist message from MQTT and published to ROS2: {spin_msg}")
    except json.JSONDecodeError:
        print("Failed to decode MQTT message as JSON")

def retry_until_connected(mqtt_client, broker_address, port):
    # Retry connecting to MQTT broker
    while True:
        try:
            mqtt_client.connect(broker_address, port)
            break
        except:
            print("Failed to connect to MQTT broker. Retrying...")
            time.sleep(1)

if __name__ == '__main__':
    # Initialize ROS2
    rclpy.init()
    bridge = ROS2MQTTBridge()

    # Initialize MQTT client
    mqtt_client = mqtt.Client()

    # Set MQTT callback
    mqtt_client.on_message = on_message

    # MQTT Broker information
    broker_address = "127.0.0.1"  # Example broker address
    port = 1883

    # MQTT Topics
    # Make this match RaP
    mqtt_lidar_topic = "/Scan"
    mqtt_twist_topic = "mqtt/twist"
    mqtt_spin_topic = "mqtt/spin_config"

    # Connect to MQTT broker
    retry_until_connected(mqtt_client, broker_address, port)

    # Subscribe to the MQTT topic for Spin messages
    mqtt_client.subscribe(mqtt_spin_topic)

    # Subscribe to the MQTT topic for Twist messages
    # mqtt_client.subscribe(mqtt_twist_topic)

    # Start the MQTT client loop
    mqtt_client.loop_start()

    # Run the ROS2 node
    try:
        while rclpy.ok():
            rclpy.spin_once(bridge, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("Shutting down bridge.")
    finally:
        # Stop MQTT client loop and disconnect
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        # Shutdown ROS2
        bridge.destroy_node()
        rclpy.shutdown()