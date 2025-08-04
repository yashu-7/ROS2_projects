import os
import yaml
import rclpy
from rclpy.node import Node
from datetime import datetime
from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_listener import TransformListener

class SaveCoordinates(Node):
    def __init__(self):
        super().__init__("coordinates_saver")

        self.coord_sub = self.create_subscription(String, 'map_coordinates', self.coord_sub_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_frame = 'map'
        self.source_frame = 'base_link'

        self.waypoints = {}
        self.yaml_file_path = '/home/yaswanth/ros2_ws/src/corbis/config/waypoints.yaml'

        if not os.path.exists(self.yaml_file_path):
            with open(self.yaml_file_path, 'w') as f:
                f.write('')
            self.get_logger().info("File Created")

    def coord_sub_callback(self, msg):
        area = msg.data
        self.get_logger().info(f"Received coordinates for area: {area}")

        try:
            transform = self.get_transform(self.target_frame, self.source_frame)
            position = transform.transform.translation
            orientation = transform.transform.rotation

            self.store_waypoint(area, position, orientation)

            self.save_yaml()

        except Exception as e:
            self.get_logger().error(f"Error processing coordinates: {e}")

    def get_transform(self, target_frame: str, source_frame: str) -> TransformStamped:
        try:
            return self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except Exception as e:
            raise RuntimeError(f"Failed to get transform from {source_frame} to {target_frame}: {e}")

    def store_waypoint(self, area: str, position, orientation):
        self.waypoints[area] = {
            'position': {
                'x': position.x,
                'y': position.y,
                'z': position.z
            },
            'orientation': {
                'x': orientation.x,
                'y': orientation.y,
                'z': orientation.z,
                'w': orientation.w
            }
        }

    def save_yaml(self):
        data = {
            'metadata': {
                'Target Frame': self.target_frame,
                'Source Frame': self.source_frame,
                'timestamp': datetime.now().isoformat(),
                'total_waypoints': len(self.waypoints)
            },
            'waypoints': self.waypoints
        }

        with open(self.yaml_file_path, 'w') as f:
            yaml.dump(data, f, sort_keys=False)

        self.get_logger().info("Waypoints saved to YAML file.")

def main():
    rclpy.init()
    node = SaveCoordinates()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()