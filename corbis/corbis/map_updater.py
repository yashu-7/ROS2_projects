import os
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MapUpdater(Node):
    def __init__(self):
        super().__init__("map_updater")
        self.map_path = '/home/yaswanth/ros2_ws/src/corbis/maps'
        self.mapping_yaml_path = '/home/yaswanth/ros2_ws/src/corbis/config/slam_toolbox_mapping.yaml' # Update the 'map_file_name:' parameter with full map_path with the map_name

        self.map_name_sub = self.create_subscription(String, 'map_name', self.map_update_callback, 10)
        self.map_name_sub
    
    def map_update_callback(self, msg):
        map_full_path = os.path.join(self.map_path, msg.data)

        # Update this path to the parameter map_file_name in mapping_yaml_path

def main():
    rclpy.init()
    node = MapUpdater()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()