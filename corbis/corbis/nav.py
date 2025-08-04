import os
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class Navigate(Node):
    def __init__(self):
        super().__init__("navigation_node")
        
        # Navigation action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Waypoint configuration
        self.yaml_path = '/home/yaswanth/ros2_ws/src/corbis/config/waypoints.yaml'
        self.waypoints = []
        self.current_waypoint_index = 0
        
        # State management
        self.navigation_active = False
        self.goal_handle = None
        
        # ROS subscribers
        self.create_subscription(String, 'nav_status', self.status_callback, 10)

    def status_callback(self, msg):
        if msg.data == "Start" and not self.navigation_active:
            if self.check_map_files():
                self.load_waypoints()
                self.start_navigation()
            else:
                self.get_logger().warn("Map files missing, cannot start navigation")
        elif msg.data == "Stop" and self.navigation_active:
            self.cancel_navigation()

    def check_map_files(self):
        return all(os.path.exists(f'/home/yaswanth/ros2_ws/src/corbis/maps/outdoormap.{ext}') 
                   for ext in ['posegraph', 'data'])

    def load_waypoints(self):
        try:
            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                self.waypoints = list(data['waypoints'].values())
                self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {str(e)}")

    def start_navigation(self):
        if not self.waypoints:
            self.get_logger().error("No waypoints loaded")
            return
            
        self.navigation_active = True
        self.current_waypoint_index = 0
        self.navigate_to_waypoint()

    def navigate_to_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints completed")
            self.navigation_active = False
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        goal_msg = self.create_nav_goal(waypoint)
        
        self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index+1}")
        self.nav_to_pose_client.wait_for_server()
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def create_nav_goal(self, waypoint):
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        pose.pose.position.x = waypoint['position']['x']
        pose.pose.position.y = waypoint['position']['y']
        pose.pose.position.z = waypoint['position']['z']
        
        # Orientation
        pose.pose.orientation.x = waypoint['orientation']['x']
        pose.pose.orientation.y = waypoint['orientation']['y']
        pose.pose.orientation.z = waypoint['orientation']['z']
        pose.pose.orientation.w = waypoint['orientation']['w']
        
        goal_msg.pose = pose
        return goal_msg

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
            
        self.get_logger().info("Goal accepted")
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index+1}")
            self.current_waypoint_index += 1
            self.navigate_to_waypoint()

    def nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f}m")

    def cancel_navigation(self):
        if self.goal_handle:
            self.get_logger().info("Canceling current navigation goal")
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done)
        self.navigation_active = False

    def cancel_done(self, future):
        self.get_logger().info("Navigation stopped")

def main():
    rclpy.init()
    node = Navigate()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cancel_navigation()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
