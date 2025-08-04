import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class Navigate(Node):
    def __init__(self):
        super().__init__("navigation_node")
        self.get_logger().info("Navigation node starting...")

        self.action_client_callback_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_callback_group = MutuallyExclusiveCallbackGroup()

        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.action_client_callback_group
        )
        
        self._server_ready = False
        self._check_server_timer = self.create_timer(1.0, self._check_action_server_ready)
        self.get_logger().info("Setting up timer to wait for NavigateToPose action server...")

        self.yaml_path = '/home/yaswanth/ros2_ws/src/corbis/config/waypoints.yaml'
        self.waypoints_data = {}
        self.route_names = ['Cement', 'Loader', 'Disposal']
        self.current_waypoint_index = 0 # This will track the index for resumption
        self.load_waypoints()

        self.navigation_active = False # Overall navigation state (running/paused/stopped)
        self.goal_handle = None

        self.create_subscription(
            String,
            'nav_status',
            self.status_callback,
            10,
            callback_group=self.subscriber_callback_group
        )

    def _check_action_server_ready(self):
        if self.nav_to_pose_client.server_is_ready():
            self.get_logger().info("NavigateToPose action server is ready!")
            self._server_ready = True
            self._check_server_timer.destroy()
        else:
            self.get_logger().info("NavigateToPose action server not ready yet...")

    def status_callback(self, msg):
        status = msg.data
        self.get_logger().info(f"Received navigation status: '{status}'")

        if status == "Start":
            # If navigation is already active, do nothing
            if self.navigation_active:
                self.get_logger().info("Navigation already in progress.")
                return
            
            # Check server readiness and map files (important before any start)
            if not self._server_ready:
                self.get_logger().warn("NavigateToPose action server is not ready. Cannot start navigation.")
                return
            if not self.check_map_files():
                self.get_logger().warn("Map files missing, cannot start navigation.")
                return
            if not self.waypoints_data:
                self.get_logger().error("No waypoints loaded, cannot start navigation.")
                return
            
            # Here's the change: Only reset index to 0 if it's a fresh start,
            # otherwise, we continue from the current index.
            # A simple way to check for a "fresh start" vs. "resume"
            # could be if current_waypoint_index is already 0 AND navigation_active is False.
            # However, the `self.navigation_active` check above already handles active.
            # So, if we reach here, it's either a fresh start or a resume.
            # The current_waypoint_index already holds the correct value for resumption.
            
            self.start_navigation() # This function will now handle where to start

        elif status == "Stop":
            if self.navigation_active:
                self.cancel_navigation()
                # self.navigation_active is set to False in cancel_done_callback or get_result_callback
            else:
                self.get_logger().info("No navigation in progress to stop.")

    def check_map_files(self):
        posegraph_path = '/home/yaswanth/ros2_ws/src/corbis/maps/outdoormap.posegraph'
        data_path = '/home/yaswanth/ros2_ws/src/corbis/maps/outdoormap.data'
        if not (os.path.exists(posegraph_path) and os.path.exists(data_path)):
            self.get_logger().error(f"Map files not found: {posegraph_path} or {data_path}")
            return False
        return True

    def load_waypoints(self):
        try:
            if not os.path.exists(self.yaml_path):
                 self.get_logger().error(f"Waypoints YAML file not found at: {self.yaml_path}")
                 return

            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                self.waypoints_data = data.get('waypoints', {}) 
                self.get_logger().info(f"Loaded {len(self.waypoints_data)} waypoints from {self.yaml_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {str(e)}")

    def start_navigation(self):
        """Starts or resumes the sequential navigation through waypoints."""
        self.navigation_active = True
        # The current_waypoint_index already holds the correct position for resumption
        # if 'Stop' was pressed. If it's a fresh start, it's 0.
        self.get_logger().info(f"Starting/Resuming navigation sequence from waypoint index {self.current_waypoint_index}...")
        self.navigate_to_waypoint()

    def navigate_to_waypoint(self):
        if not self.navigation_active:
            self.get_logger().info("Navigation sequence deactivated. Stopping.")
            return

        if self.current_waypoint_index >= len(self.route_names):
            self.get_logger().info("All waypoints completed.")
            self.navigation_active = False
            self.goal_handle = None
            return

        waypoint_name = self.route_names[self.current_waypoint_index]
        
        if waypoint_name not in self.waypoints_data:
            self.get_logger().warn(f"Waypoint '{waypoint_name}' not found in loaded data. Skipping to next.")
            self.current_waypoint_index += 1
            self.navigate_to_waypoint()
            return
        
        waypoint_details = self.waypoints_data[waypoint_name]
        pose_data = waypoint_details

        goal_msg = self.create_nav_goal(pose_data)
        
        self.get_logger().info(f"Navigating to waypoint '{waypoint_name}' ({self.current_waypoint_index+1}/{len(self.route_names)})")
        
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def create_nav_goal(self, pose_dict):
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = pose_dict['position']['x']
        pose.pose.position.y = pose_dict['position']['y']
        pose.pose.position.z = pose_dict['position']['z']
        
        pose.pose.orientation.x = pose_dict['orientation']['x']
        pose.pose.orientation.y = pose_dict['orientation']['y']
        pose.pose.orientation.z = pose_dict['orientation']['z']
        pose.pose.orientation.w = pose_dict['orientation']['w']
        
        goal_msg.pose = pose
        return goal_msg

    def goal_response_callback(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Exception while waiting for goal response: {e}")
            self.navigation_active = False
            return

        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal rejected by action server.")
            self.navigation_active = False
            return
            
        self.get_logger().info("Goal accepted by server.")
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        
        current_waypoint_name = self.route_names[self.current_waypoint_index]

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Waypoint '{current_waypoint_name}' reached successfully.")
            self.current_waypoint_index += 1 # Move to the next waypoint
            if self.navigation_active: # ONLY proceed if navigation is still active
                self.navigate_to_waypoint()
            else:
                self.get_logger().info("Navigation stopped externally after waypoint reached.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"Navigation to waypoint '{current_waypoint_name}' was canceled.")
            # Do NOT increment current_waypoint_index here, so it resumes from this one
            self.navigation_active = False
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f"Navigation to waypoint '{current_waypoint_name}' aborted (failed).")
            # Do NOT increment current_waypoint_index here, allowing retry if desired
            self.navigation_active = False
        else:
            self.get_logger().warn(f"Navigation to waypoint '{current_waypoint_name}' finished with status: {status}")
            # For other unexpected statuses, also don't increment, and stop active state
            self.navigation_active = False

        self.goal_handle = None

    def nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback for '{self.route_names[self.current_waypoint_index]}': Distance remaining: {feedback.distance_remaining:.2f}m")

    def cancel_navigation(self):
        if self.goal_handle and self.goal_handle.status in [
            GoalStatus.STATUS_ACCEPTED, 
            GoalStatus.STATUS_EXECUTING
            ]:
            self.get_logger().info("Attempting to cancel current navigation goal...")
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info("No active goal to cancel or goal already finished/rejected.")
            self.navigation_active = False
            self.goal_handle = None

    def cancel_done_callback(self, future):
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info("Current navigation goal successfully canceled.")
            else:
                self.get_logger().warn("Current navigation goal could not be canceled or was already finished.")
        except Exception as e:
            self.get_logger().error(f"Exception during cancel done callback: {e}")
        finally:
            self.navigation_active = False
            self.goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = Navigate()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Navigation interrupted by user (KeyboardInterrupt)")
        # On KeyboardInterrupt, ensure navigation state is cleaned up
        if node.navigation_active:
            node.cancel_navigation()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()