import rclpy
import os
import shutil
from rclpy.node import Node
from std_msgs.msg import String
from slam_toolbox.srv import SerializePoseGraph # Import for response codes
import time

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')

        self.cli = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')

        self.subscription = self.create_subscription(
            String,
            'map_name',
            self.saver_callback,
            10
        )

        self.maps_path = '/home/yaswanth/ros2_ws/src/corbis/maps'
        os.makedirs(self.maps_path, exist_ok=True) # Ensure the main maps directory exists

        self.get_logger().info("Map saver node initialized.")
        self.get_logger().info(f"Maps will be saved directly under: {self.maps_path}")
        self.get_logger().info(f"Temporary files during save will be created in: {os.getcwd()}")


    def saver_callback(self, msg):
        map_name = msg.data.strip()
        if not map_name:
            self.get_logger().error("Received empty map name, not saving.")
            return

        self.get_logger().info(f"Received save request for map: '{map_name}'")

        # slam_toolbox saves to <base_service_filename>.posegraph and .data
        # These temporary files will be created in the current working directory of this node.
        base_service_filename = os.path.join(os.getcwd(), map_name) # No extension here!
        
        self.get_logger().info(f"Requesting SLAM Toolbox to serialize map with temporary base name: {base_service_filename}")

        request = SerializePoseGraph.Request()
        request.filename = base_service_filename

        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SLAM Toolbox SerializeMap service not available. Is slam_toolbox running?")
            return

        future = self.cli.call_async(request)
        future.add_done_callback(
            lambda future_obj: self.handle_service_response(future_obj, base_service_filename, map_name)
        )

    def handle_service_response(self, future, temp_base_filename_used, map_name):
        # These are the temporary files SLAM Toolbox should have created in os.getcwd()
        temp_posegraph_file = temp_base_filename_used + ".posegraph"
        temp_data_file = temp_base_filename_used + ".data"

        try:
            response = future.result()
            
            # Check the service response result field. 0 means SUCCESS.
            if response.result != SerializePoseGraph.Response.RESULT_SUCCESS:
                self.get_logger().error(
                    f"SLAM Toolbox service call for '{map_name}' failed with SLAM Toolbox result code: {response.result}."
                )
                # If the service failed, it might not have created the files, so cleanup might not be needed here,
                # but we'll attempt it in the finally block of the outer try-except if needed.
                return # Exit early as SLAM toolbox indicated failure

            self.get_logger().info(f"SLAM Toolbox service call for '{map_name}' reported success (code: {response.result}).")
            
            # A very short delay can sometimes help ensure filesystem operations are complete,
            # especially if running on slower storage or networked filesystems.
            time.sleep(0.2) # Try with 0.2 or 0.5 if issues persist, otherwise can be removed.

            # Verify temporary files were actually created by SLAM Toolbox
            if not os.path.exists(temp_posegraph_file):
                self.get_logger().error(
                    f"Critical Error: {temp_posegraph_file} was NOT found after SLAM Toolbox reported success."
                )
                self.get_logger().info(f"For debugging, current CWD is: {os.getcwd()}")
                self.get_logger().info(f"Contents of CWD: {os.listdir(os.getcwd())}")
                raise FileNotFoundError(f"Expected temporary file {temp_posegraph_file} not found.")
            
            if not os.path.exists(temp_data_file):
                # This is a warning because sometimes a .data file might be empty or not critical for some users,
                # though usually it's expected.
                self.get_logger().warn(
                    f"Warning: {temp_data_file} was not found. The map data might be incomplete."
                )

            self.get_logger().info(f"Temporary map files found: {temp_posegraph_file}" +
                                   (f" and {temp_data_file}" if os.path.exists(temp_data_file) else " (.data missing or not found)."))

            # --- Define final destination paths directly in self.maps_path ---
            # No separate subdirectory for each map name.
            final_posegraph_path = os.path.join(self.maps_path, map_name + ".posegraph")
            final_data_path = os.path.join(self.maps_path, map_name + ".data")
            
            self.get_logger().info(f"Target directory for map files: {self.maps_path}")
            self.get_logger().info(f"Final posegraph path: {final_posegraph_path}")
            if os.path.exists(temp_data_file):
                 self.get_logger().info(f"Final data path: {final_data_path}")


            # Move the .posegraph file
            self.get_logger().info(f"Moving '{temp_posegraph_file}' to '{final_posegraph_path}'")
            shutil.move(temp_posegraph_file, final_posegraph_path)
            self.get_logger().info(f"Moved .posegraph file successfully.")

            # Move the .data file if it exists
            if os.path.exists(temp_data_file): # Re-check existence before moving
                self.get_logger().info(f"Moving '{temp_data_file}' to '{final_data_path}'")
                shutil.move(temp_data_file, final_data_path)
                self.get_logger().info(f"Moved .data file successfully.")
            else:
                self.get_logger().warn(f"Could not move '{temp_data_file}' as it was not found at the time of moving.")

            self.get_logger().info(f"Successfully saved map '{map_name}' to '{self.maps_path}'")

        except Exception as e:
            self.get_logger().error(f"Map save process for '{map_name}' encountered an error: {type(e).__name__} - {str(e)}")
            # Clean up partial temporary files if they exist from os.getcwd()
            self.get_logger().info(f"Attempting to clean up temporary files for '{map_name}' from CWD: {os.getcwd()}")
            if os.path.exists(temp_posegraph_file):
                try:
                    os.remove(temp_posegraph_file)
                    self.get_logger().info(f"Cleaned up temporary file: {temp_posegraph_file}")
                except OSError as rm_e:
                    self.get_logger().error(f"Error cleaning up {temp_posegraph_file}: {rm_e}")
            if os.path.exists(temp_data_file):
                try:
                    os.remove(temp_data_file)
                    self.get_logger().info(f"Cleaned up temporary file: {temp_data_file}")
                except OSError as rm_e:
                    self.get_logger().error(f"Error cleaning up {temp_data_file}: {rm_e}")

def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Map saver node shutting down due to user interrupt (Ctrl-C).")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()