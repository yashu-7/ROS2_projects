#!/usr/bin/env python3
import cv2
import rclpy
import threading
from rclpy.node import Node
import customtkinter as ctk
from cv_bridge import CvBridge
from std_msgs.msg import String
from PIL import Image as PILImage
from tkintermapview import TkinterMapView
from sensor_msgs.msg import Image, NavSatFix

class RobotGUI(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Robot GUI")
        self.geometry("800x600")
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure((0, 1), weight=1)

        # Upper frames
        self.left_frame = ctk.CTkFrame(self)
        self.left_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        self.right_frame = ctk.CTkFrame(self)
        self.right_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

        # Configure frame weights
        self.left_frame.grid_rowconfigure(0, weight=1)
        self.left_frame.grid_columnconfigure(0, weight=1)
        self.right_frame.grid_rowconfigure(0, weight=1)
        self.right_frame.grid_columnconfigure(0, weight=1)

        # Camera display
        self.camera_label = ctk.CTkLabel(self.left_frame, text="Waiting for camera feed...")
        self.camera_label.grid(row=0, column=0, sticky="nsew")

        # Map display
        self.map_view = TkinterMapView(self.right_frame, width=400, height=300, corner_radius=0)
        self.map_view.grid(row=0, column=0, sticky="nsew")
        self.map_view.set_position(12.9716, 77.5946)  # Bangalore
        self.map_view.set_zoom(25)
        self.current_marker = None

        # Bottom frame
        self.bottom_frame = ctk.CTkFrame(self)
        self.bottom_frame.grid(row=1, column=0, columnspan=2, sticky="ew", padx=10, pady=10)
        self.bottom_frame.grid_columnconfigure((0, 1, 2, 3), weight=1)
        self.bottom_frame.grid_rowconfigure((0, 1), weight=1)

        # Row 0: Cement / Loader / Disposal buttons
        self.button1 = ctk.CTkButton(self.bottom_frame, text="Cement", command=self.cement_click)
        self.button1.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.button2 = ctk.CTkButton(self.bottom_frame, text="Loader", command=self.loader_click)
        self.button2.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")
        self.button3 = ctk.CTkButton(self.bottom_frame, text="Disposal", command=self.disposal_click)
        self.button3.grid(row=0, column=2, padx=5, pady=5, sticky="nsew")

        # Row 1: Start/Stop, New Button, and Text Entry
        self.button4 = ctk.CTkButton(self.bottom_frame, text="Start/Stop", command=self.status_click)
        self.button4.grid(row=1, column=2, padx=5, pady=5, sticky="nsew")

        self.button5 = ctk.CTkButton(self.bottom_frame, text="Save map", command=self.map_save_click)
        self.button5.grid(row=1, column=1, padx=5, pady=5, sticky="nsew")

        self.entry_field = ctk.CTkEntry(self.bottom_frame, placeholder_text="Enter Map Name")
        self.entry_field.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")

        # Leave column 3 free or add more widgets later

        # Publishers will be set by external code
        self.coord_pub = None
        self.status_pub = None
        self.map_save = None

        # Define valid waypoints
        self.valid_waypoints = {'Cement', 'Loader', 'Disposal'}

    # Button callbacks with added logging and validation
    def cement_click(self):
        waypoint = 'Cement'
        print(f"Publishing '{waypoint}' to map_coordinates")
        if waypoint in self.valid_waypoints:
            msg = String(data=waypoint)
            self.coord_pub.publish(msg)
        else:
            print(f"Invalid waypoint '{waypoint}' not published")

    def loader_click(self):
        waypoint = 'Loader'
        print(f"Publishing '{waypoint}' to map_coordinates")
        if waypoint in self.valid_waypoints:
            msg = String(data=waypoint)
            self.coord_pub.publish(msg)
        else:
            print(f"Invalid waypoint '{waypoint}' not published")

    def disposal_click(self):
        waypoint = 'Disposal'
        print(f"Publishing '{waypoint}' to map_coordinates")
        if waypoint in self.valid_waypoints:
            msg = String(data=waypoint)
            self.coord_pub.publish(msg)
        else:
            print(f"Invalid waypoint '{waypoint}' not published")

    def status_click(self):
        msg = String()
        self.status = 'Start' if getattr(self, 'status', 'Stop') == 'Stop' else 'Stop'
        msg.data = self.status
        self.status_pub.publish(msg)

    def map_save_click(self):
        # map_name = self.entry_field.get().strip()
        map_name = ''
        
        if not map_name: # If the entry field is empty
            map_name = "outdoormap" # Default to "outdoormap"
            print(f"Map name field empty. Defaulting to map name: '{map_name}'")
        else:
            print(f"Using map name from entry field: '{map_name}'")

        if self.map_save: # Check if publisher is initialized
            msg = String(data=map_name)
            self.map_save.publish(msg)
            print(f"Publishing map name '{map_name}' to /map_name topic.")
        else:
            print("Map save publisher not initialized.")
            
        self.entry_field.delete(0, 'end')  # Clear the entry field after attempting to save

    def set_publisher(self, coord_pub, status_pub, map_save):
        self.coord_pub = coord_pub
        self.status_pub = status_pub
        self.map_save = map_save

    def update_camera_image(self, cv_image):
        try:
            cv_image = cv2.resize(cv_image, (640, 480))
            img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(img)
            ctk_image = ctk.CTkImage(light_image=pil_image, size=(640, 480))
            self.camera_label.configure(image=ctk_image, text="")
            self.camera_label._image = ctk_image
        except Exception as e:
            print(f"Error updating camera image: {e}")

    def update_gps_data(self, lat, lon, alt):
        try:
            if self.current_marker:
                self.current_marker.delete()
            self.map_view.set_position(lat, lon)
            self.current_marker = self.map_view.set_marker(lat, lon, text="Robot")
        except Exception as e:
            print(f"Error updating map: {e}")

class RobotGUINode(Node):
    def __init__(self, gui_app):
        super().__init__('robot_gui_node')
        self.gui = gui_app
        self.bridge = CvBridge()

        self.create_subscription(Image,    '/camera/image', self.image_callback, 10)
        self.create_subscription(NavSatFix,'/navsat',      self.gps_callback,   10)

        self.coord_pub = self.create_publisher(String, 'map_coordinates', 10)
        self.status_pub = self.create_publisher(String, 'nav_status',      10)
        self.map_save = self.create_publisher(String, 'map_name', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.gui.after(0, self.gui.update_camera_image, cv_image)

    def gps_callback(self, msg):
        self.gui.after(0, self.gui.update_gps_data,
                       msg.latitude, msg.longitude, msg.altitude)

def main(args=None):
    rclpy.init(args=args)
    gui = RobotGUI()
    node = RobotGUINode(gui)
    gui.set_publisher(node.coord_pub, node.status_pub, node.map_save)

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,))
    ros_thread.start()

    try:
        gui.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join()

if __name__ == '__main__':
    main()