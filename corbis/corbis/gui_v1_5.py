#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge
import customtkinter as ctk
import cv2
from PIL import Image as PILImage, ImageTk
import threading

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

        # GPS display
        self.gps_label = ctk.CTkLabel(self.right_frame, text="Waiting for GPS data...")
        self.gps_label.grid(row=0, column=0, sticky="nsew")

        # Bottom buttons
        self.bottom_frame = ctk.CTkFrame(self)
        self.bottom_frame.grid(row=1, column=0, columnspan=2, sticky="ew", padx=10, pady=10)
        
        # Configure button grid
        self.bottom_frame.grid_columnconfigure((0, 1, 2), weight=1)
        self.bottom_frame.grid_rowconfigure((0, 1), weight=1)

        # Buttons
        self.button1 = ctk.CTkButton(self.bottom_frame, text="Cement")
        self.button1.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.button2 = ctk.CTkButton(self.bottom_frame, text="Loader")
        self.button2.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")
        self.button3 = ctk.CTkButton(self.bottom_frame, text="Disposal")
        self.button3.grid(row=0, column=2, padx=5, pady=5, sticky="nsew")
        
        self.button4 = ctk.CTkButton(self.bottom_frame, text="Start/Stop")
        self.button4.grid(row=1, column=0, columnspan=3, padx=5, pady=5, sticky="nsew")

    def update_camera_image(self, cv_image):
        """Thread-safe image update"""
        cv_image = cv2.resize(cv_image, (640, 480))
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img = PILImage.fromarray(img)
        imgtk = ImageTk.PhotoImage(image=img)
        self.camera_label.configure(image=imgtk)
        self.camera_label._image = imgtk  # Keep reference

    def update_gps_data(self, lat, lon, alt):
        """Thread-safe GPS update"""
        text = f"Latitude: {lat:.6f}\nLongitude: {lon:.6f}\nAltitude: {alt:.2f}m"
        self.gps_label.configure(text=text)

class RobotGUINode(Node):
    def __init__(self, gui_app):
        super().__init__('robot_gui_node')
        self.gui = gui_app
        self.bridge = CvBridge()

        # Verify your actual topic names!
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',  # Update with your actual camera topic
            self.image_callback,
            10
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/navsat',  # Update with your actual GPS topic
            self.gps_callback,
            10
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.gui.after(0, self.gui.update_camera_image, cv_image)
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")

    def gps_callback(self, msg):
        self.gui.after(0, self.gui.update_gps_data, 
                      msg.latitude, msg.longitude, msg.altitude)

def main(args=None):
    rclpy.init(args=args)
    gui = RobotGUI()
    node = RobotGUINode(gui)
    
    # Use separate thread for ROS2 node
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