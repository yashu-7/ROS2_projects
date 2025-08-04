#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge
import customtkinter as ctk
import cv2
from PIL import Image as PILImage, ImageTk

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

        # Bottom frame for buttons
        self.bottom_frame = ctk.CTkFrame(self)
        self.bottom_frame.grid(row=1, column=0, columnspan=2, sticky="ew", padx=10, pady=10)
        self.bottom_frame.grid_columnconfigure((0, 1, 2), weight=1)

        # Three buttons side by side
        self.button1 = ctk.CTkButton(self.bottom_frame, text="Cement", command=self.button1_callback)
        self.button1.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        self.button2 = ctk.CTkButton(self.bottom_frame, text="Loader", command=self.button2_callback)
        self.button2.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.button3 = ctk.CTkButton(self.bottom_frame, text="Disposal", command=self.button3_callback)
        self.button3.grid(row=0, column=2, padx=5, pady=5, sticky="ew")

        # Fourth button below the three buttons
        self.button4 = ctk.CTkButton(self.bottom_frame, text="Start/Stop", command=self.button4_callback)
        self.button4.grid(row=1, column=0, columnspan=3, padx=5, pady=5, sticky="ew")

        # Labels to display camera and GPS data
        self.camera_label = ctk.CTkLabel(self.left_frame, text="Camera Feed")
        self.camera_label.pack(expand=True, fill="both", padx=10, pady=10)
        self.gps_label = ctk.CTkLabel(self.right_frame, text="GPS Data")
        self.gps_label.pack(expand=True, fill="both", padx=10, pady=10)

    def button1_callback(self):
        print("Button 1 pressed")

    def button2_callback(self):
        print("Button 2 pressed")

    def button3_callback(self):
        print("Button 3 pressed")

    def button4_callback(self):
        print("Button 4 pressed")

    def update_camera_image(self, cv_image):
        # Convert OpenCV image to PIL image
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(cv_image_rgb)
        tk_image = ImageTk.PhotoImage(pil_image)
        self.camera_label.configure(image=tk_image)
        self.camera_label.image = tk_image  # Keep a reference

    def update_gps_data(self, latitude, longitude, altitude):
        gps_text = f"Latitude: {latitude:.6f}\nLongitude: {longitude:.6f}\nAltitude: {altitude:.2f} m"
        self.gps_label.configure(text=gps_text)

class RobotGUINode(Node):
    def __init__(self, gui_app):
        super().__init__('robot_gui_node')
        self.gui_app = gui_app
        self.bridge = CvBridge()

        self.create_subscription(Image, '/camera/image', self.camera_callback, 10)
        self.create_subscription(NavSatFix, '/navsat', self.gps_callback, 10)

    def camera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.gui_app.update_camera_image(cv_image)

    def gps_callback(self, msg):
        self.gui_app.update_gps_data(msg.latitude, msg.longitude, msg.altitude)

def main(args=None):
    rclpy.init(args=args)
    gui_app = RobotGUI()
    node = RobotGUINode(gui_app)
    try:
        gui_app.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  