import cv2
import math
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class BotControl(Node):
    def __init__(self):
        super().__init__("Control")
        self.sub = self.create_subscription(Image, '/seg_image', self.corners_and_center, 10)
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.time = 0.1
        self.timer = self.create_timer(self.time, self.control)
        self.twist = Twist()
        self.cx = 0
        self.cy = 0
        self.roi_center_x = 0
        self.roi_center_y = 0
    
    def corners_and_center(self, img):
        cv_image = self.bridge.imgmsg_to_cv2(img)
        roi = cv_image[300:480, 0:640]  
        roi_g = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)  
        contours, _ = cv2.findContours(roi_g, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        cv2.drawContours(roi, contours, -1, (255, 0, 0), 3)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                self.cx = int(M["m10"] / M["m00"])
                self.cy = int(M["m01"] / M["m00"])
                cv2.circle(roi, (self.cx, self.cy), 5, (255, 0, 0), -1)

        self.roi_center_x = roi.shape[1] // 2 
        self.roi_center_y = roi.shape[0] // 2 
        cv2.circle(roi, (self.roi_center_x, self.roi_center_y), 5, (0, 255, 0), -1) 
        cv2.line(roi, (self.roi_center_x, self.roi_center_y), (self.cx, self.cy), (0,0,255), 2)
        cv2.imshow('ROI', roi)
        cv2.waitKey(1)
    
    def error(self, icx, icy, cx, cy):
        error = math.sqrt((icx - cx)**2 + (icy - cy)**2)
        return error
    
    def p_control(self, kp):
        err = self.error(self.roi_center_x, self.roi_center_y, self.cx, self.cy)
        angle = kp * (self.roi_center_x - self.cx)
        self.get_logger().info(f'Error: {err}')
        return angle
    
    def control(self):
        angular_vel = self.p_control(0.003)
        self.twist.linear.x = 0.3
        self.twist.angular.z = angular_vel
        
        self.pub.publish(self.twist)

def main():
    rclpy.init()
    node = BotControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
