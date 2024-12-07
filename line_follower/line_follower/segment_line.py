import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge  
from sensor_msgs.msg import Image

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('Image_Segment')
        self.sub = self.create_subscription(Image, 'camera/image', self.image_process, 10)
        self.pub = self.create_publisher(Image, '/seg_image', 10)
        self.time = 0.05
        self.timer = self.create_timer(self.time, self.send_seg)
        self.bridge = CvBridge()
        self.segmented_image = None

    def image_process(self, msg):
        image_data = np.frombuffer(msg.data, np.uint8)
        image = image_data.reshape((msg.height, msg.width, 3)) 
        image = image[:,:,::-1]
        cv2.imshow("Camera Image", image)
        self.segmented_image = self.segment(image)
        cv2.imshow('Segmented Image', self.segmented_image)
        cv2.waitKey(1)  
    
    def segment(self, img):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(img_hsv, lower_red2, upper_red2)

        mask = mask1 + mask2
        res = cv2.bitwise_and(img, img, mask=mask)
        res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        _,thresh = cv2.threshold(res, 50, 255, cv2.THRESH_BINARY)
        res = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        return res

    def send_seg(self):
        if self.segmented_image is not None:
            msg = self.bridge.cv2_to_imgmsg(self.segmented_image, encoding="bgr8")
            self.pub.publish(msg)

def main():
    rclpy.init()
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()