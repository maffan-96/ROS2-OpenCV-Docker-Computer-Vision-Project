import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ResizeNode(Node):
    def __init__(self):
        super().__init__('resize_node')
        self.subscription = self.create_subscription(
            Image,
            'input_image_topic',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Image, 'output_image_topic', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            resized_image = cv2.resize(cv_image, (300, 300))
            s = str(cv_image.shape)
            #self.get_logger().info("resize node %s " %s)   
            resized_msg = self.bridge.cv2_to_imgmsg(resized_image, 'bgr8')
            self.publisher.publish(resized_msg)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ResizeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
