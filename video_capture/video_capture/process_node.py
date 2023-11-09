import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ProcessNode(Node):
    def __init__(self):
        super().__init__('process_node')
        self.subscription = self.create_subscription(
            Image,
            'output_image_topic',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Image, 'processed_image_topic', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            processed_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            s = str(processed_image.shape)
            #self.get_logger().info("proccess node %s " %s) 
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, 'mono8')
            self.publisher.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ProcessNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
