import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 
    
class ImagePub(Node):
    def __init__(self):
        super().__init__('image_pub')

        topic_name= 'input_image_topic'

        self.publisher_ = self.create_publisher(Image, topic_name , 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error('Error: Could not open the camera.')
            return
        self.br = CvBridge()

        # self.subscription = self.create_subscription(Image, topic_name, self.img_callback, 10)
        # self.subscription 
        self.br = CvBridge()


    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Error: Could not read a frame.')
            return
        s = str(frame.shape)
        #self.get_logger().info("capture node %s " %s)   
        if ret == True:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))
        #self.get_logger().info('Publishing video frame')




def main(args=None):
    rclpy.init(args=args)
    image_pub = ImagePub()
    rclpy.spin(image_pub)
    image_pub.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()
