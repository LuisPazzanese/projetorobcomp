# https://codeshare.io/XLmkpn

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import String
import cv2
from robcomp_util.module_net import MobileNetDetector
from robcomp_interfaces.msg import DetectionArray, Detection

"""
ros2 topic pub -1 /vision/mobilenet_flag std_msgs/msg/String "{data: 'False'}"

"""

class ImageNode(Node, MobileNetDetector):

    def __init__(self):
        super().__init__('robcomp_eh_legal_mobilenet_node')
        self.running = True
        self.mobilenet = MobileNetDetector(
            args_prototxt = "/home/borg/colcon_ws/src/robcomp-util/robcomp_util/config/MobileNetSSD_deploy.prototxt.txt",
            args_model = "/home/borg/colcon_ws/src/robcomp-util/robcomp_util/config/MobileNetSSD_deploy.caffemodel"
        )

        # Subscribers
        ## Coloque aqui os subscribers
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.flag_sub = self.create_subscription(
            String,
            '/vision/mobilenet_flag', # Mude o nome do tópico
            self.flag_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        


        # Publishers
        self.detection_pub = self.create_publisher(DetectionArray, 'mobilenet_detection', 10)
        ## Coloque aqui os publishers

    def flag_callback(self, msg):
        if msg.data.lower() == 'false':
            self.running = False
        elif msg.data.lower() == 'true':
            self.running = True
        print(self.running )

    def image_callback(self, msg):
        if self.running:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # if CompressedImage

            image, resultados = self.mobilenet.detect(cv_image)

            self.detection_array = DetectionArray()
            for resultado in resultados:
                deteccao = Detection()
                deteccao.classe = resultado["classe"]
                x1,y1,x2,y2 = resultado["bbox"]
                deteccao.cx = (x1+x2)/2
                deteccao.cy = (y1+y2)/2

                self.detection_array.deteccoes.append(deteccao)
            
            self.detection_pub.publish(self.detection_array)

            cv2.imshow('Image', image)
            cv2.waitKey(1)
            
            # Faça aqui o processamento da imagem
            # ou chame uma classe ou função que faça isso
        else:
            print('Image processing is paused')

    
def main(args=None):
    rclpy.init(args=args)
    ros_node = ImageNode() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()