import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import String
from robcomp_interfaces.msg import DetectionArray, Detection
from robcomp_util.module_aruco import Aruco3d
import cv2

class ArucoNode(Node, Aruco3d): # Mude o nome da classe

    def __init__(self):
        super().__init__('aruco_node')
        self.running = True

        self.aruco = Aruco3d()

        # Subscribers
        ## Coloque aqui os subscribers
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            CompressedImage, # or Image
            'camera/image_raw/compressed', # or '/camera/image_raw'
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.flag_sub = self.create_subscription(
            String,
            '/vision/image_flag', # Mude o nome do tópico
            self.flag_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Publishers
        ## Coloque aqui os publishers
        self.detection_pub = self.create_publisher(DetectionArray, 'aruco_detection', 10)

    def flag_callback(self, msg):
        if msg.data.lower() == 'false':
            self.running = False
        elif msg.data.lower() == 'true':
            self.running = True
        print(self.running)

    def image_callback(self, msg):
        if self.running:
            #cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # if Image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # if CompressedImage

            image, resultados = self.aruco.detectaAruco(cv_image)
            self.detection_array = DetectionArray()
            self.detection_array.deteccoes = []

            for resultado in resultados:
                deteccao = Detection() # Detection 
                deteccao.classe = str(resultado['id'][0])
                cx, cy = resultado['centro']
                deteccao.cx = float(cx)
                deteccao.cy = float(cy)

                self.aruco.drawAruco(image, resultado)
                self.detection_array.deteccoes.append(deteccao)
            
               #print(f"ID {resultado['id']} Centro {deteccao.cx, deteccao.cy}")

            self.detection_pub.publish(self.detection_array) # Publicando a mensagem
            cv2.imshow('Aruco', image)
            cv2.waitKey(1)

        else:
            print('Image processing is paused')

    
def main(args=None):
    rclpy.init(args=args)
    ros_node = ArucoNode() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()