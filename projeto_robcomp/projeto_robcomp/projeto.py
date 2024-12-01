import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
# Adicione aqui os imports necessários
from robcomp_interfaces.msg import DetectionArray, Detection
from sensor_msgs.msg import CompressedImage
from robcomp_util.odom import Odom
import numpy as np
import cv2
from cv_bridge import CvBridge

class MissaoC(Node, Odom): # Mude o nome da classe

    def __init__(self):
        super().__init__('projeto') # Mude o nome do nó
        Odom.__init__(self)

        self.robot_state = 'segue'
        self.state_machine = {
            'segue' :self.segue,
            'creepers': self.creepers,
            'anda_creepers': self.anda_creepers,
            'viu_creepers': self.viu_creepers,
            'volta_creepers': self.volta_creepers,
            'meio': self.meio,
            'anda_meio': self.anda_meio,
            'viu_meio': self.viu_meio,
            'volta_meio': self.volta_meio,
            'stop': self.stop
        }

        # Inicialização de variáveis
        self.twist = Twist()
        self.bridge = CvBridge()
        self.cyellow = {
            'lower': (20, 50, 50),
            'upper': (30, 255, 255)
        }
        self.kernel = np.ones((10,10), np.uint8)
        self.threshold = 5
        self.kp = 0.005
        self.velx = 0.3
        self.pos_init = (self.x, self.y)
        self.saiu = False
        
        self.aruco250_direita = False
        self.ja_viu_creepers = False

        self.aruco150_esquerda = False
        self.ja_viu_meio = False
        

        self.creepers = {
        'green-32': 'labirinto',
        'green-13': 'labirinto',
        'blue-11': 'labirinto',
        'blue-21': 'labirinto',
        'bicicleta': (0,0),
        'cachorro': (0,0),
        'gato': (0,0)
        }
        
        # Subscribers
        self.mobilenet_sub = self.create_subscription(
            DetectionArray,
            '/mobilenet_detection',
            self.mobilenet_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.creepers_sub = self.create_subscription(
            DetectionArray,
            '/creeper',
            self.creeper_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        )
        self.aruco_sub = self.create_subscription(
            DetectionArray,
            '/aruco_detection',
            self.aruco_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        )
        self.subcomp = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

        self.timer = self.create_timer(0.25, self.control)

    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")  # if CompressedImage
        h, w, _ = cv_image.shape
        self.w = w / 2
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, self.cyellow['lower'], self.cyellow['upper'])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(cv_image, contour, -1, [255, 0, 0], 3)

            M = cv2.moments(contour)
            self.cx = int(M["m10"] / M["m00"])
            self.cy = int(M["m01"] / M["m00"])

            cv2.circle(cv_image, (self.cx, self.cy), 5, (0, 0, 255), -1)

        else:
            self.cx = np.inf

    def mobilenet_callback(self,msg):
        for detection in msg.deteccoes:
            if detection.classe == 'cat':
                self.creepers['gato'] = (self.x , self.y)
            if detection.classe == 'dog':
                self.creepers['cachorro'] = (self.x , self.y)
            if detection.classe == 'bicycle':
                self.creepers['bicileta'] = (self.x , self.y)

    def aruco_callback(self, msg):
        for detection in msg.deteccoes:
            if detection.classe == '250':
                self.aruco250_direita = True
            elif detection.classe == '150':
                self.aruco150_esquerda = True
    
    def creeper_callback(self, msg):
        for detection in msg.deteccoes:
            creeper = detection.classe
            if creeper in self.creepers.keys():
                self.creepers[creeper] = 'creeper_place'

    def segue(self):
        dist = np.sqrt((self.x - self.pos_init[0])**2 + (self.y - self.pos_init[1])**2)
        if self.cx == np.inf:
            self.twist.angular.z = 0.4
        else:
            self.calc_erro()
            self.twist.linear.x = self.velx
            self.twist.angular.z = self.rot
        
        if self.saiu == False and dist > 0.5:
            self.saiu = True
        
        if self.saiu == True and dist < 0.5:
            self.robot_state = 'stop'

        if self.aruco250_direita == True and self.ja_viu_creepers == False:
            self.goal_yaw = self.yaw - np.pi/2
            self.tempo_inicial = self.get_clock().now()
            self.robot_state = 'creepers'

        if self.aruco150_esquerda == True and self.ja_viu_meio == False:
            self.goal_yaw = self.yaw + np.pi/2
            self.tempo_inicial = self.get_clock().now()
            self.robot_state = 'meio'
    
    def creepers(self):
        if (self.get_clock().now().nanoseconds - self.tempo_inicial.nanoseconds) < 5*1e9:
            self.twist.linear.x = self.velx
        
        else:
            erro = self.goal_yaw - self.yaw
            erro = np.arctan2(np.sin(erro), np.cos(erro))
            self.twist.angular.z = 0.3*erro
        
            if abs(erro) < np.deg2rad(8):
                self.robot_state = 'anda_creepers'
                self.tempo_inicial = self.get_clock().now()

        self.ja_viu_creepers = True

    def anda_creepers(self):
        if (self.get_clock().now().nanoseconds - self.tempo_inicial.nanoseconds) < 3*1e9:
            self.twist.linear.x = self.velx
        else:
            self.robot_state = 'viu_creepers'
            self.goal_yaw = self.yaw + 2*np.pi/3
    
    def viu_creepers(self):
        erro = self.goal_yaw - self.yaw
        erro = np.arctan2(np.sin(erro), np.cos(erro))
        self.twist.angular.z = 0.3*erro
    
        if abs(erro) < np.deg2rad(8):
            self.robot_state = 'volta_creepers'
            self.tempo_inicial = self.get_clock().now()

    def volta_creepers(self):
        if (self.get_clock().now().nanoseconds - self.tempo_inicial.nanoseconds) < 6*1e9:
            self.twist.linear.x = self.velx
        else:
            self.robot_state = 'segue'

    def meio(self):
        if (self.get_clock().now().nanoseconds - self.tempo_inicial.nanoseconds) < 5*1e9:
            self.twist.linear.x = self.velx
        
        else:
            erro = self.goal_yaw - self.yaw
            erro = np.arctan2(np.sin(erro), np.cos(erro))
            self.twist.angular.z = 0.3*erro
        
            if abs(erro) < np.deg2rad(8):
                self.robot_state = 'anda_meio'
                self.tempo_inicial = self.get_clock().now()

        self.ja_viu_meio = True

    def anda_meio(self):
        if (self.get_clock().now().nanoseconds - self.tempo_inicial.nanoseconds) < 3*1e9:
            self.twist.linear.x = self.velx
        else:
            self.robot_state = 'viu_meio'
            self.goal_yaw = self.yaw - 2*np.pi/3
    
    def viu_meio(self):
        erro = self.goal_yaw - self.yaw
        erro = np.arctan2(np.sin(erro), np.cos(erro))
        self.twist.angular.z = 0.3*erro
    
        if abs(erro) < np.deg2rad(8):
            self.robot_state = 'volta_meio'
            self.tempo_inicial = self.get_clock().now()
    
    def volta_meio(self):
        if (self.get_clock().now().nanoseconds - self.tempo_inicial.nanoseconds) < 4*1e9:
            self.twist.linear.x = self.velx
        else:
            self.robot_state = 'segue'       

    def stop(self):
        self.twist = Twist()
        print(self.creepers)

    def calc_erro(self):
        self.erro = self.w - self.cx
        self.rot = self.erro * self.kp

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = MissaoC() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()