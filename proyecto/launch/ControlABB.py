
import time
import socket
import math
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointPublisher(Node):

    def __init__(self, topic):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            f'/{topic}/joint_trajectory',
            10
        )

    def mover(self, posicion):
        msg = JointTrajectory()

        # EXACTAMENTE lo mismo que tu comando
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        pos_radians= [math.radians(val) for val in posicion]
        punto = JointTrajectoryPoint()
        punto.positions = pos_radians
        punto.time_from_start.sec = 0
        punto.time_from_start.nanosec = int(0.2 * 1e9)

        msg.points.append(punto)

        intentos = 0
        while self.publisher_.get_subscription_count() < 1:
            time.sleep(0.1) # Espera 100ms
            intentos += 1
            if intentos > 50: # Timeout tras 5 segundos
                self.get_logger().error('No se encontró ningún suscriptor (Gazebo). ¿Está el controlador activo?')
                return

        self.publisher_.publish(msg)
        #self.get_logger().info('Trayectoria enviada')

def conectarSocket(IP, Puerto):
    
    socket1=socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    socket1.connect((IP, Puerto))

    return socket1


