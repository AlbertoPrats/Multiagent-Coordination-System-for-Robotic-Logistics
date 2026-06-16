import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, PointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient # <--- NUEVO
from nav2_msgs.action import NavigateToPose # <--- NUEVO
from action_msgs.msg import GoalStatus
import time

class RobotCommander(Node):
    def __init__(self, r_id, initial_coords):
        super().__init__(f'robot_commander_{r_id}', # Nombre único para evitar el WARNING
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ])
        
        self.r_id = r_id
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.current_pos = [initial_coords['x'], initial_coords['y'], initial_coords['w']]
        # Cliente de Acción para navegación
        self._nav_client = ActionClient(self, NavigateToPose, f'/{r_id}/navigate_to_pose')

        # Publishers y Subscribers anteriores
        self.publishers_initial = self.create_publisher(PoseWithCovarianceStamped, f'{r_id}/initialpose', qos_profile)
        self.publishers_goal = self.create_publisher(PoseStamped, f'{r_id}/goal_pose', qos_profile)
        self.stop_rob = self.create_publisher(Twist, f'{r_id}/cmd_vel', 10)
        self.odometry_subscription = self.create_subscription(PointStamped, f'{r_id}/pos_actual', self.pos_callback,10)

        # Flag para saber si hemos llegado
        self.goal_finished = False

        # Inicialización
        self.get_logger().info(f"Iniciando robot {r_id}...")
        # Nota: En un script real, evita sleep() en __init__, pero lo mantenemos para no romper tu lógica
        time.sleep(2) 
        self.create_initial_pose(initial_coords)

    def create_initial_pose(self, initial_coords):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = initial_coords['x']
        msg.pose.pose.position.y = initial_coords['y']
        msg.pose.pose.orientation.w = initial_coords['w']
        msg.pose.covariance = [0.1] * 36
        self.publishers_initial.publish(msg)
        self.get_logger().info(f'Posicion inicial enviada para {self.r_id}')

    def create_goal_pose(self, coords):
        """Envía la meta mediante Action para poder rastrear el éxito"""
        self.goal_finished = False
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = coords['x']
        goal_msg.pose.pose.position.y = coords['y']
        goal_msg.pose.pose.orientation.w = coords['w']

        self.get_logger().info(f'Enviando {self.r_id} a meta...')
        
        # Esperamos al servidor de acciones
        self._nav_client.wait_for_server()
        
        send_goal_future = self._nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('¡Meta rechazada!')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'--- ¡EL ROBOT {self.r_id} HA LLEGADO A SU DESTINO! ---')
            self.goal_finished = True
            self.ejecutar_tras_llegada()

    def ejecutar_tras_llegada(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.stop_rob.publish(msg)

    def get_current_pos(self):
        return self.current_pos
    
    def pos_callback(self, msg):
        self.current_pos = [msg.point.x, msg.point.y, msg.point.z]

    def check_goal(self):
        return self.goal_finished

    def get_current_pos(self):
        return self.current_pos

def main():
    rclpy.init()
    
    # Ejemplo de uso
    coords_ini = {'x': 0.0, 'y': 0.0, 'w': 1.0}
    coords_fin = {'x': 2.0, 'y': 5.0, 'w': 1.0}
    
    commander = RobotCommander('robot_0', coords_ini)
    
    # Lanzar la meta
    commander.create_goal_pose(coords_fin)
    
    # Mantenemos el script vivo para procesar los callbacks
    rclpy.spin(commander)
    
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()