import os
import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Constantes del robot
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

# METERS
DISTANCE_THRESHOLD = 0.35
SPEED = 0.1825
TURN_SPEED = 0.5
ANGLE = 20
L_ANGLE = 10

# Inicializa el modelo del TurtleBot3 desde las variables de entorno
TURTLEBOT3_MODEL = os.getenv('TURTLEBOT3_MODEL', 'burger')

class ObstacleAvoidanceNode(Node):
    def __init__(self, node_name, qos_profile):
        super().__init__(node_name)
        
        self.data = {}
        self.data["true_left"] = 0
        self.data["true_right"] = 0
        self.data["front_left"] = 0
        self.data["front_right"] = 0
        self.data["front"] = 0
        
        self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, qos_profile)

    def normalize_ranges_to_360(self, msg):
        """Normaliza los datos del LIDAR para cubrir 360 grados."""
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        message_range = msg.ranges

        # Crear un arreglo vacío para 360 grados
        normalized_ranges = [float('inf')] * 360

        # Calcular los ángulos en grados
        current_angle = math.degrees(angle_min)
        
        # Rellenar el arreglo normalizado
        for distance in message_range:
            if not math.isnan(distance):
                index = int(round(current_angle)) % 360
                normalized_ranges[index] = distance
            current_angle += math.degrees(angle_increment)

        return normalized_ranges

    def laser_scan_callback(self, msg):
        global ANGLE
        """Procesa los datos del LIDAR para detectar obstáculos."""
        message_range = self.normalize_ranges_to_360(msg)
        if not message_range:
            return

        # Filtrar los valores válidos (sin NaN ni inf) para las distancias a la izquierda
        front = [d for d in (message_range[-ANGLE:] + message_range[:ANGLE]) if not math.isnan(d) and not math.isinf(d)]
        f_right = [d for d in message_range[(-ANGLE - L_ANGLE):(-ANGLE + L_ANGLE)] if not math.isnan(d) and not math.isinf(d)]
        f_left = [d for d in message_range[(ANGLE):(ANGLE + L_ANGLE)] if not math.isnan(d) and not math.isinf(d)]
        t_right = [d for d in message_range[90:91] if not math.isnan(d) and not math.isinf(d)]
        t_left = [d for d in message_range[-90:-89] if not math.isnan(d) and not math.isinf(d)]

        # Distancias a cada lado
        distF = min(front) if len(front) > 0 else -1
        distL_f = min(f_left) if len(f_left) > 0 else -1
        distR_f = min(f_right) if len(f_right) > 0 else -1
        distL_t = min(t_left) if len(t_left) > 0 else -1
        distR_t = min(t_right) if len(t_right) > 0 else -1

        # Almacenar datos procesados
        self.data["true_left"] = distL_t
        self.data["true_right"] = distR_t
        self.data["front_left"] = distL_f
        self.data["front_right"] = distR_f
        self.data["front"] = distF

    def get_values(self):
        return self.data


def create_qos_profile():
    """Crea un perfil QoS con confiabilidad 'best effort'."""
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )


def processState(state, pub, laser):
    twist = Twist()
    dist = laser.get_values()

    if dist["front_right"] > 0.78:
        a = -TURN_SPEED * 0.18
    elif dist["front_left"] > 0.78:
        a = TURN_SPEED * 0.18
    else:
        a = 0.0

    # Obtener datos : izquierda, derecha, frente
    if state == 'ADVANCE':
        twist.linear.x = SPEED
        twist.angular.z = a
    elif state == 'LEFT':
        twist.linear.x = 0.0
        twist.angular.z = TURN_SPEED
    elif state == 'RIGHT':
        twist.linear.x = 0.0
        twist.angular.z = -TURN_SPEED
    elif state == 'IDLE':
        twist.linear.x = 0.0
        twist.angular.z = 0.0
    elif state == 'BACK':
        twist.linear.x = -SPEED * 0.4
        twist.angular.z = 0.0

    pub.publish(twist)

def nextstate(state, laser):
    dist = laser.get_values()
    # Lógica del estado
    if state == 'IDLE':
        state = 'ADVANCE'
    elif state == 'ADVANCE':
        if dist['front'] < DISTANCE_THRESHOLD:
            state = 'TURN'
    elif state == 'TURN':
        if dist['front_left'] > dist['front_right']:
            state = 'LEFT'
        else:
            state = 'RIGHT'

    elif state == 'LEFT':
        if dist['front_right'] > DISTANCE_THRESHOLD:
            state = 'IDLE'
    elif state == 'RIGHT':
        if dist['front_left'] > DISTANCE_THRESHOLD:
            state = 'IDLE'
    elif state == 'BACK':
        if dist['front'] > DISTANCE_THRESHOLD:
            state = 'IDLE'
    return state

def main():
    rclpy.init()

    qos_profile = create_qos_profile()
    laser_node = ObstacleAvoidanceNode('turtlebot_obstacle', qos_profile)

    # Configuración del publicador para controlar el robot
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    # Configuración del ejecutor multithread
    executor = MultiThreadedExecutor()
    executor.add_node(laser_node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Configurar la cámara con su propio hilo

    state = 'IDLE'
    try:
        while rclpy.ok():
            # Obtener acción más reciente desde la cámara

            # Procesar el estado del robot
            processState(state, pub,laser_node)
            state = nextstate(state, laser_node)
    except KeyboardInterrupt:
        print("Finalizando...")
    finally:
        # Apagar ROS2, cámara y limpiar nodos
        rclpy.shutdown()
        executor.shutdown()
        laser_node.destroy_node()
        #camera_handler.stop()


if __name__ == '__main__':
    print("ANLEC BOT TEST PROGRAM!!")
    print("Booting up....")
    main()
