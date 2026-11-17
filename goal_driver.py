#!/usr/bin/env python3
"""
goal_driver.py - Nodo de control por objetivo (ROS 2 Jazzy)
===========================================================
Permite darle al carro un objetivo de posición (x, y en metros, theta en rad).
Lee la telemetría del carro (encoders) para estimar odometría diferencial
y publica comandos Twist en /carro/cmd_vel_goal para llevar el carro al objetivo.

Temas:
  Suscribe:
    - /carro/telemetria (Float32MultiArray) -> odometría del carro
    - /carro/set_goal (geometry_msgs/Pose2D) -> nuevo objetivo (x, y, theta)
  Publica:
    - /carro/cmd_vel_goal (Twist) -> velocidades para alcanzar el objetivo
    - /carro/goal_status (String) -> estado: IDLE, MOVING, REACHED
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float32MultiArray, String
import math
import threading


class GoalDriver(Node):
    def __init__(self):
        super().__init__('goal_driver')

        # --- Parámetros de odometría ---
        self.declare_parameter('ticks_per_rev', 1000.0)   # ticks por vuelta del encoder
        self.declare_parameter('wheel_radius', 0.035)     # radio de rueda en metros
        self.declare_parameter('track_width', 0.20)       # distancia entre ruedas en metros

        # --- Parámetros de control ---
        self.declare_parameter('goal_x', 0.5)             # objetivo X en metros
        self.declare_parameter('goal_y', 0.0)             # objetivo Y en metros
        self.declare_parameter('goal_theta', 0.0)         # objetivo theta en rad
        self.declare_parameter('linear_gain', 1.0)        # ganancia P velocidad lineal
        self.declare_parameter('angular_gain', 2.0)       # ganancia P velocidad angular
        self.declare_parameter('max_linear_vel', 0.5)     # m/s
        self.declare_parameter('max_angular_vel', 1.0)    # rad/s
        self.declare_parameter('tolerance', 0.05)         # tolerancia en metros
        self.declare_parameter('angle_tolerance', 0.05)   # tolerancia en rad

        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_theta = self.get_parameter('goal_theta').value

        self.linear_gain = self.get_parameter('linear_gain').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.tolerance = self.get_parameter('tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value

        # --- Estado de odometría ---
        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.enc_l_prev = None
        self.enc_r_prev = None
        self.odom_initialized = False

        # --- Estado del objetivo ---
        self.status = 'IDLE'  # IDLE, MOVING, REACHED

        # --- Suscriptores ---
        self.sub_telemetry = self.create_subscription(
            Float32MultiArray, '/carro/telemetria', self.callback_telemetry, 10)
        self.sub_set_goal = self.create_subscription(
            Pose2D, '/carro/set_goal', self.callback_set_goal, 10)

        # --- Publicadores ---
        self.pub_cmd_vel_goal = self.create_publisher(Twist, '/carro/cmd_vel_goal', 10)
        self.pub_goal_status = self.create_publisher(String, '/carro/goal_status', 10)

        # --- Timer de control (20 Hz) ---
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'Goal driver iniciado. Objetivo inicial: ({self.goal_x:.2f}, {self.goal_y:.2f}, {self.goal_theta:.2f})'
        )

    def callback_telemetry(self, msg: Float32MultiArray):
        """Actualiza odometría diferencial a partir de los encoders del carro."""
        data = msg.data
        if len(data) < 2:
            return

        enc_l = data[0]  # encoder izquierdo (ticks o unidades)
        enc_r = data[1]  # encoder derecho

        with self.lock:
            if not self.odom_initialized:
                self.enc_l_prev = enc_l
                self.enc_r_prev = enc_r
                self.odom_initialized = True
                return

            delta_l = enc_l - self.enc_l_prev
            delta_r = enc_r - self.enc_r_prev
            self.enc_l_prev = enc_l
            self.enc_r_prev = enc_r

            # Convertir ticks a metros
            dist_l = (delta_l / self.ticks_per_rev) * (2.0 * math.pi * self.wheel_radius)
            dist_r = (delta_r / self.ticks_per_rev) * (2.0 * math.pi * self.wheel_radius)

            # Cinemática diferencial
            dc = (dist_l + dist_r) / 2.0
            dtheta = (dist_r - dist_l) / self.track_width

            # Actualizar posición (integración simple)
            self.x += dc * math.cos(self.theta + dtheta / 2.0)
            self.y += dc * math.sin(self.theta + dtheta / 2.0)
            self.theta += dtheta

            # Normalizar theta a [-pi, pi]
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def callback_set_goal(self, msg: Pose2D):
        """Establece un nuevo objetivo de posición."""
        with self.lock:
            self.goal_x = msg.x
            self.goal_y = msg.y
            self.goal_theta = msg.theta
            self.status = 'MOVING'

        self.get_logger().info(
            f'Nuevo objetivo recibido: x={self.goal_x:.2f} m, y={self.goal_y:.2f} m, theta={self.goal_theta:.2f} rad'
        )
        self.publish_status()

    def publish_status(self):
        status_msg = String()
        status_msg.data = self.status
        self.pub_goal_status.publish(status_msg)

    def control_loop(self):
        """Calcula el error al objetivo y publica comandos de velocidad."""
        with self.lock:
            x = self.x
            y = self.y
            theta = self.theta
            goal_x = self.goal_x
            goal_y = self.goal_y
            goal_theta = self.goal_theta
            status = self.status

        if status == 'IDLE':
            twist = Twist()
            self.pub_cmd_vel_goal.publish(twist)
            return

        # Calcular error de posición
        dx = goal_x - x
        dy = goal_y - y
        distance_error = math.sqrt(dx**2 + dy**2)

        # Ángulo deseado hacia el objetivo
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Error de orientación final
        final_angle_error = goal_theta - theta
        final_angle_error = math.atan2(math.sin(final_angle_error), math.cos(final_angle_error))

        # Lógica de control: primero girar hacia el objetivo, luego avanzar, luego ajustar theta
        twist = Twist()

        if distance_error > self.tolerance:
            self.status = 'MOVING'
            # Control P para giro hacia el objetivo y avance
            twist.angular.z = self.angular_gain * angle_error
            twist.linear.x = self.linear_gain * distance_error * math.cos(angle_error)

            # Limitar velocidades
            twist.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, twist.linear.x))
            twist.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, twist.angular.z))
        else:
            # Llegamos a la posición, ajustar orientación final
            if abs(final_angle_error) > self.angle_tolerance:
                self.status = 'MOVING'
                twist.linear.x = 0.0
                twist.angular.z = self.angular_gain * final_angle_error
                twist.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, twist.angular.z))
            else:
                self.status = 'REACHED'
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        self.pub_cmd_vel_goal.publish(twist)

        # Publicar estado si cambió
        status_msg = String()
        status_msg.data = self.status
        self.pub_goal_status.publish(status_msg)

        if distance_error <= self.tolerance and abs(final_angle_error) <= self.angle_tolerance:
            self.get_logger().info(f'Objetivo alcanzado: ({x:.2f}, {y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = GoalDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
