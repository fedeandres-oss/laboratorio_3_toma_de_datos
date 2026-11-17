#!/usr/bin/env python3
"""
movement_processor.py - Nodo MEDIO NIVEL (ROS 2 Jazzy)
======================================================
Procesa la IMU del guante y publica comandos de movimiento en /carro/cmd_vel.

ORIENTACIÓN DEL MPU6050 (según configuración del hardware):
  - Eje X negativo = ADELANTE (inclinar guante hacia adelante = ax negativo, ej. -1.0 g)
  - Eje X positivo = ATRÁS (inclinar guante hacia atrás = ax positivo, ej. +1.0 g)
  - Eje Y positivo = DERECHA (inclinar guante a la derecha = ay positivo)
  - Eje Z: cuando el sensor está quieto sobre superficie plana, debe leer ≈ -1.0 g
    (gravedad hacia abajo). Si lee +1.0, el sensor está montado al revés.

El nodo convierte m/s^2 a g dividiendo por 9.81, aplica filtros pasa-bajo,
zona muerta y ganancia para generar velocidades lineal y angular.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


class MovementProcessor(Node):
    def __init__(self):
        super().__init__('movement_processor')

        # --- Parámetros de movimiento lineal (eje X) ---
        self.declare_parameter('deadzone_x', 0.15)     # umbral en g (aumentado para evitar movimiento accidental)
        self.declare_parameter('ganancia_x', 2.0)      # escala g -> rad/s
        self.declare_parameter('max_speed', 1.0)       # rad/s máximos
        self.declare_parameter('invertir_x', True)  # True: ax negativo = adelante

        # --- Parámetros de giro (eje Y) ---
        self.declare_parameter('deadzone_y', 0.15)     # umbral en g (aumentado para evitar movimiento accidental)
        self.declare_parameter('ganancia_y', 1.5)      # escala g -> rad/s
        self.declare_parameter('max_giro', 1.0)        # rad/s máximos
        self.declare_parameter('invertir_y', True)  # True = y positivo gira a la derecha

        # --- Parámetros de filtro ---
        self.declare_parameter('alpha_filtro', 0.3)

        # Cargar parámetros
        self.deadzone_x = self.get_parameter('deadzone_x').value
        self.ganancia_x = self.get_parameter('ganancia_x').value
        self.max_speed = self.get_parameter('max_speed').value
        self.invertir_x = self.get_parameter('invertir_x').value
        
        self.deadzone_y = self.get_parameter('deadzone_y').value
        self.ganancia_y = self.get_parameter('ganancia_y').value
        self.max_giro = self.get_parameter('max_giro').value
        self.invertir_y = self.get_parameter('invertir_y').value

        self.alpha = self.get_parameter('alpha_filtro').value

        # Estados de filtros pasa-bajo
        self.x_filtrado = 0.0
        self.y_filtrado = 0.0

        # Suscriptor y publicador
        self.sub = self.create_subscription(Imu, '/guante/imu', self.callback_imu, 10)
        self.pub = self.create_publisher(Twist, '/carro/cmd_vel', 10)

        self.get_logger().info('Movement processor optimizado e iniciado correctamente.')

    def callback_imu(self, msg):
        # 1. CONVERSIÓN CRÍTICA: ROS 2 entrega linear_acceleration en m/s^2.
        # Los dividimos entre 9.81 para llevarlos a la escala de 'g' (-1.0 a +1.0 aprox).
        ax = msg.linear_acceleration.x / 9.81
        ay = msg.linear_acceleration.y / 9.81

        # --- Filtros pasa-bajo ---
        self.x_filtrado = self.alpha * ax + (1.0 - self.alpha) * self.x_filtrado
        self.y_filtrado = self.alpha * ay + (1.0 - self.alpha) * self.y_filtrado

        # --- Velocidad lineal desde eje X (con rampa suave post-deadzone) ---
        if abs(self.x_filtrado) <= self.deadzone_x:
            v_lineal = 0.0
        else:
            # Restar el deadzone para que la velocidad inicie suavemente en 0
            mag_x = abs(self.x_filtrado) - self.deadzone_x
            signo_x = 1.0 if self.x_filtrado > 0 else -1.0  # x positivo = atrás; negativo = adelante (se invierte con invertir_x)
            v_lineal = mag_x * self.ganancia_x * signo_x
            if self.invertir_x:
                v_lineal = -v_lineal

        v_lineal = max(-self.max_speed, min(self.max_speed, v_lineal))

        # --- Velocidad de giro desde eje Y ---
        if abs(self.y_filtrado) <= self.deadzone_y:
            v_giro = 0.0
        else:
            mag_y = abs(self.y_filtrado) - self.deadzone_y
            signo_y = 1.0 if self.y_filtrado > 0 else -1.0  # y positivo = derecha, y negativo = izquierda
            v_giro = mag_y * self.ganancia_y * signo_y
            if self.invertir_y:
                v_giro = -v_giro

        v_giro = max(-self.max_giro, min(self.max_giro, v_giro))

        # --- Publicar Twist ---
        twist = Twist()
        twist.linear.x = v_lineal
        twist.angular.z = v_giro
        self.pub.publish(twist)

        # Log de diagnóstico (SIEMPRE visible)
        self.get_logger().info(
            f'RAW: ax={ax:+.2f}, ay={ay:+.2f} | FILT: x={self.x_filtrado:+.2f}, y={self.y_filtrado:+.2f} --> v_lin={v_lineal:+.2f}, v_giro={v_giro:+.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MovementProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()