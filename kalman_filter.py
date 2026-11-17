#!/usr/bin/env python3
"""
kalman_filter.py - Filtro de Kalman Extendido para odometría del carro (ROS 2 Jazzy)
===================================================================================
Fusiona encoders (predicción de movimiento diferencial) con MPU del carro
(corrección de heading vía giroscopio y detección de quieto).

Temas ROS:
  Suscribe:
    - /carro/telemetria (Float32MultiArray)
      Índices: [enc_l, enc_r, enc_l_filt, enc_r_filt, ax, ay, az, gx, gy, gz]
  Publica:
    - /carro/posicion_kalman (Pose2D)      -> x, y, theta estimados
    - /carro/trayectoria_kalman (Float32MultiArray) -> [x, y, theta, v, w, quieto]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray
import numpy as np
import threading


class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter')

        # --- Parámetros físicos ---
        self.declare_parameter('wheel_radius', 0.0325)   # m
        self.declare_parameter('track_width', 0.135)     # m
        self.declare_parameter('dt', 0.01)               # s (100 Hz de telemetría)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        self.dt = self.get_parameter('dt').value

        # --- Bias / offset de calibración en reposo (se resta antes de filtrar) ---
        # Valores fijos calculados a partir de datos del carro quieto
        self.declare_parameter('bias_ax', 0.045)
        self.declare_parameter('bias_ay', 0.045)
        self.declare_parameter('bias_az', 1.070)
        self.declare_parameter('bias_gx', -1.30)
        self.declare_parameter('bias_gy', 9.00)
        self.declare_parameter('bias_gz', -1.50)

        self.bias_ax = self.get_parameter('bias_ax').value
        self.bias_ay = self.get_parameter('bias_ay').value
        self.bias_az = self.get_parameter('bias_az').value
        self.bias_gx = self.get_parameter('bias_gx').value
        self.bias_gy = self.get_parameter('bias_gy').value
        self.bias_gz = self.get_parameter('bias_gz').value

        # --- Estado EKF: [x, y, theta] ---
        self.x = np.zeros(3, dtype=float)

        # --- Covarianza ---
        self.P = np.diag([0.1, 0.1, 0.1])

        # --- Ruido de proceso (encoders) ---
        self.Q = np.diag([0.02, 0.02, 0.01])

        # --- Ruido de medición (giroscopio MPU para theta) ---
        self.R = np.diag([0.03])

        # --- Integración de giroscopio para theta IMU ---
        self.theta_imu = 0.0
        self.last_imu_time = None

        # --- Estado quieto ---
        self.quieto = False

        # --- Publicadores ---
        self.pub_pos = self.create_publisher(Pose2D, '/carro/posicion_kalman', 10)
        self.pub_traj = self.create_publisher(
            Float32MultiArray, '/carro/trayectoria_kalman', 10)

        # --- Suscriptor telemetría del carro ---
        self.sub = self.create_subscription(
            Float32MultiArray, '/carro/telemetria', self.callback_telemetry, 10)

        self.lock = threading.Lock()
        self.trayectoria_x = [0.0]
        self.trayectoria_y = [0.0]

        self.get_logger().info('Kalman filter (EKF) para odometría iniciado.')

    # ------------------------------------------------------------------
    # Callback de telemetría
    # ------------------------------------------------------------------
    def callback_telemetry(self, msg):
        data = msg.data
        if len(data) < 10:
            return

        # Índices de la telemetría:
        # [0] enc_l, [1] enc_r, [2] enc_l_filt, [3] enc_r_filt,
        # [4] ax, [5] ay, [6] az, [7] gx, [8] gy, [9] gz
        wl = data[2]   # enc_l_filt (rad/s)
        wr = data[3]   # enc_r_filt (rad/s)
        ax = data[4]   # g
        ay = data[5]   # g
        az = data[6]   # g
        gx = data[7]   # dps
        gy = data[8]   # dps
        gz = data[9]   # dps

        # --- Restar bias de calibración en reposo ---
        ax_c = ax - self.bias_ax
        ay_c = ay - self.bias_ay
        az_c = az - self.bias_az
        gx_c = gx - self.bias_gx
        gy_c = gy - self.bias_gy
        gz_c = gz - self.bias_gz

        now = self.get_clock().now()
        if self.last_imu_time is None:
            self.last_imu_time = now
            self.theta_imu = self.x[2]
            return

        dt = (now - self.last_imu_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 0.2:
            self.last_imu_time = now
            return

        # --- Detección de quieto (para congelar posición y resetear drift) ---
        accel_mag = float(np.sqrt(ax_c * ax_c + ay_c * ay_c))
        giro_poco = abs(gz_c) < 1.5
        accel_poco = accel_mag < 0.15
        quieto = accel_poco and giro_poco

        # --- 1. Predicción con encoders (odometría diferencial) ---
        v = self.wheel_radius * (wl + wr) / 2.0          # m/s
        w = self.wheel_radius * (wr - wl) / self.track_width  # rad/s

        x_prev = self.x.copy()
        theta_prev = x_prev[2]

        # Ecuaciones de movimiento
        self.x[0] = x_prev[0] + v * np.cos(theta_prev) * dt
        self.x[1] = x_prev[1] + v * np.sin(theta_prev) * dt
        self.x[2] = x_prev[2] + w * dt
        self.x[2] = float(np.arctan2(np.sin(self.x[2]), np.cos(self.x[2])))

        # Jacobiano de la predicción F
        F = np.array([
            [1.0, 0.0, -v * np.sin(theta_prev) * dt],
            [0.0, 1.0,  v * np.cos(theta_prev) * dt],
            [0.0, 0.0, 1.0]
        ], dtype=float)

        # Propagar covarianza
        self.P = F @ self.P @ F.T + self.Q

        # --- 2. Corrección con giroscopio MPU (theta) ---
        gz_rad = gz_c * np.pi / 180.0
        self.theta_imu += gz_rad * dt
        self.theta_imu = float(np.arctan2(np.sin(self.theta_imu), np.cos(self.theta_imu)))

        # Medición: theta del IMU
        z = np.array([self.theta_imu], dtype=float)
        H = np.array([[0.0, 0.0, 1.0]], dtype=float)  # medimos theta directamente

        # Innovación
        y_inn = z - H @ self.x
        y_inn[0] = float(np.arctan2(np.sin(y_inn[0]), np.cos(y_inn[0])))

        # Covarianza de innovación
        S = H @ self.P @ H.T + self.R

        # Ganancia de Kalman
        K = self.P @ H.T @ np.linalg.inv(S)

        # Actualizar estado
        self.x = self.x + K @ y_inn
        self.x[2] = float(np.arctan2(np.sin(self.x[2]), np.cos(self.x[2])))

        # Actualizar covarianza (Joseph form simplificada)
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P

        # --- 3. Si está quieto: congelar posición y corregir theta al IMU ---
        if quieto:
            # Si la velocidad es casi cero, no avanzar
            if abs(v) < 0.02:
                self.x[0] = x_prev[0]
                self.x[1] = x_prev[1]
            # Forzar heading al del IMU para evitar deriva acumulada
            self.x[2] = self.theta_imu
            self.quieto = True
        else:
            self.quieto = False

        self.last_imu_time = now

        # --- 4. Publicar ---
        with self.lock:
            pose = Pose2D()
            pose.x = float(self.x[0])
            pose.y = float(self.x[1])
            pose.theta = float(self.x[2])
            self.pub_pos.publish(pose)

            traj = Float32MultiArray()
            traj.data = [
                float(self.x[0]), float(self.x[1]), float(self.x[2]),
                float(v), float(w), float(1.0 if self.quieto else 0.0)
            ]
            self.pub_traj.publish(traj)

            self.trayectoria_x.append(float(self.x[0]))
            self.trayectoria_y.append(float(self.x[1]))


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
