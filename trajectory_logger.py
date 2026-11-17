#!/usr/bin/env python3
"""
trajectory_logger.py - Logger de trayectoria cruda + filtrada + Kalman (ROS 2 Jazzy)
======================================================================================
Guarda en un solo archivo CSV todos los datos del carro para comparar configuraciones:

  - CRUDOS del MPU (ax, ay, az, gx, gy, gz)
  - CRUDOS y FILTRADOS FIR de encoders (enc_l, enc_r, enc_l_filt, enc_r_filt)
  - ESTIMADOS por Kalman (kal_x, kal_y, kal_theta, kal_v, kal_w, kal_quieto)

Columnas del CSV:
  configuracion, t_ms, seq,
  enc_l_crudo, enc_r_crudo, enc_l_fir, enc_r_fir,
  ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps,
  kal_x, kal_y, kal_theta, kal_v, kal_w, kal_quieto

Uso:
    source /opt/ros/jazzy/setup.bash
    python3 trajectory_logger.py --ros-args -p configuracion:=crudo
    python3 trajectory_logger.py --ros-args -p configuracion:=fir
    python3 trajectory_logger.py --ros-args -p configuracion:=kalman
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray
import csv
import os
from datetime import datetime


class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')

        # Parámetro de configuración: crudo, fir, kalman
        self.declare_parameter('configuracion', 'crudo')
        self.configuracion = self.get_parameter('configuracion').value

        # Ruta del archivo en la carpeta laboratorio_3_toma_de_datos
        self.log_dir = '/media/federico/Nuevo_vol/7mo_semestre/procesamiento_de_senales/lab/laboratorio_3_toma_de_datos'
        os.makedirs(self.log_dir, exist_ok=True)

        # Nombre del archivo incluye la configuración y timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.log_dir,
            f'datos_{self.configuracion}_{timestamp}.csv'
        )

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'configuracion', 't_ms', 'seq',
            # Encoders crudos y filtrados FIR (rad/s)
            'enc_l_crudo', 'enc_r_crudo', 'enc_l_fir', 'enc_r_fir',
            # MPU crudo (g y dps)
            'ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps',
            # Estimación Kalman
            'kal_x', 'kal_y', 'kal_theta', 'kal_v', 'kal_w', 'kal_quieto'
        ])

        # Estado más reciente de cada tópico
        self.last_telemetry = None
        self.last_kalman = None
        self.seq = 0

        # Suscriptores
        self.sub_tele = self.create_subscription(
            Float32MultiArray, '/carro/telemetria', self.callback_telemetry, 10)
        self.sub_kalman_pos = self.create_subscription(
            Pose2D, '/carro/posicion_kalman', self.callback_kalman_pos, 10)
        self.sub_kalman_traj = self.create_subscription(
            Float32MultiArray, '/carro/trayectoria_kalman', self.callback_kalman_traj, 10)

        # Timer para escribir a 22 Hz (sincronizado con el carro)
        self.timer = self.create_timer(0.045, self.timer_callback)

        self.get_logger().info(
            f'Trajectory logger iniciado. Configuración: {self.configuracion}. '
            f'Guardando en: {self.csv_path}'
        )

    def callback_telemetry(self, msg):
        data = msg.data
        if len(data) < 10:
            return
        # Índices: [enc_l, enc_r, enc_l_filt, enc_r_filt, ax, ay, az, gx, gy, gz]
        self.last_telemetry = {
            'enc_l': data[0],
            'enc_r': data[1],
            'enc_l_filt': data[2],
            'enc_r_filt': data[3],
            'ax': data[4],
            'ay': data[5],
            'az': data[6],
            'gx': data[7],
            'gy': data[8],
            'gz': data[9],
        }

    def callback_kalman_pos(self, msg: Pose2D):
        self.last_kalman = {
            'x': msg.x,
            'y': msg.y,
            'theta': msg.theta,
        }

    def callback_kalman_traj(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) < 6:
            return
        if self.last_kalman is not None:
            self.last_kalman['v'] = data[3]
            self.last_kalman['w'] = data[4]
            self.last_kalman['quieto'] = data[5]

    def timer_callback(self):
        """Escribe una fila al CSV cada 45 ms (22 Hz) si hay datos."""
        if self.last_telemetry is None or self.last_kalman is None:
            return

        t_ms = self.get_clock().now().nanoseconds // 1_000_000

        self.writer.writerow([
            self.configuracion,
            t_ms, self.seq,
            f"{self.last_telemetry['enc_l']:.6f}",
            f"{self.last_telemetry['enc_r']:.6f}",
            f"{self.last_telemetry['enc_l_filt']:.6f}",
            f"{self.last_telemetry['enc_r_filt']:.6f}",
            f"{self.last_telemetry['ax']:.6f}",
            f"{self.last_telemetry['ay']:.6f}",
            f"{self.last_telemetry['az']:.6f}",
            f"{self.last_telemetry['gx']:.6f}",
            f"{self.last_telemetry['gy']:.6f}",
            f"{self.last_telemetry['gz']:.6f}",
            f"{self.last_kalman['x']:.6f}",
            f"{self.last_kalman['y']:.6f}",
            f"{self.last_kalman['theta']:.6f}",
            f"{self.last_kalman.get('v', 0.0):.6f}",
            f"{self.last_kalman.get('w', 0.0):.6f}",
            f"{self.last_kalman.get('quieto', 0.0):.6f}",
        ])
        self.seq += 1

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info(f'Archivo cerrado: {self.csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()