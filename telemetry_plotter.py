#!/usr/bin/env python3
"""
telemetry_plotter.py - Nodo de visualización (ROS 2 Jazzy)
==========================================================
Suscribe a /carro/telemetria (Float32MultiArray) y /carro/cmd_vel (Twist).
Grafica en tiempo real:
  - Velocidad de ruedas izquierda/derecha (real vs esperada)
  - Posición esperada (X, Y) integrando cmd_vel
  - Aceleración del carro (ax, ay, az)
  - Velocidad angular del carro (gz)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import threading
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Backend con ventana interactiva
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class TelemetryPlotter(Node):
    def __init__(self):
        super().__init__('telemetry_plotter')

        # Parámetros
        self.declare_parameter('wheel_radius', 0.065)   # metros (radio 65 mm)
        self.declare_parameter('track_width', 0.135)     # distancia entre ruedas (m) (135 mm)
        self.declare_parameter('max_history', 500)      # puntos a mostrar
        self.declare_parameter('update_interval_ms', 100)  # ms entre actualizaciones

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        self.max_history = self.get_parameter('max_history').value
        self.update_interval = self.get_parameter('update_interval_ms').value / 1000.0

        # Buffers de datos
        self.lock = threading.Lock()
        self.times = []
        self.enc_l = []
        self.enc_r = []
        self.ax = []
        self.ay = []
        self.az = []
        self.gz = []

        # Datos de cmd_vel (esperados)
        self.cmd_times = []
        self.cmd_vx = []
        self.cmd_vz = []
        self.pos_x = [0.0]   # posición esperada integrada
        self.pos_y = [0.0]
        self.theta = [0.0]

        self.last_cmd_time = None
        self.last_vx = 0.0
        self.last_vz = 0.0

        # Suscriptores
        self.sub_tele = self.create_subscription(
            Float32MultiArray, '/carro/telemetria', self.callback_telemetria, 10)
        self.sub_cmd = self.create_subscription(
            Twist, '/carro/cmd_vel', self.callback_cmd_vel, 10)

        # Configuración de gráfica
        self.fig, self.axs = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('Teleoperación - Telemetría del Carro', fontsize=14)

        self.ani = None
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info('Telemetry plotter iniciado. Cerrar la ventana de gráfica para salir.')

    def callback_telemetria(self, msg: Float32MultiArray):
        with self.lock:
            t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
            data = msg.data
            self.times.append(t)
            self.enc_l.append(data[0] if len(data) > 0 else 0.0)
            self.enc_r.append(data[1] if len(data) > 1 else 0.0)
            self.ax.append(data[4] if len(data) > 4 else 0.0)
            self.ay.append(data[5] if len(data) > 5 else 0.0)
            self.az.append(data[6] if len(data) > 6 else 0.0)
            self.gz.append(data[9] if len(data) > 9 else 0.0)

            # Mantener ventana fija
            if len(self.times) > self.max_history:
                self.times = self.times[-self.max_history:]
                self.enc_l = self.enc_l[-self.max_history:]
                self.enc_r = self.enc_r[-self.max_history:]
                self.ax = self.ax[-self.max_history:]
                self.ay = self.ay[-self.max_history:]
                self.az = self.az[-self.max_history:]
                self.gz = self.gz[-self.max_history:]

    def callback_cmd_vel(self, msg: Twist):
        with self.lock:
            t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
            vx = msg.linear.x
            vz = msg.angular.z

            # Integrar posición esperada (cinemática diferencial simple)
            if self.last_cmd_time is not None:
                dt = t - self.last_cmd_time
                if dt > 0:
                    # Usar promedio trapezoidal simple
                    vx_avg = (self.last_vx + vx) / 2.0
                    vz_avg = (self.last_vz + vz) / 2.0

                    theta_mid = self.theta[-1] + vz_avg * dt / 2.0
                    dx = vx_avg * dt * np.cos(theta_mid)
                    dy = vx_avg * dt * np.sin(theta_mid)
                    dtheta = vz_avg * dt

                    self.pos_x.append(self.pos_x[-1] + dx)
                    self.pos_y.append(self.pos_y[-1] + dy)
                    self.theta.append(self.theta[-1] + dtheta)

            self.cmd_times.append(t)
            self.cmd_vx.append(vx)
            self.cmd_vz.append(vz)
            self.last_cmd_time = t
            self.last_vx = vx
            self.last_vz = vz

            # Mantener ventana fija
            if len(self.cmd_times) > self.max_history:
                self.cmd_times = self.cmd_times[-self.max_history:]
                self.cmd_vx = self.cmd_vx[-self.max_history:]
                self.cmd_vz = self.cmd_vz[-self.max_history:]
                self.pos_x = self.pos_x[-self.max_history:]
                self.pos_y = self.pos_y[-self.max_history:]
                self.theta = self.theta[-self.max_history:]

    def update_plot(self, frame):
        with self.lock:
            times = np.array(self.times)
            enc_l = np.array(self.enc_l)
            enc_r = np.array(self.enc_r)
            ax = np.array(self.ax)
            ay = np.array(self.ay)
            az = np.array(self.az)
            gz = np.array(self.gz)

            cmd_times = np.array(self.cmd_times)
            cmd_vx = np.array(self.cmd_vx)
            cmd_vz = np.array(self.cmd_vz)
            pos_x = np.array(self.pos_x)
            pos_y = np.array(self.pos_y)

        # --- Subplot 1: Velocidad ruedas (real vs esperada) ---
        self.axs[0, 0].clear()
        if len(times) > 0:
            self.axs[0, 0].plot(times, enc_l, label='Enc L (real)', color='tab:blue')
            self.axs[0, 0].plot(times, enc_r, label='Enc R (real)', color='tab:red')
        if len(cmd_times) > 0:
            # Convertir cmd_vel a velocidad de ruedas aproximada
            vl_exp = (cmd_vx - cmd_vz * self.track_width / 2.0) / self.wheel_radius
            vr_exp = (cmd_vx + cmd_vz * self.track_width / 2.0) / self.wheel_radius
            self.axs[0, 0].plot(cmd_times, vl_exp, '--', label='VL (esperada)', color='tab:cyan', alpha=0.7)
            self.axs[0, 0].plot(cmd_times, vr_exp, '--', label='VR (esperada)', color='tab:orange', alpha=0.7)
        self.axs[0, 0].set_title('Velocidad de Ruedas')
        self.axs[0, 0].set_xlabel('t (s)')
        self.axs[0, 0].set_ylabel('Vel (units/s)')
        if len(self.axs[0, 0].get_lines()) > 0:
            self.axs[0, 0].legend(loc='upper right', fontsize=8)
        self.axs[0, 0].grid(True, alpha=0.3)

        # --- Subplot 2: Posición esperada (X, Y) ---
        self.axs[0, 1].clear()
        if len(pos_x) > 0:
            self.axs[0, 1].plot(pos_x, pos_y, '-o', label='Trayectoria esperada', color='tab:green', markersize=2)
            self.axs[0, 1].plot(pos_x[0], pos_y[0], 'go', label='Inicio')
            self.axs[0, 1].plot(pos_x[-1], pos_y[-1], 'ro', label='Actual')
        self.axs[0, 1].set_title('Posición Esperada (X-Y)')
        self.axs[0, 1].set_xlabel('X (m)')
        self.axs[0, 1].set_ylabel('Y (m)')
        if len(self.axs[0, 1].get_lines()) > 0:
            self.axs[0, 1].legend(loc='upper right', fontsize=8)
        self.axs[0, 1].grid(True, alpha=0.3)
        self.axs[0, 1].axis('equal')

        # --- Subplot 3: Aceleración del carro ---
        self.axs[1, 0].clear()
        if len(times) > 0:
            self.axs[1, 0].plot(times, ax, label='ax', color='tab:red')
            self.axs[1, 0].plot(times, ay, label='ay', color='tab:green')
            self.axs[1, 0].plot(times, az, label='az', color='tab:blue')
        self.axs[1, 0].set_title('Aceleración del Carro (IMU)')
        self.axs[1, 0].set_xlabel('t (s)')
        self.axs[1, 0].set_ylabel('a (m/s²)')
        if len(self.axs[1, 0].get_lines()) > 0:
            self.axs[1, 0].legend(loc='upper right', fontsize=8)
        self.axs[1, 0].grid(True, alpha=0.3)

        # --- Subplot 4: Velocidad angular (gz) y comando ---
        self.axs[1, 1].clear()
        if len(times) > 0:
            self.axs[1, 1].plot(times, gz, label='gz (real)', color='tab:purple')
        if len(cmd_times) > 0:
            self.axs[1, 1].plot(cmd_times, cmd_vz, '--', label='ωz (cmd)', color='tab:orange', alpha=0.7)
        self.axs[1, 1].set_title('Velocidad Angular Z')
        self.axs[1, 1].set_xlabel('t (s)')
        self.axs[1, 1].set_ylabel('ω (rad/s)')
        if len(self.axs[1, 1].get_lines()) > 0:
            self.axs[1, 1].legend(loc='upper right', fontsize=8)
        self.axs[1, 1].grid(True, alpha=0.3)

        plt.tight_layout(rect=[0, 0, 1, 0.96])
        return []

    def run(self):
        """Inicia la animación de matplotlib y el spin de ROS."""
        # Configurar animación
        self.ani = FuncAnimation(
            self.fig, self.update_plot, interval=self.update_interval * 1000, blit=False)

        # Mostrar ventana de gráfica (bloqueante)
        plt.show()

        # Después de cerrar la ventana, apagar ROS
        self.get_logger().info('Ventana de gráfica cerrada. Finalizando nodo...')


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryPlotter()
    try:
        # Ejecutar spin de ROS en hilo separado para no bloquear matplotlib
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
