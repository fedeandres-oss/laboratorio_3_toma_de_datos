#!/usr/bin/env python3
"""
sensor_bridge.py - Nodo BAJO NIVEL (ROS 2 Jazzy)
==============================================
Recibe datos por UDP del Micro 1 (guante, puerto 5000) y los publica en ROS.
También responde al DISCOVER_PC del guante y envía ACK al puerto 5001.

Temas publicados:
  - /guante/imu (sensor_msgs/Imu) -> datos del MPU6050 del guante
"""

import rclpy
from rclpy.node import Node
import socket
import threading
import math

from sensor_msgs.msg import Imu


def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


class SensorBridge(Node):
    def __init__(self):
        super().__init__('sensor_bridge')

        # Publicador ROS
        self.pub_guante = self.create_publisher(Imu, '/guante/imu', 10)

        # Socket UDP para el guante (Micro 1) - puerto 5000
        self.sock_guante = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_guante.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock_guante.bind(('0.0.0.0', 5000))
            self.get_logger().info(f'Bind exitoso al puerto 5000 (guante)')
        except OSError as e:
            self.get_logger().error(f'No se pudo hacer bind al puerto 5000: {e}')
            raise

        # Hilo de recepción
        self.thread_guante = threading.Thread(target=self.recibir_guante, daemon=True)
        self.thread_guante.start()

        local_ip = get_local_ip()
        self.get_logger().info(f'IP local del PC: {local_ip}')
        self.get_logger().info(
            'Sensor bridge iniciado. Escuchando UDP 5000 (guante).'
        )

    def recibir_guante(self):
        """Hilo: recibe MPU del guante por UDP y publica en /guante/imu.
        También responde al DISCOVER_PC y envía ACK al puerto 5001 del guante."""
        while rclpy.ok():
            try:
                data, addr = self.sock_guante.recvfrom(1024)
                msg = data.decode(errors='ignore').strip()
                ip_remota = addr[0]

                if msg == "DISCOVER_PC":
                    local_ip = get_local_ip()
                    respuesta = f"PC_IP:{local_ip}\n"
                    try:
                        self.sock_guante.sendto(respuesta.encode(), (ip_remota, 5001))
                        self.get_logger().info(f'Respondido DISCOVER_PC a {ip_remota} con IP {local_ip}')
                    except Exception as e:
                        self.get_logger().warn(f'Error enviando respuesta DISCOVER_PC: {e}')
                    continue

                if msg.startswith('MPU|'):
                    self.publicar_imu(msg)
                    # ACK al puerto 5001 del guante
                    try:
                        self.sock_guante.sendto(b"ACK", (ip_remota, 5001))
                    except Exception as e:
                        self.get_logger().warn(f'Error enviando ACK al guante: {e}')
                elif msg.startswith('TEST|'):
                    try:
                        self.sock_guante.sendto(b"ACK", (ip_remota, 5001))
                    except Exception as e:
                        self.get_logger().warn(f'Error enviando ACK al guante (TEST): {e}')
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().warn(f'Error recibiendo guante: {e}')

    def publicar_imu(self, msg):
        """Convierte 'MPU|seq=1|ax=...|ay=...|az=...|gx=...|gy=...|gz=...|temp=...' a Imu"""
        try:
            parts = msg.split('|')
            datos = {}
            for p in parts[1:]:
                if '=' in p:
                    k, v = p.split('=', 1)
                    datos[k] = float(v)

            imu = Imu()
            imu.header.stamp = self.get_clock().now().to_msg()
            imu.header.frame_id = 'guante_imu'

            # El guante envia aceleracion en g y giroscopio en grados/s.
            # sensor_msgs/Imu exige m/s^2 y rad/s.
            g = 9.80665
            imu.linear_acceleration.x = datos.get('ax', 0.0) * g
            imu.linear_acceleration.y = datos.get('ay', 0.0) * g
            imu.linear_acceleration.z = datos.get('az', 0.0) * g

            imu.angular_velocity.x = math.radians(datos.get('gx', 0.0))
            imu.angular_velocity.y = math.radians(datos.get('gy', 0.0))
            imu.angular_velocity.z = math.radians(datos.get('gz', 0.0))

            self.pub_guante.publish(imu)
        except Exception as e:
            self.get_logger().warn(f'Error parseando IMU: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()