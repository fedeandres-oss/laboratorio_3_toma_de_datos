#!/usr/bin/env python3
"""
carro_commander.py - Nodo ALTO NIVEL (ROS 2 Jazzy)
======================================================================
Suscribe a /carro/cmd_vel (teleoperación guante) y /carro/set_goal (posición objetivo desde GUI).
Envía comandos al Micro 2 (carro) por UDP en dos formatos:
  - Modo teleop: "v,w" (velocidad lineal m/s y angular rad/s)
  - Modo goal:   "POS,x,y,theta" (posición objetivo en metros y radianes)
Recibe telemetría del carro por UDP y la publica en /carro/telemetria.
Permite configurar una IP fija del carro mediante parámetro.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float32MultiArray, String
import socket
import threading
import time


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


def get_network_prefix(ip):
    parts = ip.split('.')
    if len(parts) == 4:
        return '.'.join(parts[:3]) + '.'
    return None


class CarroCommander(Node):
    def __init__(self):
        super().__init__('carro_commander')

        # --- Parámetros del nodo ---
        self.declare_parameter('carro_port', 5005)
        self.declare_parameter('max_linear_vel', 1.53)   # m/s (100% PWM medido)
        self.declare_parameter('max_angular_vel', 23.0)  # rad/s
        self.declare_parameter('track_width', 0.135)     # m
        self.declare_parameter('wheel_radius', 0.0325)   # m
        self.declare_parameter('left_trim', 1.00)        # ajuste fino rueda izquierda
        self.declare_parameter('right_trim', 1.16)       # ajuste fino rueda derecha (calibrado)
        self.declare_parameter('carro_ip_fija', '')      # IP fija opcional
        self.declare_parameter('control_mode', 'teleop')  # 'teleop' o 'goal'
        self.declare_parameter('test_velocity', 0.0)     # m/s (prueba directa)
        self.declare_parameter('invertir_x_goal', False)  # True: invierte X en modo objetivo
        self.declare_parameter('invertir_y_goal', False)  # True: invierte Y en modo objetivo
        self.declare_parameter('invertir_theta_goal', False)  # True: invierte Theta en modo objetivo
        self.declare_parameter('filtrar_enc', True)           # True: FIR en encoders del carro; False: crudo

        self.carro_port = self.get_parameter('carro_port').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.left_trim = self.get_parameter('left_trim').value
        self.right_trim = self.get_parameter('right_trim').value
        self.control_mode = self.get_parameter('control_mode').value
        self.test_velocity = self.get_parameter('test_velocity').value
        self.invertir_x_goal = self.get_parameter('invertir_x_goal').value
        self.invertir_y_goal = self.get_parameter('invertir_y_goal').value
        self.invertir_theta_goal = self.get_parameter('invertir_theta_goal').value
        self.filtrar_enc = self.get_parameter('filtrar_enc').value
        ip_fija = self.get_parameter('carro_ip_fija').value

        # IP del carro
        self.carro_ip = ip_fija if ip_fija else None
        self.carro_ip_lock = threading.Lock()
        self.carro_descubierto = True if ip_fija else False

        # IP local del PC
        local_ip = get_local_ip()
        self.local_ip = local_ip
        self.get_logger().info(f'IP local del PC: {local_ip}')

        # Socket UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        try:
            self.sock.bind(('0.0.0.0', self.carro_port))
            self.get_logger().info(f'Bind exitoso al puerto {self.carro_port}')
        except OSError as e:
            self.get_logger().error(f'No se pudo hacer bind al puerto {self.carro_port}: {e}')
            raise

        self.sock.settimeout(0.5)

        # --- Suscriptores y publicadores ROS 2 ---
        self.sub_cmd_teleop = self.create_subscription(
            Twist, '/carro/cmd_vel', self.callback_cmd_vel_teleop, 10)
        self.sub_set_goal = self.create_subscription(
            Pose2D, '/carro/set_goal', self.callback_set_goal, 10)
        self.sub_control_mode = self.create_subscription(
            String, '/carro/set_control_mode', self.callback_control_mode, 10)

        self.pub_telemetria = self.create_publisher(Float32MultiArray, '/carro/telemetria', 10)
        self.pub_carro_ip = self.create_publisher(String, '/carro/ip_detectada', 10)

        # --- Estado de comandos ---
        self.cmd_lock = threading.Lock()
        self.last_teleop_twist = Twist()
        self.last_goal_pose = Pose2D()
        self.goal_has_been_set = False

        # Hilos de red
        self.thread_rx = threading.Thread(target=self.recibir_telemetria, daemon=True)
        self.thread_rx.start()

        if not self.carro_ip:
            self.thread_descubrimiento = threading.Thread(target=self.descubrimiento_automatico, daemon=True)
            self.thread_descubrimiento.start()
            self.get_logger().info('Iniciando búsqueda automática de la IP del carro...')
        else:
            self.get_logger().info(f'IP del carro configurada estáticamente: {self.carro_ip}')

        self.get_logger().info(
            f'Carro commander listo. Modo: {self.control_mode}. '
            f'vmax: {self.max_linear_vel} m/s, wmax: {self.max_angular_vel} rad/s, '
            f'trim: izq={self.left_trim:.3f} der={self.right_trim:.3f}'
        )

        if self.carro_ip is not None:
            ip_msg = String()
            ip_msg.data = self.carro_ip
            self.pub_carro_ip.publish(ip_msg)

        # Timer de control (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.log_counter = 0
        self.trim_counter = 0

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def callback_cmd_vel_teleop(self, msg: Twist):
        with self.cmd_lock:
            self.last_teleop_twist = msg

    def callback_set_goal(self, msg: Pose2D):
        with self.cmd_lock:
            self.last_goal_pose = msg
            self.goal_has_been_set = True
        self.get_logger().info(
            f'OBJETIVO RECIBIDO: x={msg.x:.2f} m, y={msg.y:.2f} m, theta={msg.theta:.2f} rad'
        )

    def callback_control_mode(self, msg: String):
        new_mode = msg.data.strip().lower()
        if new_mode in ('teleop', 'goal'):
            with self.cmd_lock:
                self.control_mode = new_mode
            self.get_logger().info(f'MODO DE CONTROL CAMBIADO A: {self.control_mode}')
        else:
            self.get_logger().warn(f'Modo desconocido: {msg.data}. Use "teleop" o "goal".')

    # ------------------------------------------------------------------
    # Bucle de control principal (20 Hz)
    # ------------------------------------------------------------------
    def control_loop(self):
        with self.cmd_lock:
            mode = self.control_mode
            test_vel = self.test_velocity

        # Reenviar el trim cada 2 s para que el carro lo conserve
        # aunque se reinicie. No actualiza la cola de comandos del carro.
        self.trim_counter += 1
        if self.trim_counter % 40 == 1:
            self._enviar_comando_str(
                f"TRIM,{self.left_trim:.3f},{self.right_trim:.3f}\n"
            )
            # Reenviar estado de filtro FIR de encoders para que el carro lo conserve
            fir_val = 1 if self.filtrar_enc else 0
            self._enviar_comando_str(f"FIR,{fir_val}\n")

        # --- Prueba directa de motores (velocidad lineal en m/s) ---
        if abs(test_vel) > 0.001:
            v = max(-self.max_linear_vel, min(self.max_linear_vel, test_vel))
            comando = f"{v:.3f},0.000\n"
            self._enviar_comando_str(comando)
            self.log_counter += 1
            if self.log_counter % 40 == 0:
                self.get_logger().info(f'[TEST] v={v:.3f} m/s, w=0.000 rad/s')
            return

        if mode == 'goal':
            # --- MODO GOAL: Enviar POSICIÓN OBJETIVO ---
            with self.cmd_lock:
                pose = self.last_goal_pose
                has_goal = self.goal_has_been_set

            if not has_goal:
                self.log_counter += 1
                if self.log_counter % 40 == 0:
                    self.get_logger().debug('[goal-pos] Esperando objetivo desde la GUI...')
                return

            if self.invertir_x_goal or self.invertir_y_goal or self.invertir_theta_goal:
                gx = -pose.x if self.invertir_x_goal else pose.x
                gy = -pose.y if self.invertir_y_goal else pose.y
                gt = -pose.theta if self.invertir_theta_goal else pose.theta
                comando = f"POS,{gx:.3f},{gy:.3f},{gt:.3f}\n"
            else:
                comando = f"POS,{pose.x:.3f},{pose.y:.3f},{pose.theta:.3f}\n"
            self._enviar_comando_str(comando)
            self.log_counter += 1
            if self.log_counter % 40 == 0:
                self.get_logger().info(
                    f'[goal-pos] Enviando POS -> X:{pose.x:.2f}m, Y:{pose.y:.2f}m, '
                    f'theta:{pose.theta:.2f}rad'
                )
        else:
            # --- MODO TELEOP: Enviar VELOCIDADES (m/s, rad/s) ---
            with self.cmd_lock:
                twist = self.last_teleop_twist

            v_lineal = max(-self.max_linear_vel,
                           min(self.max_linear_vel, twist.linear.x))
            v_giro = max(-self.max_angular_vel,
                         min(self.max_angular_vel, twist.angular.z))

            comando = f"{v_lineal:.3f},{v_giro:.3f}\n"
            self._enviar_comando_str(comando)

            self.log_counter += 1
            if abs(v_lineal) > 0.01 or abs(v_giro) > 0.01 or self.log_counter % 40 == 0:
                self.get_logger().info(
                    f'[teleop] v={v_lineal:.3f} m/s, w={v_giro:.3f} rad/s'
                )

    # ------------------------------------------------------------------
    # Comunicación UDP
    # ------------------------------------------------------------------
    def _enviar_comando_str(self, comando_str: str):
        with self.carro_ip_lock:
            ip = self.carro_ip
        if ip is not None:
            try:
                self.sock.sendto(comando_str.encode(), (ip, self.carro_port))
            except Exception as e:
                self.get_logger().warn(f'Error enviando comando a {ip}: {e}')

    def recibir_telemetria(self):
        import rclpy
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                msg = data.decode(errors='ignore').strip()
                ip_remota = addr[0]

                # Ignorar paquetes de nuestra propia IP
                if ip_remota == self.local_ip:
                    continue

                # Descubrir IP del carro automáticamente si no está fijada
                with self.carro_ip_lock:
                    if not self.carro_ip or self.carro_ip != ip_remota:
                        if not self.get_parameter('carro_ip_fija').value:
                            self.carro_ip = ip_remota
                            self.carro_descubierto = True
                            self.get_logger().info(
                                f'IP del carro detectada automáticamente: {self.carro_ip}'
                            )
                            ip_msg = String()
                            ip_msg.data = self.carro_ip
                            self.pub_carro_ip.publish(ip_msg)

                # Procesar telemetría del carro
                if msg.startswith('ROBOT|'):
                    self.publicar_telemetria(msg)
                    self.get_logger().debug(f'Telemetría recibida de {ip_remota}: {msg[:80]}...')
                else:
                    self.get_logger().debug(f'Paquete desconocido de {ip_remota}: {msg[:60]}')

                # Publicar IP detectada periódicamente
                with self.carro_ip_lock:
                    ip = self.carro_ip
                if ip is not None:
                    ip_msg = String()
                    ip_msg.data = ip
                    self.pub_carro_ip.publish(ip_msg)

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().warn(f'Error recibiendo telemetría: {e}')

    def descubrimiento_automatico(self):
        local_ip = get_local_ip()
        prefix = get_network_prefix(local_ip)

        while rclpy.ok():
            with self.carro_ip_lock:
                ya_descubierto = self.carro_descubierto

            if ya_descubierto:
                time.sleep(2.0)
                continue

            try:
                self.sock.sendto(b"0,0\n", ("255.255.255.255", self.carro_port))
            except Exception:
                pass

            if prefix:
                for i in range(1, 101):
                    ip = f"{prefix}{i}"
                    if ip == local_ip:
                        continue
                    try:
                        self.sock.sendto(b"0,0\n", (ip, self.carro_port))
                    except Exception:
                        pass

            time.sleep(2.0)

    def publicar_telemetria(self, msg):
        try:
            parts = msg.split('|')
            datos = {}
            for p in parts[1:]:
                if '=' in p:
                    k, v = p.split('=', 1)
                    datos[k] = float(v)

            tel = Float32MultiArray()
            tel.data = [
                float(datos.get('enc_l', 0.0)),
                float(datos.get('enc_r', 0.0)),
                float(datos.get('enc_l_filt', 0.0)),
                float(datos.get('enc_r_filt', 0.0)),
                float(datos.get('ax', 0.0)),
                float(datos.get('ay', 0.0)),
                float(datos.get('az', 0.0)),
                float(datos.get('gx', 0.0)),
                float(datos.get('gy', 0.0)),
                float(datos.get('gz', 0.0)),
            ]
            self.pub_telemetria.publish(tel)

            # Publicar IP detectada
            with self.carro_ip_lock:
                ip = self.carro_ip
            if ip is not None:
                ip_msg = String()
                ip_msg.data = ip
                self.pub_carro_ip.publish(ip_msg)
        except Exception as e:
            self.get_logger().warn(f'Error parseando telemetría: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CarroCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
