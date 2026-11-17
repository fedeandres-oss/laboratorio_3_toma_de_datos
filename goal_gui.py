#!/usr/bin/env python3
"""
goal_gui.py - Interfaz de control y visualización (PC - Alto nivel)
====================================================================
Ventana Tkinter para:
  - Enviar objetivos de posición (X, Y, Theta) al carro
  - Cambiar modo teleop / goal
  - Recibir telemetría del MPU del carro (sin filtros)
  - Calcular posición X, Y desde MPU (doble integración de aceleración)
  - Mostrar trayectoria en tiempo real

Temas ROS:
  Publica:
    - /carro/set_goal (Pose2D)      -> objetivo de posición
    - /carro/set_control_mode (String) -> 'teleop' o 'goal'
  Suscribe:
    - /carro/telemetria (Float32MultiArray) -> datos crudos del MPU del carro
      Índices: [enc_l, enc_r, enc_l_filt, enc_r_filt, ax, ay, az, gx, gy, gz]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String, Float32MultiArray
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


class FIRFilter:
    """FIR pasa-bajos simple con ventana Hamming para filtrado de sensores en Python."""
    def __init__(self, ntaps: int, fc_hz: float, fs_hz: float):
        self.ntaps = ntaps
        self.fc_hz = fc_hz
        self.fs_hz = fs_hz
        self.buffer = [0.0] * ntaps
        self.idx = 0
        self._coeffs = self._design_coeffs()

    def _design_coeffs(self):
        import math
        M = (self.ntaps - 1) / 2.0
        wc = 2.0 * math.pi * self.fc_hz / self.fs_hz
        h = []
        total = 0.0
        for n in range(self.ntaps):
            m = n - M
            if m == 0:
                ideal = wc / math.pi
            else:
                ideal = math.sin(wc * m) / (math.pi * m)
            # Ventana de Hamming
            w = 0.54 - 0.46 * math.cos(2.0 * math.pi * n / (self.ntaps - 1))
            h.append(ideal * w)
            total += h[-1]
        # Normalizar ganancia DC = 1
        h = [v / total for v in h]
        return h

    def update(self, x: float) -> float:
        self.buffer[self.idx] = x
        self.idx = (self.idx + 1) % self.ntaps
        y = 0.0
        for k in range(self.ntaps):
            i = (self.idx - 1 - k) % self.ntaps
            y += self._coeffs[k] * self.buffer[i]
        return y

    def reset(self, x: float = 0.0):
        self.buffer = [x] * self.ntaps
        self.idx = 0


class GoalGui(Node):
    def __init__(self):
        super().__init__('goal_gui')

        # Parámetros físicos
        self.declare_parameter('gravity', 9.81)   # m/s^2
        self.gravity = self.get_parameter('gravity').value

        self.declare_parameter('wheel_radius', 0.0325)   # m
        self.declare_parameter('track_width', 0.135)     # m
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value

        # Parámetros de filtrado y deadzone para evitar drift de doble integración
        self.declare_parameter('deadzone_accel_g', 0.06)   # g: por debajo se trata como 0
        self.declare_parameter('deadzone_gz_dps', 1.5)     # dps: umbral de giro para considerar quieto
        self.declare_parameter('alpha_filtro', 0.4)        # IIR 1er orden para ax/ay (0.1-0.5)
        self.declare_parameter('muestras_quieto', 5)       # muestras consecutivas para considerar quieto
        self.declare_parameter('compensar_gravedad', True) # restar componente vertical de gravedad
        self.declare_parameter('filtrar_imu', False)        # True: aplicar FIR; False: datos crudos (sin filtro)

        self.deadzone_accel_g = self.get_parameter('deadzone_accel_g').value
        self.deadzone_gz_dps  = self.get_parameter('deadzone_gz_dps').value
        self.alpha_filtro     = self.get_parameter('alpha_filtro').value
        self.muestras_quieto  = self.get_parameter('muestras_quieto').value
        self.compensar_gravedad = self.get_parameter('compensar_gravedad').value
        self.filtrar_imu      = self.get_parameter('filtrar_imu').value

        # Estados de filtro IIR para aceleración
        self.ax_filt = 0.0
        self.ay_filt = 0.0
        self.az_filt = 0.0

        # Contador de muestras consecutivas quietas
        self.quieto_cont = 0

        # Estimación del vector de gravedad en marco mundial (se inicializa con az)
        self.gravity_world_z = -1.0  # se actualiza con los primeros datos

        # --- Bias / offset de calibración en reposo (se resta antes de filtrar) ---
        # Valores fijos calculados a partir de tus datos del carro quieto
        self.bias_ax = 0.045
        self.bias_ay = 0.045
        self.bias_az = 1.070
        self.bias_gx = -1.30
        self.bias_gy = 9.00
        self.bias_gz = -1.50
        self.calibrando = False
        # Auto-ajuste continuo del bias de gz cuando el carro está quieto
        self.gz_bias_alpha = 0.01   # cuánto se mueve el bias por muestra quieta (0.01-0.1)
        self.calib_muestras = 0
        self.calib_total = 100
        self.calib_suma = [0.0] * 6

        # Etiqueta de estado de calibración (se crea en _construir_interfaz)
        self.lbl_calib = None

        # --- FILTRO FIR pasa-bajos para ax/ay del MPU (solo si filtrar_imu=True) ---
        self.declare_parameter('fir_ntaps', 15)          # número de coeficientes (impar)
        self.declare_parameter('fir_fc_hz', 5.0)         # frecuencia de corte en Hz
        self.declare_parameter('fir_fs_hz', 100.0)       # frecuencia de muestreo del carro (Hz)
        fir_ntaps = self.get_parameter('fir_ntaps').value
        fir_fc_hz = self.get_parameter('fir_fc_hz').value
        fir_fs_hz = self.get_parameter('fir_fs_hz').value
        if self.filtrar_imu:
            self.fir_ax = FIRFilter(fir_ntaps, fir_fc_hz, fir_fs_hz)
            self.fir_ay = FIRFilter(fir_ntaps, fir_fc_hz, fir_fs_hz)
            self.fir_az = FIRFilter(fir_ntaps, fir_fc_hz, fir_fs_hz)
            self.fir_gz = FIRFilter(fir_ntaps, fir_fc_hz, fir_fs_hz)
        else:
            self.fir_ax = self.fir_ay = self.fir_az = self.fir_gz = None

        # Variable para almacenar valores filtrados o crudos según el parámetro
        self.ax_filtrado = 0.0
        self.ay_filtrado = 0.0
        self.az_filtrado = 0.0
        self.gz_filtrado = 0.0

        # Publicadores
        self.pub_set_goal = self.create_publisher(Pose2D, '/carro/set_goal', 10)
        self.pub_control_mode = self.create_publisher(String, '/carro/set_control_mode', 10)

        # Suscriptor telemetría del carro (MPU crudo)
        self.sub_tele = self.create_subscription(
            Float32MultiArray, '/carro/telemetria', self.callback_telemetry, 10)

        # Suscriptor estado del objetivo
        self.sub_status = self.create_subscription(
            String, '/carro/goal_status', self.callback_status, 10)

        # Suscriptor posición estimada por Kalman (fusión MPU + encoders)
        self.sub_kalman_pos = self.create_subscription(
            Pose2D, '/carro/posicion_kalman', self.callback_kalman_pos, 10)
        # Suscriptor trayectoria completa del Kalman
        self.sub_kalman_traj = self.create_subscription(
            Float32MultiArray, '/carro/trayectoria_kalman', self.callback_kalman_traj, 10)

        # Estado de la estimación Kalman
        self.pos_kal_x = 0.0
        self.pos_kal_y = 0.0
        self.pos_kal_theta = 0.0
        self.kal_v = 0.0
        self.kal_w = 0.0
        self.kal_quieto = False
        self.trayectoria_kal_x = [0.0]
        self.trayectoria_kal_y = [0.0]

        # Estado de posición calculada desde MPU (en la GUI/PC)
        self.lock = threading.Lock()
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_theta = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0

        # Estado de odometría por encoders (ruedas)
        self.pos_enc_x = 0.0
        self.pos_enc_y = 0.0
        self.pos_enc_theta = 0.0
        self.last_time_enc = None
        self.trayectoria_enc_x = [0.0]
        self.trayectoria_enc_y = [0.0]

        # Última lectura de IMU para integración
        self.last_ax = 0.0
        self.last_ay = 0.0
        self.last_gz = 0.0
        self.last_time = None

        # Historial para trayectoria
        self.trayectoria_x = [0.0]
        self.trayectoria_y = [0.0]

        # Estado del objetivo
        self.goal_status = 'IDLE'

        # --- Interfaz Tkinter ---
        self.root = tk.Tk()
        self.root.title('Teleoperación - Control y Trayectoria (MPU)')
        self.root.geometry('950x750')
        self.root.resizable(True, True)

        self._construir_interfaz()

        # Ventana aparte: trayectoria por encoders
        self._construir_interfaz_encoders()

        # Ventana principal: trayectoria estimada por Kalman (fusión MPU + encoders)
        self._construir_interfaz_kalman()

        # Timer para spin de ROS (50 Hz)
        self.timer = self.root.after(20, self.spin_once)

        self.get_logger().info('Interfaz de control MPU iniciada.')

    # ------------------------------------------------------------------
    # Construcción de la interfaz
    # ------------------------------------------------------------------
    def _construir_interfaz(self):
        # Título
        title = tk.Label(self.root, text='Teleoperación Minisumo - Control y Trayectoria (MPU)',
                         font=('Arial', 14, 'bold'))
        title.pack(pady=8)

        # Frame superior: modo y entrada de objetivo
        frame_top = ttk.Frame(self.root)
        frame_top.pack(padx=15, pady=5, fill='x')

        # Modo de control
        frame_mode = ttk.LabelFrame(frame_top, text='Modo de control')
        frame_mode.pack(side='left', padx=10, pady=5, fill='y')

        self.mode_var = tk.StringVar(value='teleop')
        rb_teleop = tk.Radiobutton(frame_mode, text='Teleoperación (Guante)',
                                   variable=self.mode_var, value='teleop',
                                   command=self.change_mode, font=('Arial', 10))
        rb_teleop.pack(anchor='w', padx=8, pady=2)
        rb_goal = tk.Radiobutton(frame_mode, text='Control por Objetivo (GUI)',
                                 variable=self.mode_var, value='goal',
                                 command=self.change_mode, font=('Arial', 10))
        rb_goal.pack(anchor='w', padx=8, pady=2)

        # Entrada de coordenadas
        frame_coords = ttk.LabelFrame(frame_top, text='Posición objetivo (metros)')
        frame_coords.pack(side='left', padx=10, pady=5, fill='y')

        tk.Label(frame_coords, text='X (m):').grid(row=0, column=0, padx=5, pady=4, sticky='e')
        self.entry_x = tk.Entry(frame_coords, width=10)
        self.entry_x.insert(0, '0.5')
        self.entry_x.grid(row=0, column=1, padx=5, pady=4)

        tk.Label(frame_coords, text='Y (m):').grid(row=1, column=0, padx=5, pady=4, sticky='e')
        self.entry_y = tk.Entry(frame_coords, width=10)
        self.entry_y.insert(0, '0.0')
        self.entry_y.grid(row=1, column=1, padx=5, pady=4)

        tk.Label(frame_coords, text='Theta (rad):').grid(row=2, column=0, padx=5, pady=4, sticky='e')
        self.entry_theta = tk.Entry(frame_coords, width=10)
        self.entry_theta.insert(0, '0.0')
        self.entry_theta.grid(row=2, column=1, padx=5, pady=4)

        btn_send = tk.Button(frame_coords, text='ENVIAR OBJETIVO', bg='#4CAF50',
                             fg='white', font=('Arial', 10, 'bold'),
                             command=self.send_goal, width=16)
        btn_send.grid(row=3, column=0, columnspan=2, padx=5, pady=8)

        # Estado actual
        frame_status = ttk.LabelFrame(frame_top, text='Estado')
        frame_status.pack(side='left', padx=10, pady=5, fill='y')

        self.lbl_mode = tk.Label(frame_status, text='Modo: teleop', font=('Arial', 10, 'bold'))
        self.lbl_mode.pack(anchor='w', padx=8, pady=2)

        self.lbl_goal_status = tk.Label(frame_status, text='Estado objetivo: IDLE',
                                        font=('Arial', 10), fg='blue')
        self.lbl_goal_status.pack(anchor='w', padx=8, pady=2)

        # Frame medio: posición actual (desde MPU)
        frame_pos = ttk.LabelFrame(self.root, text='Posición actual del carro (calculada desde MPU del carro)')
        frame_pos.pack(padx=15, pady=5, fill='x')

        self.lbl_x = tk.Label(frame_pos, text='X: 0.000 m', font=('Arial', 12, 'bold'), fg='green')
        self.lbl_x.pack(side='left', padx=20, pady=8)

        self.lbl_y = tk.Label(frame_pos, text='Y: 0.000 m', font=('Arial', 12, 'bold'), fg='green')
        self.lbl_y.pack(side='left', padx=20, pady=8)

        self.lbl_theta = tk.Label(frame_pos, text='Theta: 0.000 rad', font=('Arial', 12, 'bold'), fg='green')
        self.lbl_theta.pack(side='left', padx=20, pady=8)

        self.lbl_ax = tk.Label(frame_pos, text='ax: 0.000 g', font=('Arial', 10), fg='gray')
        self.lbl_ax.pack(side='left', padx=15, pady=8)

        self.lbl_ay = tk.Label(frame_pos, text='ay: 0.000 g', font=('Arial', 10), fg='gray')
        self.lbl_ay.pack(side='left', padx=15, pady=8)

        self.lbl_gz = tk.Label(frame_pos, text='gz: 0.000 dps', font=('Arial', 10), fg='gray')
        self.lbl_gz.pack(side='left', padx=15, pady=8)

        # Botón reset posición
        btn_reset = tk.Button(frame_pos, text='Reset posición', bg='#f44336',
                              fg='white', command=self.reset_posicion, width=14)
        btn_reset.pack(side='right', padx=5, pady=8)

        # Botón calibrar cero en reposo
        btn_calibrar = tk.Button(frame_pos, text='Calibrar cero (reposo)', bg='#ff9800',
                                 fg='white', command=self.iniciar_calibracion, width=18)
        btn_calibrar.pack(side='right', padx=5, pady=8)

        # Botón limpiar calibración
        btn_limpiar = tk.Button(frame_pos, text='Limpiar calibración', bg='#757575',
                                fg='white', command=self.limpiar_calibracion, width=16)
        btn_limpiar.pack(side='right', padx=5, pady=8)

        # Etiqueta de estado de calibración
        self.lbl_calib = tk.Label(frame_pos, text='', font=('Arial', 9), fg='#ff9800')
        self.lbl_calib.pack(side='right', padx=10, pady=8)

        # Frame inferior: gráfica de trayectoria
        frame_plot = ttk.LabelFrame(self.root, text='Trayectoria en tiempo real (desde MPU)')
        frame_plot.pack(padx=15, pady=10, fill='both', expand=True)

        self.fig = Figure(figsize=(7, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title('Trayectoria del carro (Y-X) - Desde MPU del carro')
        self.ax.set_xlabel('Y (m)')
        self.ax.set_ylabel('X (m)')
        self.ax.grid(True, alpha=0.3)
        self.ax.axis('equal')

        self.line_trayectoria, = self.ax.plot([], [], 'b-', linewidth=2, label='Trayectoria')
        self.point_actual, = self.ax.plot([], [], 'ro', markersize=8, label='Posición actual')
        self.point_inicio, = self.ax.plot([0], [0], 'go', markersize=8, label='Inicio')
        self.ax.legend(loc='upper right', fontsize=8)

        self.canvas = FigureCanvasTkAgg(self.fig, master=frame_plot)
        self.canvas.get_tk_widget().pack(fill='both', expand=True, padx=5, pady=5)

        # Instrucciones
        lbl_inst = tk.Label(self.root,
                            text='X,Y en metros. Ej: X=0.5, Y=0.0 mueve 50 cm adelante.\n'
                                 'Theta en radianes. 0 = frente, 1.57 ≈ 90° izquierda.\n'
                                 'Con filtro IIR + deadzone: si el carro está quieto, la posición se congela cerca de cero.',
                            font=('Arial', 9), fg='gray', justify='center')
        lbl_inst.pack(pady=5)

    # ------------------------------------------------------------------
    # Callbacks ROS
    # ------------------------------------------------------------------
    def callback_telemetry(self, msg: Float32MultiArray):
        """Recibe telemetría del carro y calcula posición desde MPU."""
        data = msg.data
        if len(data) < 10:
            return

        # Índices de la telemetría:
        # [enc_l, enc_r, enc_l_filt, enc_r_filt, ax, ay, az, gx, gy, gz]
        ax = data[4]   # g
        ay = data[5]   # g
        az = data[6]   # g
        gx = data[7]   # dps
        gy = data[8]   # dps
        gz = data[9]   # dps

        # --- Restar bias de calibración (reposo) antes de filtrar ---
        ax_c = ax - self.bias_ax
        ay_c = ay - self.bias_ay
        az_c = az - self.bias_az
        gx_c = gx - self.bias_gx
        gy_c = gy - self.bias_gy
        gz_c = gz - self.bias_gz

        # Si estamos calibrando, acumular muestras y no integrar
        if self.calibrando:
            self.calib_suma[0] += ax_c
            self.calib_suma[1] += ay_c
            self.calib_suma[2] += az_c
            self.calib_suma[3] += gx_c
            self.calib_suma[4] += gy_c
            self.calib_suma[5] += gz_c
            self.calib_muestras += 1
            progreso = f'Calibrando... {self.calib_muestras}/{self.calib_total}'
            self.lbl_calib.config(text=progreso)
            if self.calib_muestras >= self.calib_total:
                self.finalizar_calibracion()
            return

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            self.last_ax = ax_c
            self.last_ay = ay_c
            self.last_gz = gz_c
            if self.filtrar_imu:
                self.fir_ax.reset(ax_c)
                self.fir_ay.reset(ay_c)
                self.fir_az.reset(az_c)
                self.fir_gz.reset(gz_c)
            self.ax_filt = ax_c
            self.ay_filt = ay_c
            self.az_filt = az_c
            # Inicializar estimación de gravedad con la primera lectura de az
            self.gravity_world_z = az_c if abs(az_c) > 0.5 else -1.0
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0 or dt > 0.2:
            self.last_time = now
            self.last_ax = ax_c
            self.last_ay = ay_c
            self.last_gz = gz_c
            return

        # --- 1. Filtrar aceleración (FIR pasa-bajos) o usar crudos ---
        if self.filtrar_imu:
            ax_f = self.fir_ax.update(ax_c)
            ay_f = self.fir_ay.update(ay_c)
            az_f = self.fir_az.update(az_c)
            gz_f = self.fir_gz.update(gz_c)
        else:
            ax_f = ax_c
            ay_f = ay_c
            az_f = az_c
            gz_f = gz_c

        # --- 2. Estimar y compensar gravedad en marco mundial ---
        # Cuando el carro está quieto sobre superficie plana, az ≈ ±1 g (gravedad).
        # Ese componente vertical se rota al marco mundial y se "filtra" en X/Y
        # si theta tiene drift. Lo estimamos y lo restamos antes de integrar.
        g_mundo_z = self.gravity_world_z
        if self.compensar_gravedad and self.pos_theta is not None:
            # Vector gravedad en marco del sensor (aprox. [0, 0, g_mundo_z])
            # Rotarlo al marco mundial y restar su componente horizontal
            gx_world = g_mundo_z * np.sin(self.pos_theta)   # ≈ g * sin(theta)
            gy_world = -g_mundo_z * np.cos(self.pos_theta)  # ≈ -g * cos(theta)
        else:
            gx_world = 0.0
            gy_world = 0.0

        # --- 3. Deadzone: aceleraciones pequeñas se tratan como cero ---
        ax_used = ax_f - gx_world / self.gravity
        ay_used = ay_f - gy_world / self.gravity
        ax_used = ax_used if abs(ax_used) > self.deadzone_accel_g else 0.0
        ay_used = ay_used if abs(ay_used) > self.deadzone_accel_g else 0.0

        # --- 4. Detectar si el carro está quieto (varias muestras consecutivas) ---
        giro_poco = abs(gz_f) < self.deadzone_gz_dps
        accel_poco = (ax_used == 0.0 and ay_used == 0.0)

        if accel_poco and giro_poco:
            self.quieto_cont += 1
        else:
            self.quieto_cont = 0

        quieto = self.quieto_cont >= self.muestras_quieto

        # --- 5. Integrar giroscopio para obtener ángulo theta ---
        gz_rad_s = gz_f * np.pi / 180.0
        self.pos_theta += gz_rad_s * dt
        self.pos_theta = np.arctan2(np.sin(self.pos_theta), np.cos(self.pos_theta))

        # --- 6. Convertir aceleración (sin gravedad) de g a m/s^2 ---
        ax_m_s2 = ax_used * self.gravity
        ay_m_s2 = ay_used * self.gravity

        # --- 7. Rotar aceleración al marco mundial usando theta ---
        ax_world = ax_m_s2 * np.cos(self.pos_theta) - ay_m_s2 * np.sin(self.pos_theta)
        ay_world = ax_m_s2 * np.sin(self.pos_theta) + ay_m_s2 * np.cos(self.pos_theta)

        # --- 8. Integración de velocidad y posición ---
        if quieto:
            # Carro quieto: resetear velocidad y NO integrar posición
            # (mantener pos_x, pos_y congelados)
            self.vel_x = 0.0
            self.vel_y = 0.0
            # Resetear posición MPU a la del Kalman para que coincidan
            self.pos_x = self.pos_kal_x
            self.pos_y = self.pos_kal_y
            # Auto-ajuste continuo del bias de gz para eliminar deriva de theta
            self.bias_gz += self.gz_bias_alpha * (gz_f - self.bias_gz)
        else:
            self.vel_x += ax_world * dt
            self.vel_y += ay_world * dt
            self.pos_x += self.vel_x * dt
            self.pos_y += self.vel_y * dt

        # Guardar para siguiente ciclo
        self.last_ax = ax_f
        self.last_ay = ay_f
        self.last_gz = gz_f
        self.last_time = now

        # Guardar trayectoria
        with self.lock:
            self.trayectoria_x.append(self.pos_x)
            self.trayectoria_y.append(self.pos_y)

        # Guardar valores para mostrar
        with self.lock:
            self._last_ax_g = ax
            self._last_ay_g = ay
            self._last_gz_dps = gz

        # --- Odometría por encoders (velocidades de rueda en rad/s) ---
        wl = data[2]  # enc_l_filt
        wr = data[3]  # enc_r_filt

        now_enc = self.get_clock().now()
        if self.last_time_enc is None:
            self.last_time_enc = now_enc
        else:
            dt_enc = (now_enc - self.last_time_enc).nanoseconds / 1e9
            if 0 < dt_enc <= 0.2:
                v_enc = self.wheel_radius * (wl + wr) / 2.0
                w_enc = self.wheel_radius * (wr - wl) / self.track_width
                self.pos_enc_theta += w_enc * dt_enc
                self.pos_enc_theta = np.arctan2(np.sin(self.pos_enc_theta), np.cos(self.pos_enc_theta))
                dx = v_enc * np.cos(self.pos_enc_theta) * dt_enc
                dy = v_enc * np.sin(self.pos_enc_theta) * dt_enc
                self.pos_enc_x += dx
                self.pos_enc_y += dy
                with self.lock:
                    self.trayectoria_enc_x.append(self.pos_enc_x)
                    self.trayectoria_enc_y.append(self.pos_enc_y)
            self.last_time_enc = now_enc

    def callback_status(self, msg: String):
        self.goal_status = msg.data
        self.root.after(0, self._actualizar_estado_objetivo)

    def _actualizar_estado_objetivo(self):
        color = 'green' if self.goal_status == 'REACHED' else ('orange' if self.goal_status == 'MOVING' else 'blue')
        self.lbl_goal_status.config(text=f'Estado objetivo: {self.goal_status}', fg=color)

    # ------------------------------------------------------------------
    # Callbacks del filtro de Kalman (trayectoria estimada principal)
    # ------------------------------------------------------------------
    def callback_kalman_pos(self, msg: Pose2D):
        with self.lock:
            self.pos_kal_x = msg.x
            self.pos_kal_y = msg.y
            self.pos_kal_theta = msg.theta

    def callback_kalman_traj(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) < 6:
            return
        with self.lock:
            self.kal_v = data[3]
            self.kal_w = data[4]
            self.kal_quieto = data[5] > 0.5
            self.trayectoria_kal_x.append(data[0])
            self.trayectoria_kal_y.append(data[1])

    # ------------------------------------------------------------------
    # Acciones de la interfaz
    # ------------------------------------------------------------------
    def send_goal(self):
        try:
            x = float(self.entry_x.get())
            y = float(self.entry_y.get())
            theta = float(self.entry_theta.get())
        except ValueError:
            messagebox.showerror('Error', 'Ingresa valores numéricos válidos')
            return

        # Cambiar a modo goal automáticamente
        self.mode_var.set('goal')
        self.change_mode()

        # Publicar objetivo
        goal = Pose2D()
        goal.x = x
        goal.y = y
        goal.theta = theta
        self.pub_set_goal.publish(goal)

        self.get_logger().info(f'Objetivo enviado: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')

    def change_mode(self):
        mode = self.mode_var.get()
        msg = String()
        msg.data = mode
        self.pub_control_mode.publish(msg)
        self.lbl_mode.config(text=f'Modo: {mode}')
        self.get_logger().info(f'Modo cambiado a: {mode}')

    def reset_posicion(self):
        with self.lock:
            self.pos_x = 0.0
            self.pos_y = 0.0
            self.pos_theta = 0.0
            self.vel_x = 0.0
            self.vel_y = 0.0
            self.trayectoria_x = [0.0]
            self.trayectoria_y = [0.0]
            # Reset odometría encoders
            self.pos_enc_x = 0.0
            self.pos_enc_y = 0.0
            self.pos_enc_theta = 0.0
            self.last_time_enc = None
            self.trayectoria_enc_x = [0.0]
            self.trayectoria_enc_y = [0.0]
        self.last_time = None
        self.quieto_cont = 0
        self.ax_filt = 0.0
        self.ay_filt = 0.0
        self.az_filt = 0.0
        self.get_logger().info('Posición reiniciada (MPU + encoders).')

    # ------------------------------------------------------------------
    # Calibración de bias en reposo (zeroing)
    # ------------------------------------------------------------------
    def iniciar_calibracion(self):
        """Inicia la recolección de muestras para calcular el bias del MPU."""
        self.calibrando = True
        self.calib_muestras = 0
        self.calib_suma = [0.0] * 6
        self.lbl_calib.config(text='Calibrando... 0/100  (mantén el carro quieto)')
        self.get_logger().info('Iniciando calibración de bias en reposo...')

    def finalizar_calibracion(self):
        """Calcula el bias promedio y lo guarda para restar en lecturas futuras."""
        if self.calib_total <= 0:
            return
        self.bias_ax = self.calib_suma[0] / self.calib_total
        self.bias_ay = self.calib_suma[1] / self.calib_total
        self.bias_az = self.calib_suma[2] / self.calib_total
        self.bias_gx = self.calib_suma[3] / self.calib_total
        self.bias_gy = self.calib_suma[4] / self.calib_total
        self.bias_gz = self.calib_suma[5] / self.calib_total
        self.calibrando = False
        self.lbl_calib.config(
            text=f'Cero calibrado: ax={self.bias_ax:+.3f} ay={self.bias_ay:+.3f} '
                 f'az={self.bias_az:+.3f} gz={self.bias_gz:+.2f} dps'
        )
        self.get_logger().info(
            f'Bias calibrado: ax={self.bias_ax:.4f} ay={self.bias_ay:.4f} '
            f'az={self.bias_az:.4f} gx={self.bias_gx:.4f} gy={self.bias_gy:.4f} '
            f'gz={self.bias_gz:.4f}'
        )

    def limpiar_calibracion(self):
        """Elimina el bias de calibración (vuelve a valores crudos)."""
        self.bias_ax = 0.0
        self.bias_ay = 0.0
        self.bias_az = 0.0
        self.bias_gx = 0.0
        self.bias_gy = 0.0
        self.bias_gz = 0.0
        self.calibrando = False
        self.calib_muestras = 0
        self.lbl_calib.config(text='Calibración limpiada')
        self.get_logger().info('Calibración de bias limpiada.')

    def _actualizar_interfaz(self):
        """Actualiza etiquetas y gráfica (llamado desde el hilo principal de Tkinter)."""
        with self.lock:
            x = self.pos_x
            y = self.pos_y
            theta = self.pos_theta
            traj_x = list(self.trayectoria_x)
            traj_y = list(self.trayectoria_y)

        # Intercambio X/Y para visualizar mejor (lbl_x muestra Y, lbl_y muestra X)
        self.lbl_x.config(text=f'X: {y:+.3f} m')
        self.lbl_y.config(text=f'Y: {x:+.3f} m')
        self.lbl_theta.config(text=f'Theta: {theta:+.3f} rad')

        # Mostrar valores crudos del MPU
        if hasattr(self, '_last_ax_g'):
            self.lbl_ax.config(text=f'ax: {self._last_ax_g:+.3f} g')
            self.lbl_ay.config(text=f'ay: {self._last_ay_g:+.3f} g')
            self.lbl_gz.config(text=f'gz: {self._last_gz_dps:+.1f} dps')

        # Actualizar gráfica (ejes X/Y intercambiados: eje horizontal = Y, vertical = X)
        self.line_trayectoria.set_data(traj_y, traj_x)
        self.point_actual.set_data([y], [x])

        # Ajustar límites automáticamente (basados en valores intercambiados)
        if len(traj_x) > 1:
            xmin = min(min(traj_y), 0) - 1.0
            xmax = max(max(traj_y), 0) + 1.0
            ymin = min(min(traj_x), 0) - 1.0
            ymax = max(max(traj_x), 0) + 1.0
            self.ax.set_xlim(xmin, xmax)
            self.ax.set_ylim(ymin, ymax)

        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    # Interfaz aparte: trayectoria por encoders
    # ------------------------------------------------------------------
    def _construir_interfaz_encoders(self):
        self.win_enc = tk.Toplevel(self.root)
        self.win_enc.title('Trayectoria por Encoders (Ruedas)')
        self.win_enc.geometry('700x550')

        tk.Label(self.win_enc, text='Odometría diferencial por encoders del carro',
                 font=('Arial', 12, 'bold')).pack(pady=8)

        self.fig_enc = Figure(figsize=(7, 5), dpi=100)
        self.ax_enc = self.fig_enc.add_subplot(111)
        self.ax_enc.set_title('Trayectoria del carro (Y-X) - Desde Encoders')
        self.ax_enc.set_xlabel('Y (m)')
        self.ax_enc.set_ylabel('X (m)')
        self.ax_enc.grid(True, alpha=0.3)
        self.ax_enc.axis('equal')

        self.line_enc, = self.ax_enc.plot([], [], 'm-', linewidth=2, label='Encoders')
        self.point_enc, = self.ax_enc.plot([], [], 'mo', markersize=8, label='Posición actual')
        self.point_enc_inicio, = self.ax_enc.plot([0], [0], 'go', markersize=8, label='Inicio')
        self.ax_enc.legend(loc='upper right', fontsize=8)

        self.canvas_enc = FigureCanvasTkAgg(self.fig_enc, master=self.win_enc)
        self.canvas_enc.get_tk_widget().pack(fill='both', expand=True, padx=5, pady=5)

        self.lbl_enc_pos = tk.Label(self.win_enc, text='X: 0.000 m  Y: 0.000 m  Theta: 0.000 rad',
                                    font=('Arial', 10), fg='gray')
        self.lbl_enc_pos.pack(pady=4)

        btn_reset_enc = tk.Button(self.win_enc, text='Reset encoders', bg='#f44336',
                                  fg='white', command=self.reset_posicion, width=14)
        btn_reset_enc.pack(pady=4)

    def _actualizar_interfaz_encoders(self):
        if not hasattr(self, 'win_enc') or not self.win_enc.winfo_exists():
            return
        with self.lock:
            x = self.pos_enc_x
            y = self.pos_enc_y
            theta = self.pos_enc_theta
            traj_x = list(self.trayectoria_enc_x)
            traj_y = list(self.trayectoria_enc_y)

        # Intercambio X/Y para visualizar mejor
        self.lbl_enc_pos.config(text=f'X: {y:+.3f} m  Y: {x:+.3f} m  Theta: {theta:+.3f} rad')

        self.line_enc.set_data(traj_y, traj_x)
        self.point_enc.set_data([y], [x])

        if len(traj_x) > 1:
            xmin = min(min(traj_y), 0) - 1.0
            xmax = max(max(traj_y), 0) + 1.0
            ymin = min(min(traj_x), 0) - 1.0
            ymax = max(max(traj_x), 0) + 1.0
            self.ax_enc.set_xlim(xmin, xmax)
            self.ax_enc.set_ylim(ymin, ymax)

        self.canvas_enc.draw_idle()

    # ------------------------------------------------------------------
    # Interfaz aparte: trayectoria estimada por Kalman (principal)
    # ------------------------------------------------------------------
    def _construir_interfaz_kalman(self):
        self.win_kal = tk.Toplevel(self.root)
        self.win_kal.title('Trayectoria Estimada - Filtro de Kalman (MPU + Encoders)')
        self.win_kal.geometry('800x600')

        tk.Label(self.win_kal, text='Trayectoria del carro estimada por EKF (fusión MPU + encoders)',
                 font=('Arial', 12, 'bold')).pack(pady=8)

        self.fig_kal = Figure(figsize=(8, 5.5), dpi=100)
        self.ax_kal = self.fig_kal.add_subplot(111)
        self.ax_kal.set_title('Trayectoria estimada (Y-X) - Filtro de Kalman')
        self.ax_kal.set_xlabel('Y (m)')
        self.ax_kal.set_ylabel('X (m)')
        self.ax_kal.grid(True, alpha=0.3)
        self.ax_kal.axis('equal')

        self.line_kal, = self.ax_kal.plot([], [], 'b-', linewidth=2.5, label='Kalman (estimada)')
        self.point_kal, = self.ax_kal.plot([], [], 'bo', markersize=8, label='Posición actual')
        self.point_kal_inicio, = self.ax_kal.plot([0], [0], 'go', markersize=8, label='Inicio')
        self.ax_kal.legend(loc='upper right', fontsize=8)

        self.canvas_kal = FigureCanvasTkAgg(self.fig_kal, master=self.win_kal)
        self.canvas_kal.get_tk_widget().pack(fill='both', expand=True, padx=5, pady=5)

        self.lbl_kal_pos = tk.Label(self.win_kal, text='X: 0.000 m  Y: 0.000 m  Theta: 0.000 rad',
                                    font=('Arial', 11, 'bold'), fg='blue')
        self.lbl_kal_pos.pack(pady=4)

        self.lbl_kal_vel = tk.Label(self.win_kal, text='v: 0.000 m/s  w: 0.000 rad/s  Quieto: No',
                                    font=('Arial', 10), fg='gray')
        self.lbl_kal_vel.pack(pady=2)

        btn_reset_kal = tk.Button(self.win_kal, text='Reset Kalman', bg='#f44336',
                                  fg='white', command=self.reset_posicion, width=14)
        btn_reset_kal.pack(pady=4)

    def _actualizar_interfaz_kalman(self):
        if not hasattr(self, 'win_kal') or not self.win_kal.winfo_exists():
            return
        with self.lock:
            x = self.pos_kal_x
            y = self.pos_kal_y
            theta = self.pos_kal_theta
            v = self.kal_v
            w = self.kal_w
            quieto = self.kal_quieto
            traj_x = list(self.trayectoria_kal_x)
            traj_y = list(self.trayectoria_kal_y)

        self.lbl_kal_pos.config(text=f'X: {x:+.3f} m  Y: {y:+.3f} m  Theta: {theta:+.3f} rad')
        estado_q = 'Sí' if quieto else 'No'
        self.lbl_kal_vel.config(text=f'v: {v:+.3f} m/s  w: {w:+.3f} rad/s  Quieto: {estado_q}')

        self.line_kal.set_data(traj_y, traj_x)
        self.point_kal.set_data([y], [x])

        if len(traj_x) > 1:
            xmin = min(min(traj_y), 0) - 1.0
            xmax = max(max(traj_y), 0) + 1.0
            ymin = min(min(traj_x), 0) - 1.0
            ymax = max(max(traj_x), 0) + 1.0
            self.ax_kal.set_xlim(xmin, xmax)
            self.ax_kal.set_ylim(ymin, ymax)

        self.canvas_kal.draw_idle()

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self._actualizar_interfaz()
        self._actualizar_interfaz_encoders()
        self._actualizar_interfaz_kalman()
        self.timer = self.root.after(20, self.spin_once)

    def run(self):
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GoalGui()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
