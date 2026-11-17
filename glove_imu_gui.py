#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
glove_imu_gui.py - GUI separada para la inclinación del MPU del guante
=====================================================================
Se suscribe a /guante/imu (sensor_msgs/Imu) y muestra:

  - Un cubo 3D que se inclina con roll/pitch.
  - Un nivel de burbuja (ax, ay) para ver hacia dónde se inclina.
  - Valores numéricos de aceleración (g), giroscopio (deg/s),
    roll, pitch e inclinación total.
  - Botón "Poner cero" para tomar la posición actual como neutra.

El mensaje del guante NO trae cuaternión de orientación: la inclinación
se calcula desde el acelerómetro. El eje Z del guante se detecta
automáticamente (puede estar montado con Z hacia arriba o hacia abajo).

Uso:
    source /opt/ros/jazzy/setup.bash
    python3 glove_imu_gui.py
"""

import math
import time
import tkinter as tk

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

G = 9.80665


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class GloveImuGui(Node):
    def __init__(self):
        super().__init__('glove_imu_gui')

        # --- Suscripción al MPU del guante ---
        self.sub = self.create_subscription(Imu, '/guante/imu', self.cb_imu, 10)

        # --- Últimos valores ---
        self.ax = self.ay = self.az = 0.0       # en g
        self.gx = self.gy = self.gz = 0.0       # en deg/s
        self.last_rx_wall = 0.0
        self.msg_count = 0
        self.rate_hz = 0.0
        self._rate_t0 = time.time()
        self._rate_n0 = 0

        # --- Cero / orientación neutra ---
        self.zero_done = False
        self.ax0 = self.ay0 = self.az0 = 0.0
        self.roll0 = 0.0
        self.pitch0 = 0.0
        self.z_sign = 1.0

        # --- Ventana ---
        self.root = tk.Tk()
        self.root.title('Inclinación MPU del guante')
        self.root.geometry('1020x700')
        self.root.minsize(860, 620)
        self.root.configure(bg='#0f1720')
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        self._closed = False
        self._timer = None
        self._last_drawn = -1

        self._build_ui()
        self._timer = self.root.after(20, self._spin)

    # ------------------------------------------------------------------
    # Interfaz
    # ------------------------------------------------------------------
    def _build_ui(self):
        bg = '#0f1720'
        fg = '#d8e6ef'
        muted = '#7fa6ba'
        panel = '#0b1118'

        title = tk.Label(
            self.root,
            text='Inclinación del MPU del guante',
            font=('Arial', 16, 'bold'),
            bg=bg, fg=fg,
        )
        title.pack(pady=(12, 2))

        self.lbl_status = tk.Label(
            self.root,
            text='Esperando datos de /guante/imu...',
            font=('Arial', 10),
            bg=bg, fg=muted,
        )
        self.lbl_status.pack(pady=(0, 8))

        body = tk.Frame(self.root, bg=bg)
        body.pack(fill='both', expand=True, padx=14, pady=4)

        # ---------- Panel izquierdo: cubo 3D ----------
        left = tk.Frame(body, bg=bg)
        left.pack(side='left', fill='both', expand=True, padx=(0, 10))

        tk.Label(left, text='Orientación (roll / pitch)',
                 font=('Arial', 11, 'bold'), bg=bg, fg=muted).pack(anchor='w', padx=4)

        self.cube = tk.Canvas(left, width=470, height=470,
                              bg=panel, highlightthickness=0)
        self.cube.pack(fill='both', expand=True, padx=4, pady=4)

        tk.Label(left,
                 text='X rojo = adelante   |   Y verde = derecha   |   Z azul = arriba',
                 font=('Arial', 9), bg=bg, fg=muted).pack(anchor='w', padx=4, pady=(0, 4))

        # ---------- Panel derecho: burbuja + datos ----------
        right = tk.Frame(body, bg=bg)
        right.pack(side='right', fill='y', padx=(10, 0))

        tk.Label(right, text='Nivel de inclinación (ax / ay)',
                 font=('Arial', 11, 'bold'), bg=bg, fg=muted).pack(anchor='w', padx=4)

        self.bubble = tk.Canvas(right, width=350, height=350,
                                bg=panel, highlightthickness=0)
        self.bubble.pack(padx=4, pady=4)

        self.lbl_values = tk.Label(
            right,
            text='',
            font=('DejaVu Sans Mono', 10),
            bg=bg, fg=fg,
            justify='left',
            anchor='nw',
        )
        self.lbl_values.pack(fill='x', padx=6, pady=(6, 4))

        self.lbl_zero = tk.Label(
            right,
            text='Cero: todavía no fijado',
            font=('Arial', 9),
            bg=bg, fg=muted,
            anchor='w',
        )
        self.lbl_zero.pack(fill='x', padx=6, pady=(0, 4))

        buttons = tk.Frame(right, bg=bg)
        buttons.pack(fill='x', padx=4, pady=(2, 8))

        tk.Button(buttons, text='Poner cero', command=self._zero_button,
                  bg='#2e6f8f', fg='white', activebackground='#3a8ab0',
                  font=('Arial', 10, 'bold'), relief='flat', padx=10, pady=4
                  ).pack(side='left', padx=(0, 6))

        tk.Button(buttons, text='Quitar cero', command=self._reset_view,
                  bg='#243a48', fg='white', activebackground='#2e4f60',
                  font=('Arial', 10), relief='flat', padx=10, pady=4
                  ).pack(side='left')

    # ------------------------------------------------------------------
    # ROS
    # ------------------------------------------------------------------
    def cb_imu(self, msg):
        self.ax = msg.linear_acceleration.x / G
        self.ay = msg.linear_acceleration.y / G
        self.az = msg.linear_acceleration.z / G

        self.gx = math.degrees(msg.angular_velocity.x)
        self.gy = math.degrees(msg.angular_velocity.y)
        self.gz = math.degrees(msg.angular_velocity.z)

        now = time.time()
        self.last_rx_wall = now
        self.msg_count += 1

        if now - self._rate_t0 >= 1.0:
            self.rate_hz = (self.msg_count - self._rate_n0) / (now - self._rate_t0)
            self._rate_t0 = now
            self._rate_n0 = self.msg_count

        # Primer dato: tomarlo como cero automático.
        if not self.zero_done:
            self._set_zero_from_current(auto=True)

    # ------------------------------------------------------------------
    # Cálculos de inclinación
    # ------------------------------------------------------------------
    def _az_up(self):
        return self.az * self.z_sign

    def _roll_raw(self):
        return math.atan2(self.ay, self._az_up())

    def _pitch_raw(self):
        # ax negativo = inclinación hacia adelante (según usuario)
        return math.atan2(-self.ax, self._az_up())

    def _roll_rel(self):
        return wrap_pi(self._roll_raw() - self.roll0)

    def _pitch_rel(self):
        return wrap_pi(self._pitch_raw() - self.pitch0)

    def _tilt_deg(self):
        if not self.zero_done:
            return 0.0
        a = np.array([self.ax, self.ay, self.az], dtype=float)
        n = np.array([self.ax0, self.ay0, self.az0], dtype=float)
        na = np.linalg.norm(a)
        nn = np.linalg.norm(n)
        if na < 1e-6 or nn < 1e-6:
            return 0.0
        c = float(np.dot(a, n) / (na * nn))
        c = max(-1.0, min(1.0, c))
        return math.degrees(math.acos(c))

    def _set_zero_from_current(self, auto=False):
        if self.msg_count == 0 and not auto:
            return
        self.ax0 = self.ax
        self.ay0 = self.ay
        self.az0 = self.az
        self.z_sign = 1.0 if self.az0 >= 0.0 else -1.0
        self.roll0 = self._roll_raw()
        self.pitch0 = self._pitch_raw()
        self.zero_done = True
        self.lbl_zero.config(
            text=(f'Cero {"automático" if auto else "manual"}: '
                  f'ax={self.ax0:+.2f}  ay={self.ay0:+.2f}  az={self.az0:+.2f} g')
        )

    def _zero_button(self):
        self._set_zero_from_current(auto=False)

    def _reset_view(self):
        # Quitar el cero: usar la orientación absoluta del sensor.
        self.ax0 = 0.0
        self.ay0 = 0.0
        self.z_sign = 1.0 if self.az >= 0.0 else -1.0
        self.az0 = self.z_sign
        self.roll0 = 0.0
        self.pitch0 = 0.0
        self.zero_done = True
        self.lbl_zero.config(text='Cero: desactivado (orientación absoluta)')

    # ------------------------------------------------------------------
    # Loop Tkinter + ROS
    # ------------------------------------------------------------------
    def _spin(self):
        if self._closed:
            return
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except Exception:
            pass
        self._update_ui()
        if not self._closed:
            self._timer = self.root.after(20, self._spin)

    # ------------------------------------------------------------------
    # Actualización visual
    # ------------------------------------------------------------------
    def _update_ui(self):
        now = time.time()
        if self.last_rx_wall == 0.0:
            self.lbl_status.config(text='Esperando datos de /guante/imu...', fg='#d6a84a')
        else:
            age = now - self.last_rx_wall
            if age > 0.5:
                self.lbl_status.config(
                    text=f'Sin datos desde hace {age:.1f} s  |  {self.rate_hz:.0f} Hz',
                    fg='#e06c6c')
            else:
                self.lbl_status.config(
                    text=f'Conectado  |  {self.rate_hz:.0f} Hz  |  última muestra {age*1000:.0f} ms',
                    fg='#7fd6a0')

        if self.msg_count == self._last_drawn:
            return
        self._last_drawn = self.msg_count

        roll = math.degrees(self._roll_rel())
        pitch = math.degrees(self._pitch_rel())
        tilt = self._tilt_deg()

        self.lbl_values.config(text=(
            f'ax = {self.ax:+.3f} g      gx = {self.gx:+8.1f} °/s\n'
            f'ay = {self.ay:+.3f} g      gy = {self.gy:+8.1f} °/s\n'
            f'az = {self.az:+.3f} g      gz = {self.gz:+8.1f} °/s\n'
            f'\nroll  = {roll:+7.1f}°\n'
            f'pitch = {pitch:+7.1f}°\n'
            f'tilt  = {tilt:+7.1f}°'
        ))

        self._draw_bubble()
        self._draw_cube()

    # ------------------------------------------------------------------
    # Nivel de burbuja
    # ------------------------------------------------------------------
    def _draw_bubble(self):
        c = self.bubble
        c.delete('all')

        w = c.winfo_width()
        h = c.winfo_height()
        if w <= 1:
            w = int(c.cget('width'))
        if h <= 1:
            h = int(c.cget('height'))
        cx, cy = w / 2.0, h / 2.0
        r = min(w, h) / 2.0 - 38.0

        # Círculos de referencia
        c.create_oval(cx - r, cy - r, cx + r, cy + r,
                      outline='#2e6078', width=2)
        for f in (1.0 / 3.0, 2.0 / 3.0):
            rr = r * f
            c.create_oval(cx - rr, cy - rr, cx + rr, cy + rr,
                          outline='#1d3a4a', dash=(3, 5))
        c.create_line(cx - r, cy, cx + r, cy, fill='#1d3a4a')
        c.create_line(cx, cy - r, cx, cy + r, fill='#1d3a4a')

        # Etiquetas
        c.create_text(cx, cy - r - 14, text='ADELANTE',
                      fill='#6fa8c0', font=('Arial', 8, 'bold'))
        c.create_text(cx, cy + r + 14, text='ATRÁS',
                      fill='#6fa8c0', font=('Arial', 8, 'bold'))
        c.create_text(cx - r - 14, cy, text='IZQ',
                      fill='#6fa8c0', font=('Arial', 8, 'bold'), anchor='e')
        c.create_text(cx + r + 14, cy, text='DER',
                      fill='#6fa8c0', font=('Arial', 8, 'bold'), anchor='w')

        # Vector de inclinación: diferencia respecto al cero
        dax = self.ax - self.ax0
        day = self.ay - self.ay0

        max_g = 0.7  # 0.7 g ≈ 45° de inclinación -> borde
        scale = r / max_g
        dx = day * scale
        dy = dax * scale  # ax negativo -> arriba (adelante)

        mag = math.hypot(dx, dy)
        if mag > r:
            dx *= r / mag
            dy *= r / mag

        ex, ey = cx + dx, cy + dy

        # Línea desde el centro y burbuja
        c.create_line(cx, cy, ex, ey, fill='#3a8ab0', width=2)
        c.create_oval(ex - 11, ey - 11, ex + 11, ey + 11,
                      fill='#ffb347', outline='#ffffff', width=2)
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3,
                      fill='#4a7a90', outline='')

    # ------------------------------------------------------------------
    # Cubo 3D
    # ------------------------------------------------------------------
    def _draw_cube(self):
        c = self.cube
        c.delete('all')

        w = c.winfo_width()
        h = c.winfo_height()
        if w <= 1:
            w = int(c.cget('width'))
        if h <= 1:
            h = int(c.cget('height'))
        cx, cy = w / 2.0, h / 2.0 + 18.0
        scale = min(w, h) * 0.23

        # Referencia de gravedad y "piso"
        c.create_oval(cx - 150, cy + 80, cx + 150, cy + 150,
                      outline='#1d3a4a', dash=(3, 5))
        c.create_line(cx, cy - 175, cx, cy + 125,
                      fill='#345a6a', dash=(4, 6), arrow='last')
        c.create_text(cx + 8, cy - 180, anchor='w', text='gravedad',
                      fill='#5f8fa5', font=('Arial', 8))

        # Rotaciones: roll alrededor de X, pitch alrededor de Y.
        # El signo de roll se invierte para que el lado derecho baje
        # cuando ay > 0.
        roll = -self._roll_rel()
        pitch = self._pitch_rel()

        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)

        Rx = np.array([[1, 0, 0],
                       [0, cr, -sr],
                       [0, sr, cr]], dtype=float)
        Ry = np.array([[cp, 0, sp],
                       [0, 1, 0],
                       [-sp, 0, cp]], dtype=float)
        R = Ry @ Rx

        s = 0.58
        verts = np.array([
            [-s, -s, -s], [s, -s, -s], [s, s, -s], [-s, s, -s],
            [-s, -s,  s], [s, -s,  s], [s, s,  s], [-s, s,  s],
        ], dtype=float)
        rv = (R @ verts.T).T

        def project(p):
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            # Proyección isométrica (Z hacia arriba en pantalla)
            sx = (x - y) * 0.8660254
            sy = (x + y) * 0.5 + z
            return cx + sx * scale, cy - sy * scale

        # Aristas del cubo
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]
        for i, j in edges:
            x1, y1 = project(rv[i])
            x2, y2 = project(rv[j])
            c.create_line(x1, y1, x2, y2, fill='#7fb0c8', width=2)

        # Ejes del sensor (rotados)
        axes = [
            ((1.0, 0.0, 0.0), '#ff6b6b', 'X'),
            ((0.0, 1.0, 0.0), '#6bff9b', 'Y'),
            ((0.0, 0.0, 1.0), '#6bb5ff', 'Z'),
        ]
        for vec, color, label in axes:
            p = R @ np.array(vec, dtype=float)
            x2, y2 = project(p)
            c.create_line(cx, cy, x2, y2, fill=color, width=3, arrow='last')
            c.create_text(x2, y2, text=label, fill=color,
                          font=('Arial', 11, 'bold'))

        # Texto de ayuda
        c.create_text(12, 12, anchor='nw',
                      text='Cubo = sensor\nLínea punteada = gravedad',
                      fill='#6f9ab0', font=('Arial', 9), justify='left')

    # ------------------------------------------------------------------
    # Cierre
    # ------------------------------------------------------------------
    def on_close(self):
        if self._closed:
            return
        self._closed = True
        if self._timer is not None:
            try:
                self.root.after_cancel(self._timer)
            except Exception:
                pass
            self._timer = None
        try:
            self.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()


def main():
    rclpy.init()
    gui = GloveImuGui()
    try:
        gui.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if not gui._closed:
            gui.on_close()


if __name__ == '__main__':
    main()
