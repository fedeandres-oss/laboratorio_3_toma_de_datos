#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibra el trim izquierda/derecha del carro para que avance recto.

El firmware del carro acepta el comando UDP:
    TRIM,<izq>,<der>      (1.00 = sin ajuste, rango 0.50 a 1.50)

Este script:
  1. Detiene el carro y mide el sesgo del giroscopio Z (gz) en reposo.
  2. Aplica el trim indicado.
  3. Avanza a velocidad lineal v con giro w=0 durante unos segundos.
  4. Promedia gz y las velocidades de los encoders.
  5. Te dice si el carro giró a la izquierda o a la derecha.

Uso:
    python3 calibrar_recto.py --v 0.70 --left 1.05 --right 1.00
    python3 calibrar_recto.py --v 0.70 --left 1.10 --right 1.00
    ...

Si el giro neto sale positivo (izquierda), sube --left o baja --right.
Si sale negativo (derecha), baja --left o sube --right.
Repite hasta que el giro neto quede cerca de 0.

IMPORTANTE: detén antes carro_commander.py / launch_ros2_monitor.sh,
porque ambos usan el puerto UDP 5005.
"""

import argparse
import select
import socket
import statistics
import sys
import time

PORT = 5005


def parse_telemetry(msg):
    datos = {}
    if not msg.startswith("ROBOT|"):
        return None
    for parte in msg.split("|")[1:]:
        if "=" in parte:
            k, v = parte.split("=", 1)
            try:
                datos[k] = float(v)
            except ValueError:
                pass
    return datos


def descubrir_carro(sock, port=PORT, timeout=4.0):
    """Busca el carro en la /24 local enviando 0,0 y esperando ROBOT|."""
    local_ip = None
    tmp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        tmp.connect(("8.8.8.8", 80))
        local_ip = tmp.getsockname()[0]
    except OSError:
        pass
    finally:
        tmp.close()
    if not local_ip or local_ip.startswith("127."):
        return None

    prefix = ".".join(local_ip.split(".")[:3]) + "."
    candidatos = [f"{prefix}{i}" for i in range(1, 255)]
    t0 = time.time()
    while time.time() - t0 < timeout:
        for ip in candidatos:
            try:
                sock.sendto(b"0,0\n", (ip, port))
            except OSError:
                pass
        try:
            sock.sendto(b"0,0\n", ("255.255.255.255", port))
        except OSError:
            pass

        t1 = time.time()
        while time.time() - t1 < 0.4:
            r, _, _ = select.select([sock], [], [], 0.05)
            if r:
                try:
                    data, addr = sock.recvfrom(2048)
                except OSError:
                    continue
                if data.startswith(b"ROBOT|"):
                    return addr[0]
    return None


def main():
    ap = argparse.ArgumentParser(description="Calibra el trim izquierda/derecha del carro")
    ap.add_argument("--ip", default="auto", help="IP del carro o 'auto' para buscarlo (default: auto)")
    ap.add_argument("--v", type=float, default=0.70, help="velocidad lineal de prueba en m/s")
    ap.add_argument("--left", type=float, default=1.00, help="trim rueda izquierda (1.00 = sin ajuste)")
    ap.add_argument("--right", type=float, default=1.00, help="trim rueda derecha (1.00 = sin ajuste)")
    ap.add_argument("--duration", type=float, default=4.0, help="segundos avanzando")
    ap.add_argument("--bias-time", type=float, default=1.5, help="segundos midiendo el sesgo en reposo")
    ap.add_argument("--no-bias", action="store_true", help="no medir sesgo; usar 0")
    args = ap.parse_args()

    if not (0.50 <= args.left <= 1.50) or not (0.50 <= args.right <= 1.50):
        print("ERROR: los trim deben estar entre 0.50 y 1.50")
        return 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    try:
        sock.bind(("0.0.0.0", PORT))
    except OSError as e:
        print(f"ERROR: no se pudo usar el puerto UDP {PORT}: {e}")
        print("Cierra carro_commander.py / launch_ros2_monitor.sh y vuelve a intentar.")
        return 1
    sock.setblocking(False)

    if args.ip.lower() == "auto":
        print("Buscando carro en la red local...")
        ip = descubrir_carro(sock, PORT)
        if not ip:
            print("ERROR: no se encontró ningún carro respondiendo en el puerto 5005.")
            print("Revisa que el carro esté encendido y en la misma red WiFi que la PC.")
            return 1
        carro = (ip, PORT)
        print(f"Carro encontrado: {carro[0]}:{PORT}")
    else:
        carro = (args.ip, PORT)

    def enviar(cmd):
        for _ in range(30):
            try:
                sock.sendto(cmd, carro)
                return True
            except BlockingIOError:
                time.sleep(0.003)
            except OSError:
                time.sleep(0.01)
        return False

    def drenar_y_promediar(dur, skip=0.0):
        """Envía 0,0 y acumula telemetría. Ignora los primeros 'skip' segundos."""
        t0 = time.time()
        vals = {}
        n = 0
        while time.time() - t0 < dur:
            enviar(b"0,0\n")
            r, _, _ = select.select([sock], [], [], 0.02)
            if r:
                try:
                    data, _ = sock.recvfrom(2048)
                except OSError:
                    continue
                d = parse_telemetry(data.decode("utf-8", "ignore").strip())
                if d and (time.time() - t0) >= skip:
                    n += 1
                    for k, v in d.items():
                        vals.setdefault(k, []).append(v)
            time.sleep(0.02)
        return vals, n

    def avanzar(dur):
        t0 = time.time()
        vals = {}
        n = 0
        ultimo_trim_env = 0.0
        while time.time() - t0 < dur:
            # Reenviar TRIM cada 1 s por si el carro se reinició
            if time.time() - ultimo_trim_env > 1.0:
                enviar(f"TRIM,{args.left:.3f},{args.right:.3f}\n".encode())
                ultimo_trim_env = time.time()
            enviar(f"{args.v:.3f},0.000\n".encode())
            r, _, _ = select.select([sock], [], [], 0.02)
            if r:
                try:
                    data, _ = sock.recvfrom(2048)
                except OSError:
                    continue
                d = parse_telemetry(data.decode("utf-8", "ignore").strip())
                if d:
                    # Ignorar los primeros 0.4 s (transitorio de arranque)
                    if time.time() - t0 > 0.4:
                        n += 1
                        for k, v in d.items():
                            vals.setdefault(k, []).append(v)
            time.sleep(0.02)
        return vals, n

    def media(d, k, default=0.0):
        v = d.get(k)
        return statistics.mean(v) if v else default

    def mediana(d, k, default=0.0):
        v = d.get(k)
        return statistics.median(v) if v else default

    print(f"Carro: {carro[0]}:{PORT}")
    print(f"Trim a probar: izquierda={args.left:.3f}  derecha={args.right:.3f}")

    # 1) Detener y medir sesgo (ignorar el primer 0.4 s de frenado)
    print("Deteniendo carro y midiendo sesgo de gz...")
    vals_rep, n_rep = drenar_y_promediar(args.bias_time, skip=0.4)
    bias = 0.0 if args.no_bias else mediana(vals_rep, "gz", 0.0)
    print(f"  gz_bias = {bias:+.3f} deg/s   (n={n_rep})")
    if not args.no_bias and abs(bias) > 5.0:
        print("  [AVISO] El sesgo es alto: el carro se movió durante la medición.")
        print("          Déjalo completamente quieto y repite.")
    if n_rep < 20:
        print(f"  [AVISO] Solo {n_rep} paquetes en reposo; puede haber pérdida por WiFi.")

    # Aplicar trim antes de arrancar
    for _ in range(5):
        enviar(f"TRIM,{args.left:.3f},{args.right:.3f}\n".encode())
        time.sleep(0.02)

    # 2) Avanzar recto
    print(f"Avanzando v={args.v:.2f} m/s durante {args.duration:.1f} s...")
    vals, n = avanzar(args.duration)

    # 3) Detener
    for _ in range(10):
        enviar(b"0,0\n")
        time.sleep(0.02)

    if n == 0:
        print("ERROR: no llegó telemetría. Revisa la IP del carro y que esté encendido.")
        return 1

    gz = mediana(vals, "gz", 0.0)
    giro_neto = gz - bias
    enc_l = media(vals, "enc_l_filt", 0.0)
    enc_r = media(vals, "enc_r_filt", 0.0)
    raw_l = media(vals, "raw_l", 0.0)
    raw_r = media(vals, "raw_r", 0.0)
    trim_l = media(vals, "trim_l", args.left)
    trim_r = media(vals, "trim_r", args.right)
    pwm_l = media(vals, "pwm_l", 0.0)
    pwm_r = media(vals, "pwm_r", 0.0)

    print("\n--- Resultado ---")
    print(f"  gz medio        = {gz:+.2f} deg/s")
    print(f"  giro neto       = {giro_neto:+.2f} deg/s  (gz - sesgo)")
    print(f"  enc_l_filt      = {enc_l:+.2f} rad/s")
    print(f"  enc_r_filt      = {enc_r:+.2f} rad/s")
    print(f"  raw_l / raw_r   = {raw_l:+.1f} / {raw_r:+.1f} cuentas por ciclo")
    print(f"  pwm_l / pwm_r   = {pwm_l:+.1f}% / {pwm_r:+.1f}%")
    print(f"  trim confirmado = izq {trim_l:.3f} / der {trim_r:.3f}")

    if abs(raw_l) < 0.05 and abs(raw_r) < 0.05:
        print("\n[AVISO] Los encoders no están contando (raw_l/raw_r = 0).")
        print("        Revisa VCC/GND de los encoders y que las señales lleguen a")
        print("        GPIO32 (izquierdo) y GPIO25 (derecho).")
    else:
        if abs(enc_r) > 0.1:
            ratio = enc_l / enc_r
            print(f"\n  Relación enc_l/enc_r = {ratio:.3f}  (1.000 = misma velocidad)")

    print("\nPara corregir:")
    if giro_neto > 0.5:
        print("  El carro gira a la IZQUIERDA -> sube --left o baja --right.")
    elif giro_neto < -0.5:
        print("  El carro gira a la DERECHA  -> baja --left o sube --right.")
    else:
        print("  El carro va prácticamente recto. Deja este trim.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
