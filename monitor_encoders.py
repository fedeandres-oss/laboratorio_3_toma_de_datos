#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Monitor de encoders del carro.

Muestra en vivo raw_l / raw_r (pulsos por ciclo de 10 ms) y la velocidad
calculada. Sirve para probar los encoders girando las ruedas A MANO:

  1. Levanta el carro para que las ruedas giren libremente.
  2. Ejecuta:  python3 monitor_encoders.py
  3. Gira despacio la rueda IZQUIERDA con la mano.
     raw_l debe cambiar entre 0, +1, -1, +2, ...
  4. Gira la rueda DERECHA.
     raw_r debe cambiar.

Si raw_l/raw_r se quedan en 0 mientras giras la rueda, el pulso no está
llegando al ESP32. Revisa VCC/GND del encoder, el cable de señal y el pin:
  - encoder izquierdo -> GPIO32
  - encoder derecho   -> GPIO25

Detén carro_commander.py / launch_ros2_monitor.sh antes de ejecutarlo.
"""

import argparse
import select
import socket
import sys
import time

from calibrar_recto import PORT, descubrir_carro, parse_telemetry


def main():
    ap = argparse.ArgumentParser(description="Monitor en vivo de los encoders del carro")
    ap.add_argument("--ip", default="auto", help="IP del carro o 'auto' (default: auto)")
    args = ap.parse_args()

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
            print("ERROR: no se encontró el carro en el puerto 5005.")
            return 1
        carro = (ip, PORT)
    else:
        carro = (args.ip, PORT)

    print(f"Carro: {carro[0]}:{PORT}")
    print("Moviendo el carro a 0,0 (detenido). Gira las ruedas a mano.")
    print("raw_l / raw_r = pulsos en el último ciclo de 10 ms.")
    print("Ctrl+C para salir.\n")

    def enviar():
        try:
            sock.sendto(b"0,0\n", carro)
        except OSError:
            pass

    ultimo_print = 0.0
    try:
        while True:
            enviar()
            r, _, _ = select.select([sock], [], [], 0.05)
            if r:
                try:
                    data, _ = sock.recvfrom(2048)
                except OSError:
                    data = b""
                msg = data.decode("utf-8", "ignore").strip()
                d = parse_telemetry(msg)
                if d and time.time() - ultimo_print > 0.25:
                    ultimo_print = time.time()
                    print(
                        f"raw_l={int(d.get('raw_l', 0)):+4d}  "
                        f"raw_r={int(d.get('raw_r', 0)):+4d}  |  "
                        f"enc_l={d.get('enc_l_filt', 0.0):+7.2f} rad/s  "
                        f"enc_r={d.get('enc_r_filt', 0.0):+7.2f} rad/s  |  "
                        f"trim=({d.get('trim_l', 1.0):.3f},{d.get('trim_r', 1.0):.3f})"
                    )
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nSaliendo...")
    finally:
        for _ in range(10):
            enviar()
            time.sleep(0.02)
    return 0


if __name__ == "__main__":
    sys.exit(main())
