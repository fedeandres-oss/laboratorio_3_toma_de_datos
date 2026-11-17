#!/usr/bin/env python3
"""
discover_carro.py
=================
Script para descubrir el carro y que empiece a enviar telemetría.
Ejecuta este script UNA VEZ antes de lanzar los nodos ROS 2.
"""

import socket
import time

CARRO_PORT = 5005
CARRO_IP_BROADCAST = "255.255.255.255"

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.5)

    print("=== Descubrimiento del Carro ===")
    print(f"Enviando comando '0,0' a {CARRO_IP_BROADCAST}:{CARRO_PORT}")
    print("Esto hará que el carro empiece a enviar telemetría al PC.\n")

    # Enviar 10 paquetes de descubrimiento con 1 segundo de diferencia
    for i in range(10):
        try:
            sock.sendto(b"0,0\n", (CARRO_IP_BROADCAST, CARRO_PORT))
            print(f"[{i+1}/10] Enviado descubrimiento (broadcast)...")
        except Exception as e:
            print(f"Error: {e}")
        time.sleep(1.0)

    print("\nDescubrimiento completado.")
    print("Ahora ejecuta: ./launch_ros2_monitor.sh")
    print("El carro debería estar enviando telemetría y el sensor_bridge detectará su IP.")

    sock.close()


if __name__ == "__main__":
    main()
