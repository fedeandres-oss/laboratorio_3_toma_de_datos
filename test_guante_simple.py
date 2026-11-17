#!/usr/bin/env python3
"""
Script simple para probar si el guante envía datos por UDP al puerto 5000.
"""

import socket
import sys
import time

PORT = 5000

print(f"Probando recepción UDP en puerto {PORT}...")
print("Esperando datos del guante...")
print("(Presiona Ctrl+C para salir)")
print("-" * 50)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("0.0.0.0", PORT))
sock.settimeout(2.0)

count = 0

try:
    while True:
        try:
            data, addr = sock.recvfrom(2048)
            count += 1
            texto = data.decode('utf-8', errors='ignore')
            print(f"[#{count}] Desde {addr[0]}:{addr[1]} -> {texto[:100]}")
        except socket.timeout:
            print(".", end="", flush=True)
except KeyboardInterrupt:
    print(f"\n\nTotal: {count} paquetes recibidos")
    if count > 0:
        print("✅ El guante SÍ envía datos por UDP")
    else:
        print("❌ No se recibieron datos del guante")
finally:
    sock.close()