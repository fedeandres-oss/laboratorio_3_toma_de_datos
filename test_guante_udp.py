#!/usr/bin/env python3
"""
test_guante_udp.py - Prueba de recepción UDP del guante (puerto 5000)
=====================================================================
Escucha en el puerto 5000 y muestra TODO lo que reciba del guante.
Útil para diagnosticar si el guante está enviando datos.
"""

import socket
import sys
import signal
import time

# Configuración
GUANTE_PORT = 5000
LOCAL_IP = "0.0.0.0"  # Escuchar en todas las interfaces

def signal_handler(sig, frame):
    print("\n\n=== Deteniendo prueba ===")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def main():
    print("=" * 60)
    print("PRUEBA DE RECEPCIÓN UDP - GUANTE (puerto 5000)")
    print("=" * 60)
    print(f"Escuchando en {LOCAL_IP}:{GUANTE_PORT}")
    print("Esperando datos del guante...")
    print("Presiona Ctrl+C para detener")
    print("=" * 60)
    print()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        sock.bind((LOCAL_IP, GUANTE_PORT))
    except Exception as e:
        print(f"ERROR: No se pudo hacer bind al puerto {GUANTE_PORT}: {e}")
        print("POSIBLE CAUSA: Otro proceso ya está usando el puerto (sensor_bridge.py)")
        print("  SOLUCIÓN: Detener sensor_bridge.py y volver a ejecutar este test")
        print(f"    pkill -f sensor_bridge.py")
        sys.exit(1)

    sock.settimeout(2.0)

    print(f"Escuchando en puerto {GUANTE_PORT}...")
    print("Esperando datos del guante...")
    print("-" * 60)

    paquetes_recibidos = 0
    ultima_vez = None

    try:
        while True:
            try:
                data, addr = sock.recvfrom(2048)
                paquetes_recibidos += 1
                ultima_vez = time.strftime("%H:%M:%S")

                # Mostrar información del paquete
                print(f"\n[#{paquetes_recibidos}] {ultima_vez} - Desde {addr[0]}:{addr[1]} - {len(data)} bytes")

                # Mostrar como texto
                try:
                    texto = data.decode('utf-8', errors='replace')
                    print(f"  Texto: {texto[:200]}")
                except:
                    print(f"  Hex: {data[:50].hex()}")

                # Si parece ser MPU, mostrar parseado
                if msg.startswith(b'MPU|') or msg.startswith(b'MPU'):
                    try:
                        texto = data.decode('utf-8', errors='replace')
                        partes = texto.split('|')
                        print(f"  Tipo: MPU")
                        for p in partes[1:]:
                            if '=' in p:
                                print(f"    {p}")
                    except:
                        pass
                elif msg.startswith(b'DISCOVER_PC'):
                    print(f"  Tipo: DISCOVER_PC (el guante busca al PC)")
                    print(f"  Respuesta esperada: PC_IP:{socket.gethostbyname(socket.gethostname())}")

            except socket.timeout:
                # Timeout cada 2 segundos, mostrar estado
                if paquetes_recibidos > 0:
                    print(f"\n[...] Paquetes recibidos totales: {paquetes_recibidos} (último a las {ultima_vez})")
                else:
                    print(".", end="", flush=True)  # Punto cada 2s sin datos
            except KeyboardInterrupt:
                break

    except KeyboardInterrupt:
        pass

    print(f"\n{'='*60}")
    print(f"RESULTADO: {paquetes_recibidos} paquetes recibidos en total")
    if paquetes_recibidos > 0:
        print(f"✅ EL GUANTE ESTÁ ENVIANDO DATOS - El sistema debería funcionar")
        print(f"Último paquete a las {ultima_vez}")
    else:
        print("❌ NO SE RECIBIERON DATOS DEL GUANTE")
        print()
        print("POSIBLES CAUSAS:")
        print("  1. El guante no está encendido o no está enviando datos")
        print("  2. El guante y el PC están en REDES DIFERENTES")
        print("  3. El guante y el PC tienen IPs en rangos diferentes")
        print("  4. Firewall bloqueando el puerto 5000")
        print("  5. El sensor_bridge.py ya está corriendo y ocupó el puerto")
        print()
        print("SOLUCIONES:")
        print("  - Verificar que el guante esté encendido y conectado a la MISMA WiFi que el PC")
        print("  - Verificar que ambas IPs estén en el mismo rango (ej: 192.168.80.x)")
        print("  - Detener sensor_bridge.py antes de ejecutar este test:")
        print("    pkill -f sensor_bridge.py")
    print("=" * 60)

    sock.close()

if __name__ == '__main__':
    main()