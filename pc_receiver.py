#!/usr/bin/env python3
"""
pc_receiver.py
Servidor UDP para el Guante ESP32 + MPU6050.
Descubre al ESP32 por broadcast, recibe sus mediciones y responde con ACK.
Imprime TODO lo que llega al puerto 5000 para depuración.
"""

import socket
import time
from datetime import datetime

DISCOVER_PORT = 5000
LOCAL_PORT = 5001
BUFFER_SIZE = 1024
GUANTE_TIMEOUT_SEC = 10  # segundos sin datos antes de olvidar al guante


def get_local_ip():
    """Obtiene la IP local de la interfaz de red del PC."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


def log(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {msg}", flush=True)


def parse_mpu_data(msg):
    """Parsea tramas MPU|... o TEST|..."""
    parts = msg.split('|')
    if len(parts) < 2 or parts[0] not in ("MPU", "TEST"):
        return None
    data = {"type": parts[0]}
    for item in parts[1:]:
        if '=' in item:
            k, v = item.split('=', 1)
            try:
                data[k] = float(v) if '.' in v else int(v)
            except ValueError:
                data[k] = v
    return data


def main():
    local_ip = get_local_ip()
    log(f"IP local detectada: {local_ip}")
    log(f"Escuchando en UDP 0.0.0.0:{DISCOVER_PORT} ...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except Exception:
        pass
    sock.bind(("0.0.0.0", DISCOVER_PORT))
    sock.settimeout(1.0)

    guante_ip = None
    last_seen = None

    log("Esperando descubrimiento del ESP32 (DISCOVER_PC)...")

    try:
        while True:
            try:
                data, addr = sock.recvfrom(BUFFER_SIZE)
                msg = data.decode(errors="ignore").strip()
                ip_remota = addr[0]
                puerto_remoto = addr[1]
                now = time.time()

                # Imprimir SIEMPRE lo que llegue al puerto 5000
                log(f"RECIBIDO de {ip_remota}:{puerto_remoto} -> {msg}")

                if msg == "DISCOVER_PC":
                    log(f"Descubrimiento de {ip_remota}")
                    guante_ip = ip_remota
                    last_seen = now
                    respuesta = f"PC_IP:{local_ip}\n"
                    sock.sendto(respuesta.encode(), addr)
                    log(f"Respondido a {guante_ip} con IP {local_ip}")

                elif guante_ip and ip_remota == guante_ip:
                    last_seen = now
                    mpu = parse_mpu_data(msg)
                    if mpu:
                        if mpu.get("type") == "MPU":
                            log(
                                f"[Seq {mpu.get('seq', 0):>4}] "
                                f"Accel(g): X={mpu.get('ax', 0):+5.2f}, "
                                f"Y={mpu.get('ay', 0):+5.2f}, "
                                f"Z={mpu.get('az', 0):+5.2f} | "
                                f"Gyro(dps): X={mpu.get('gx', 0):+6.1f}, "
                                f"Y={mpu.get('gy', 0):+6.1f}, "
                                f"Z={mpu.get('gz', 0):+6.1f} | "
                                f"Temp: {mpu.get('temp', 0):.1f}°C"
                            )
                        else:
                            log(f"TEST del guante: {msg}")
                    # Responder ACK al ESP32 (puerto 5001 del ESP32)
                    try:
                        sock.sendto(b"ACK", addr)
                    except Exception as e:
                        log(f"Error enviando ACK: {e}")
                else:
                    log(f"Mensaje de dispositivo desconocido ({ip_remota}): {msg}")

            except socket.timeout:
                if guante_ip is not None and last_seen is not None:
                    if now - last_seen > GUANTE_TIMEOUT_SEC:
                        log(f"⚠️ Guante {guante_ip} perdido ({GUANTE_TIMEOUT_SEC}s sin datos). Esperando nuevo descubrimiento...")
                        guante_ip = None
                        last_seen = None
                continue

    except KeyboardInterrupt:
        log("Saliendo...")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
