#!/usr/bin/env python3
"""
pc_coordinator.py
=================
PC coordinador para el Laboratorio 3 - Procesamiento de Señales.

Maneja simultáneamente:
  - MICRO 1 (guante, ESP32-S3)  <- UDP puerto 5000  (descubrimiento + MPU)
                                    -> Puerto 5001 (ACK al guante)
  - MICRO 2 (carro, ESP32)      <- UDP puerto 5005  (telemetría ROBOT + envío de comandos vl,vr)

Capas del sistema:
  - Bajo nivel (micros): captura, filtrado, control PID, transmisión UDP.
  - Medio nivel (PC): lectura del eje Y del MPU, deadzone, estimación de velocidad.
  - Alto nivel (PC): envío de comandos al carro, modo automático/manual, consola.
"""

import socket
import threading
import time
from datetime import datetime

# ---------------------------------------------------------------------------
# Puertos
# ---------------------------------------------------------------------------
DISCOVER_PORT = 5000   # Micro 1 (guante) - recibe MPU del guante
LOCAL_PORT    = 5001   # Puerto local del PC para enviar ACK al guante
CARRO_PORT    = 5005   # Micro 2 (carro) - recibe telemetría y envía comandos
BUFFER_SIZE   = 1024

# ---------------------------------------------------------------------------
# Configuración de coordinación (Medio y Alto Nivel)
# ---------------------------------------------------------------------------
MAX_SPEED      = 1.5   # rad/s máximos por rueda
AUTO_MODE      = False # False por defecto: no enviar comandos solos al carro
SEND_INTERVAL  = 0.05  # segundos entre envíos automáticos (20 Hz)

# --- Mapeo MPU (eje Y) -> velocidad del carro ---
# El MPU va "derecho": eje Z apunta hacia arriba (az ≈ 1.0 g).
# Al inclinar el guante hacia adelante/atrás, el eje Y detecta gravedad.
# Mantener inclinado = moverse constantemente. Volver a nivel = detenerse.
DEADZONE_AY    = 0.03  # umbral en g para que no se mueva solo con ruido
GANANCIA_AY    = 2.0   # factor de escala (rad/s por g de inclinación)
INVERTIR_AY    = False # pon True si el carro va hacia atrás cuando debería ir adelante
USAR_GIRO_GX   = False # usa giroscopio X para giro diferencial (opcional)

# Filtros de consola (para no saturar)
QUIET_GUANTE = True    # oculta logs del Micro 1 por defecto (anti-saturación)
QUIET_CARRO  = False   # oculta telemetría del Micro 2

# Colores ANSI (opcional, se desactivan si la terminal no los soporta)
C_RESET  = "\033[0m"
C_GUANTE = "\033[94m"   # azul
C_CARRO  = "\033[92m"   # verde
C_CMD    = "\033[93m"   # amarillo
C_SYS    = "\033[96m"   # cian
C_ERR    = "\033[91m"   # rojo


def use_color():
    try:
        import sys
        return sys.stdout.isatty()
    except Exception:
        return False


COLOR = use_color()
def c(code, text):
    return f"{code}{text}{C_RESET}" if COLOR else text

# ---------------------------------------------------------------------------
# Estado compartido
# ---------------------------------------------------------------------------
state_lock = threading.Lock()
guante_ip   = None
carro_ip    = None
last_mpu    = None
last_robot  = None


def log(msg, color=C_RESET):
    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {msg}", flush=True)


def log_guante(msg):
    global QUIET_GUANTE
    if QUIET_GUANTE:
        return
    log(c(C_GUANTE, "[MICRO 1/guante] " + msg))


def log_carro(msg):
    global QUIET_CARRO
    if QUIET_CARRO:
        return
    log(c(C_CARRO, "[MICRO 2/carro]  " + msg))


def log_cmd(msg):
    log(c(C_CMD, "[CMD] " + msg))


def log_sys(msg):
    log(c(C_SYS, "[SYS] " + msg))


def log_err(msg):
    log(c(C_ERR, "[ERR] " + msg))


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


# ---------------------------------------------------------------------------
# Parsers
# ---------------------------------------------------------------------------
def parse_mpu_data(msg):
    """'MPU|seq=1|ax=0.12|ay=...|az=...|gx=...|gy=...|gz=...|temp=25.0'"""
    parts = msg.split("|")
    if len(parts) < 2 or parts[0] not in ("MPU", "TEST"):
        return None
    data = {"type": parts[0]}
    for item in parts[1:]:
        if "=" in item:
            k, v = item.split("=", 1)
            try:
                data[k] = float(v) if "." in v else int(v)
            except ValueError:
                data[k] = v
    return data


def parse_robot_data(msg):
    """'ROBOT|seq=1|t_ms=123|enc_l=0.50|enc_r=0.48|...|gz=0.1'"""
    if not msg.startswith("ROBOT|"):
        return None
    parts = msg.split("|")
    data = {"type": "ROBOT"}
    for item in parts[1:]:
        if "=" in item:
            k, v = item.split("=", 1)
            try:
                data[k] = float(v) if "." in v else int(v)
            except ValueError:
                data[k] = v
    return data


# ---------------------------------------------------------------------------
# Capa MEDIA: Estimación de intención de movimiento desde el MPU del guante
# ---------------------------------------------------------------------------
def estimar_velocidad_desde_mpu(mpu):
    """
    Convierte la aceleración del eje Y del guante en velocidad lineal para el carro.
    
    El MPU va montado "derecho":
      - az ≈ +1.0 g (gravedad hacia Z)
      - Cuando inclinas el guante hacia adelante, ay se desvía de 0.
      - Cuando inclinas hacia atrás, ay se desvía al signo opuesto.
    
    Devuelve (vl, vr, v_lineal, v_giro) en rad/s.
    """
    ay = mpu.get("ay", 0.0)   # aceleración eje Y en g
    gx = mpu.get("gx", 0.0)   # giroscopio eje X en dps

    # --- Deadzone para evitar movimiento por ruido ---
    if abs(ay) <= DEADZONE_AY:
        v_lineal = 0.0
    else:
        signo = 1.0 if ay > 0 else -1.0
        v_lineal = (abs(ay) - DEADZONE_AY) * GANANCIA_AY * signo
        if INVERTIR_AY:
            v_lineal = -v_lineal

    # Limitar velocidad lineal
    v_lineal = max(-MAX_SPEED, min(MAX_SPEED, v_lineal))

    # --- Giro diferencial opcional (giroscopio X) ---
    v_giro = 0.0
    if USAR_GIRO_GX:
        v_giro = (gx / 180.0) * MAX_SPEED
        v_giro = max(-MAX_SPEED, min(MAX_SPEED, v_giro))

    # Combinar: vl = lineal + giro, vr = lineal - giro
    vl = v_lineal + v_giro
    vr = v_lineal - v_giro

    # Saturación final por rueda
    vl = max(-MAX_SPEED, min(MAX_SPEED, vl))
    vr = max(-MAX_SPEED, min(MAX_SPEED, vr))

    return vl, vr, v_lineal, v_giro


# ---------------------------------------------------------------------------
# Envío de comandos al carro
# ---------------------------------------------------------------------------
def send_to_carro(sock, vl, vr, addr):
    msg = f"{vl:.3f},{vr:.3f}\n"
    try:
        sock.sendto(msg.encode(), addr)
        log_cmd(f"enviado a {addr[0]}: {msg.strip()}")
    except Exception as e:
        log_err(f"No se pudo enviar al carro: {e}")


# ---------------------------------------------------------------------------
# Hilo A: Micro 1 / Guante (puerto 5000) + ACK al puerto 5001
# ---------------------------------------------------------------------------
def hilo_guante():
    global guante_ip, last_mpu, QUIET_GUANTE
    local_ip = get_local_ip()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except Exception:
        pass

    sock.bind(("0.0.0.0", DISCOVER_PORT))
    sock.settimeout(1.0)

    log_sys(f"Escuchando guante (Micro 1) en UDP 0.0.0.0:{DISCOVER_PORT}")
    log_sys(f"Enviando ACK al puerto {LOCAL_PORT} del guante")

    while True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            msg = data.decode(errors="ignore").strip()
            ip_remota = addr[0]

            if msg == "DISCOVER_PC":
                with state_lock:
                    guante_ip = ip_remota
                log_sys(f"Guante descubierto en {guante_ip}")
                respuesta = f"PC_IP:{local_ip}\n"
                sock.sendto(respuesta.encode(), addr)
                log_sys(f"Respondido PC_IP:{local_ip} a {ip_remota}")

            elif guante_ip and ip_remota == guante_ip:
                mpu = parse_mpu_data(msg)
                if mpu:
                    with state_lock:
                        last_mpu = mpu
                    if mpu.get("type") == "MPU":
                        log_guante(
                            f"seq={mpu.get('seq', 0):>4} "
                            f"Accel(g) X={mpu.get('ax', 0):+6.2f} "
                            f"Y={mpu.get('ay', 0):+6.2f} "
                            f"Z={mpu.get('az', 0):+6.2f} | "
                            f"Gyro(dps) X={mpu.get('gx', 0):+7.1f} "
                            f"Y={mpu.get('gy', 0):+7.1f} "
                            f"Z={mpu.get('gz', 0):+7.1f} | "
                            f"Temp={mpu.get('temp', 0):.1f}°C"
                        )
                    else:
                        log_guante(f"TEST: {msg}")
                # --- ACK al puerto 5001 del guante (LOCAL_PORT) ---
                try:
                    sock.sendto(b"ACK", (ip_remota, LOCAL_PORT))
                except Exception as e:
                    log_err(f"ACK guante: {e}")
            else:
                log_guante(f"DESCONOCIDO {ip_remota}: {msg}", force=True)

        except socket.timeout:
            continue


# ---------------------------------------------------------------------------
# Hilo B: Micro 2 / Carro (puerto 5005) - recibe telemetría
# ---------------------------------------------------------------------------
def hilo_carro(sock_carro):
    global carro_ip, last_robot
    sock_carro.settimeout(1.0)
    log_sys(f"Escuchando carro (Micro 2) en UDP 0.0.0.0:{CARRO_PORT}")

    while True:
        try:
            data, addr = sock_carro.recvfrom(BUFFER_SIZE)
            msg = data.decode(errors="ignore").strip()
            ip_remota = addr[0]

            with state_lock:
                carro_ip = ip_remota

            robot = parse_robot_data(msg)
            if robot:
                with state_lock:
                    last_robot = robot
                log_carro(
                    f"seq={robot.get('seq', 0):>4} "
                    f"enc_l={robot.get('enc_l', 0):+7.3f} "
                    f"enc_r={robot.get('enc_r', 0):+7.3f} | "
                    f"enc_l_filt={robot.get('enc_l_filt', 0):+7.3f} "
                    f"enc_r_filt={robot.get('enc_r_filt', 0):+7.3f}"
                )
            else:
                log_carro(f"mensaje sin parsear: {msg}", force=True)

        except socket.timeout:
            continue


# ---------------------------------------------------------------------------
# Hilo C: Descubrimiento periódico del carro
# ---------------------------------------------------------------------------
def hilo_descubrimiento_carro(sock_carro):
    log_sys("Hilo de descubrimiento del carro iniciado (broadcast cada 1s).")
    while True:
        with state_lock:
            ya_sabemos = carro_ip is not None
        if not ya_sabemos:
            try:
                bcast = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                bcast.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                bcast.sendto(b"0,0\n", ("255.255.255.255", CARRO_PORT))
                bcast.close()
                log_sys("Broadcast descubrimiento enviado al carro (puerto 5005).")
            except Exception as e:
                log_err(f"Error en descubrimiento: {e}")
        time.sleep(1.0)


# ---------------------------------------------------------------------------
# Hilo D: Coordinación automática guante -> carro
# ---------------------------------------------------------------------------
def hilo_coordinacion(sock_carro):
    global AUTO_MODE
    log_sys(f"Coordinación automática (Medio/Alto nivel): {'ACTIVADA' if AUTO_MODE else 'DESACTIVADA'}")

    while True:
        time.sleep(SEND_INTERVAL)
        if not AUTO_MODE:
            continue

        with state_lock:
            mpu = last_mpu
            c_ip = carro_ip

        if mpu is None or c_ip is None:
            continue

        vl, vr, v_lineal, v_giro = estimar_velocidad_desde_mpu(mpu)
        addr = (c_ip, CARRO_PORT)
        send_to_carro(sock_carro, vl, vr, addr)

        # Log de la capa media (estimación)
        ay = mpu.get("ay", 0.0)
        estado = "MOVIÉNDOSE" if abs(v_lineal) > 0.01 else "DETENIDO"
        log_cmd(
            f"[CAPA MEDIA] {estado} | ay={ay:+7.4f}g | v_lineal={v_lineal:+.4f} rad/s | "
            f"vl={vl:+.4f} vr={vr:+.4f}"
        )


# ---------------------------------------------------------------------------
# Hilo E: Consola para comandos manuales al carro
# ---------------------------------------------------------------------------
def hilo_consola(sock_carro):
    global carro_ip, AUTO_MODE, QUIET_GUANTE, QUIET_CARRO

    log_sys("Consola activa. Comandos:")
    log_sys("  vl,vr        -> enviar velocidad al carro (ej: 1.0,-0.5)")
    log_sys("  auto on/off  -> activar/desactivar control automático desde guante")
    log_sys("  quiet on/off -> ocultar/mostrar logs del guante (Micro 1)")
    log_sys("  carro on/off -> ocultar/mostrar telemetría del carro (Micro 2)")
    log_sys("  discover     -> rediscover del carro (reenvía broadcast puerto 5005)")
    log_sys("  q            -> salir")

    while True:
        try:
            linea = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if linea.lower() in ("q", "quit", "exit"):
            break

        if linea.lower() == "auto on":
            AUTO_MODE = True
            log_sys("Modo automático ACTIVADO")
            continue
        if linea.lower() == "auto off":
            AUTO_MODE = False
            log_sys("Modo automático DESACTIVADO")
            continue
        if linea.lower() == "quiet on":
            QUIET_GUANTE = True
            log_sys("Logs del guante OCULTOS (quiet on)")
            continue
        if linea.lower() == "quiet off":
            QUIET_GUANTE = False
            log_sys("Logs del guante MOSTRADOS (quiet off)")
            continue
        if linea.lower() == "carro on":
            QUIET_CARRO = False
            log_sys("Telemetría del carro MOSTRADA (carro on)")
            continue
        if linea.lower() == "carro off":
            QUIET_CARRO = True
            log_sys("Telemetría del carro OCULTA (carro off)")
            continue
        if linea.lower() == "discover":
            try:
                bcast = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                bcast.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                bcast.sendto(b"0,0\n", ("255.255.255.255", CARRO_PORT))
                bcast.close()
                log_sys("Descubrimiento reenviado al carro (broadcast puerto 5005).")
            except Exception as e:
                log_err(f"No se pudo enviar descubrimiento: {e}")
            continue
        if not linea:
            continue

        partes = linea.split(",")
        if len(partes) != 2:
            print("Formato: vl,vr  (ejemplo: 0.5,-0.5)")
            continue

        try:
            vl = float(partes[0])
            vr = float(partes[1])
        except ValueError:
            print("Valores no numéricos")
            continue

        with state_lock:
            c_ip = carro_ip

        if c_ip is None:
            print("Aún no se conoce la IP del carro. Escribe 'discover' para reenviar el broadcast.")
            continue

        addr = (c_ip, CARRO_PORT)
        send_to_carro(sock_carro, vl, vr, addr)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    local_ip = get_local_ip()
    log_sys(f"IP local del PC: {local_ip}")
    log_sys(f"Puerto guante (Micro 1): {DISCOVER_PORT}  (ACK al puerto {LOCAL_PORT})")
    log_sys(f"Puerto carro  (Micro 2): {CARRO_PORT}")
    log_sys(f"Modo automático inicial: {'ON' if AUTO_MODE else 'OFF'}")
    log_sys(f"Filtro inicial: guante SILENCIADO (escribe 'quiet off' para verlo)")

    sock_carro = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_carro.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock_carro.bind(("0.0.0.0", CARRO_PORT))
    except Exception as e:
        log_err(f"No se pudo hacer bind al puerto {CARRO_PORT}: {e}")
        return

    threads = [
        threading.Thread(target=hilo_guante, daemon=True, name="guante"),
        threading.Thread(target=hilo_carro, args=(sock_carro,), daemon=True, name="carro_rx"),
        threading.Thread(target=hilo_descubrimiento_carro, args=(sock_carro,), daemon=True, name="descubrimiento"),
        threading.Thread(target=hilo_coordinacion, args=(sock_carro,), daemon=True, name="coordinacion"),
    ]
    for t in threads:
        t.start()

    try:
        hilo_consola(sock_carro)
    except KeyboardInterrupt:
        log_sys("Saliendo por teclado...")
    finally:
        sock_carro.close()
        log_sys("Fin.")


if __name__ == "__main__":
    main()
