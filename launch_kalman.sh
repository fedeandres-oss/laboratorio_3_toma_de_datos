#!/bin/bash
# ============================================================================
# launch_kalman.sh - Lanza nodos ROS 2 con énfasis en KALMAN
# Laboratorio 3 - Toma de datos para comparación
# Configuración: filtrar_enc:=True, filtrar_imu:=True, configuracion:=kalman
# ============================================================================

source /opt/ros/jazzy/setup.bash
if [ -f /home/federico/ros2_ws/install/setup.bash ]; then
    source /home/federico/ros2_ws/install/setup.bash
fi

NODO_DIR="/media/federico/Nuevo_vol/7mo_semestre/procesamiento_de_senales/lab/laboratorio_3_toma_de_datos"

echo "============================================================"
echo "  ROS 2 Jazzy - TOMA DE DATOS: KALMAN (EKF fusión)"
echo "  Modos: teleop (guante) | goal (objetivo X,Y con GUI)"
echo "  Configuración: filtrar_enc:=True, filtrar_imu:=True"
echo "============================================================"
echo ""

# Matar nodos previos
pkill -f sensor_bridge.py 2>/dev/null
pkill -f movement_processor.py 2>/dev/null
pkill -f carro_commander.py 2>/dev/null
pkill -f goal_gui.py 2>/dev/null
pkill -f glove_imu_gui.py 2>/dev/null
pkill -f kalman_filter.py 2>/dev/null
pkill -f trajectory_logger.py 2>/dev/null
sleep 1

echo "[1/7] Lanzando sensor_bridge (UDP 5000 guante -> ROS)..."
python3 "$NODO_DIR/sensor_bridge.py" &
PID_BRIDGE=$!
echo "      PID: $PID_BRIDGE"

echo "[2/7] Lanzando movement_processor (/guante/imu -> /carro/cmd_vel)..."
python3 "$NODO_DIR/movement_processor.py" --ros-args -p invertir_x:=True &
PID_PROC=$!
echo "      PID: $PID_PROC"

echo "[3/7] Lanzando carro_commander (descubre carro + envía v,w o POS al carro)..."
# FIR en encoders ACTIVADO
python3 "$NODO_DIR/carro_commander.py" --ros-args -p left_trim:=1.00 -p right_trim:=1.16 -p filtrar_enc:=True &
PID_CMD=$!
echo "      PID: $PID_CMD"

echo "[4/7] Lanzando goal_gui (Interfaz: objetivo + posición desde MPU + trayectoria)..."
# FIR en GUI ACTIVADO
python3 "$NODO_DIR/goal_gui.py" --ros-args -p filtrar_imu:=True &
PID_GUI=$!
echo "      PID: $PID_GUI"

echo "[5/7] Lanzando glove_imu_gui (GUI aparte: inclinación del MPU del guante)..."
python3 "$NODO_DIR/glove_imu_gui.py" &
PID_IMU_GUI=$!
echo "      PID: $PID_IMU_GUI"

echo "[6/7] Lanzando kalman_filter (EKF: fusión MPU + encoders)..."
python3 "$NODO_DIR/kalman_filter.py" &
PID_KAL=$!
echo "      PID: $PID_KAL"

echo "[7/7] Lanzando trajectory_logger (guarda datos KALMAN a CSV)..."
python3 "$NODO_DIR/trajectory_logger.py" --ros-args -p configuracion:=kalman &
PID_LOGGER=$!
echo "      PID: $PID_LOGGER"

echo ""
echo "============================================================"
echo "  Esperando 5 segundos para descubrir el carro..."
echo "============================================================"
echo ""
sleep 5

echo ""
echo "============================================================"
echo "  Topics activos:"
echo "    - /guante/imu           (datos del MPU del guante)"
echo "    - /carro/cmd_vel        (comandos de teleoperación guante)"
echo "    - /carro/set_goal       (objetivo de posición desde la GUI)"
echo "    - /carro/telemetria     (datos del MPU del carro)"
echo "    - /carro/ip_detectada   (IP del carro detectada)"
echo "    - /carro/goal_status    (estado del objetivo)"
echo ""
echo "  INTERFAZ GRÁFICA (goal_gui):"
echo "    - Cambia entre Teleop y Objetivo"
echo "    - Ingresa X (m), Y (m), Theta (rad) y presiona ENVIAR OBJETIVO"
echo "    - Ve la posición calculada desde el MPU del carro (CON filtro FIR)"
echo ""
echo "  GUI APARTE (glove_imu_gui):"
echo "    - Muestra cómo se inclina el MPU del guante (cubo 3D + nivel)"
echo ""
echo "  EKF KALMAN (kalman_filter):"
echo "    - Fusiona encoders (filtrados FIR) + MPU (corrección de heading)"
echo "    - Publica /carro/posicion_kalman y /carro/trayectoria_kalman"
echo ""
echo "  LOGGER DE TRAYECTORIA (trajectory_logger):"
echo "    - Guarda datos KALMAN a: laboratorio_3_toma_de_datos/datos_kalman_*.csv"
echo ""
echo "  Para detener todo: Ctrl+C en esta terminal"
echo "  O ejecuta: pkill -f sensor_bridge.py; pkill -f movement_processor.py;"
echo "             pkill -f carro_commander.py; pkill -f goal_gui.py;"
echo "             pkill -f glove_imu_gui.py; pkill -f kalman_filter.py;"
echo "             pkill -f trajectory_logger.py"
echo "============================================================"
echo ""

wait $PID_BRIDGE $PID_PROC $PID_CMD $PID_GUI $PID_IMU_GUI $PID_KAL $PID_LOGGER