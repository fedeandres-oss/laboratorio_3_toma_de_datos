/* =============================================================================
 * MICRO 2 - CARRO ROBOT (ESP32, ESP-IDF 5.4) - CONTROL CINEMÁTICO Y VELOCIDAD
 * Laboratorio 3 - Procesamiento de Señales
 * ============================================================================= */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <errno.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"
#include "fir_filter.h"

static const char *TAG = "CARRO_MICRO2";

/* ============================================================================
 * CONFIGURACIÓN DE RED Y PUERTOS
 * ============================================================================ */
#define WIFI_SSID      "fedex"
#define WIFI_PASS      "todasmienten"
#define UDP_CMD_PORT   5005

/* ============================================================================
 * PINES - MOTORES L298N
 * ============================================================================ */
#define GPIO_MD_ENA        17   
#define GPIO_MD_IN1        18   
#define GPIO_MD_IN2        19   

#define GPIO_MI_ENA        21   
#define GPIO_MI_IN1        22   
#define GPIO_MI_IN2        23   

/* ============================================================================
 * PINES - ENCODERS MONOFÁSICOS (PCNT) - 20 PPR
 * ============================================================================ */
#define GPIO_ENC_MI        32
#define GPIO_ENC_MD        25
/* El encoder es de 20 PPR, pero el PCNT cuenta flanco de subida Y bajada:
 * 20 pulsos/vuelta x 2 flancos = 40 cuentas por vuelta. */
#define ENC_PPR            40.0f

/* ============================================================================
 * MPU6050 (I2C) - Actualizado a SDA=4, SCL=5
 * ============================================================================ */
#define GPIO_I2C_SDA       4
#define GPIO_I2C_SCL       5
#define I2C_PORT_NUM       I2C_NUM_0

#define MPU6050_ADDR_LOW     0x68
#define MPU6050_ADDR_HIGH    0x69
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

#define I2C_PROBE_TIMEOUT_MS 100
#define I2C_XFER_TIMEOUT_MS  10

/* ============================================================================
 * PWM Y TEMPORIZACIÓN
 * ============================================================================ */
#define PWM_FREQ_HZ        2000
#define CONTROL_DT_MS      10
#define CONTROL_DT         0.010f

#define ALPHA_ENC          0.10f
#define WATCHDOG_MAX       200
#define WATCHDOG_GRACE_MS  5000

/* ============================================================================
 * FILTRO FIR - Encoders de rueda (para control PI por rueda)
 * fs = 100 Hz, fc = 15 Hz (retardo ~70 ms, aceptable para control de velocidad)
 * ============================================================================ */
#define FIR_FS_CARRO    100.0f
#define FIR_FC_CARRO    15.0f
#define FIR_NTAPS_CARRO 15          /* impar -> fase lineal tipo I */

static fir_filter_t g_fir_enc_l;
static fir_filter_t g_fir_enc_r;
static bool g_fir_enc_primed = false;

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

typedef struct {
    float linear_vel;
    float angular_vel;
    float goal_x;
    float goal_y;
    float goal_theta;
    int control_mode;
} command_msg_t;

typedef struct {
    uint32_t seq;
    uint32_t t_ms;
    int32_t raw_left;
    int32_t raw_right;
    float enc_left_raw;
    float enc_right_raw;
    float enc_left_filt;
    float enc_right_filt;
    float pwm_left;
    float pwm_right;
    float accel_raw[3];
    float gyro_raw[3];
} telemetry_msg_t;

typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
} mpu6050_data_t;

static QueueHandle_t xQueueCommand = NULL;
static SemaphoreHandle_t xTelemetryMutex = NULL;
static telemetry_msg_t g_telemetry_data;
static uint32_t g_telemetry_seq = 0;

static int g_udp_socket = -1;
static struct sockaddr_in g_dest_addr;
static bool g_pc_known = false;
static bool g_wifi_connected = false;

static mcpwm_cmpr_handle_t cmpr_mi_in1 = NULL, cmpr_mi_in2 = NULL;
static mcpwm_cmpr_handle_t cmpr_md_in1 = NULL, cmpr_md_in2 = NULL;
static pcnt_unit_handle_t pcnt_unit_mi = NULL, pcnt_unit_md = NULL;

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t mpu6050_dev = NULL;
static bool g_mpu6050_online = false;

static float enc_left_filt_state  = 0.0f;
static float enc_right_filt_state = 0.0f;
static bool g_fir_enc_enabled = true;   /* true = usar FIR en encoders; false = usar crudo */
static int dir_sign_mi = 1;
static int dir_sign_md = 1;

static float g_pos_x = 0.0f;
static float g_pos_y = 0.0f;
static float g_pos_theta = 0.0f;

static float g_wheel_radius = 0.0325f;  /* 32.5 mm */
static float g_track_width  = 0.135f;   

static float g_kp_linear = 2.0f;
static float g_kp_angular = 3.5f;
static float g_pos_tolerance = 0.05f;
static float g_angle_tolerance = 0.05f;

static float g_max_linear_vel = 1.53f;  /* m/s medidos al 100% */
static float g_max_angular_vel = 23.0f; 
static float g_start_percent = 50.0f;

/* Ganancias del lazo cerrado de velocidad (PI por rueda) */
#define KP_WHEEL      1.0f    /* % PWM por rad/s */
#define KI_WHEEL      3.0f    /* % PWM por (rad/s * s) */
#define PI_INT_MAX   35.0f    /* límite del integrador, % */
#define PI_CORR_MAX  35.0f    /* límite de la corrección, % */

static float g_pi_int_l = 0.0f;
static float g_pi_int_r = 0.0f;
static float g_last_target_l = 0.0f;
static float g_last_target_r = 0.0f;

/* Trim por motor para compensar diferencias de fricción/curva.
 * 1.00 = sin ajuste. Si una rueda va más lenta, subir su valor
 * (ej. 1.05 = +5% de PWM). Se puede cambiar en vivo con el comando
 * UDP:  TRIM,<izq>,<der>   por ejemplo:  TRIM,1.08,1.00
 * Rango permitido: 0.50 a 1.50 */
static volatile float g_left_trim  = 1.00f;
static volatile float g_right_trim = 1.00f;

static inline float ema_filter(float *state, float new_value, float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    *state = alpha * new_value + (1.0f - alpha) * (*state);
    return *state;
}

static float apply_start_threshold(float value, float threshold)
{
    float abs_v = fabsf(value);
    if (abs_v > 0.5f && abs_v < threshold) {
        return threshold * (value > 0 ? 1.0f : -1.0f);
    }
    return value;
}

/* PI de velocidad de una rueda.
 * target_vel y measured_vel están en rad/s. measured_vel es positiva en el
 * sentido de avance ordenado (el firmware ya la multiplica por dir_sign).
 * feedforward es el PWM de lazo abierto en %. */
static float wheel_pwm_pi(float target_vel, float measured_vel,
                          float feedforward, float *integral)
{
    float target_abs = fabsf(target_vel);
    if (target_abs < 0.1f) {
        *integral = 0.0f;
        return 0.0f;
    }

    float error = target_abs - measured_vel;
    float p_term = KP_WHEEL * error;
    float corr_now = p_term + *integral;
    float pwm_now = feedforward + (target_vel >= 0.0f ? corr_now : -corr_now);

    /* Anti-windup: no integrar si el PWM ya está saturado y el error
     * sigue pidiendo más velocidad. Si el error es negativo, se permite
     * descargar el integrador. */
    bool saturated = (error > 0.0f) &&
                     (pwm_now >= 100.0f || pwm_now <= -100.0f);
    if (!saturated) {
        *integral += error * CONTROL_DT * KI_WHEEL;
        if (*integral >  PI_INT_MAX) *integral =  PI_INT_MAX;
        if (*integral < -PI_INT_MAX) *integral = -PI_INT_MAX;
    }

    float correction = p_term + *integral;
    if (correction >  PI_CORR_MAX) correction =  PI_CORR_MAX;
    if (correction < -PI_CORR_MAX) correction = -PI_CORR_MAX;

    float pwm = feedforward + (target_vel >= 0.0f ? correction : -correction);
    if (pwm > 100.0f)  pwm = 100.0f;
    if (pwm < -100.0f) pwm = -100.0f;
    return pwm;
}

static void init_motors_l298n(void)
{
    gpio_config_t enable_conf = {
        .pin_bit_mask = (1ULL << GPIO_MI_ENA) | (1ULL << GPIO_MD_ENA),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&enable_conf);
    gpio_set_level(GPIO_MI_ENA, 1);
    gpio_set_level(GPIO_MD_ENA, 1);

    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_conf = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,
        .period_ticks = 10000000 / PWM_FREQ_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_conf, &timer));

    mcpwm_oper_handle_t oper_mi = NULL, oper_md = NULL;
    mcpwm_operator_config_t oper_conf = {.group_id = 0};
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_conf, &oper_mi));
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_conf, &oper_md));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_mi, timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_md, timer));

    mcpwm_comparator_config_t cmpr_conf = {.flags.update_cmp_on_tez = true};
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_mi, &cmpr_conf, &cmpr_mi_in1));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_mi, &cmpr_conf, &cmpr_mi_in2));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_md, &cmpr_conf, &cmpr_md_in1));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_md, &cmpr_conf, &cmpr_md_in2));

    mcpwm_gen_handle_t gen_mi_in1 = NULL, gen_mi_in2 = NULL;
    mcpwm_gen_handle_t gen_md_in1 = NULL, gen_md_in2 = NULL;

    mcpwm_generator_config_t gen_mi1_cfg = {.gen_gpio_num = GPIO_MI_IN1};
    mcpwm_generator_config_t gen_mi2_cfg = {.gen_gpio_num = GPIO_MI_IN2};
    mcpwm_generator_config_t gen_md1_cfg = {.gen_gpio_num = GPIO_MD_IN1};
    mcpwm_generator_config_t gen_md2_cfg = {.gen_gpio_num = GPIO_MD_IN2};

    ESP_ERROR_CHECK(mcpwm_new_generator(oper_mi, &gen_mi1_cfg, &gen_mi_in1));
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_mi, &gen_mi2_cfg, &gen_mi_in2));
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_md, &gen_md1_cfg, &gen_md_in1));
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_md, &gen_md2_cfg, &gen_md_in2));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_mi_in1,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_mi_in1,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_mi_in1, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_mi_in2,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_mi_in2,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_mi_in2, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_md_in1,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_md_in1,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_md_in1, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_md_in2,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_md_in2,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_md_in2, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

static void set_motor_speed_2pins(mcpwm_cmpr_handle_t cmpr_in1,
                                  mcpwm_cmpr_handle_t cmpr_in2,
                                  float speed_percent, int *dir_sign)
{
    if (speed_percent > 100.0f) speed_percent = 100.0f;
    if (speed_percent < -100.0f) speed_percent = -100.0f;

    uint32_t max_duty = 10000000 / PWM_FREQ_HZ;
    uint32_t duty_ticks = (uint32_t)(max_duty * (fabsf(speed_percent) / 100.0f));

    if (speed_percent > 0.5f) {
        mcpwm_comparator_set_compare_value(cmpr_in1, duty_ticks);
        mcpwm_comparator_set_compare_value(cmpr_in2, 0);
        *dir_sign = 1;
    } else if (speed_percent < -0.5f) {
        mcpwm_comparator_set_compare_value(cmpr_in1, 0);
        mcpwm_comparator_set_compare_value(cmpr_in2, duty_ticks);
        *dir_sign = -1;
    } else {
        mcpwm_comparator_set_compare_value(cmpr_in1, 0);
        mcpwm_comparator_set_compare_value(cmpr_in2, 0);
    }
}

static pcnt_unit_handle_t init_single_phase_encoder(int gpio_num)
{
    pcnt_unit_config_t unit_config = { .high_limit = 32767, .low_limit = -32768 };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {.max_glitch_ns = 2000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_config = { .edge_gpio_num = gpio_num, .level_gpio_num = -1 };
    pcnt_channel_handle_t chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &chan));

    /* Muchos encoders son de salida open-collector y necesitan pull-up.
     * Si el módulo ya trae pull-up externo, esto no molesta. */
    gpio_set_pull_mode(gpio_num, GPIO_PULLUP_ONLY);

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    return pcnt_unit;
}

static void init_mpu6050(void)
{
    g_mpu6050_online = false;
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT_NUM,
        .sda_io_num = GPIO_I2C_SDA,
        .scl_io_num = GPIO_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_config, &i2c_bus) != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo iniciar el bus I2C (MPU6050 omitido)");
        return;
    }

    uint8_t found_addr = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(i2c_bus, addr, I2C_PROBE_TIMEOUT_MS) == ESP_OK) {
            if (addr == MPU6050_ADDR_LOW || addr == MPU6050_ADDR_HIGH) {
                found_addr = addr;
                break;
            }
        }
    }

    if (found_addr == 0) {
        ESP_LOGW(TAG, "MPU6050 no detectado en I2C");
        return;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = found_addr,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(i2c_bus, &dev_config, &mpu6050_dev) != ESP_OK) {
        return;
    }

    uint8_t wake_cmd[2] = {MPU6050_PWR_MGMT_1, 0x00};
    if (i2c_master_transmit(mpu6050_dev, wake_cmd, sizeof(wake_cmd), I2C_PROBE_TIMEOUT_MS) == ESP_OK) {
        g_mpu6050_online = true;
        uint8_t dlpf_cmd[2] = {0x1A, 0x03};
        i2c_master_transmit(mpu6050_dev, dlpf_cmd, sizeof(dlpf_cmd), I2C_PROBE_TIMEOUT_MS);
        ESP_LOGI(TAG, "MPU6050 inicializado correctamente en 0x%02X (SDA=%d, SCL=%d)", found_addr, GPIO_I2C_SDA, GPIO_I2C_SCL);
    }
}

static esp_err_t read_mpu6050(mpu6050_data_t *imu_data)
{
    if (!g_mpu6050_online || mpu6050_dev == NULL) {
        memset(imu_data, 0, sizeof(*imu_data));
        return ESP_FAIL;
    }

    uint8_t reg_addr = MPU6050_ACCEL_XOUT_H;
    uint8_t raw_buffer[14];

    esp_err_t ret = i2c_master_transmit_receive(mpu6050_dev, &reg_addr, 1, raw_buffer, sizeof(raw_buffer), I2C_XFER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        memset(imu_data, 0, sizeof(*imu_data));
        return ret;
    }

    imu_data->accel_x = (float)((int16_t)((raw_buffer[0] << 8) | raw_buffer[1])) / 16384.0f;
    imu_data->accel_y = (float)((int16_t)((raw_buffer[2] << 8) | raw_buffer[3])) / 16384.0f;
    imu_data->accel_z = (float)((int16_t)((raw_buffer[4] << 8) | raw_buffer[5])) / 16384.0f;
    imu_data->gyro_x  = (float)((int16_t)((raw_buffer[8] << 8) | raw_buffer[9])) / 131.0f;
    imu_data->gyro_y  = (float)((int16_t)((raw_buffer[10] << 8) | raw_buffer[11])) / 131.0f;
    imu_data->gyro_z  = (float)((int16_t)((raw_buffer[12] << 8) | raw_buffer[13])) / 131.0f;

    return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        g_wifi_connected = false;
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        g_wifi_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void udp_socket_init(void)
{
    struct sockaddr_in local_addr = {0};
    g_udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (g_udp_socket < 0) return;

    int reuse = 1;
    setsockopt(g_udp_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(UDP_CMD_PORT);

    if (bind(g_udp_socket, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        close(g_udp_socket);
        g_udp_socket = -1;
        return;
    }

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;
    setsockopt(g_udp_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
}

void vTaskCommunication_Core0(void *pvParameters)
{
    char rx_buffer[128];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    struct sockaddr_in broadcast_addr;
    memset(&broadcast_addr, 0, sizeof(broadcast_addr));
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(UDP_CMD_PORT);
    broadcast_addr.sin_addr.s_addr = inet_addr("255.255.255.255");

    while (1) {
        if (!g_wifi_connected) {
            if (g_udp_socket >= 0) {
                close(g_udp_socket);
                g_udp_socket = -1;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (g_udp_socket < 0) {
            udp_socket_init();
            if (g_udp_socket < 0) {
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
        }

        int len = recvfrom(g_udp_socket, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&source_addr, &socklen);

        if (len > 0) {
            rx_buffer[len] = 0;
            command_msg_t cmd = {0};

            g_dest_addr = source_addr;
            g_pc_known = true;

            if (strncmp(rx_buffer, "POS,", 4) == 0) {
                float gx = 0.0f, gy = 0.0f, gt = 0.0f;
                if (sscanf(rx_buffer + 4, "%f,%f,%f", &gx, &gy, &gt) == 3) {
                    cmd.control_mode = 1;
                    cmd.goal_x = gx;
                    cmd.goal_y = gy;
                    cmd.goal_theta = gt;
                    if (xQueueCommand) xQueueOverwrite(xQueueCommand, &cmd);
                }
            } else if (strncmp(rx_buffer, "TRIM,", 5) == 0) {
                float lt = 0.0f, rt = 0.0f;
                if (sscanf(rx_buffer + 5, "%f,%f", &lt, &rt) == 2) {
                    if (lt >= 0.50f && lt <= 1.50f) g_left_trim = lt;
                    if (rt >= 0.50f && rt <= 1.50f) g_right_trim = rt;
                    ESP_LOGI(TAG, "TRIM actualizado: izq=%.3f der=%.3f",
                             g_left_trim, g_right_trim);
                }
            } else if (strncmp(rx_buffer, "FIR,", 4) == 0) {
                int fir_val = 0;
                if (sscanf(rx_buffer + 4, "%d", &fir_val) == 1) {
                    g_fir_enc_enabled = (fir_val != 0);
                    ESP_LOGI(TAG, "FIR encoders %s", g_fir_enc_enabled ? "ACTIVADO" : "DESACTIVADO");
                }
            } else {
                float v = 0.0f, w = 0.0f;
                if (sscanf(rx_buffer, "%f,%f", &v, &w) == 2) {
                    cmd.control_mode = 0;
                    cmd.linear_vel = v;
                    cmd.angular_vel = w;
                    if (xQueueCommand) xQueueOverwrite(xQueueCommand, &cmd);
                }
            }
        }

        telemetry_msg_t tx = {0};
        if (xTelemetryMutex && xSemaphoreTake(xTelemetryMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            memcpy(&tx, &g_telemetry_data, sizeof(telemetry_msg_t));
            xSemaphoreGive(xTelemetryMutex);
        }

        char msg[384];
        int n = snprintf(msg, sizeof(msg),
            "ROBOT|seq=%lu|t_ms=%lu|"
            "raw_l=%ld|raw_r=%ld|"
            "enc_l=%.3f|enc_r=%.3f|"
            "enc_l_filt=%.3f|enc_r_filt=%.3f|"
            "trim_l=%.3f|trim_r=%.3f|"
            "pwm_l=%.1f|pwm_r=%.1f|"
            "ax=%.3f|ay=%.3f|az=%.3f|"
            "gx=%.3f|gy=%.3f|gz=%.3f\n",
            (unsigned long)tx.seq, (unsigned long)tx.t_ms,
            (long)tx.raw_left, (long)tx.raw_right,
            tx.enc_left_raw, tx.enc_right_raw,
            tx.enc_left_filt, tx.enc_right_filt,
            g_left_trim, g_right_trim,
            tx.pwm_left, tx.pwm_right,
            tx.accel_raw[0], tx.accel_raw[1], tx.accel_raw[2],
            tx.gyro_raw[0], tx.gyro_raw[1], tx.gyro_raw[2]);

        struct sockaddr_in target = g_pc_known ? g_dest_addr : broadcast_addr;
        if (g_udp_socket >= 0 && g_wifi_connected) {
            sendto(g_udp_socket, msg, n, 0, (struct sockaddr *)&target, sizeof(target));
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void vTaskControlAndSensors_Core1(void *pvParameters)
{
    init_motors_l298n();
    pcnt_unit_mi = init_single_phase_encoder(GPIO_ENC_MI);
    pcnt_unit_md = init_single_phase_encoder(GPIO_ENC_MD);
    init_mpu6050();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(CONTROL_DT_MS);

    command_msg_t current_cmd = {0};
    int raw_count_l = 0, raw_count_r = 0;
    mpu6050_data_t imu_data = {0};
    uint32_t watchdog_counter = 0;
    uint32_t t_start = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t grace_start = xTaskGetTickCount() * portTICK_PERIOD_MS;

    /* Inicializar filtros FIR pasa-bajos una sola vez */
    if (!g_fir_enc_primed) {
        const fir_window_t win = FIR_WIN_HAMMING;
        fir_init_lowpass(&g_fir_enc_l, FIR_NTAPS_CARRO, FIR_FC_CARRO, FIR_FS_CARRO, win);
        fir_init_lowpass(&g_fir_enc_r, FIR_NTAPS_CARRO, FIR_FC_CARRO, FIR_FS_CARRO, win);
        g_fir_enc_primed = true;
        ESP_LOGI(TAG, "FIR carro iniciado: ntaps=%d fc=%.1f Hz fs=%.1f Hz retardo=%.0f ms",
                 FIR_NTAPS_CARRO, FIR_FC_CARRO, FIR_FS_CARRO,
                 fir_group_delay_ms(&g_fir_enc_l, FIR_FS_CARRO));
    }

    ESP_LOGI(TAG, "Tarea de control iniciada en Core 1");

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, frequency);

        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        bool in_grace_period = (now_ms - grace_start) < WATCHDOG_GRACE_MS;

        if (xQueueCommand && xQueueReceive(xQueueCommand, &current_cmd, 0) == pdTRUE) {
            watchdog_counter = 0;
        } else {
            watchdog_counter++;
            if (!in_grace_period && watchdog_counter > WATCHDOG_MAX) {
                current_cmd.linear_vel = 0.0f;
                current_cmd.angular_vel = 0.0f;
                current_cmd.control_mode = 0;
            }
        }

        pcnt_unit_get_count(pcnt_unit_mi, &raw_count_l);
        pcnt_unit_get_count(pcnt_unit_md, &raw_count_r);
        pcnt_unit_clear_count(pcnt_unit_mi);
        pcnt_unit_clear_count(pcnt_unit_md);

        float rad_s_left  = (((float)raw_count_l / ENC_PPR) * 2.0f * M_PI / CONTROL_DT) * (float)dir_sign_mi;
        float rad_s_right = (((float)raw_count_r / ENC_PPR) * 2.0f * M_PI / CONTROL_DT) * (float)dir_sign_md;

        /* FIR pasa-bajos para medición de velocidad de rueda (se puede desactivar con comando FIR,0) */
        float rad_s_left_filt  = g_fir_enc_enabled ? fir_update(&g_fir_enc_l, rad_s_left)  : rad_s_left;
        float rad_s_right_filt = g_fir_enc_enabled ? fir_update(&g_fir_enc_r, rad_s_right) : rad_s_right;

        float dist_l = rad_s_left_filt * g_wheel_radius * CONTROL_DT;
        float dist_r = rad_s_right_filt * g_wheel_radius * CONTROL_DT;

        float dc = (dist_l + dist_r) / 2.0f;
        float dtheta = (dist_r - dist_l) / g_track_width;

        g_pos_x += dc * cosf(g_pos_theta + dtheta / 2.0f);
        g_pos_y += dc * sinf(g_pos_theta + dtheta / 2.0f);
        g_pos_theta += dtheta;
        g_pos_theta = atan2f(sinf(g_pos_theta), cosf(g_pos_theta));

        float target_vel_l = 0.0f;
        float target_vel_r = 0.0f;
        float turn_factor = g_track_width / (2.0f * g_wheel_radius);

        if (current_cmd.control_mode == 0) {
            float v = current_cmd.linear_vel;   /* m/s */
            float w = current_cmd.angular_vel;  /* rad/s */

            /* Limitar a los máximos físicos medidos */
            if (v >  g_max_linear_vel)  v =  g_max_linear_vel;
            if (v < -g_max_linear_vel)  v = -g_max_linear_vel;
            if (w >  g_max_angular_vel) w =  g_max_angular_vel;
            if (w < -g_max_angular_vel) w = -g_max_angular_vel;

            /* v (m/s) -> velocidad angular de rueda (rad/s).
             * w positivo = giro antihorario (izquierda):
             *   rueda derecha más rápida, izquierda más lenta. */
            float v_wheel = v / g_wheel_radius;
            target_vel_l = v_wheel - w * turn_factor;
            target_vel_r = v_wheel + w * turn_factor;
        } else if (current_cmd.control_mode == 1) {
            float dx = current_cmd.goal_x - g_pos_x;
            float dy = current_cmd.goal_y - g_pos_y;
            float distance_error = sqrtf(dx*dx + dy*dy);

            float target_angle = atan2f(dy, dx);
            float angle_error = target_angle - g_pos_theta;
            angle_error = atan2f(sinf(angle_error), cosf(angle_error));

            if (distance_error > g_pos_tolerance) {
                float v_linear = g_kp_linear * distance_error * cosf(angle_error);  /* m/s */
                float v_angular = g_kp_angular * angle_error;                       /* rad/s */

                if (v_linear >  g_max_linear_vel)  v_linear =  g_max_linear_vel;
                if (v_linear < -g_max_linear_vel)  v_linear = -g_max_linear_vel;
                if (v_angular >  g_max_angular_vel) v_angular =  g_max_angular_vel;
                if (v_angular < -g_max_angular_vel) v_angular = -g_max_angular_vel;

                float v_wheel = v_linear / g_wheel_radius;
                target_vel_l = v_wheel - v_angular * turn_factor;
                target_vel_r = v_wheel + v_angular * turn_factor;
            }
        }

        float max_wheel_rad_s = g_max_linear_vel / g_wheel_radius; 
        float ff_left  = (target_vel_l / max_wheel_rad_s) * 100.0f;
        float ff_right = (target_vel_r / max_wheel_rad_s) * 100.0f;

        if (fabsf(ff_left) > 0.5f)  ff_left  = apply_start_threshold(ff_left,  g_start_percent);
        if (fabsf(ff_right) > 0.5f) ff_right = apply_start_threshold(ff_right, g_start_percent);

        /* El trim queda como ajuste grueso de feedforward; el PI corrige
         * la diferencia real de velocidad medida por los encoders. */
        ff_left  *= g_left_trim;
        ff_right *= g_right_trim;

        /* Resetear el integrador al cambiar de sentido */
        if (target_vel_l * g_last_target_l < 0.0f) g_pi_int_l = 0.0f;
        if (target_vel_r * g_last_target_r < 0.0f) g_pi_int_r = 0.0f;
        g_last_target_l = target_vel_l;
        g_last_target_r = target_vel_r;

        float pwm_left  = wheel_pwm_pi(target_vel_l, rad_s_left_filt,  ff_left,  &g_pi_int_l);
        float pwm_right = wheel_pwm_pi(target_vel_r, rad_s_right_filt, ff_right, &g_pi_int_r);

        set_motor_speed_2pins(cmpr_mi_in1, cmpr_mi_in2, pwm_left,  &dir_sign_mi);
        set_motor_speed_2pins(cmpr_md_in1, cmpr_md_in2, pwm_right, &dir_sign_md);

        if (g_mpu6050_online) {
            read_mpu6050(&imu_data);
        }

        if (xTelemetryMutex && xSemaphoreTake(xTelemetryMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            g_telemetry_data.seq = g_telemetry_seq++;
            g_telemetry_data.t_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) - t_start;
            g_telemetry_data.raw_left  = raw_count_l;
            g_telemetry_data.raw_right = raw_count_r;
            g_telemetry_data.enc_left_raw  = rad_s_left;
            g_telemetry_data.enc_right_raw = rad_s_right;
            g_telemetry_data.enc_left_filt  = rad_s_left_filt;
            g_telemetry_data.enc_right_filt = rad_s_right_filt;
            g_telemetry_data.pwm_left  = pwm_left;
            g_telemetry_data.pwm_right = pwm_right;
            g_telemetry_data.accel_raw[0] = imu_data.accel_x;
            g_telemetry_data.accel_raw[1] = imu_data.accel_y;
            g_telemetry_data.accel_raw[2] = imu_data.accel_z;
            g_telemetry_data.gyro_raw[0]  = imu_data.gyro_x;
            g_telemetry_data.gyro_raw[1]  = imu_data.gyro_y;
            g_telemetry_data.gyro_raw[2]  = imu_data.gyro_z;
            xSemaphoreGive(xTelemetryMutex);
        }
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    xTelemetryMutex = xSemaphoreCreateMutex();
    // CORRECCIÓN CRÍTICA: La cola se crea de tamaño 1 para permitir xQueueOverwrite
    xQueueCommand = xQueueCreate(1, sizeof(command_msg_t));

    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Sin ahorro de energía: evita perder comandos UDP por latencia del modem */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "WiFi power save: OFF");

    ESP_LOGI(TAG, "Iniciando sistema...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    xTaskCreatePinnedToCore(vTaskCommunication_Core0, "comm_task", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(vTaskControlAndSensors_Core1, "control_task", 4096, NULL, 5, NULL, 1);
}