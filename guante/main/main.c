/* =============================================================================
 * MICRO 1 - GUANTE (ESP32, ESP-IDF 5.4)
 * Laboratorio 3 - Procesamiento de Señales
 * =============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <errno.h>

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
#include "esp_timer.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "fir_filter.h"

static const char *TAG = "GUANTE";

/* ============================================================================
 * CONFIGURACIÓN DE RED Y PUERTOS
 * ============================================================================ */
#define WIFI_SSID      "fedex"
#define WIFI_PASS      "todasmienten"

#define DISCOVER_PORT      5000
#define LOCAL_PORT         5001
/* El PC se descubre por broadcast (DISCOVER_PC); no se usa IP fija. */

/* ============================================================================
 * MPU6050 (I2C)
 * ============================================================================ */
#define I2C_PORT_NUM       I2C_NUM_0

#define MPU6050_ADDR_LOW     0x68
#define MPU6050_ADDR_HIGH    0x69
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

#define I2C_PROBE_TIMEOUT_MS 50
#define I2C_XFER_TIMEOUT_MS  10

/* Pares SDA/SCL candidatos. Se prueban en orden hasta encontrar el MPU6050.
 * Incluye el orden invertido por si el cableado quedo al revés. */
typedef struct {
    int sda;
    int scl;
} i2c_pin_pair_t;

static const i2c_pin_pair_t I2C_CANDIDATOS[] = {
    {21, 19},             /* SDA=21, SCL=19 (cableado actual) */
    {19, 21},             /* orden invertido, por si acaso */
    {15, 16}, {16, 15},   /* proyecto muestreo_mpu */
    {4, 5},   {5, 4},     /* pines del carro */
    {22, 21}, {21, 22},   /* pinout clasico del MPU6050 */
    {32, 33}, {33, 32},
    {25, 26}, {26, 25},
};

/* ============================================================================
 * TASAS DE MUESTREO
 * ============================================================================ */
#define MPU_READ_INTERVAL_MS   10   /* Lectura del sensor: 100 Hz */
#define UDP_SEND_INTERVAL_MS   10   /* Envío por UDP: 100 Hz */

/* ============================================================================
 * FILTRO FIR - MPU6050 del guante
 * fs = 100 Hz, fc = 15 Hz (quita ruido de alta frecuencia, retardo ~70 ms)
 * ============================================================================ */
#define FIR_FS_GUANTE   100.0f
#define FIR_FC_GUANTE   15.0f
#define FIR_NTAPS       15          /* impar -> fase lineal tipo I */
#define FIR_N_EJES      6           /* ax, ay, az, gx, gy, gz */

static fir_filter_t g_fir_mpu[FIR_N_EJES];
static bool g_fir_primed = false;

/* ============================================================================
 * ESTRUCTURAS DE DATOS
 * ============================================================================ */
typedef struct {
    uint32_t seq;
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;
    float temp_c;
} mpu_packet_t;

/* ============================================================================
 * VARIABLES GLOBALES
 * ============================================================================ */
static QueueHandle_t xQueueMpu = NULL;
static int sock = -1;
static uint32_t g_seq = 0;

static volatile bool g_wifi_connected = false;
static bool g_mpu6050_online = false;

/* Dirección del PC descubierta por broadcast (sin IP fija) */
static struct sockaddr_in g_pc_addr;
static bool g_pc_known = false;
static int64_t g_last_pc_rx_us = 0;
static int64_t g_last_discover_us = 0;

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t mpu6050_dev = NULL;

/* ============================================================================
 * DECLARACIONES DE FUNCIONES (forward declarations)
 * ============================================================================ */
static void init_mpu6050(void);
static void init_wifi(void);
static void init_socket(void);
static void vTaskCommunication_Core0(void *pvParameters);
static void vTaskMpuReader(void *pvParameters);
static esp_err_t mpu6050_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                                  int16_t *gx, int16_t *gy, int16_t *gz,
                                  int16_t *temp);

/* ============================================================================
 * FUNCIONES AUXILIARES
 * ============================================================================ */
static esp_err_t mpu6050_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                                  int16_t *gx, int16_t *gy, int16_t *gz,
                                  int16_t *temp)
{
    if (mpu6050_dev == NULL) return ESP_ERR_INVALID_STATE;

    uint8_t reg_addr = MPU6050_ACCEL_XOUT_H;
    uint8_t raw[14];

    esp_err_t ret = i2c_master_transmit_receive(mpu6050_dev, &reg_addr, 1,
                                                raw, sizeof(raw),
                                                I2C_XFER_TIMEOUT_MS);
    if (ret != ESP_OK) return ret;

    *ax   = (int16_t)((raw[0]  << 8) | raw[1]);
    *ay   = (int16_t)((raw[2]  << 8) | raw[3]);
    *az   = (int16_t)((raw[4]  << 8) | raw[5]);
    *temp = (int16_t)((raw[6]  << 8) | raw[7]);
    *gx   = (int16_t)((raw[8]  << 8) | raw[9]);
    *gy   = (int16_t)((raw[10] << 8) | raw[11]);
    *gz   = (int16_t)((raw[12] << 8) | raw[13]);

    return ESP_OK;
}

/* Diagnóstico eléctrico:
 * - Con pull-up interno: ambas líneas deben leer 1 en reposo. Si leen 0, algo
 *   las está sujetando a GND.
 * - Con pull-down interno: si el módulo está conectado y alimentado, sus
 *   pull-ups externos (4.7k) ganan y las líneas siguen leyendo 1. Si leen 0,
 *   los cables SDA/SCL no llegan al módulo (o el módulo no tiene VCC). */
static void i2c_diagnostico_niveles(int sda, int scl)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << sda) | (1ULL << scl),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    esp_rom_delay_us(200);
    int pu_sda = gpio_get_level(sda);
    int pu_scl = gpio_get_level(scl);

    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io);
    esp_rom_delay_us(200);
    int pd_sda = gpio_get_level(sda);
    int pd_scl = gpio_get_level(scl);

    ESP_LOGI(TAG, "SDA(%d): pullup=%d pulldown=%d | SCL(%d): pullup=%d pulldown=%d",
             sda, pu_sda, pd_sda, scl, pu_scl, pd_scl);

    if (pu_sda == 0 || pu_scl == 0) {
        ESP_LOGE(TAG, "Linea I2C atada a GND: revisa que SDA/SCL no toquen GND");
    } else if (pd_sda == 1 && pd_scl == 1) {
        ESP_LOGI(TAG, "Pull-ups externos detectados: el modulo esta conectado y alimentado");
    } else {
        ESP_LOGE(TAG, "Sin pull-ups externos: los cables SDA/SCL no llegan al modulo o falta VCC/GND");
    }

    /* Prueba de corto entre SDA y SCL: se baja una linea y se lee la otra */
    gpio_config_t io2 = {
        .pin_bit_mask = (1ULL << sda) | (1ULL << scl),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io2);
    gpio_set_level(sda, 1);
    gpio_set_level(scl, 1);
    esp_rom_delay_us(200);

    gpio_set_level(sda, 0);
    esp_rom_delay_us(200);
    int scl_con_sda_baja = gpio_get_level(scl);
    gpio_set_level(sda, 1);
    esp_rom_delay_us(200);

    gpio_set_level(scl, 0);
    esp_rom_delay_us(200);
    int sda_con_scl_baja = gpio_get_level(sda);
    gpio_set_level(scl, 1);

    if (scl_con_sda_baja == 0 || sda_con_scl_baja == 0) {
        ESP_LOGE(TAG, "SDA y SCL parecen CORTOCIRCUITADAS entre si");
    } else {
        ESP_LOGI(TAG, "Sin corto entre SDA y SCL");
    }

    gpio_reset_pin(sda);
    gpio_reset_pin(scl);
}

/* Prueba un par SDA/SCL. Devuelve true si encuentra el MPU6050 y deja el bus
 * creado en i2c_bus. Si no lo encuentra, destruye el bus y devuelve false. */
static bool i2c_probar_par(const i2c_pin_pair_t *pins, uint8_t *found_addr,
                           bool escaneo_completo)
{
    /* Liberar el bus I2C si algún esclavo quedó sujetando SDA */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pins->sda) | (1ULL << pins->scl),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(pins->sda, 1);
    gpio_set_level(pins->scl, 1);
    esp_rom_delay_us(10);

    for (int i = 0; i < 9; i++) {
        if (gpio_get_level(pins->sda) == 1) break;
        gpio_set_level(pins->scl, 0);
        esp_rom_delay_us(10);
        gpio_set_level(pins->scl, 1);
        esp_rom_delay_us(10);
    }
    gpio_reset_pin(pins->sda);
    gpio_reset_pin(pins->scl);

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT_NUM,
        .sda_io_num = pins->sda,
        .scl_io_num = pins->scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_config, &i2c_bus) != ESP_OK) {
        i2c_bus = NULL;
        return false;
    }

    /* Dar tiempo al MPU a completar su arranque */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* No usar i2c_master_probe: en algunos ESP32 no detecta aunque el sensor
     * responda. Se agrega el dispositivo y se lee WHO_AM_I (0x75), que es una
     * transaccion I2C real. */
    uint8_t addr = 0;
    uint8_t who = 0;
    const uint8_t candidatos[2] = {MPU6050_ADDR_LOW, MPU6050_ADDR_HIGH};

    for (int k = 0; k < 2; k++) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = candidatos[k],
            .scl_speed_hz = 100000,
        };
        i2c_master_dev_handle_t tmp_dev = NULL;
        if (i2c_master_bus_add_device(i2c_bus, &dev_cfg, &tmp_dev) != ESP_OK) {
            continue;
        }

        uint8_t reg = 0x75;  /* WHO_AM_I */
        esp_err_t r = ESP_FAIL;

        for (int intento = 1; intento <= 3; intento++) {
            r = i2c_master_transmit_receive(tmp_dev, &reg, 1, &who, 1,
                                            I2C_PROBE_TIMEOUT_MS);
            if (r == ESP_OK) {
                break;
            }
            ESP_LOGW(TAG, "  WHO_AM_I 0x%02X intento %d fallo: %s",
                     candidatos[k], intento, esp_err_to_name(r));
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        if (r != ESP_OK) {
            /* Fallback: escribir registro con STOP y luego leer con STOP */
            if (i2c_master_transmit(tmp_dev, &reg, 1, I2C_PROBE_TIMEOUT_MS) == ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(5));
                r = i2c_master_receive(tmp_dev, &who, 1, I2C_PROBE_TIMEOUT_MS);
                if (r == ESP_OK) {
                    ESP_LOGI(TAG, "  WHO_AM_I leido con write+read separado");
                }
            }
        }

        i2c_master_bus_rm_device(tmp_dev);

        if (r == ESP_OK) {
            ESP_LOGI(TAG, "  ACK real en 0x%02X, WHO_AM_I=0x%02X",
                     candidatos[k], who);
            addr = candidatos[k];
            break;
        }
    }

    if (addr == 0) {
        if (escaneo_completo) {
            ESP_LOGW(TAG, "Sin respuesta en 0x68/0x69. Prueba de escaneo completa...");
            for (uint8_t a = 0x08; a < 0x78; a++) {
                if (i2c_master_probe(i2c_bus, a, I2C_PROBE_TIMEOUT_MS) == ESP_OK) {
                    ESP_LOGI(TAG, "  dispositivo I2C en 0x%02X", a);
                }
            }
        }
        i2c_del_master_bus(i2c_bus);
        i2c_bus = NULL;
        return false;
    }

    *found_addr = addr;
    return true;
}

static void init_mpu6050(void)
{
    g_mpu6050_online = false;
    uint8_t found_addr = 0;

    /* Diagnóstico eléctrico del par cableado: SDA=21, SCL=19 */
    i2c_diagnostico_niveles(21, 19);

    for (size_t i = 0; i < sizeof(I2C_CANDIDATOS) / sizeof(I2C_CANDIDATOS[0]); i++) {
        const i2c_pin_pair_t *pins = &I2C_CANDIDATOS[i];
        ESP_LOGI(TAG, "Probando I2C SDA=%d SCL=%d...", pins->sda, pins->scl);

        if (!i2c_probar_par(pins, &found_addr, i == 0)) {
            continue;
        }

        ESP_LOGI(TAG, "MPU6050 detectado en 0x%02X (SDA=%d, SCL=%d)",
                 found_addr, pins->sda, pins->scl);

        i2c_device_config_t dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = found_addr,
            .scl_speed_hz = 100000,
        };
        if (i2c_master_bus_add_device(i2c_bus, &dev_config, &mpu6050_dev) != ESP_OK) {
            ESP_LOGE(TAG, "Error agregando MPU6050");
            i2c_del_master_bus(i2c_bus);
            i2c_bus = NULL;
            continue;
        }

        /* Sacar el MPU6050 del modo sleep (con reintentos: la primera
         * transaccion tras agregar el dispositivo a veces falla) */
        uint8_t wake_cmd[2] = {MPU6050_PWR_MGMT_1, 0x00};
        bool despertado = false;
        for (int intento = 1; intento <= 3; intento++) {
            if (i2c_master_transmit(mpu6050_dev, wake_cmd, sizeof(wake_cmd),
                                    I2C_PROBE_TIMEOUT_MS) == ESP_OK) {
                despertado = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        if (despertado) {
            g_mpu6050_online = true;
            ESP_LOGI(TAG, "MPU6050 listo en 0x%02X", found_addr);
        } else {
            ESP_LOGE(TAG, "MPU6050 detectado pero no responde al despertar");
        }
        return;
    }

    ESP_LOGE(TAG, "MPU6050 no detectado en ningun par de pines. Revisa VCC/GND/SDA/SCL.");
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi desconectado, reconectando...");
        g_wifi_connected = false;
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        char ip_str[16];
        esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "Conectado. IP: %s", ip_str);
        g_wifi_connected = true;
    }
}

static void init_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
        ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
        IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Esperando conexión Wi-Fi...");
}

static void init_socket(void)
{
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "No se pudo crear el socket (errno %d)", errno);
        return;
    }

    int bc = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &bc, sizeof(bc));

    /* recvfrom no bloqueante infinito: timeout de 200 ms */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in local;
    memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(LOCAL_PORT);
    if (bind(sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
        ESP_LOGE(TAG, "Error en bind (errno %d)", errno);
        close(sock);
        sock = -1;
        return;
    }

    ESP_LOGI(TAG, "Socket UDP listo en puerto %d", LOCAL_PORT);
}

/* ============================================================================
 * TAREAS
 * ============================================================================ */
static void send_discover_pc(void)
{
    if (sock < 0) return;

    struct sockaddr_in bcast;
    memset(&bcast, 0, sizeof(bcast));
    bcast.sin_family = AF_INET;
    bcast.sin_port = htons(DISCOVER_PORT);
    bcast.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    const char *msg = "DISCOVER_PC\n";
    sendto(sock, msg, strlen(msg), 0,
           (struct sockaddr *)&bcast, sizeof(bcast));
}

static void vTaskCommunication_Core0(void *pvParameters)
{
    memset(&g_pc_addr, 0, sizeof(g_pc_addr));
    g_pc_addr.sin_family = AF_INET;
    g_pc_addr.sin_port = htons(DISCOVER_PORT);

    ESP_LOGI(TAG, "Tarea de comunicación iniciada. Buscando PC por broadcast...");

    uint32_t enviados_ok = 0;
    uint32_t siguiente_log = 100;

    while (1) {
        /* Esperar Wi-Fi y crear el socket si hace falta */
        if (!g_wifi_connected) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (sock < 0) {
            init_socket();
            if (sock < 0) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }

        int64_t now_us = esp_timer_get_time();

        /* Si no conocemos el PC, o dejamos de recibir ACK, redescubrir.
         * Se envía DISCOVER_PC al broadcast cada 1 s. */
        if (!g_pc_known || (now_us - g_last_pc_rx_us) > 5000000LL) {
            if ((now_us - g_last_discover_us) > 1000000LL) {
                send_discover_pc();
                g_last_discover_us = now_us;
            }
        }

        /* Drenar ACK / respuestas sin bloquear. */
        char rx[64];
        struct sockaddr_in src;
        socklen_t slen = sizeof(src);
        int n;
        while ((n = recvfrom(sock, rx, sizeof(rx) - 1, MSG_DONTWAIT,
                             (struct sockaddr *)&src, &slen)) > 0) {
            rx[n] = '\0';

            if (strncmp(rx, "PC_IP:", 6) == 0 ||
                strncmp(rx, "ACK", 3) == 0) {
                /* La IP de origen de la respuesta es la IP real del PC. */
                g_pc_addr.sin_addr.s_addr = src.sin_addr.s_addr;
                g_last_pc_rx_us = now_us;
                if (!g_pc_known) {
                    g_pc_known = true;
                    ESP_LOGI(TAG, "PC descubierto: %s", rx);
                }
            }
        }

        /* Enviar todos los paquetes MPU pendientes directamente al PC */
        mpu_packet_t pkt;

        if (xQueueMpu != NULL) {
            while (xQueueReceive(xQueueMpu, &pkt, 0) == pdTRUE) {
                if (!g_pc_known) {
                    continue;  /* todavía no sabemos a dónde enviar */
                }

                char msg[256];
                int len = snprintf(msg, sizeof(msg),
                    "MPU|seq=%lu|ax=%.4f|ay=%.4f|az=%.4f|gx=%.2f|gy=%.2f|gz=%.2f|temp=%.1f\n",
                    (unsigned long)pkt.seq,
                    pkt.ax_g, pkt.ay_g, pkt.az_g,
                    pkt.gx_dps, pkt.gy_dps, pkt.gz_dps, pkt.temp_c);
                if (len <= 0) {
                    continue;
                }

                int err = sendto(sock, msg, len, 0,
                                 (struct sockaddr *)&g_pc_addr,
                                 sizeof(g_pc_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error enviando al PC (errno %d)", errno);
                } else {
                    enviados_ok++;
                }
            }
        }

        if (enviados_ok >= siguiente_log) {
            ESP_LOGI(TAG, "Enviados %lu paquetes al PC",
                     (unsigned long)enviados_ok);
            siguiente_log = enviados_ok + 100;
        }

        /* Si el MPU no está presente, latido cada 1 s para verificar la red */
        static int64_t last_test_us = 0;
        if (g_pc_known && !g_mpu6050_online &&
            (now_us - last_test_us) >= 1000000LL) {
            const char *test_msg = "TEST|sin_mpu\n";
            int err = sendto(sock, test_msg, strlen(test_msg), 0,
                             (struct sockaddr *)&g_pc_addr,
                             sizeof(g_pc_addr));
            if (err >= 0) {
                last_test_us = now_us;
                ESP_LOGI(TAG, "Latido TEST enviado al PC");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(UDP_SEND_INTERVAL_MS));
    }
}

static void vTaskMpuReader(void *pvParameters)
{
    mpu_packet_t pkt;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(MPU_READ_INTERVAL_MS);

    /* Inicializar filtros FIR pasa-bajos una sola vez */
    if (!g_fir_primed) {
        const fir_window_t win = FIR_WIN_HAMMING;
        for (int i = 0; i < FIR_N_EJES; i++) {
            if (!fir_init_lowpass(&g_fir_mpu[i], FIR_NTAPS, FIR_FC_GUANTE, FIR_FS_GUANTE, win)) {
                ESP_LOGE(TAG, "Error inicializando FIR eje %d", i);
            }
        }
        g_fir_primed = true;
        ESP_LOGI(TAG, "FIR guante iniciado: ntaps=%d fc=%.1f Hz fs=%.1f Hz retardo=%.0f ms",
                 FIR_NTAPS, FIR_FC_GUANTE, FIR_FS_GUANTE,
                 fir_group_delay_ms(&g_fir_mpu[0], FIR_FS_GUANTE));
    }

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, period);

        if (!g_mpu6050_online) {
            continue;
        }

        int16_t rax, ray, raz, rtemp, rgx, rgy, rgz;
        if (mpu6050_read_raw(&rax, &ray, &raz, &rgx, &rgy, &rgz, &rtemp) == ESP_OK) {
            pkt.seq    = g_seq++;
            float ax_g   = rax / 16384.0f;
            float ay_g   = ray / 16384.0f;
            float az_g   = raz / 16384.0f;
            float gx_dps = rgx / 131.0f;
            float gy_dps = rgy / 131.0f;
            float gz_dps = rgz / 131.0f;
            float temp_c = (rtemp / 340.0f) + 36.53f;

            /* FIR pasa-bajos por eje (100 Hz -> 15 Hz) */
            pkt.ax_g   = fir_update(&g_fir_mpu[0], ax_g);
            pkt.ay_g   = fir_update(&g_fir_mpu[1], ay_g);
            pkt.az_g   = fir_update(&g_fir_mpu[2], az_g);
            pkt.gx_dps = fir_update(&g_fir_mpu[3], gx_dps);
            pkt.gy_dps = fir_update(&g_fir_mpu[4], gy_dps);
            pkt.gz_dps = fir_update(&g_fir_mpu[5], gz_dps);
            pkt.temp_c = temp_c;

            if (xQueueMpu != NULL) {
                /* Cola llena: descartar el paquete más viejo */
                if (uxQueueSpacesAvailable(xQueueMpu) == 0) {
                    mpu_packet_t dummy;
                    xQueueReceive(xQueueMpu, &dummy, 0);
                }
                xQueueSend(xQueueMpu, &pkt, 0);
            }
        } else {
            ESP_LOGW(TAG, "Error leyendo MPU6050");
        }
    }
}

/* ============================================================================
 * MAIN
 * ============================================================================ */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_mpu6050();
    init_wifi();

    xQueueMpu = xQueueCreate(10, sizeof(mpu_packet_t));
    if (xQueueMpu == NULL) {
        ESP_LOGE(TAG, "No se pudo crear la cola de MPU");
        return;
    }

    /* Si tu chip es de un solo núcleo (C3, C6, etc.), usa xTaskCreate(...) */
    xTaskCreatePinnedToCore(vTaskMpuReader, "mpu_reader", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(vTaskCommunication_Core0, "udp_sender", 6144, NULL, 5, NULL, 0);

    ESP_LOGI(TAG, "Sistema iniciado. Esperando conexión Wi-Fi...");
}