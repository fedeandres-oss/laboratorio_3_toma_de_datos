/*
 * =============================================================================
 *  CODIGO DE PRUEBA - CARRO ROBOT (ESP32, ESP-IDF 5.4)
 *  Laboratorio 3 - Procesamiento de Señales
 *  =============================================================================
 *
 *  Este codigo hace avanzar el carro hacia adelante a velocidad constante
 *  durante 5 segundos, luego se detiene. Lee encoders y MPU6050 para verificar
 *  que motores y sensores esten bien conectados.
 *
 *  Pines (respetando configuracion original):
 *    MI: ENA=17, IN1=18, IN2=19   |  MD: ENA=21, IN1=22, IN2=23
 *    Enc MI=32, Enc MD=25         |  I2C SDA=4, SCL=5 (MPU6050 0x68/0x69)
 * =============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"

static const char *TAG = "CARRO_TEST";

// ============================================================================
// PINES MOTORES L298N
// ============================================================================
#define GPIO_MI_ENA        17
#define GPIO_MI_IN1        18
#define GPIO_MI_IN2        19

#define GPIO_MD_ENA        21
#define GPIO_MD_IN1        22
#define GPIO_MD_IN2        23

// ============================================================================
// PINES ENCODERS
// ============================================================================
#define GPIO_ENC_MI        32
#define GPIO_ENC_MD        25

// ============================================================================
// MPU6050 I2C
// ============================================================================
#define GPIO_I2C_SDA       4
#define GPIO_I2C_SCL       5
#define I2C_PORT_NUM       I2C_NUM_0

#define MPU6050_ADDR_LOW     0x68
#define MPU6050_ADDR_HIGH    0x69
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

// ============================================================================
// PWM
// ============================================================================
#define PWM_FREQ_HZ        2000
#define PWM_DUTY_PERCENT   50.0f   // 50% de velocidad para prueba

static mcpwm_cmpr_handle_t cmpr_mi_in1 = NULL, cmpr_mi_in2 = NULL;
static mcpwm_cmpr_handle_t cmpr_md_in1 = NULL, cmpr_md_in2 = NULL;
static pcnt_unit_handle_t pcnt_unit_mi = NULL, pcnt_unit_md = NULL;

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t mpu6050_dev = NULL;
static bool g_mpu6050_online = false;

/* ============================================================================
 *  MOTORES L298N (MCPWM)
 * ============================================================================ */
static void init_motors_l298n(void)
{
    gpio_config_t enable_conf = {
        .pin_bit_mask = (1ULL << GPIO_MI_ENA) | (1ULL << GPIO_MD_ENA),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
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

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        gen_mi_in1, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen_mi_in1, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_mi_in1, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        gen_mi_in2, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen_mi_in2, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_mi_in2, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        gen_md_in1, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen_md_in1, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_md_in1, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        gen_md_in2, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen_md_in2, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_md_in2, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "Motores inicializados (MCPWM %d Hz).", PWM_FREQ_HZ);
}

static void set_motor_speed_2pins(mcpwm_cmpr_handle_t cmpr_in1,
                                  mcpwm_cmpr_handle_t cmpr_in2,
                                  float speed_percent)
{
    if (speed_percent > 100.0f) speed_percent = 100.0f;
    if (speed_percent < -100.0f) speed_percent = -100.0f;

    uint32_t max_duty = 10000000 / PWM_FREQ_HZ;
    uint32_t duty_ticks = (uint32_t)(max_duty * (fabsf(speed_percent) / 100.0f));

    if (speed_percent > 0.5f) {
        mcpwm_comparator_set_compare_value(cmpr_in1, duty_ticks);
        mcpwm_comparator_set_compare_value(cmpr_in2, 0);
    } else if (speed_percent < -0.5f) {
        mcpwm_comparator_set_compare_value(cmpr_in1, 0);
        mcpwm_comparator_set_compare_value(cmpr_in2, duty_ticks);
    } else {
        mcpwm_comparator_set_compare_value(cmpr_in1, 0);
        mcpwm_comparator_set_compare_value(cmpr_in2, 0);
    }
}

/* ============================================================================
 *  ENCODERS (PCNT)
 * ============================================================================ */
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

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    return pcnt_unit;
}

/* ============================================================================
 *  MPU6050 (I2C)
 * ============================================================================ */
static void force_i2c_bus_free(gpio_num_t sda_pin, gpio_num_t scl_pin)
{
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << sda_pin) | (1ULL << scl_pin),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&conf);

    gpio_set_level(sda_pin, 1);
    gpio_set_level(scl_pin, 1);
    esp_rom_delay_us(10);

    for (int i = 0; i < 9; i++) {
        if (gpio_get_level(sda_pin) == 1) break;
        gpio_set_level(scl_pin, 0);
        esp_rom_delay_us(10);
        gpio_set_level(scl_pin, 1);
        esp_rom_delay_us(10);
    }

    gpio_set_level(sda_pin, 0);
    esp_rom_delay_us(10);
    gpio_set_level(scl_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(sda_pin, 1);
    esp_rom_delay_us(10);

    gpio_reset_pin(sda_pin);
    gpio_reset_pin(scl_pin);
}

static void init_mpu6050(void)
{
    g_mpu6050_online = false;

    force_i2c_bus_free(GPIO_I2C_SDA, GPIO_I2C_SCL);
    vTaskDelay(pdMS_TO_TICKS(50));

    if (mpu6050_dev != NULL) {
        i2c_master_bus_rm_device(mpu6050_dev);
        mpu6050_dev = NULL;
    }
    if (i2c_bus != NULL) {
        i2c_del_master_bus(i2c_bus);
        i2c_bus = NULL;
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT_NUM,
        .sda_io_num = GPIO_I2C_SDA,
        .scl_io_num = GPIO_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_config, &i2c_bus) != ESP_OK) {
        ESP_LOGE(TAG, "Error creando bus I2C");
        return;
    }

    uint8_t addr = 0;
    if (i2c_master_probe(i2c_bus, MPU6050_ADDR_LOW, 100) == ESP_OK) {
        addr = MPU6050_ADDR_LOW;
    } else if (i2c_master_probe(i2c_bus, MPU6050_ADDR_HIGH, 100) == ESP_OK) {
        addr = MPU6050_ADDR_HIGH;
    }

    if (addr == 0) {
        ESP_LOGE(TAG, "MPU6050 no responde en 0x68 ni 0x69");
        return;
    }
    ESP_LOGI(TAG, "MPU6050 detectado en 0x%02X", addr);

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(i2c_bus, &dev_config, &mpu6050_dev) != ESP_OK) {
        ESP_LOGE(TAG, "Error agregando MPU6050");
        return;
    }

    uint8_t wake_cmd[2] = {MPU6050_PWR_MGMT_1, 0x00};
    if (i2c_master_transmit(mpu6050_dev, wake_cmd, sizeof(wake_cmd), 100) == ESP_OK) {
        g_mpu6050_online = true;
        ESP_LOGI(TAG, "MPU6050 inicializado.");
    } else {
        ESP_LOGE(TAG, "Fallo al despertar MPU6050");
    }
}

static esp_err_t read_mpu6050(float *ax, float *ay, float *az,
                              float *gx, float *gy, float *gz)
{
    if (!g_mpu6050_online) return ESP_FAIL;

    uint8_t reg_addr = MPU6050_ACCEL_XOUT_H;
    uint8_t raw_buffer[14];

    esp_err_t ret = i2c_master_transmit_receive(mpu6050_dev, &reg_addr, 1,
                                                raw_buffer, sizeof(raw_buffer),
                                                100);
    if (ret != ESP_OK) return ret;

    int16_t raw_ax = (int16_t)((raw_buffer[0] << 8) | raw_buffer[1]);
    int16_t raw_ay = (int16_t)((raw_buffer[2] << 8) | raw_buffer[3]);
    int16_t raw_az = (int16_t)((raw_buffer[4] << 8) | raw_buffer[5]);
    int16_t raw_gx = (int16_t)((raw_buffer[8] << 8) | raw_buffer[9]);
    int16_t raw_gy = (int16_t)((raw_buffer[10] << 8) | raw_buffer[11]);
    int16_t raw_gz = (int16_t)((raw_buffer[12] << 8) | raw_buffer[13]);

    *ax = (float)raw_ax / 16384.0f;
    *ay = (float)raw_ay / 16384.0f;
    *az = (float)raw_az / 16384.0f;
    *gx = (float)raw_gx / 131.0f;
    *gy = (float)raw_gy / 131.0f;
    *gz = (float)raw_gz / 131.0f;

    return ESP_OK;
}

/* ============================================================================
 *  APP MAIN - PRUEBA DE MOTORES Y SENSORES
 * ============================================================================ */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "=== INICIANDO PRUEBA DE CARRO ===");

    /* Inicializar hardware */
    init_motors_l298n();
    pcnt_unit_mi = init_single_phase_encoder(GPIO_ENC_MI);
    pcnt_unit_md = init_single_phase_encoder(GPIO_ENC_MD);
    init_mpu6050();

    /* Leer sensores iniciales */
    int raw_l = 0, raw_r = 0;
    float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;

    pcnt_unit_get_count(pcnt_unit_mi, &raw_l);
    pcnt_unit_get_count(pcnt_unit_md, &raw_r);
    read_mpu6050(&ax, &ay, &az, &gx, &gy, &gz);

    ESP_LOGI(TAG, "Sensores iniciales:");
    ESP_LOGI(TAG, "  Encoder MI: %d  |  Encoder MD: %d", raw_l, raw_r);
    ESP_LOGI(TAG, "  MPU6050: ax=%.3f ay=%.3f az=%.3f | gx=%.3f gy=%.3f gz=%.3f",
             ax, ay, az, gx, gy, gz);

    /* ------------------------------------------------------------
     * PRUEBA: AVANZAR HACHA ADELANTE durante 5 segundos
     * ------------------------------------------------------------ */
    ESP_LOGI(TAG, "Avanzando a %.0f%% durante 5 segundos...", PWM_DUTY_PERCENT);
    set_motor_speed_2pins(cmpr_mi_in1, cmpr_mi_in2, PWM_DUTY_PERCENT);
    set_motor_speed_2pins(cmpr_md_in1, cmpr_md_in2, PWM_DUTY_PERCENT);

    for (int i = 0; i < 50; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));

        /* Leer sensores cada 100 ms */
        pcnt_unit_get_count(pcnt_unit_mi, &raw_l);
        pcnt_unit_get_count(pcnt_unit_md, &raw_r);
        read_mpu6050(&ax, &ay, &az, &gx, &gy, &gz);

        ESP_LOGI(TAG, "[%2d] Enc MI=%d  Enc MD=%d  |  ax=%.2f ay=%.2f az=%.2f  gx=%.1f gy=%.1f gz=%.1f",
                 i+1, raw_l, raw_r, ax, ay, az, gx, gy, gz);
    }

    /* Detener motores */
    ESP_LOGI(TAG, "Deteniendo motores.");
    set_motor_speed_2pins(cmpr_mi_in1, cmpr_mi_in2, 0.0f);
    set_motor_speed_2pins(cmpr_md_in1, cmpr_md_in2, 0.0f);

    /* Lectura final */
    pcnt_unit_get_count(pcnt_unit_mi, &raw_l);
    pcnt_unit_get_count(pcnt_unit_md, &raw_r);
    read_mpu6050(&ax, &ay, &az, &gx, &gy, &gz);

    ESP_LOGI(TAG, "=== PRUEBA FINALIZADA ===");
    ESP_LOGI(TAG, "Encoder MI final: %d  |  Encoder MD final: %d", raw_l, raw_r);
    ESP_LOGI(TAG, "MPU6050 final: ax=%.3f ay=%.3f az=%.3f | gx=%.3f gy=%.3f gz=%.3f",
             ax, ay, az, gx, gy, gz);

    if (raw_l > 0 && raw_r > 0) {
        ESP_LOGI(TAG, "✓ MOTORES: Ambos encoders detectaron movimiento (ruedas giraron).");
    } else if (raw_l == 0 && raw_r == 0) {
        ESP_LOGW(TAG, "✗ MOTORES: Ningun encoder detecto movimiento. Revisar cableado de motores/encoders.");
    } else {
        ESP_LOGW(TAG, "⚠ MOTORES: Solo un encoder detecto movimiento. Revisar el otro.");
    }

    if (g_mpu6050_online) {
        ESP_LOGI(TAG, "✓ SENSORES: MPU6050 detectado y leyendo datos.");
    } else {
        ESP_LOGW(TAG, "✗ SENSORES: MPU6050 no responde. Revisar I2C (SDA=4, SCL=5).");
    }

    /* Mantener vivo el programa (motores detenidos) */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
