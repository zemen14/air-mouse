#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_hidd.h"
#include "esp_hid_gap.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

static const char *TAG = "HID_DEV_DEMO";

#define I2C_PORT     I2C_NUM_0
#define SDA_PIN      GPIO_NUM_7
#define SCL_PIN      GPIO_NUM_8
#define ICM_ADDR     0x68
#define BOOT_BTN     GPIO_NUM_9
#define THRESH_SMALL 500
#define THRESH_BIG   2000

typedef struct {
    TaskHandle_t task_hdl;
    esp_hidd_dev_t *hid_dev;
    uint8_t protocol_mode;
    uint8_t *buffer;
} local_param_t;

static local_param_t s_ble_hid_param = {0};

static const unsigned char mouseReportMap[] = {
    0x05, 0x01, 0x09, 0x02, 0xa1, 0x01,
    0x09, 0x01, 0xa1, 0x00,
    0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
    0x15, 0x00, 0x25, 0x01,
    0x95, 0x03, 0x75, 0x01, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x05, 0x81, 0x03,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
    0x15, 0x81, 0x25, 0x7f,
    0x75, 0x08, 0x95, 0x03, 0x81, 0x06,
    0xc0, 0xc0
};

static esp_hid_raw_report_map_t ble_report_maps[] = {
    { .data = mouseReportMap, .len = sizeof(mouseReportMap) }
};

static esp_hid_device_config_t ble_hid_config = {
    .vendor_id         = 0x16C0,
    .product_id        = 0x05DF,
    .version           = 0x0100,
    .device_name       = "AirMouse",
    .manufacturer_name = "Zemen",
    .serial_number     = "1234567890",
    .report_maps       = ble_report_maps,
    .report_maps_len   = 1
};

static void icm_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (ICM_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(h, buf, 2, true);
    i2c_master_stop(h);
    i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(h);
}

static void icm_read(uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (ICM_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h, reg, true);
    i2c_master_start(h);
    i2c_master_write_byte(h, (ICM_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(h, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(h);
    i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(h);
}

static void imu_init(void) {
    i2c_driver_delete(I2C_PORT);
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_PIN;
    conf.scl_io_num = SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    icm_write(0x1F, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(100));
    icm_write(0x21, 0x06);
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_config_t btn = {};
    btn.pin_bit_mask = (1ULL << BOOT_BTN);
    btn.mode = GPIO_MODE_INPUT;
    btn.pull_up_en = GPIO_PULLUP_ENABLE;
    btn.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&btn);
    ESP_LOGI(TAG, "IMU initialized");
}

static void send_mouse(uint8_t buttons, char dx, char dy, char wheel) {
    static uint8_t buffer[4] = {0};
    buffer[0] = buttons;
    buffer[1] = dx;
    buffer[2] = dy;
    buffer[3] = wheel;
    esp_hidd_dev_input_set(s_ble_hid_param.hid_dev, 0, 0, buffer, 4);
}

static void ble_hid_demo_task_mouse(void *pvParameters) {
    ESP_LOGI(TAG, "IMU mouse task started");
    int accel = 1;
    TickType_t tilt_start = 0;
    bool was_tilted = false;

    while (1) {
        uint8_t raw[6];
        icm_read(0x0B, raw, 6);
        int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
        int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);

        int x_delta = 0, y_delta = 0;

        if (ax > THRESH_BIG)        x_delta =  15;
        else if (ax > THRESH_SMALL) x_delta =  5;
        else if (ax < -THRESH_BIG)  x_delta = -15;
        else if (ax < -THRESH_SMALL) x_delta = -5;

        if (ay > THRESH_BIG)        y_delta = -15;
        else if (ay > THRESH_SMALL) y_delta = -5;
        else if (ay < -THRESH_BIG)  y_delta =  15;
        else if (ay < -THRESH_SMALL) y_delta =  5;

        bool is_tilted = (x_delta != 0 || y_delta != 0);
        if (is_tilted) {
            if (!was_tilted) { tilt_start = xTaskGetTickCount(); accel = 1; }
            else {
                uint32_t ms = (xTaskGetTickCount() - tilt_start) * portTICK_PERIOD_MS;
                if (ms > 200) accel = 3;
                else if (ms > 50) accel = 2;
            }
            was_tilted = true;
        } else {
            accel = 1;
            was_tilted = false;
        }

        int final_x = x_delta * accel;
        int final_y = y_delta * accel;

        uint8_t buttons = 0;
        if (gpio_get_level(BOOT_BTN) == 0) buttons = 1;

        if (final_x != 0 || final_y != 0 || buttons != 0) {
            send_mouse(buttons, (char)final_x, (char)final_y, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void ble_hid_task_start_up(void) {
    if (s_ble_hid_param.task_hdl) return;
    xTaskCreate(ble_hid_demo_task_mouse, "ble_hid_demo_task_mouse", 4 * 1024,
                NULL, configMAX_PRIORITIES - 3, &s_ble_hid_param.task_hdl);
}

void ble_hid_task_shut_down(void) {
    if (s_ble_hid_param.task_hdl) {
        vTaskDelete(s_ble_hid_param.task_hdl);
        s_ble_hid_param.task_hdl = NULL;
    }
}

static void ble_hidd_event_callback(void *handler_args, esp_event_base_t base,
                                     int32_t id, void *event_data) {
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    static const char *TAG = "HID_DEV_BLE";

    switch (event) {
    case ESP_HIDD_START_EVENT:
        ESP_LOGI(TAG, "START");
        esp_hid_ble_gap_adv_start();
        break;
    case ESP_HIDD_CONNECT_EVENT:
        ESP_LOGI(TAG, "CONNECT");
        break;
    case ESP_HIDD_CONTROL_EVENT:
        if (param->control.control) ble_hid_task_start_up();
        else ble_hid_task_shut_down();
        break;
    case ESP_HIDD_DISCONNECT_EVENT:
        ESP_LOGI(TAG, "DISCONNECT");
        ble_hid_task_shut_down();
        esp_hid_ble_gap_adv_start();
        break;
    case ESP_HIDD_STOP_EVENT:
        ESP_LOGI(TAG, "STOP");
        break;
    default:
        break;
    }
}

void app_main(void) {
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    imu_init();

    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_DEV_MODE);
    ret = esp_hid_gap_init(HID_DEV_MODE);
    ESP_ERROR_CHECK(ret);

    ret = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_MOUSE, ble_hid_config.device_name);
    ESP_ERROR_CHECK(ret);

    if ((ret = esp_ble_gatts_register_callback(esp_hidd_gatts_event_handler)) != ESP_OK) {
        ESP_LOGE(TAG, "GATTS register callback failed: %d", ret);
        return;
    }

    ESP_LOGI(TAG, "setting ble device");
    ESP_ERROR_CHECK(esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE,
                                       ble_hidd_event_callback, &s_ble_hid_param.hid_dev));
}
