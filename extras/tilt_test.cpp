#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG       "IMU_TEST"
#define I2C_PORT  I2C_NUM_0
#define SDA_PIN   GPIO_NUM_7
#define SCL_PIN   GPIO_NUM_8
#define ICM_ADDR  0x68

#define THRESHOLD 500

void icm_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (ICM_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(h, buf, 2, true);
    i2c_master_stop(h);
    i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(h);
}

void icm_read(uint8_t reg, uint8_t *data, size_t len) {
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

extern "C" void app_main() {
    // Init I2C
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

    // Check WHO_AM_I
    uint8_t who = 0;
    icm_read(0x75, &who, 1);
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (expect 0x67 or 0x6F)", who);

    // Wake up + enable accel and gyro
    icm_write(0x1F, 0x0F);  // PWR_MGMT0: accel+gyro low noise
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set accel to 2G range, 200Hz ODR
    icm_write(0x21, 0x06);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "ICM-42670-P ready, reading accel...");

    while (true) {
        uint8_t raw[6];
        icm_read(0x0B, raw, 6);  // ACCEL_DATA_X1 through ACCEL_DATA_Z0

        int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
        int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
        int16_t az = (int16_t)((raw[4] << 8) | raw[5]);

        ESP_LOGI(TAG, "ax=%d ay=%d az=%d", ax, ay, az);

        char msg[64] = "";
        bool any = false;

        if (ay > THRESHOLD)       { strcat(msg, "UP ");    any = true; }
        else if (ay < -THRESHOLD) { strcat(msg, "DOWN ");  any = true; }

        if (ax < -THRESHOLD)      { strcat(msg, "LEFT ");  any = true; }
        else if (ax > THRESHOLD)  { strcat(msg, "RIGHT "); any = true; }

        if (any) ESP_LOGI(TAG, "%s", msg);

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

