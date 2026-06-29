#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "LSM6DS3_driver.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "LSM6DS3_conf.h"

#define BLINK_LED_PIN   2

typedef struct {
    int16_t gyro_x;   
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t accel_x;    
    int16_t accel_y;
    int16_t accel_z;
    uint8_t ts0;
    uint8_t ts1;
    uint8_t ts2;
    uint8_t step_l;
    uint8_t step_h;
    uint8_t reserved;
} __attribute__((packed)) sensor_sample;

const char * TAG = "MAIN";

void led_init(void) {
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << BLINK_LED_PIN),
        .mode = GPIO_MODE_OUTPUT,               // Set as OUTPUT
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&led_conf));
}

void app_main(void)
{
    i2c_init();
    led_init();

    lsm6ds3_init();
    lsm6ds3_setup();
    lsm6ds3_start();

    uint8_t data[FIFO_THR_BYTES];

    while (1)
    {
        ESP_LOGI(TAG, "Reading FIFO...");

        lsm6ds3_read_fifo(data, FIFO_THR_BYTES);

        gpio_set_level(BLINK_LED_PIN, 1);

        vTaskDelay(pdMS_TO_TICKS(10));

        gpio_set_level(BLINK_LED_PIN, 0);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}