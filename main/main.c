#include <stdio.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

#include "LSM6DS3_driver.h"
#include "LSM6DS3_conf.h"

#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"

#define BLINK_LED_PIN   2
#define LOG_FILE_PATH   "/sdcard/sensor_log.txt"

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

typedef struct {
    int16_t gyro_x;   
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t accel_x;    
    int16_t accel_y;
    int16_t accel_z;
} __attribute__((packed)) sensor_sample;

static const char *TAG = "MAIN";
sdmmc_card_t *card = NULL;
bool sd_card_ready = false;
char current_log_path[128] = LOG_FILE_PATH;

void led_init(void) {
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << BLINK_LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&led_conf));
}

void init_internal_clock(void) {
    struct tm t = {
        .tm_year = 2026 - 1900,
        .tm_mon = 6,
        .tm_mday = 9,
        .tm_hour = 17,
        .tm_min = 5,
        .tm_sec = 0
    };
    
    time_t t_of_day = mktime(&t);
    struct timeval tv = { .tv_sec = t_of_day, .tv_usec = 0 };
    settimeofday(&tv, NULL);
}

void init_sd_card(void) {
    esp_err_t ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_PROBING;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus.");
        return;
    }

    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_MISO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CLK,  GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CS,   GPIO_PULLUP_ONLY);

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card.");
        sd_card_ready = false;
        return;
    }
    
    sd_card_ready = true;
    ESP_LOGI(TAG, "SD Card mounted successfully!");
}

void create_next_session_folder(void) {
    if (!sd_card_ready) return;

    int next_session_id = 0;
    DIR *dir = opendir("/sdcard");
    
    if (dir != NULL) {
        struct dirent *de;
        while ((de = readdir(dir)) != NULL) {
            int current_id;
            if (sscanf(de->d_name, "%d", &current_id) == 1) {
                if (current_id >= next_session_id) {
                    next_session_id = current_id + 1;
                }
            }
        }
        closedir(dir);
    }

    char folder_path[64];
    snprintf(folder_path, sizeof(folder_path), "/sdcard/%d", next_session_id);

    struct stat st = {0};
    if (stat(folder_path, &st) == -1) {
        mkdir(folder_path, 0777);
    }

    snprintf(current_log_path, sizeof(current_log_path), "%s/sensor_log.txt", folder_path);

    FILE *f = fopen(current_log_path, "w");
    if (f != NULL) {
        fprintf(f, "Timestamp,Sample_Index,Gyro_X,Gyro_Y,Gyro_Z,Accel_X,Accel_Y,Accel_Z\n");
        fclose(f);
    }
}

void log_text_to_sd(const char *timestamp, sensor_sample *samples, int count) {
    if (!sd_card_ready || count <= 0) return;

    FILE *f = fopen(current_log_path, "a");
    if (f == NULL) return;

    for (int i = 0; i < count; i++) {
        fprintf(f, "%s,%d,%d,%d,%d,%d,%d,%d\n",
                timestamp, i,
                samples[i].gyro_x,  samples[i].gyro_y,  samples[i].gyro_z,
                samples[i].accel_x, samples[i].accel_y, samples[i].accel_z);
    }
    fclose(f);
}

void app_main(void)
{
    led_init();
    init_sd_card();
    create_next_session_folder();
    init_internal_clock();
    
    i2c_init();
    lsm6ds3_init();
    vTaskDelay(pdMS_TO_TICKS(15)); 

    interrupt_init();
    lsm6ds3_setup_continuous_with_wakeup();

    static uint8_t dummy_buffer[4096];
    static uint8_t fifo_buffer[4096];
    uint8_t wakeup_status = 0;
    uint32_t capture_round = 0;
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];

    lsm6ds3_read_wakeup_source(&wakeup_status);
    lsm6ds3_reset_fifo();

    ESP_LOGI(TAG, "System Ready. Waiting for motion...");

    #define PRE_TRIGGER_SAMPLES   52
    #define POST_TRIGGER_SAMPLES  104

    #define WORDS_PER_SAMPLE      6
    #define BYTES_PER_SAMPLE      12

    #define PRE_TRIGGER_WORDS     (PRE_TRIGGER_SAMPLES * WORDS_PER_SAMPLE)
    #define POST_TRIGGER_WORDS    (POST_TRIGGER_SAMPLES * WORDS_PER_SAMPLE)

    #define TOTAL_WORDS           (PRE_TRIGGER_WORDS + POST_TRIGGER_WORDS)
    #define TOTAL_BYTES           (TOTAL_WORDS * 2)

    while (1)
    {
        esp_light_sleep_start();
        gpio_set_level(BLINK_LED_PIN, 1);

        // -------------------------------------------------------------------------------------------

        // Clear interrupt source
        lsm6ds3_read_wakeup_source(&wakeup_status);


        // How many FIFO words currently exist?
        uint16_t fifo_words = lsm6ds3_get_fifo_word_count();

        ESP_LOGI(TAG, "FIFO contains %d words (%d samples)",
                fifo_words,
                fifo_words / WORDS_PER_SAMPLE);


        // We need to keep the newest PRE_TRIGGER samples.
        // Remove the oldest ones.

        if (fifo_words < PRE_TRIGGER_WORDS)
        {
            ESP_LOGW(TAG, "Not enough samples before trigger");
            lsm6ds3_reset_fifo();
            continue;
        }


        uint16_t words_to_discard = fifo_words - PRE_TRIGGER_WORDS;


        // Convert words to bytes because read_fifo() takes bytes
        uint32_t discard_bytes = words_to_discard * 2;


        if(discard_bytes > 0)
        {
            lsm6ds3_read_fifo(dummy_buffer, discard_bytes);
        }


        ESP_LOGI(TAG, "Waiting for post-trigger data");


        // Now FIFO contains exactly PRE_TRIGGER samples
        // Wait until it contains PRE + POST samples

        while(lsm6ds3_get_fifo_word_count() < TOTAL_WORDS)
        {
            uint16_t words = lsm6ds3_get_fifo_word_count();

            ESP_LOGI(TAG,
                    "FIFO: %d words (%d/%d samples)",
                    words,
                    words / WORDS_PER_SAMPLE,
                    PRE_TRIGGER_SAMPLES + POST_TRIGGER_SAMPLES);

            vTaskDelay(pdMS_TO_TICKS(10));
        }


        // Now read everything at once
        ESP_ERROR_CHECK(
            lsm6ds3_read_fifo(
                fifo_buffer,
                TOTAL_BYTES
            )
        );


        lsm6ds3_reset_fifo();
        
        // -------------------------------------------------------------------------------------------

        // 6. Log event to SD card
        time(&now);
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
        log_text_to_sd(strftime_buf, (sensor_sample *)fifo_buffer, PRE_TRIGGER_SAMPLES + POST_TRIGGER_SAMPLES);


        // -------------------------------------------------------------------------------------------

        gpio_set_level(BLINK_LED_PIN, 0);
    }
}