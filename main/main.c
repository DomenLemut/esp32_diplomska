#include <stdio.h>
#include <string.h>
#include <time.h>
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
#define SENSOR_DATA_FILE_PATH  "/sdcard/sensor_data.txt"

// SPI Pins for SD Card Reader
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

const char * TAG = "MAIN";
sdmmc_card_t *card = NULL;
bool sd_card_ready = false;

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
        .tm_year = 2026 - 1900, // 2026
        .tm_mon = 6,            // July (0-11)
        .tm_mday = 9,           // Day of the month
        .tm_hour = 17,
        .tm_min = 5,
        .tm_sec = 0
    };
    
    time_t t_of_day = mktime(&t);
    struct timeval tv = {
        .tv_sec = t_of_day,
        .tv_usec = 0
    };
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

    ESP_LOGI("SD_CARD", "Initializing SPI bus...");
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
        ESP_LOGE("SD_CARD", "Failed to initialize SPI bus.");
        return;
    }

    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_MISO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CLK,  GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CS,   GPIO_PULLUP_ONLY);

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI("SD_CARD", "Mounting filesystem...");
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        ESP_LOGE("SD_CARD", "Failed to mount filesystem (0x%X).", ret);
        sd_card_ready = false;
        return;
    }
    
    sd_card_ready = true;
    ESP_LOGI("SD_CARD", "SD Card mounted successfully!");

    // Create a CSV header row if the file doesn't exist yet
    FILE *f = fopen(LOG_FILE_PATH, "r");
    if (f == NULL) {
        f = fopen(LOG_FILE_PATH, "w");
        if (f != NULL) {
            fprintf(f, "Timestamp Sample_Index Gyro_X Gyro_Y Gyro_Z Accel_X Accel_Y Accel_Z\n");
            fclose(f);
        }
    } else {
        fclose(f);
    }
}

char current_log_path[128] = LOG_FILE_PATH; // Fallback default

// Function to scan the SD card and find the next sequential folder number
void create_next_session_folder(void) {
    if (!sd_card_ready) return;

    int next_session_id = 0;
    DIR *dir = opendir("/sdcard");
    
    if (dir != NULL) {
        struct dirent *de;
        // Scan all items in the root directory
        while ((de = readdir(dir)) != NULL) {
            // Check if the folder name is a number
            int current_id;
            if (sscanf(de->d_name, "%d", &current_id) == 1) {
                if (current_id >= next_session_id) {
                    next_session_id = current_id + 1; // Set to next available ID
                }
            }
        }
        closedir(dir);
    }

    // Create the full path string for the new directory
    char folder_path[64];
    snprintf(folder_path, sizeof(folder_path), "/sdcard/%d", next_session_id);

    // Create the physical directory on the FAT32 filesystem
    struct stat st = {0};
    if (stat(folder_path, &st) == -1) {
        if (mkdir(folder_path, 0777) == 0) {
            ESP_LOGW("SYSTEM", "Created brand new session folder: %s", folder_path);
        } else {
            ESP_LOGE("SYSTEM", "Failed to create directory %s", folder_path);
            return;
        }
    }

    // Update the global path variable for your logging loops to use
    snprintf(current_log_path, sizeof(current_log_path), "%s/sensor_log.txt", folder_path);
}

void log_acc_info(const char *timestamp) {
    if (!sd_card_ready) return;

    FILE *f = fopen(current_log_path, "a"); // "a" opens in standard ASCII text append mode
    if (f == NULL) return;

    fprintf(f, "freq: %d\n i2c speed: %d\n samples per interrupt: %d\n start time: %s\n", FREQ, I2C_MASTER_CLK_SPEED, NUM_SAMPLES_PER_BATCH, timestamp);

    fclose(f);
}

// Formats and logs the samples as standard readable text strings
void log_text_to_sd(const char *timestamp, sensor_sample *samples, int count) {
    if (!sd_card_ready) return;

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
    init_internal_clock();
    
    i2c_init();
    lsm6ds3_init();
    vTaskDelay(pdMS_TO_TICKS(15)); 

    interrupt_init();
    lsm6ds3_setup_event_window();
    lsm6ds3_configure_motion_interrupt();

    lsm6ds3_start();

    static uint8_t data[FIFO_THR_BYTES];
    uint8_t wakeup_status = 0;
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];

    lsm6ds3_read_wakeup_source(&wakeup_status);
    uint16_t fifo_word_count = lsm6ds3_get_fifo_word_count();
    lsm6ds3_read_fifo(data, fifo_word_count > 0 ? fifo_word_count : 12); 
    ESP_LOGI(TAG, "Hardware flushes completed. Entering operational loop...");

    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);   
    log_acc_info(strftime_buf);

    while (1)
    {
        esp_light_sleep_start();

        gpio_set_level(BLINK_LED_PIN, 1);

        uint16_t fifo_word_count = lsm6ds3_get_fifo_word_count();
        uint16_t bytes_available = fifo_word_count * 2;

        if (bytes_available >= FIFO_THR_BYTES)
        {
            time(&now);
            localtime_r(&now, &timeinfo);
            strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
            
            lsm6ds3_read_fifo(data, FIFO_THR_BYTES);
            lsm6ds3_read_wakeup_source(&wakeup_status);

            sensor_sample *samples = (sensor_sample *)data;
            int total_samples = FIFO_THR_BYTES / sizeof(sensor_sample);

            // Write the array out as clean readable text strings
            log_text_to_sd(strftime_buf, samples, total_samples);

            ESP_LOGI(TAG, "[%s] Successfully logged %d text rows to SD Card.", strftime_buf, total_samples);
        }
        else {
            ESP_LOGI(TAG, "Accumulating... FIFO word count: %d", fifo_word_count);
        }
        
        gpio_set_level(BLINK_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}