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
#include "esp_adc/adc_oneshot.h"

#include "LSM6DS3_driver.h"
#include "LSM6DS3_conf.h"
#include "RTC.h"
#include "WiFi.h"

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

static adc_oneshot_unit_handle_t adc1_handle;

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

void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(&init_config, &adc1_handle)
    );

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc1_handle,
            ADC_CHANNEL_0,   // GPIO36
            &channel_config
        )
    );
}

int read_adc_raw(void)
{
    int adc_value = 0;

    ESP_ERROR_CHECK(
        adc_oneshot_read(
            adc1_handle,
            ADC_CHANNEL_0,
            &adc_value
        )
    );

    return adc_value;
}

int read_adc(void)
{
    int sum = 0;
    int samples = 16;

    for(int i = 0; i < samples; i++)
    {
        sum += read_adc_raw();
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    return sum / samples;
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

char current_log_path[128] = LOG_FILE_PATH;
char current_setup_path[128] = SENSOR_DATA_FILE_PATH;

// Function to scan the SD card and find the next sequential folder number
void create_next_session_folder(void)
{
    if (!sd_card_ready)
        return;

    int next_session_id = 0;

    DIR *dir = opendir("/sdcard");

    if (dir != NULL)
    {
        struct dirent *de;

        while ((de = readdir(dir)) != NULL)
        {
            int current_id;

            if (sscanf(de->d_name, "%d", &current_id) == 1)
            {
                if (current_id >= next_session_id)
                    next_session_id = current_id + 1;
            }
        }

        closedir(dir);
    }

    char folder_path[64];

    snprintf(folder_path,
            sizeof(folder_path),
            "/sdcard/%d",
            next_session_id);

    if (mkdir(folder_path, 0777) != 0)
    {
        ESP_LOGE(TAG, "Failed to create directory %s", folder_path);
        return;
    }

    ESP_LOGI(TAG, "Created session folder %s", folder_path);

    snprintf(current_log_path,
            sizeof(current_log_path),
            "%s/sensor_log.txt",
            folder_path);

    snprintf(current_setup_path,
            sizeof(current_setup_path),
            "%s/sensor_data.txt",
            folder_path);

    FILE *f = fopen(current_log_path, "w");
    if (f)
    {
        fprintf(f,
                "Timestamp,Sample_Index,Gyro_X,Gyro_Y,Gyro_Z,Accel_X,Accel_Y,Accel_Z\n");
        fclose(f);
    }

    f = fopen(current_setup_path, "w");
    if (f)
    {
        fclose(f);
    }
}

void log_acc_info(const char *timestamp)
{
    if (!sd_card_ready) return;

    FILE *f = fopen(current_setup_path, "a");
    if (f == NULL) 
    {
        ESP_LOGE(TAG, "Failed to open %s", current_setup_path);
        return;
    }

    fprintf(f,
            "Frequency: %d Hz\n"
            "I2C speed: %d Hz\n"
            "Samples per interrupt: %d\n"
            "Start time: %s\n",
            FREQ,
            I2C_MASTER_CLK_SPEED,
            NUM_SAMPLES_PER_BATCH,
            timestamp);

    fclose(f);
}

// Formats and logs the samples as standard readable text strings
void log_text_to_sd(const char *timestamp, sensor_sample *samples, int count) {
    if (!sd_card_ready) return;

    FILE *f = fopen(current_log_path, "a");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open %s", current_log_path);
        return;
    }

    for (int i = 0; i < count; i++) {
        fprintf(f, "%s,%d,%d,%d,%d,%d,%d,%d\n",
                timestamp, i,
                samples[i].gyro_x,  samples[i].gyro_y,  samples[i].gyro_z,
                samples[i].accel_x, samples[i].accel_y, samples[i].accel_z);
    }
    fclose(f);
}

#define HOUR_US (3600ULL * 1000000ULL)
#define MIN_US (60ULL * 1000000ULL)
#define HALF_HOUR_US (1800ULL * 1000000ULL)

void app_main(void)
{
    //LED
    led_init();
    adc_init();

    //RTC
    get_time_remote();

    //SD Card
    init_sd_card();
    create_next_session_folder();
    
    i2c_init();
    lsm6ds3_init();
    vTaskDelay(pdMS_TO_TICKS(15)); 

    interrupt_init();
    lsm6ds3_setup_event_window();
    lsm6ds3_configure_motion_interrupt();

    print_time();

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

    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(MIN_US));

    while (1)
    {   
        esp_light_sleep_start();

        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

        if (cause == ESP_SLEEP_WAKEUP_TIMER)
        {
            float voltage = ((float) read_adc() / 4095.0f) * 3.3f * 5.7; // Assuming a 12-bit ADC and 3.3V reference

            ESP_LOGI(TAG, "Voltage: %.2f V", voltage);

            send_thingspeak_data(voltage);

            vTaskDelay(pdMS_TO_TICKS(1000));

            ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(HOUR_US));

            continue;
        } 

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