#include <stdio.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdbool.h>

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


/* ============================================================
 * Configuration
 * ============================================================ */

#define BLINK_LED_PIN       2

#define LOG_FILE_PATH       "/sdcard/sensor_log.txt"
#define LOG_SETUP_PATH      "/sdcard/setup.txt"

#define PIN_NUM_MISO        19
#define PIN_NUM_MOSI        23
#define PIN_NUM_CLK         18
#define PIN_NUM_CS          5

#define HOUR_US             (3600ULL * 1000000ULL)
#define MIN_US              (60ULL * 1000000ULL)
#define HALF_HOUR_US        (1800ULL * 1000000ULL)

#define PRE_TRIGGER_SAMPLES     104
#define POST_TRIGGER_SAMPLES    208

#define WORDS_PER_SAMPLE        6
#define BYTES_PER_SAMPLE        12

#define PRE_TRIGGER_WORDS       (PRE_TRIGGER_SAMPLES * WORDS_PER_SAMPLE)
#define POST_TRIGGER_WORDS      (POST_TRIGGER_SAMPLES * WORDS_PER_SAMPLE)

#define TOTAL_WORDS             (PRE_TRIGGER_WORDS + POST_TRIGGER_WORDS)
#define TOTAL_BYTES             (TOTAL_WORDS * 2)


/* ============================================================
 * Types
 * ============================================================ */

typedef struct __attribute__((packed))
{
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

} sensor_sample;


/* ============================================================
 * Globals
 * ============================================================ */

static const char *TAG = "MAIN";

static sdmmc_card_t *card = NULL;
static bool sd_card_ready = false;

static char current_log_path[128] = LOG_FILE_PATH;
static char current_setup_path[128] = LOG_SETUP_PATH;

static adc_oneshot_unit_handle_t adc1_handle;


/* ============================================================
 * LED
 * ============================================================ */

void led_init(void)
{
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << BLINK_LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&led_conf));

    gpio_set_level(BLINK_LED_PIN, 0);
}


/* ============================================================
 * ADC
 * ============================================================ */

void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(
            &init_config,
            &adc1_handle
        )
    );

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc1_handle,
            ADC_CHANNEL_0,       // GPIO36
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
    const int samples = 16;

    for (int i = 0; i < samples; i++)
    {
        sum += read_adc_raw();

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    return sum / samples;
}


/* ============================================================
 * Internal clock
 * ============================================================ */

void init_internal_clock(void)
{
    struct tm t = {
        .tm_year = 2026 - 1900,
        .tm_mon  = 6,
        .tm_mday = 9,
        .tm_hour = 17,
        .tm_min  = 5,
        .tm_sec  = 0
    };

    time_t t_of_day = mktime(&t);

    struct timeval tv = {
        .tv_sec = t_of_day,
        .tv_usec = 0
    };

    settimeofday(&tv, NULL);
}


/* ============================================================
 * SD card
 * ============================================================ */

void init_sd_card(void)
{
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

    ret = spi_bus_initialize(
        host.slot,
        &bus_cfg,
        SDSPI_DEFAULT_DMA
    );

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SPI bus.");
        return;
    }

    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_MISO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CS, GPIO_PULLUP_ONLY);

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();

    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(
        "/sdcard",
        &host,
        &slot_config,
        &mount_config,
        &card
    );

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount SD card.");
        sd_card_ready = false;
        return;
    }

    sd_card_ready = true;

    ESP_LOGI(TAG, "SD Card mounted successfully!");
}


/* ============================================================
 * Session folder
 * ============================================================ */

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
                {
                    next_session_id = current_id + 1;
                }
            }
        }

        closedir(dir);
    }

    char folder_path[64];

    snprintf(
        folder_path,
        sizeof(folder_path),
        "/sdcard/%d",
        next_session_id
    );

    if (mkdir(folder_path, 0777) != 0)
    {
        ESP_LOGE(
            TAG,
            "Failed to create directory %s",
            folder_path
        );

        return;
    }

    ESP_LOGI(
        TAG,
        "Created session folder %s",
        folder_path
    );

    snprintf(
        current_log_path,
        sizeof(current_log_path),
        "%s/sensor_log.txt",
        folder_path
    );

    snprintf(
        current_setup_path,
        sizeof(current_setup_path),
        "%s/sensor_data.txt",
        folder_path
    );


    /* Create sensor log */

    FILE *f = fopen(current_log_path, "w");

    if (f != NULL)
    {
        fprintf(
            f,
            "Timestamp,Sample_Index,"
            "Gyro_X,Gyro_Y,Gyro_Z,"
            "Accel_X,Accel_Y,Accel_Z\n"
        );

        fclose(f);
    }
    else
    {
        ESP_LOGE(
            TAG,
            "Failed to create %s",
            current_log_path
        );
    }


    /* Create setup file */

    f = fopen(current_setup_path, "w");

    if (f != NULL)
    {
        fclose(f);
    }
    else
    {
        ESP_LOGE(
            TAG,
            "Failed to create %s",
            current_setup_path
        );
    }
}


/* ============================================================
 * Log accelerometer configuration
 * ============================================================ */

void log_acc_info(const char *timestamp)
{
    if (!sd_card_ready)
        return;

    FILE *f = fopen(current_setup_path, "a");

    if (f == NULL)
    {
        ESP_LOGE(
            TAG,
            "Failed to open %s",
            current_setup_path
        );

        return;
    }

    fprintf(
        f,
        "Frequency: %d Hz\n"
        "I2C speed: %d Hz\n"
        "Samples per interrupt: %d\n"
        "Start time: %s\n",
        FREQ,
        I2C_MASTER_CLK_SPEED,
        NUM_SAMPLES_PER_BATCH,
        timestamp
    );

    fclose(f);
}


/* ============================================================
 * Log sensor event
 * ============================================================ */

void log_text_to_sd(
    const char *timestamp,
    const sensor_sample *samples,
    int count
)
{
    if (!sd_card_ready || count <= 0)
        return;

    FILE *f = fopen(current_log_path, "a");

    if (f == NULL)
    {
        ESP_LOGE(
            TAG,
            "Failed to open %s",
            current_log_path
        );

        return;
    }

    for (int i = 0; i < count; i++)
    {
        fprintf(
            f,
            "%s,%d,%d,%d,%d,%d,%d,%d\n",
            timestamp,
            i,

            samples[i].gyro_x,
            samples[i].gyro_y,
            samples[i].gyro_z,

            samples[i].accel_x,
            samples[i].accel_y,
            samples[i].accel_z
        );
    }

    fclose(f);
}


/* ============================================================
 * Main
 * ============================================================ */

void app_main(void)
{
    /* --------------------------------------------------------
     * LED
     * -------------------------------------------------------- */

    led_init();


    /* --------------------------------------------------------
     * ADC
     * -------------------------------------------------------- */

    adc_init();


    /* --------------------------------------------------------
     * Time / RTC
     * -------------------------------------------------------- */

    /*
     * Synchronize time from remote source first.
     *
     * Do NOT call init_internal_clock() afterwards because
     * that would overwrite the synchronized time.
     */
    get_time_remote();


    /* --------------------------------------------------------
     * SD card
     * -------------------------------------------------------- */

    init_sd_card();

    create_next_session_folder();


    /* --------------------------------------------------------
     * LSM6DS3
     * -------------------------------------------------------- */

    i2c_init();

    lsm6ds3_init();

    vTaskDelay(pdMS_TO_TICKS(15));

    interrupt_init();

    lsm6ds3_setup_continuous_with_wakeup();

    lsm6ds3_configure_motion_interrupt();


    /* --------------------------------------------------------
     * Print current time
     * -------------------------------------------------------- */

    print_time();


    /* --------------------------------------------------------
     * Buffers
     * -------------------------------------------------------- */

    static uint8_t dummy_buffer[4096];
    static uint8_t fifo_buffer[TOTAL_BYTES];

    uint8_t wakeup_status = 0;


    /* --------------------------------------------------------
     * Clear any previous interrupt
     * -------------------------------------------------------- */

    lsm6ds3_read_wakeup_source(&wakeup_status);

    lsm6ds3_reset_fifo();

    ESP_LOGI(
        TAG,
        "System Ready. Waiting for motion..."
    );


    /* --------------------------------------------------------
     * Initial FIFO flush
     *
     * Read FIFO word count in WORDS, then convert to BYTES.
     * -------------------------------------------------------- */

    uint16_t fifo_word_count =
        lsm6ds3_get_fifo_word_count();

    uint32_t fifo_bytes =
        (uint32_t)fifo_word_count * 2U;

    if (fifo_bytes > 0)
    {
        if (fifo_bytes > sizeof(dummy_buffer))
        {
            fifo_bytes = sizeof(dummy_buffer);
        }

        lsm6ds3_read_fifo(
            dummy_buffer,
            fifo_bytes
        );
    }


    ESP_LOGI(
        TAG,
        "Hardware flush completed. "
        "Entering operational loop..."
    );


    /* --------------------------------------------------------
     * Log system configuration
     * -------------------------------------------------------- */

    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];

    time(&now);

    localtime_r(
        &now,
        &timeinfo
    );

    strftime(
        strftime_buf,
        sizeof(strftime_buf),
        "%Y-%m-%d %H:%M:%S",
        &timeinfo
    );

    log_acc_info(strftime_buf);


    /* --------------------------------------------------------
     * First timer wakeup
     *
     * Wake after one minute.
     * After that, use one-hour intervals.
     * -------------------------------------------------------- */

    ESP_ERROR_CHECK(
        esp_sleep_enable_timer_wakeup(MIN_US)
    );


    /* ========================================================
     * Operational loop
     * ======================================================== */

    while (1)
    {
        /*
         * Enter light sleep.
         *
         * The LSM6DS3 interrupt should be able to wake the ESP32
         * when motion occurs.
         *
         * The timer also wakes the ESP32 periodically.
         */
        esp_light_sleep_start();


        /* ----------------------------------------------------
         * Determine why we woke up
         * ---------------------------------------------------- */

        esp_sleep_wakeup_cause_t cause =
            esp_sleep_get_wakeup_cause();


        /* ====================================================
         * TIMER WAKEUP
         * ==================================================== */

        if (cause == ESP_SLEEP_WAKEUP_TIMER)
        {
            ESP_LOGI(
                TAG,
                "Timer wakeup"
            );


            /* Read battery voltage */

            float voltage =
                ((float)read_adc() / 4095.0f)
                * 3.3f
                * 5.7f;

            ESP_LOGI(
                TAG,
                "Voltage: %.2f V",
                voltage
            );


            /* Send data */

            send_thingspeak_data(voltage);


            /*
             * Give communication enough time to finish.
             *
             * If send_thingspeak_data() is synchronous and
             * already waits for completion, this delay may not
             * be necessary.
             */
            vTaskDelay(pdMS_TO_TICKS(1000));


            /*
             * From now on, wake once per hour.
             */
            ESP_ERROR_CHECK(
                esp_sleep_enable_timer_wakeup(HOUR_US)
            );

            continue;
        }


        /* ====================================================
         * MOTION / GPIO WAKEUP
         * ==================================================== */

        gpio_set_level(
            BLINK_LED_PIN,
            1
        );


        /* ----------------------------------------------------
         * Clear interrupt source
         * ---------------------------------------------------- */

        lsm6ds3_read_wakeup_source(
            &wakeup_status
        );


        /* ----------------------------------------------------
         * Determine FIFO contents
         * ---------------------------------------------------- */

        uint16_t fifo_words =
            lsm6ds3_get_fifo_word_count();

        ESP_LOGI(
            TAG,
            "FIFO contains %u words (%u samples)",
            fifo_words,
            fifo_words / WORDS_PER_SAMPLE
        );


        /* ----------------------------------------------------
         * We need PRE_TRIGGER_SAMPLES.
         * Keep the newest samples.
         * ---------------------------------------------------- */

        if (fifo_words < PRE_TRIGGER_WORDS)
        {
            ESP_LOGW(
                TAG,
                "Not enough samples before trigger: "
                "%u / %u words",
                fifo_words,
                PRE_TRIGGER_WORDS
            );

            // lsm6ds3_reset_fifo();

            gpio_set_level(
                BLINK_LED_PIN,
                0
            );

            continue;
        }


        /* ----------------------------------------------------
         * Discard old samples.
         * ---------------------------------------------------- */

        uint16_t words_to_discard =
            fifo_words - PRE_TRIGGER_WORDS;

        uint32_t discard_bytes =
            (uint32_t)words_to_discard * 2U;


        if (discard_bytes > 0)
        {
            if (discard_bytes > sizeof(dummy_buffer))
            {
                ESP_LOGE(
                    TAG,
                    "Discard size exceeds buffer!"
                );

                lsm6ds3_reset_fifo();

                gpio_set_level(
                    BLINK_LED_PIN,
                    0
                );

                continue;
            }

            lsm6ds3_read_fifo(
                dummy_buffer,
                discard_bytes
            );
        }


        ESP_LOGI(
            TAG,
            "Waiting for post-trigger data..."
        );


        /* ----------------------------------------------------
         * Wait until PRE + POST samples exist.
         * ---------------------------------------------------- */

        while (
            lsm6ds3_get_fifo_word_count()
            < TOTAL_WORDS
        )
        {
            uint16_t words =
                lsm6ds3_get_fifo_word_count();

            ESP_LOGI(
                TAG,
                "FIFO: %u words (%u/%u samples)",
                words,
                words / WORDS_PER_SAMPLE,
                PRE_TRIGGER_SAMPLES +
                POST_TRIGGER_SAMPLES
            );

            vTaskDelay(
                pdMS_TO_TICKS(10)
            );
        }


        /* ----------------------------------------------------
         * Read complete event
         * ---------------------------------------------------- */

        ESP_ERROR_CHECK(
            lsm6ds3_read_fifo(
                fifo_buffer,
                TOTAL_BYTES
            )
        );


        /* ----------------------------------------------------
         * Reset FIFO
         * ---------------------------------------------------- */

        lsm6ds3_reset_fifo();


        /* ----------------------------------------------------
         * Timestamp event
         * ---------------------------------------------------- */

        time(&now);

        localtime_r(
            &now,
            &timeinfo
        );

        strftime(
            strftime_buf,
            sizeof(strftime_buf),
            "%Y-%m-%d %H:%M:%S",
            &timeinfo
        );


        /* ----------------------------------------------------
         * Save event
         * ---------------------------------------------------- */

        log_text_to_sd(
            strftime_buf,
            (const sensor_sample *)fifo_buffer,
            PRE_TRIGGER_SAMPLES +
            POST_TRIGGER_SAMPLES
        );


        /* ----------------------------------------------------
         * Event finished
         * ---------------------------------------------------- */

        gpio_set_level(
            BLINK_LED_PIN,
            0
        );

        vTaskDelay(
            pdMS_TO_TICKS(10)
        );
    }
}
