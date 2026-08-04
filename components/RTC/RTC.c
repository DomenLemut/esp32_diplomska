#include "RTC.h"
#include "esp_log.h"

static const char *TAG = "TIME";

void wifi_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(
        esp_wifi_init(&cfg)
    );

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        }
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    ESP_LOGI(TAG, "Connecting to WiFi...");
}


void wait_for_wifi(void)
{
    wifi_ap_record_t ap_info;

    while (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK)
    {
        ESP_LOGI(TAG, "Waiting for WiFi...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "WiFi connected");
}

void init_sntp(void)
{
    ESP_LOGI(TAG, "Starting SNTP");

    sntp_setoperatingmode(SNTP_OPMODE_POLL);

    sntp_setservername(0, "pool.ntp.org");

    sntp_init(); // <---- Blocking function for setting time
}

void wait_for_time(void)
{
    time_t now = 0;
    struct tm timeinfo = {0};

    while (timeinfo.tm_year < (2020 - 1900))
    {
        time(&now);

        localtime_r(&now, &timeinfo);

        ESP_LOGI(TAG, "Waiting for NTP time...");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    ESP_LOGI(TAG, "Time synchronized");
}

void print_time(void)
{
    time_t now;
    struct tm timeinfo;
    char buffer[64];

    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);

    ESP_LOGI(TAG, "Current time: %s", buffer);
}

void get_time_string(char *buffer) {
    time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
}

void get_time_remote(void) {
    wifi_init();

    wait_for_wifi();
    init_sntp();
    wait_for_time();

    setenv(
        "TZ",
        "CET-1CEST,M3.5.0,M10.5.0/3",
        1
    );
    tzset();
    sntp_stop();
    esp_wifi_disconnect();
    esp_wifi_stop();
}
