#include "RTC.h"
#include "WiFi.h"
#include "esp_log.h"

static const char *TAG = "TIME";

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
