#include "WiFi.h"

static const char * TAG = "WiFi";

const char* TS_server = "api.thingspeak.com";

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

void thingspeak_send(float value)
{
    char body[64];

    snprintf(body,
            sizeof(body),
            "field2=%.2f",
            value);

    esp_http_client_config_t config = {
        .url = "http://api.thingspeak.com/update",
        .transport_type = HTTP_TRANSPORT_OVER_SSL
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    if(client == NULL)
    {
        ESP_LOGE(TAG, "HTTP init failed");
        return;
    }

    esp_http_client_set_method(client, HTTP_METHOD_POST);

    esp_http_client_set_header(client,
                            "X-THINGSPEAKAPIKEY",
                            THINGSPEAK_API_KEY);

    esp_http_client_set_header(client,
                            "Content-Type",
                            "application/x-www-form-urlencoded");

    esp_http_client_set_post_field(client,
                                body,
                                strlen(body));

    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG,
                "Status %d",
                esp_http_client_get_status_code(client));
    }

    esp_http_client_cleanup(client);
}

void send_thingspeak_data(float value) {
    esp_wifi_start();
    ESP_ERROR_CHECK(esp_wifi_connect());
    wait_for_wifi();
    vTaskDelay(pdMS_TO_TICKS(3000));
    thingspeak_send(value);
    esp_wifi_disconnect();
    esp_wifi_stop();
}