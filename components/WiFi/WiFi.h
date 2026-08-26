#pragma once

#include "esp_http_client.h"
#include "esp_log.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "lwip/apps/sntp.h"

#define WIFI_SSID "MD Ventus Vipava"
#define WIFI_PASS "ventus17"

#define THINGSPEAK_API_KEY "W18MNTONIL6YN9SH"

void wifi_init(void);
void wait_for_wifi(void);
void send_thingspeak_data(float value);
