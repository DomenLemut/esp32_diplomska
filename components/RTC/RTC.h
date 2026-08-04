#pragma once

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "lwip/apps/sntp.h"

#define WIFI_SSID "S'z br'z drata"
#define WIFI_PASS "Na Gradiscu je lepo 2008."

#include "RTC.h"

void print_time(void);
void get_time_string(char *buffer);
void get_time_remote(void);