#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"

#include "RTC.h"

void app_main(void)
{
    get_time_remote();

    while (1)
    {
        print_time();

        vTaskDelay(
            pdMS_TO_TICKS(60000)
        );
    }
}