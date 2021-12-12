#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_now.h"
#include "MAX30100.h"
#include "driver/gpio.h"
#include "MHETLive.h"


static const char *TAG = "MHETLive";

static void NVS_init(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
}


void app_main(void)
{
    NVS_init();       
   

    MAX30100_init();


    for(;;) {        
        vTaskDelay(2000 / portTICK_PERIOD_MS);   
        ESP_LOGI(TAG, "Still alive");
    }
    
    //never reach this code
    fflush(stdout);
    esp_restart();
}
