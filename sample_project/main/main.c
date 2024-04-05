/*
 * Alunos:
 *              @arthurWielr
 *              @MauricioBSouza
 *              
 * 
 * Professor:
 *              @tuliocharles
 * 
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"

static const char* TAG = "SYS INFO"; 


void app_main(void){
    
    ESP_LOGI(TAG,"Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG,"This is %s chip with %d CPU core(s), mmodel %d, WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           chip_info.model,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG,"silicon revision v%d.%d, ", major_rev, minor_rev);

    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        ESP_LOGE(TAG,"Get flash size failed");
        return;
    }


}
