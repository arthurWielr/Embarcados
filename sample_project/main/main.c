/*
 * Alunos:
 *              @arthurWielr
 *              @MauricioBSouza
 *              @RaiterJ
 * Professor:
 *              @tuliocharles
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/queue.h"
#include "driver/gpio.h"


#define GPIO_OUTPUT_IO_0    02

#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)
#define GPIO_INPUT_IO_0    21
#define GPIO_INPUT_IO_1    22
#define GPIO_INPUT_IO_2    23
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2))
//#define GPIO_INPUT_PIN_CHANGE   ((1ULL<<GPIO_INPUT_IO_2))
#define ESP_INTR_FLAG_DEFAULT 0



static const char* TAG = "SYS INFO"; 

static QueueHandle_t gpio_evt_queue = NULL; /* Queue handler */

static void IRAM_ATTR gpio_isr_handler(void* arg){
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);  /* Definição do block pela queue e pelo numero do da porta */
}

static void gpio_task(void* arg){
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) { /* Detecção do evento na porta, determinação da task */
            static bool mudanca_estado = false;
            bool acende = true;
            bool apaga = false;
            bool estado =  false; //True aceso, false apagado;
            ESP_LOGI(TAG,"YAY \n");
            if(io_num == 21){ //Botão 1 - Acender
                estado = true;
                gpio_set_level(GPIO_OUTPUT_IO_0, acende);
                ESP_LOGI(TAG,"Acendeu")
            } 
            if(io_num == 22){ // Botão 2 - Apagar
                estado = false;
                gpio_set_level(GPIO_OUTPUT_IO_0, apaga);
                ESP_LOGI(TAG,"Apagou")
            } 

            if (io_num == 23) { // Botão 3 - Switch
                if (!mudanca_estado) { // Se o LED não estiver em transição
                    mudanca_estado = true; // Indica que o LED está em transição
                    
                    if (estado == true) { // Se o LED está aceso, apaga-o
                        estado = false;
                        gpio_set_level(GPIO_OUTPUT_IO_0, apaga);
                        ESP_LOGI(TAG,"Aceso -> Apagado")
                    } else { // Se o LED está apagado, acende-o
                        estado = true;
                        gpio_set_level(GPIO_OUTPUT_IO_0, acende);
                        ESP_LOGI(TAG,"Apagado -> Aceso")
                    }
                    // Define um timer para limpar o estado de mudança após um curto período
                    xTimerStartOneShot(mudanca_estado_timer, pdMS_TO_TICKS(100)); // Timer de 100 ms
                }
            }
        } 
    }
}



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

    gpio_config_t out_conf = {};
    out_conf.intr_type = GPIO_INTR_DISABLE; /* Desabilita a interrupção do sistema */
    out_conf.mode = GPIO_MODE_OUTPUT;
    out_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    out_conf.pull_down_en = 0;
    out_conf.pull_up_en = 0;
    gpio_config(&out_conf);

    gpio_config_t in_conf = {};
    in_conf.intr_type = GPIO_INTR_ANYEDGE; /* Desabilita a interrupção do sistema */
    in_conf.mode = GPIO_MODE_INPUT;
    in_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    in_conf.pull_down_en = 0;
    in_conf.pull_up_en = 1;
    gpio_config(&in_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);  /* Inicializa a task*/

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2);


    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());
      
       
    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}