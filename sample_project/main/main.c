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
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_err.h"



#define GPIO_OUTPUT_IO_0   2
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)
#define GPIO_INPUT_IO_0    21
#define GPIO_INPUT_IO_1    22
#define GPIO_INPUT_IO_2    23
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2))
#define ESP_INTR_FLAG_DEFAULT 0

#define LEDC_HS_TIMER   LEDC_TIMER_0
#define LEDC_HS_MODE    LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_CHANNEL LEDC_CHANNEL_0
#define LEDC_HS_CH0_GPIO    18
 

static const char* TAG = "SYS INFO"; 

static QueueHandle_t gpio_evt_queue = NULL; /* Queue handler */

static TimerHandle_t mudanca_estado_timer = NULL;
static bool mudanca_estado = false; // Variável global para controlar a mudança de estado
static bool estado = false; // Estado atual do LED (true: aceso, false: apagado)

//static const char *TAG = "example";

typedef struct {
    uint64_t current_count;
    uint64_t alarm_value;
} timer_queue_element_t;

typedef struct {
    int hours;
    int minutes;
    int seconds;
} real_time_clock_t;

void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);  /* Envia o número do GPIO para a fila */
}

static bool IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data){
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;
    
    // Retrieve count value and send to queue
    timer_queue_element_t ele = {
        .current_count = edata->count_value,
        .alarm_value = edata->alarm_value
    };
    xQueueSendFromISR(queue, &ele, &high_task_awoken);

    // Reconfigure alarm value for next interrupt
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + 100000, // alarm in next 100ms
    };
    gptimer_set_alarm_action(timer, &alarm_config);

    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}



void mudanca_estado_timer_callback(TimerHandle_t xTimer) {
    mudanca_estado = false; // Permite mudanças de estado novamente
}

static void gpio_task(void* arg){  // Função Prática II - Acender e apagar
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) { /* Detecção do evento na porta, determinação da task */
            bool acende = true;
            bool apaga = false;

            ESP_LOGI(TAG, "YAY \n");
            if (io_num == GPIO_INPUT_IO_0) { // Botão 1 - Acender
                estado = true;
                gpio_set_level(GPIO_OUTPUT_IO_0, acende);
                ESP_LOGI(TAG, "Acendeu");
            } else if (io_num == GPIO_INPUT_IO_1) { // Botão 2 - Apagar
                estado = false;
                gpio_set_level(GPIO_OUTPUT_IO_0, apaga);
                ESP_LOGI(TAG, "Apagou");
            } else if (io_num == GPIO_INPUT_IO_2) { // Botão 3 - Switch
                if (!mudanca_estado) { // Se o LED não estiver em transição
                    mudanca_estado = true; // Indica que o LED está em transição

                    if (estado == true) { // Se o LED está aceso, apaga-o
                        estado = false;
                        gpio_set_level(GPIO_OUTPUT_IO_0, apaga);
                        ESP_LOGI(TAG, "Aceso -> Apagado");
                    } else { // Se o LED está apagado, acende-o
                        estado = true;
                        gpio_set_level(GPIO_OUTPUT_IO_0, acende);
                        ESP_LOGI(TAG, "Apagado -> Aceso");
                    }
                    // Inicia o temporizador para limpar o estado de mudança após um curto período
                    xTimerStart(mudanca_estado_timer, 0); // Temporizador de 1000 ms
                }
            }
        } 
    }
}

void timer_task(void *arg){ // Função Prática III - Timer
    QueueHandle_t queue = (QueueHandle_t)arg;
    timer_queue_element_t ele;
    real_time_clock_t rtc = {0, 0, 0};
    uint64_t last_second = 0;
    while (1) {
        if (xQueueReceive(queue, &ele, portMAX_DELAY)) {
            uint64_t count_in_seconds = ele.current_count / 1000000;
            rtc.hours = (count_in_seconds / 3600) % 24;
            rtc.minutes = (count_in_seconds / 60) % 60;
            rtc.seconds = count_in_seconds % 60;
            if (rtc.seconds != last_second) {
                ESP_LOGI(TAG, "Time: %02d:%02d:%02d, Current count: %llu",
                rtc.hours, rtc.minutes, rtc.seconds, ele.current_count);
                last_second = rtc.seconds;
            }
        }
    }
}

gptimer_handle_t gptimer = NULL;
gptimer_config_t timer_config = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000, // 1MHz, 1 tick=1us
};

gptimer_event_callbacks_t cbs = {
    .on_alarm = timer_on_alarm_cb,
};

gptimer_alarm_config_t alarm_config = {
    .alarm_count = 100000, // period = 100ms
};

static void example_ledc_init(void){
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,            // frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,       // timer mode
        .timer_num = LEDC_HS_TIMER,       // timer index
        .clk_cfg = LEDC_AUTO_CLK,        // Auto select the source clock
    };


    ledc_channel_config_t ledc_channel = {
        .channel  = LEDC_HS_CH0_CHANNEL,
        .duty    = 0,
        .gpio_num  = LEDC_HS_CH0_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint   = 0,
        .timer_sel = LEDC_HS_TIMER,
        .flags.output_invert = 0
    };
}

void app_main(void) {

    // ------------------------------------------------------------ Prática I e II ----------------------------------------------------------- //

    // Cria uma fila para lidar com eventos GPIO a partir da ISR
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    mudanca_estado_timer = xTimerCreate("mudanca_estado_timer", pdMS_TO_TICKS(300), pdFALSE, (void*)0, mudanca_estado_timer_callback);

    ESP_LOGI(TAG, "Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);

    ESP_LOGI(TAG, "This is %s chip with %d CPU core(s), model %d, WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           chip_info.model,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG, "silicon revision v%d.%d, ", major_rev, minor_rev);

    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        ESP_LOGE(TAG, "Get flash size failed");
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
    in_conf.intr_type = GPIO_INTR_ANYEDGE; /* Configura interrupção em ambas as bordas */
    in_conf.mode = GPIO_MODE_INPUT;
    in_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    in_conf.pull_down_en = 0;
    in_conf.pull_up_en = 1;
    gpio_config(&in_conf);

    // Inicializa a task GPIO
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    // Instala o serviço de interrupção GPIO
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // Adiciona manipuladores de interrupção para pinos específicos
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2);

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

// --------------------------------------------------------------- Prática III -------------------------------------------------------------- //

    timer_queue_element_t ele;
    QueueHandle_t queue = xQueueCreate(10, sizeof(timer_queue_element_t));
    if (!queue) {
        ESP_LOGE(TAG, "Creating queue failed");
        return;
    }
    xTaskCreate(timer_task, "timer_task", 2048, queue, 10, NULL);

    ESP_LOGI(TAG, "Create timer handle");
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, queue));
    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_LOGI(TAG, "Start timer, update alarm value dynamically");
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

// --------------------------------------------------------------- Prática IV -------------------------------------------------------------- //

    example_ledc_init();


}
