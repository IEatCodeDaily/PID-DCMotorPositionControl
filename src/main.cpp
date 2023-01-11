#include "main.h"

//Logging
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
static const char *TAG = "MAIN";

//FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//ADC
#include "driver/adc.h"
#include "esp_adc_cal.h"


//=============== ADC ================//


adc1_channel_t list_adc[] = {OMEGA_FEEDBACK_ADC, OMEGA_INPUT_ADC, KP_ADC, KI_ADC, KD_ADC};
gpio_num_t list_gpio[] = {OMEGA_FEEDBACK_PIN, OMEGA_INPUT_PIN, KP_PIN, KI_PIN, KD_PIN};



static esp_adc_cal_characteristics_t adc1_chars;

bool adc_init(){

    esp_log_level_set("*", ESP_LOG_VERBOSE);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc1_chars);
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    for (adc1_channel_t ch : list_adc){
        ESP_ERROR_CHECK(adc1_config_channel_atten(ch, ADC_ATTEN_DB_11));
    }
    ESP_LOGI(TAG, "ADC initialized");
    return true;   
}

int adc_read(adc1_channel_t ch){
    return adc1_get_raw(ch);
}

//=============== PWM ================//


//=============== RTOS ================//



//=============== MAIN ================//
extern "C" void app_main() {
    adc_init();
    ESP_LOGD(TAG, "System initialized");
    while(true){
        for (adc1_channel_t ch : list_adc){
            ESP_LOGD(TAG, "ADC %d: %d", ch, adc_read(ch));
            //printf("ADC %d: %d\n", ch, adc_read(ch));
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
}