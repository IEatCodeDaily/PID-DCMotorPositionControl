#include "main.h"

//Logging
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
static const char *SYS = "SYSTEM";
static const char *COEF = "COEF";
static const char *MOTOR = "MOTOR";
static const char *DISPLAY = "DISPLAY";

//Set Debug
#define LOG_LEVEL_COEF ESP_LOG_INFO
#define LOG_LEVEL_MOTOR ESP_LOG_INFO
#define LOG_LEVEL_DISPLAY ESP_LOG_INFO

//FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//ADC
#include "driver/adc.h"
#include "esp_adc_cal.h"

//Motor PWM
#include "driver/mcpwm.h"

//Display
#include "driver/i2c.h"
#include "lcd.h"

//Global Variables
float coef[COEF_COUNT];     //init coef globally for PID
int error;                  //Error value, global so LCD can print it
float u = 0;                //Input for plant, global for LCD

//=============== ADC ================//

adc1_channel_t list_adc[] = {OMEGA_FEEDBACK_ADC, OMEGA_INPUT_ADC, KP_ADC, KI_ADC, KD_ADC};
gpio_num_t list_gpio[] = {OMEGA_FEEDBACK_PIN, OMEGA_INPUT_PIN, KP_PIN, KI_PIN, KD_PIN};

static esp_adc_cal_characteristics_t adc1_chars;

bool adc_init(){
    ESP_LOGI(SYS, "Initializing ADC...");

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc1_chars);
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    for (int i = 0; i < 5; i++){
        ESP_ERROR_CHECK(adc1_config_channel_atten(list_adc[i], ADC_ATTEN_DB_11));
    }

    ESP_LOGI(SYS, "ADC initialized");
    return true;   
}


int adc_read(adc1_channel_t ch){
    int adc = 0;
    for (int i = 0; i < N_SAMPLE; i++){
        adc += adc1_get_raw(ch);
    }
    return (adc/N_SAMPLE);
}

void coef_read(){
    for (int i = 0; i < COEF_COUNT; i++){
        coef[i] = adc_read(list_adc[i+2])/4095.0; //Normalized to 0-1
    }
    //Value adjustment
    coef[0] = coef[0]*2;
    coef[1] = coef[1]/5000;
    coef[2] = coef[2]*5;
}

//=============== PWM ================//
bool mcpwm_gpio_assign()
{
    ESP_LOGI(SYS, "Assigning GPIO pins to MCPWM...");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_IN1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B,MOTOR_IN2);
    ESP_LOGI(SYS, "GPIO pins assigned to MCPWM");

    return true;
}

void mcpwm_run(float u){
    if (u > 0){
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, u);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
    else if (u < 0){
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, -u);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else{
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    }
}

//=============== DISPLAY ================//


lcd_handle_t lcd_handle = LCD_HANDLE_DEFAULT_CONFIG();

static void display_init(void)
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    // Initialise i2c
    ESP_LOGD(DISPLAY, "Installing i2c driver in master mode on channel %d", I2C_MASTER_NUM);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGD(DISPLAY,
             "Configuring i2c parameters.\n\tMode: %d\n\tSDA pin:%d\n\tSCL pin:%d\n\tSDA pullup:%d\n\tSCL pullup:%d\n\tClock speed:%.3fkHz",
             i2c_config.mode, i2c_config.sda_io_num, i2c_config.scl_io_num,
             i2c_config.sda_pullup_en, i2c_config.scl_pullup_en,
             i2c_config.master.clk_speed / 1000.0);
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_config));

    // Modify default lcd_handle details
    lcd_handle.i2c_port = I2C_MASTER_NUM;
    lcd_handle.address = LCD_ADDR;
    lcd_handle.columns = LCD_COLUMNS;
    lcd_handle.rows = LCD_ROWS;
    lcd_handle.backlight = LCD_BACKLIGHT;

    // Initialise LCD
    ESP_ERROR_CHECK(lcd_init(&lcd_handle));

    return;
}



//=============== RTOS ================//
//Read coefficient every 200ms
void task_coef(void *pvParameters){
    adc_init();
    while(true){
        coef_read();
        ESP_LOGD(COEF, "KP: %.2f, KI: %.2f, KD: %.2f", coef[0], coef[1], coef[2]);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    vTaskDelete(NULL);
}

//update motor position every 10ms
void task_motor(void *pvParameters){
    ESP_LOGI(SYS, "Initializing motor control...");
    
    int error_prev = 0, error_sum = 0;
    //int last_time;                      //For PID

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    
    mcpwm_gpio_assign();
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    
    ESP_LOGI(SYS, "Motor control initialized");
    
    while(true){
        //insert motor control code here
        
        //read error and integrate
        error = adc_read(OMEGA_INPUT_ADC) - adc_read(OMEGA_FEEDBACK_ADC);
        ESP_LOGD(MOTOR, "Input: %d, Feedback: %d, Error: %d", adc_read(OMEGA_INPUT_ADC), adc_read(OMEGA_FEEDBACK_ADC), error);
        error_sum += error; 

        //PID calculation
        u = coef[0]*error + coef[1]*error_sum + coef[2]*(error - error_prev); 

        ESP_LOGD(MOTOR, "u: %.2f", u);
        //set motor speed
        mcpwm_run(u);


        //update error_prev for next iteration
        error_prev = error;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

//Update display every 250ms
void task_display(void *pvParameters){
    char buf[32];
    ESP_LOGI(SYS, "Initializing display...");
    display_init();

    lcd_clear_screen(&lcd_handle);
    lcd_write_str(&lcd_handle,"Initializing System");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(SYS, "Display initialized");
    while(true){
        ESP_LOGD(DISPLAY, "Updating display...");
        //lcd_clear_screen(&lcd_handle);
        
    
        sprintf(buf,"error:%d   ", error);
        lcd_write_str(&lcd_handle,buf);
        ESP_LOGD(DISPLAY, "Wrote to display: %s", buf);

        lcd_set_cursor(&lcd_handle, 0,1);
        sprintf(buf,"K %.1f %s %.1f", coef[0], coef[1].substr(1,5), coef[2]);
        lcd_write_str(&lcd_handle, buf);
        ESP_LOGD(DISPLAY, "Wrote to display: %s", buf);

        vTaskDelay(pdMS_TO_TICKS(500));
        
    }
    vTaskDelete(NULL);
}
//=============== MAIN ================//
void app_main() {
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    esp_log_level_set("DISPLAY", LOG_LEVEL_DISPLAY);
    esp_log_level_set("MOTOR", LOG_LEVEL_MOTOR);
    esp_log_level_set("COEF", LOG_LEVEL_COEF);


    xTaskCreate(task_coef, "Read Coefficient", 2048, NULL, 4, NULL);
    xTaskCreate(task_motor, "Motor Control", 4096, NULL, 5, NULL);
    xTaskCreate(task_display, "Display", 4096, NULL, 3, NULL);
    ESP_LOGI(SYS, "System initialized");
    vTaskDelete(NULL);
}