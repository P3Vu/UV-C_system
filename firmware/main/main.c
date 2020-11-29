/**
  ******************************************************************************
  * @file       main.c
  * @author     Pawel Wojciechowski
  ******************************************************************************
  */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/adc.h"



#define LAMP1           GPIO_NUM_2                    /* RELAY IN1 on board */
#define LAMP2           GPIO_NUM_0                    /* RELAY IN2 on board */
#define LAMP3           GPIO_NUM_18                   /* RELAY IN3 on board */
#define LAMP4           GPIO_NUM_19                   /* RELAY IN4 on board */
#define GPIO_OUTPUT_PIN_SEL ((1ULL<<GPIO_NUM_2) | (1ULL<<GPIO_NUM_0) | (1ULL<<GPIO_NUM_18) | (1ULL<<GPIO_NUM_19))

#define MOTION_SENSOR   GPIO_NUM_13               /* Microwave Motion Sensor */
#define GPIO_INPUT_PIN_SEL (1ULL<<MOTION_SENSOR)
#define ESP_INTR_FLAG_DEFAULT 0

#define ON      0
#define OFF     1

#define ADC_AVR_SAMPLES 64          /* Number of samples avereged for UV sensor */

static xQueueHandle gpio_evt_queue = NULL;
static const char* TAG = "Module";

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

/** Configure GPIO's for relay switch */
static void configGPIO(void){

    gpio_config_t io_conf;

    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that want to set
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins that will be inputs
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "Motion_Task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(MOTION_SENSOR, gpio_isr_handler, (void*) MOTION_SENSOR);
    // remove isr handler for gpio number
    gpio_isr_handler_remove(MOTION_SENSOR);
    // hook isr handler for specific gpio pin again
    gpio_isr_handler_add(MOTION_SENSOR, gpio_isr_handler, (void*) MOTION_SENSOR);

    ESP_LOGI(TAG, "GPIO Config finished.");
}
/** Turn On the Lamp */
static void TurnOnLamp(int gpioNum){
    int err = 0;
    err = gpio_set_level(gpioNum, ON);
    if(err != ESP_OK) ESP_LOGW(TAG, "Fail to set Lamp On at GPIO: %d", gpioNum);

    return;
}

/** Turn Off the Lamp */
static void TurnOffLamp(int gpioNum){
    int err = 0;
    err = gpio_set_level(gpioNum, OFF);
    if(err != ESP_OK) ESP_LOGW(TAG, "Fail to set Lamp Off at GPIO: %d", gpioNum);

    return;
}

/** Configures the ADC peripheral for UV sensor */
static void configADC(void){
    esp_err_t err;

    err = adc1_config_width(ADC_WIDTH_BIT_12);
    if(err != ESP_OK){
        ESP_LOGW(TAG, "Fail to config ADC Width");
        return;
    }

    err = adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    if(err != ESP_OK){
        ESP_LOGW(TAG, "Fail to config ADC channel attenuation");
    }

    ESP_LOGI(TAG, "ADC Config finished.");
    return;
}

/** Function measures UV intensity form ML8511 sensor by ADC
    @return
        - UV Sensor reading in [mW/cm^2]
*/
static float UVmeas(void){

        float vol = 0;
        float intensity = 0;
        int avr = 0;
        int sum = 0;
        uint8_t counter;

        /* Average samples */
        for(counter = 0; counter < ADC_AVR_SAMPLES; counter++){
            sum += adc1_get_raw(ADC1_CHANNEL_0);
        }

        avr = sum / 64;
        /* Prints raw adc value */
        //printf("avr value = %d", avr);

        vol = 3.3 * ((float)avr/4095);
        //printf("Voltage = %d\n", voltage);

        /* The pattern below is brought from ML8511 sensor characteristic figured
            in datasheet on Output Voltage - UV Intensity Characteristic */
        intensity = 8*(vol-1);  /* in [mW/cm^2] */
        if(intensity < 0) intensity = 0;

        return intensity;
}

void app_main()
{
    configGPIO();
    configADC();

}


