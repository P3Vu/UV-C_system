/**
  ******************************************************************************
  * @file       main.c
  * @author     Pawel Wojciechowski
  ******************************************************************************
  */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"



#define LAMP1 GPIO_NUM_2
#define LAMP2 GPIO_NUM_0
#define LAMP3 GPIO_NUM_18
#define LAMP4 GPIO_NUM_19

#define ON      0
#define OFF     1

static const char* TAG = "Module";

/** Configure GPIO's for relay switch */
static void configGPIO(void){

    int err = 0;

    err = gpio_set_direction(LAMP1, GPIO_MODE_OUTPUT);
    if(err != ESP_OK) {
            ESP_LOGW(TAG, "Fail to configure Switch Relay GPIO's at GPIO : %d", LAMP1);
            return;
    }
    gpio_set_level(LAMP1, OFF);

    err = gpio_set_direction(LAMP2, GPIO_MODE_OUTPUT);
    if(err != ESP_OK){
        ESP_LOGW(TAG, "Fail to configure Switch Relay GPIO's at GPIO : %d", LAMP2);
        return;
    }
    gpio_set_level(LAMP2, OFF);

    err = gpio_set_direction(LAMP3, GPIO_MODE_OUTPUT);
    if(err != ESP_OK){
        ESP_LOGW(TAG, "Fail to configure Switch Relay GPIO's at GPIO : %d", LAMP3);
        return;
    }
    gpio_set_level(LAMP3, OFF);

    err = gpio_set_direction(LAMP4, GPIO_MODE_OUTPUT);
    if(err != ESP_OK){
        ESP_LOGW(TAG, "Fail to configure Switch Relay GPIO's at GPIO : %d", LAMP4);
        return;
    }
    gpio_set_level(LAMP4, OFF);

    ESP_LOGI(TAG, "Switch Relay GPIO's config success.");
    return;
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

void app_main()
{
    configGPIO();

}


