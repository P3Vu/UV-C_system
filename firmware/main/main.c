/**
  ******************************************************************************
  * @file       main.c
  * @author     Pawel Wojciechowski
  * @version    V.1.0.0
  * @date       29-November-2020
  ******************************************************************************
  */

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "math.h"

#define LAMP1                       GPIO_NUM_2                    /* RELAY IN1 on board */
#define LAMP2                       GPIO_NUM_0                    /* RELAY IN2 on board */
#define LAMP3                       GPIO_NUM_18                   /* RELAY IN3 on board */
#define LAMP4                       GPIO_NUM_19                   /* RELAY IN4 on board */
#define I2C_MASTER_SCL_IO                GPIO_NUM_22                   /* Master */
#define I2C_MASTER_SDA_IO                 GPIO_NUM_21

#define GPIO_OUTPUT_PIN_SEL ((1ULL<<GPIO_NUM_2) | (1ULL<<GPIO_NUM_0) | (1ULL<<GPIO_NUM_18) | (1ULL<<GPIO_NUM_19))

#define MOTION_SENSOR               GPIO_NUM_13               /* Microwave Motion Sensor */
#define GPIO_INPUT_PIN_SEL         (1ULL<<MOTION_SENSOR)
#define ESP_INTR_FLAG_DEFAULT           0

#define LIGHT_SENSOR_ADDR   0x4A
#define LHB_addr            0x03                    /* Lux High Byte address */
#define LLB_addr            0x04                    /* Lux Low Byte address */

#define I2C_WRITE_ADDR   0x00
#define I2C_READ_ADDR    0x01


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

/** Initializes a I2C config */
void i2c_init (void) {
  int i2c_master_port = 0;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 100000;
  i2c_param_config(i2c_master_port, &conf);
  i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

/** Reads a singular byte from particular address */
uint8_t i2c_read_byte (uint8_t slave_addr, uint8_t addr) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, slave_addr|I2C_WRITE_ADDR, 1);
  i2c_master_write_byte(cmd, addr, 1);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, slave_addr|I2C_READ_ADDR, 1);

  uint8_t data;
  i2c_master_read_byte(cmd, &data, 1);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(0, cmd, 1000/portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return data;
}

/** Read 2 bytes in 1 transmission */
bool i2c_read_2bytes (uint8_t slave_addr, uint8_t addr, uint8_t* buf) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, slave_addr|I2C_WRITE_ADDR, 0);
  i2c_master_write_byte(cmd, 0x03, 0);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, slave_addr|I2C_READ_ADDR, 0);
  i2c_master_read_byte(cmd, buf, 0);

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, slave_addr|I2C_WRITE_ADDR, 0);
  i2c_master_write_byte(cmd, 0x04, 0);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, slave_addr|I2C_READ_ADDR, 0);
  i2c_master_read_byte(cmd, buf+1, 1);
  i2c_master_stop(cmd);

  i2c_master_cmd_begin(0, cmd, 1000/portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  return true;
}

/** Measures ambient light in luxes */
static int LUXMeas(void){

    uint8_t slave_addr = LIGHT_SENSOR_ADDR;
    uint8_t read_addr  = LHB_addr;
    slave_addr = slave_addr << 1;

    uint8_t buf[2] = {};

    i2c_read_2bytes(slave_addr, read_addr, buf);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    /* Calculate the readings */
    uint8_t exponent = (buf[0] & 0xF0) >> 4;
    uint8_t mantissa = ((buf[0] & 0x0F) << 4)| (buf[1] & 0x0F);
    float calc = pow(2, exponent) * mantissa * 0.72;
    int lux = (int)calc;

    //printf("Exp = %d | Mantissa  = %d | Lux = %f \n", exponent, mantissa , lux);
    return lux;
}

void app_main()
{
    configGPIO();
    configADC();
    i2c_init();

    int light = 0;

    while(1){
        light = LUXMeas();
        printf("Lux = %d\n", light);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}


