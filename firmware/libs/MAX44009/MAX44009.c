/**
  ******************************************************************************
  * @file       main.c
  * @author     Pawel Wojciechowski
  * @version    V.1.0.0
  * @date       12-December-2020
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "MAX44009.h"
#include "my_i2c.h"
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/** Measures ambient light in luxes */
int LUXMeas(void){

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
