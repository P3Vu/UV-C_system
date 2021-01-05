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
#include "ML8511.h"
#include "driver/adc.h"

/** Function measures UV intensity form ML8511 sensor by ADC
    @return
        - UV Sensor reading in [mW/cm^2]
*/
float UVmeas(void){

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
