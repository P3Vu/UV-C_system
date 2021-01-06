/**
  ******************************************************************************
  * @file       main.c
  * @author     Pawel Wojciechowski
  * @version    V.1.0.0
  * @date       12-December-2020
  ******************************************************************************
  */

#define LIGHT_SENSOR_ADDR           0x4A
#define LHB_addr                    0x03                        /* Lux High Byte address */
#define LLB_addr                    0x04                        /* Lux Low Byte address */

int LUXMeas(void);
