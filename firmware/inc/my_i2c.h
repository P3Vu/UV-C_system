/**
  ******************************************************************************
  * @file       main.c
  * @author     Pawel Wojciechowski
  * @version    V.1.0.0
  * @date       5-December-2020
  ******************************************************************************
  */

#define I2C_MASTER_SCL_IO           GPIO_NUM_22                   /* I2C Master SCL */
#define I2C_MASTER_SDA_IO           GPIO_NUM_21                   /* I2C Master SDA */

#define I2C_WRITE_ADDR   0x00
#define I2C_READ_ADDR    0x01

void i2c_init(void);
uint8_t i2c_read_byte (uint8_t, uint8_t );
bool i2c_read_2bytes (uint8_t, uint8_t, uint8_t*);
