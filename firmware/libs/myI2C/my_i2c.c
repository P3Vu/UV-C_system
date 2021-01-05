/**
  ******************************************************************************
  * @file       main.c
  * @author     Pawel Wojciechowski
  * @version    V.1.0.0
  * @date       5-December-2020
  ******************************************************************************
  */

#include <stdio.h>
#include <stdbool.h>
#include "my_i2c.h"
#include "driver/i2c.h"

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
