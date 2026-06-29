#pragma once

#include "esp_err.h"
#include <stdint.h>

#define I2C_NUM I2C_NUM_0
#define LSM6DS3_ADDR 0x6A

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define LSM6DS3_INT2                14
#define LSM6DS3_INT1                27

#define NUM_SAMPLES_PER_BATCH   20
#define BYTES_PER_SAMPLE    18
#define FIFO_THR_BYTES (BYTES_PER_SAMPLE * NUM_SAMPLES_PER_BATCH)
#define FIFO_THR (FIFO_THR_BYTES / 2)

#define SAMPLE_DIV 1

esp_err_t i2c_init(void);
esp_err_t i2c_write_reg(uint8_t reg, uint8_t data);
esp_err_t i2c_read_reg(uint8_t reg, uint8_t *data);

esp_err_t lsm6ds3_init(void);
esp_err_t lsm6ds3_start(void);
esp_err_t lsm6ds3_configure_motion_interrupt(void);
esp_err_t lsm6ds3_setup(void);
esp_err_t lsm6ds3_read_fifo(uint8_t *data, size_t len);

bool lsm6ds_check_int1(void);
bool lsm6ds_check_int2(void);
esp_err_t interrupt_init(void);