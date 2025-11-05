#ifndef _ICM_20948_I2C_H_
#define _ICM_20948_I2C_H_

#include "driver/i2c_master.h"
#include "icm20948.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

typedef struct
{
    i2c_master_dev_handle_t i2c_dev;  // Novo handle do dispositivo I2C
    uint8_t i2c_addr;                 // Endereço I2C (mantido para referência)
} icm0948_config_i2c_t;

void icm20948_init_i2c(icm20948_device_t *device, icm0948_config_i2c_t *config);

icm20948_status_e icm20948_internal_write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user);
icm20948_status_e icm20948_internal_read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user);

#endif