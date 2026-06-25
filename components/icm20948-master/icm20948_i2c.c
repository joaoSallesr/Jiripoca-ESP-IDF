#include "icm20948.h"
#include "icm20948_i2c.h"

icm20948_status_e icm20948_internal_write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
    icm20948_status_e status = ICM_20948_STAT_OK;
    icm0948_config_i2c_t *args = (icm0948_config_i2c_t*)user;
    
    uint8_t *write_data = malloc(len + 1);
    if (write_data == NULL) {
        return ICM_20948_STAT_ERR;
    }
    
    write_data[0] = reg;
    memcpy(&write_data[1], data, len);
    
    esp_err_t ret = i2c_master_transmit(args->i2c_dev, write_data, len + 1, -1);
    if (ret != ESP_OK) {
        status = ICM_20948_STAT_ERR;
    }
    
    free(write_data);
    return status;
}

icm20948_status_e icm20948_internal_read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
    icm20948_status_e status = ICM_20948_STAT_OK;
    icm0948_config_i2c_t *args = (icm0948_config_i2c_t*)user;
    
    esp_err_t ret = i2c_master_transmit(args->i2c_dev, &reg, 1, -1);
    if (ret != ESP_OK) {
        return ICM_20948_STAT_ERR;
    }
    
    ret = i2c_master_receive(args->i2c_dev, buff, len, -1);
    if (ret != ESP_OK) {
        status = ICM_20948_STAT_ERR;
    }
    
    return status;
}

icm20948_serif_t default_serif = {
    icm20948_internal_write_i2c,
    icm20948_internal_read_i2c,
    NULL,
};

void icm20948_init_i2c(icm20948_device_t *icm_device, icm0948_config_i2c_t *args)
{
    icm20948_init_struct(icm_device);
    default_serif.user = (void *)args;
    icm20948_link_serif(icm_device, &default_serif);

    // Inicializações internas do driver
    icm_device->_dmp_firmware_available = false;
    icm_device->_firmware_loaded = false;
    icm_device->_last_bank = 255;
    icm_device->_last_mems_bank = 255;
    icm_device->_gyroSF = 0;
    icm_device->_gyroSFpll = 0;
    icm_device->_enabled_Android_0 = 0;
    icm_device->_enabled_Android_1 = 0;
    icm_device->_enabled_Android_intr_0 = 0;
    icm_device->_enabled_Android_intr_1 = 0;
}