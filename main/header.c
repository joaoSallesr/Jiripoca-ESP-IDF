#include "header.h"

i2c_master_bus_handle_t bus_handle = NULL;

/* i2c bus configuration */
i2c_master_bus_config_t bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = I2C_SCL_IO,
    .sda_io_num = I2C_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true
};

/* ICM 20948 configuration */
i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM_20948_I2C_ADDR_AD0,
        .scl_speed_hz = 400000,
};

/* BMP 390 configuration */
i2c_device_config_t bmp_config = {
    .device_address = BMP390_I2C_ADDRESS,
    .scl_speed_hz = I2C_SPEED
};

uart_config_t uart_config = {
    .baud_rate = GPS_BAUDRATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
};

// Kalman Filter configuration
altitude_config_t kf_config = {
        .dt = 0.01f,
        .g = G,
        .accel_var = 0.01f,
        .gps_var_alt = 15.0f,
        .gps_var_vel = 0.01f,
        .bar_var = 0.07f,
        .acc_bias_var = 0.001f,
        .angular_error_var = 0.005f,
        .use_gps_mahalanobis = false
    };

TaskHandle_t xTaskLora = NULL;

// Queues
QueueHandle_t xAltQueue = NULL;
QueueHandle_t xLittleFSQueue = NULL;
QueueHandle_t xSDQueue = NULL;
QueueHandle_t xLoraQueue = NULL;

// Mutexes
SemaphoreHandle_t xGPSMutex = NULL;
SemaphoreHandle_t xStatusMutex = NULL;
SemaphoreHandle_t xI2CMutex = NULL;

// Status
uint16_t STATUS = 0;
uint32_t utc_time = 1; // keep 1 for the logic
float initial_temp = 0.0f;
float bmp_initial_alt = 0.0f;
float gps_initial_alt = 0.0f;