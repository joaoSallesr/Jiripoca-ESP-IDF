#ifndef GLOBAL_H
#define GLOBAL_H

#ifndef HEADER_H
#define HEADER_H
#endif

#pragma once

#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ============================================================================
// Stub: ESP-IDF & FreeRTOS types and macros
// ============================================================================
// These replace the real ESP-IDF/FreeRTOS headers so tests compile natively
// on the host (Windows/Linux) without the full ESP-IDF toolchain.

// -- esp_err / esp_check --
typedef int esp_err_t;
#define ESP_OK   0
#define ESP_FAIL (-1)

#define ESP_RETURN_ON_ERROR(x, tag, fmt, ...) \
    do {                                      \
        if ((x) != 0)                         \
            return (x);                       \
    } while (0)

// -- esp_log --
#define ESP_LOGW(tag, fmt, ...) (void)0
#define ESP_LOGI(tag, fmt, ...) (void)0
#define ESP_LOGE(tag, fmt, ...) (void)0
#define ESP_LOGD(tag, fmt, ...) (void)0
#define ESP_ERROR_CHECK(x)      (void)(x)

// -- esp_timer --
typedef void *esp_timer_handle_t;
static inline int64_t esp_timer_get_time(void) { return 0; }
static inline esp_err_t esp_timer_start_once(esp_timer_handle_t timer, uint64_t timeout_us) {
    (void)timer;
    (void)timeout_us;
    return ESP_OK;
}

// -- esp_bit_defs --
#define BIT0   (1UL << 0)
#define BIT1   (1UL << 1)
#define BIT(x) (1UL << (x))

// -- GPIO --
typedef int gpio_num_t;
#define GPIO_NUM_0  0
#define GPIO_NUM_4  4
#define GPIO_NUM_5  5
#define GPIO_NUM_6  6
#define GPIO_NUM_8  8
#define GPIO_NUM_9  9
#define GPIO_NUM_15 15
#define GPIO_NUM_38 38
#define GPIO_NUM_47 47
#define GPIO_NUM_48 48
#define HIGH 1
#define LOW  0

#define gpio_set_level(gpio, level) (void)0
#define gpio_get_level(gpio)        (0)

// -- FreeRTOS core types --
typedef uint8_t  StackType_t;
typedef uint8_t  BaseType_t;
typedef uint32_t UBaseType_t;
typedef uint32_t TickType_t;

// Prevent real FreeRTOS headers from being included
#define configNUMBER_OF_CORES 1
#define portBYTE_ALIGNMENT    16

typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED (0)
#define portENTER_CRITICAL(mux)      (void)0
#define portEXIT_CRITICAL(mux)       (void)0

#define pdTRUE            1
#define pdFALSE           0
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portMAX_DELAY     0xFFFFFFFF

// -- FreeRTOS task --
typedef void *TaskHandle_t;
#define vTaskDelay(ticks)              (void)0
#define vTaskDelete(task)              (void)0
#define xTaskNotifyWait(a, b, c, d)    (0)
#define xTaskNotifyGive(task)          (void)0
#define ulTaskNotifyTake(clearOnExit, ticks) (0)

// -- FreeRTOS queue --
typedef void *QueueHandle_t;
#define xQueueSend(q, d, t)   (0)
#define xQueueOverwrite(q, d) (0)

// -- FreeRTOS semaphore --
typedef void *SemaphoreHandle_t;
#define xSemaphoreTake(sem, ticks) (1)
#define xSemaphoreGive(sem)        (void)0

// -- FreeRTOS task notify --
typedef uint32_t eNotifyAction;
#define eSetBits ((eNotifyAction)0)
#define xTaskNotify(task, value, action) (void)0

// -- FreeRTOS task timing --
static inline TickType_t xTaskGetTickCount(void) { return 0; }
static inline void xTaskDelayUntil(TickType_t *const pxPreviousWakeTime, const TickType_t xTimeIncrement) {
    (void)pxPreviousWakeTime;
    (void)xTimeIncrement;
}

// -- FreeRTOS event group --
typedef void *EventGroupHandle_t;

// -- FreeRTOS ring buffer --
typedef void *RingbufHandle_t;

// -- I2C bus handle --
typedef void *i2c_master_bus_handle_t;

// -- esp_err / assert compatibility --
#define likely(x)   (x)
#define unlikely(x) (x)
#define __ASSERT_FUNC __func__
static inline void _esp_error_check_failed(esp_err_t err, const char *file, int line, const char *func, const char *expr) {
    (void)err;
    (void)file;
    (void)line;
    (void)func;
    (void)expr;
}

// ============================================================================
// Application constants
// ============================================================================

#define BOOT_GPIO   GPIO_NUM_0
#define RBF_GPIO    GPIO_NUM_4
#define VIN_GPIO    GPIO_NUM_5
#define ALED_GPIO   GPIO_NUM_6
#define LED_GPIO    GPIO_NUM_15
#define BUZZER_GPIO GPIO_NUM_38
#define DROGUE_GPIO GPIO_NUM_47
#define MAIN_GPIO   GPIO_NUM_48

#define ICM20948_I2C_ADDRESS 0x68
#define I2C_SPEED 400000
#define G 9.80665f // m/s²
#define ICM_SAMPLE_RATE_S 0.01f
#define BMP_SAMPLE_RATE_MS 50
#define GPS_SAMPLE_RATE_MS 200
#define MAG_SCALE 0.15f
#define TEMP_SCALE 1.0f / 333.87f
#define TEMP_OFFSET 21.0f
#define GYRO_SCALE 1.0f / 65.5f
#define ACC_SCALE 1.0f / 2048.0f

// ============================================================================
// Sensor bits
// ============================================================================
enum SENSOR_BIT {
    ICM_BIT,
    BMP_BIT,
    GPS_BIT,
    ADC_BIT,
};

// ============================================================================
// Init flags
// ============================================================================
#define BMP_INIT  BIT(0)
#define MS_INIT   BIT(1)
#define LSM_INIT  BIT(2)
#define BMM_INIT  BIT(3)
#define HX_INIT   BIT(4)
#define SD_INIT   BIT(5)
#define LFS_INIT  BIT(6)
#define LORA_INIT BIT(7)
#define GPS_INIT  BIT(8)

// ============================================================================
// Flight states
// ============================================================================
typedef enum __attribute__((packed)) {
    STATE_IDLE = 0,
    STATE_BOOST,
    STATE_COAST,
    STATE_DROGUE,
    STATE_MAIN,
    STATE_LANDING,
    STATE_LANDED,
    STATE_FATAL,
} flight_state_t;

// ============================================================================
// Status flags
// ============================================================================
#define STATUS_INITIALIZED     BIT(0)
#define STATUS_ARMED           BIT(1)
#define STATUS_DROGUE_DEPLOYED BIT(2)
#define STATUS_MAIN_DEPLOYED   BIT(3)
#define STATUS_NVS_EDIT        BIT(4)

// ============================================================================
// Sensor sample types
// ============================================================================
typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t mag_x, mag_y, mag_z;
    float   q1, q2, q3, q4;
    float   az;
    uint8_t accel;
} icm20948_sample_t;

typedef struct {
    float    latitude;
    float    longitude;
    float    altitude;
    float    vel_vertical;
    uint32_t utc_time;
    uint8_t  sAcc;
    uint8_t  fix;
} gps_sample_t;

typedef struct {
    float altitude;
    float pressure;
} bmp390_sample_t;

// ============================================================================
// ESKF stub (matches real eskf_t layout needed by acquire.c)
// ============================================================================
typedef struct {
    float h;
    float vz;
    float apogee;
} eskf_state_t;

typedef struct {
    eskf_state_t x;
} eskf_t;

static inline bool eskf_predict(eskf_t *kf, float az, float xQ) {
    (void)kf;
    (void)az;
    (void)xQ;
    return true;
}
static inline bool eskf_update_bar(eskf_t *kf, float alt, float r) {
    (void)kf;
    (void)alt;
    (void)r;
    return true;
}
static inline bool eskf_update_gps(eskf_t *kf, float alt, float vz, float xR) {
    (void)kf;
    (void)alt;
    (void)vz;
    (void)xR;
    return true;
}

// ============================================================================
// Data structures (match real header.h)
// ============================================================================
typedef struct {
    uint32_t time;

    icm20948_sample_t icm;
    gps_sample_t      gps;
    bmp390_sample_t   bmp;
    eskf_t            kf;
    uint8_t           voltage;

    _Atomic flight_state_t flight_state;
} data_t;

typedef struct __attribute__((packed)) {
    uint32_t time;

    float   pressure;
    float   latitude, longitude;
    float   gps_altitude, gps_vel_vertical;
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t mag_x, mag_y, mag_z;
    uint8_t voltage;
    uint8_t sAcc;
    uint8_t fix;

    _Atomic flight_state_t flight_state;
} save_t;

typedef struct __attribute__((packed)) {
    uint32_t time;

    float    latitude, longitude;
    uint16_t kf_altitude, kf_apogee;
    int16_t  kf_vel_vertical;
    int16_t  q1, q2, q3, q4;
    uint8_t  accel;
    uint8_t  voltage;
    uint8_t  fix;

    _Atomic flight_state_t flight_state;
} send_t;

typedef struct __attribute__((packed)) {
    uint32_t sd_files;
    uint32_t lfs_files;
    bool     format;
} file_counter_t;

typedef struct __attribute__((packed)) {
    gpio_num_t         gpio;
    const char        *label;
    esp_timer_handle_t timer;
} pyro_channel_t;

// ============================================================================
// ICM20948 / VQF stubs for host-side unit tests
// ============================================================================

typedef enum {
    ICM_20948_STAT_OK = 0x00,
    ICM_20948_STAT_ERR,
    ICM_20948_STAT_NOT_IMPL,
    ICM_20948_STAT_PARAM_ERR,
    ICM_20948_STAT_WRONG_ID,
    ICM_20948_STAT_INVALID_SENSOR,
    ICM_20948_STAT_NO_DATA,
    ICM_20948_STAT_SENSOR_NOT_SUPPORTED,
    ICM_20948_STAT_DMP_NOT_SUPPORTED,
    ICM_20948_STAT_DMP_VERIFY_FAIL,
    ICM_20948_STAT_FIFO_NO_DATA_AVAIL,
    ICM_20948_STAT_FIFO_INCOMPLETE_DATA,
    ICM_20948_STAT_FIFO_MORE_DATA_AVAIL,
    ICM_20948_STAT_UNRECOGNISED_DMP_HEADER,
    ICM_20948_STAT_UNRECOGNISED_DMP_HEADER_2,
    ICM_20948_STAT_INVALID_DMP_REGISTER,
    ICM_20948_STAT_NUM,
    ICM_20948_STAT_UNKNOWN,
} icm20948_status_e;

typedef enum {
    SAMPLE_MODE_CONTINUOUS = 0x00,
    SAMPLE_MODE_CYCLED,
} icm20948_lp_config_cycle_e;

typedef enum {
    CLOCK_INTERNAL_20MHZ = 0x00,
    CLOCK_AUTO,
    CLOCK_TIMING_RESET = 0x07,
} icm20948_pwr_mgmt_1_clksel_e;

typedef enum {
    ICM_20948_INTERNAL_ACC = (1 << 0),
    ICM_20948_INTERNAL_GYR = (1 << 1),
    ICM_20948_INTERNAL_MAG = (1 << 2),
    ICM_20948_INTERNAL_TMP = (1 << 3),
    ICM_20948_INTERNAL_MST = (1 << 4),
} icm20948_internal_sensor_id_bm;

typedef union {
    int16_t i16bit[3];
    uint8_t  u8bit[6];
} icm20948_axis3bit16_t;

typedef union {
    int16_t i16bit;
    uint8_t u8bit[2];
} icm20948_axis1bit16_t;

typedef struct {
    uint8_t a : 2;
    uint8_t g : 2;
    uint8_t reserved_0 : 4;
} icm20948_fss_t;

typedef struct {
    uint8_t a;
    uint8_t g;
} icm20948_dlpcfg_t;

typedef struct {
    uint16_t a;
    uint8_t g;
} icm20948_smplrt_t;

typedef struct {
    uint8_t i2c_addr;
    uint32_t i2c_clock_speed;
} icm20948_config_i2c_t;

typedef union {
    icm20948_axis3bit16_t raw;
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } axes;
} icm20948_axis3named_t;

typedef struct {
    icm20948_axis3named_t acc;
    icm20948_axis3named_t gyr;
    icm20948_axis3named_t mag;
    union {
        icm20948_axis1bit16_t raw;
        int16_t val;
    } tmp;
    icm20948_fss_t fss;
    uint8_t magStat1;
    uint8_t magStat2;
} icm20948_agmt_t;

typedef struct {
    const void * _serif;
    bool _dmp_firmware_available;
    bool _firmware_loaded;
    uint8_t _last_bank;
    uint8_t _last_mems_bank;
    int32_t _gyroSF;
    int8_t _gyroSFpll;
    uint32_t _enabled_Android_0;
    uint32_t _enabled_Android_1;
    uint32_t _enabled_Android_intr_0;
    uint32_t _enabled_Android_intr_1;
    uint16_t _dataOutCtl1;
    uint16_t _dataOutCtl2;
    uint16_t _dataRdyStatus;
    uint16_t _motionEventCtl;
    uint16_t _dataIntrCtl;
} icm20948_device_t;

typedef enum {
    GPM_2 = 0x00,
    GPM_4,
    GPM_8,
    GPM_16,
} icm20948_accel_config_fs_sel_e;

typedef enum {
    DPS_250 = 0x00,
    DPS_500,
    DPS_1000,
    DPS_2000,
} icm20948_gyro_config_1_fs_sel_e;

typedef enum {
    ACC_D246BW_N265BW = 0x00,
    ACC_D246BW_N265BW_1,
    ACC_D111BW4_N136BW,
    ACC_D50BW4_N68BW8,
    ACC_D23BW9_N34BW4,
    ACC_D11BW5_N17BW,
    ACC_D5BW7_N8BW3,
    ACC_D473BW_N499BW,
} icm20948_accel_config_dlpcfg_e;

typedef enum {
    GYR_D196BW6_N229BW8 = 0x00,
    GYR_D151BW8_N187BW6,
    GYR_D119BW5_B154BW3,
    GYR_D51BW2_N73BW3,
    GYR_D23BW9_N35BW9,
    GYR_D11BW6_N17BW8,
    GYR_D5BW7_N8BW9,
    GYR_D361BW4_N376BW5,
} icm20948_gyro_config_1_dlpcfg_e;

static inline icm20948_status_e icm20948_init_i2c(void *bus, const icm20948_config_i2c_t *cfg, icm20948_device_t *dev) {
    (void)bus; (void)cfg; (void)dev; return ICM_20948_STAT_OK;
}
static inline icm20948_status_e icm20948_check_id(icm20948_device_t *dev) { (void)dev; return ICM_20948_STAT_OK; }
static inline icm20948_status_e icm20948_sw_reset(icm20948_device_t *dev) { (void)dev; return ICM_20948_STAT_OK; }
static inline icm20948_status_e icm20948_sleep(icm20948_device_t *dev, bool on) { (void)dev; (void)on; return ICM_20948_STAT_OK; }
static inline icm20948_status_e icm20948_low_power(icm20948_device_t *dev, bool on) { (void)dev; (void)on; return ICM_20948_STAT_OK; }
static inline icm20948_status_e icm20948_set_sample_mode(icm20948_device_t *dev, icm20948_internal_sensor_id_bm sensors, icm20948_lp_config_cycle_e mode) {
    (void)dev; (void)sensors; (void)mode; return ICM_20948_STAT_OK;
}
static inline icm20948_status_e icm20948_set_sample_rate(icm20948_device_t *dev, icm20948_internal_sensor_id_bm sensors, icm20948_smplrt_t smplrt) {
    (void)dev; (void)sensors; (void)smplrt; return ICM_20948_STAT_OK;
}
static inline icm20948_status_e icm20948_set_full_scale(icm20948_device_t *dev, icm20948_internal_sensor_id_bm sensors, icm20948_fss_t full_scale) {
    (void)dev; (void)sensors; (void)full_scale; return ICM_20948_STAT_OK;
}
static inline icm20948_status_e icm20948_set_dlpf_cfg(icm20948_device_t *dev, icm20948_internal_sensor_id_bm sensors, icm20948_dlpcfg_t dlpf) {
    (void)dev; (void)sensors; (void)dlpf; return ICM_20948_STAT_OK;
}
static inline icm20948_status_e icm20948_enable_dlpf(icm20948_device_t *dev, icm20948_internal_sensor_id_bm sensor, bool enable) {
    (void)dev; (void)sensor; (void)enable; return ICM_20948_STAT_OK;
}
static inline icm20948_status_e icm20948_init_magnetometer(icm20948_device_t *dev) { (void)dev; return ICM_20948_STAT_OK; }
static inline icm20948_status_e icm20948_get_agmt(icm20948_device_t *dev, icm20948_agmt_t *agmt) { (void)dev; (void)agmt; return ICM_20948_STAT_OK; }

typedef struct vqf_params_s {
    float tauAcc;
    float tauMag;
    int motionBiasEstEnabled;
    int restBiasEstEnabled;
    int magDistRejectionEnabled;
    float biasSigmaInit;
    float biasForgettingTime;
    float biasClip;
    float biasSigmaMotion;
    float biasVerticalForgettingFactor;
    float biasSigmaRest;
    float restMinT;
    float restFilterTau;
    float restThGyr;
    float restThAcc;
    float magCurrentTau;
    float magRefTau;
    float magNormTh;
    float magDipTh;
    float magNewMinGyr;
} vqf_params_t;

typedef struct vqf_handle_t {
    int placeholder;
} vqf_handle_t;

static inline void vqf_params_init(vqf_params_t *params) {
    memset(params, 0, sizeof(*params));
    params->tauAcc = 3.0f;
    params->tauMag = 9.0f;
    params->motionBiasEstEnabled = 1;
    params->restBiasEstEnabled = 1;
    params->magDistRejectionEnabled = 1;
    params->biasSigmaInit = 0.5f;
    params->biasForgettingTime = 100.0f;
    params->biasClip = 2.0f;
    params->biasSigmaMotion = 0.1f;
    params->biasVerticalForgettingFactor = 0.0001f;
    params->biasSigmaRest = 0.03f;
    params->restMinT = 1.5f;
    params->restFilterTau = 0.5f;
    params->restThGyr = 2.0f;
    params->restThAcc = 0.5f;
    params->magCurrentTau = 0.05f;
    params->magRefTau = 20.0f;
    params->magNormTh = 0.1f;
    params->magDipTh = 10.0f;
    params->magNewMinGyr = 0.0f;
}

static inline vqf_handle_t *vqf_init_custom(const vqf_params_t *params, float gyrTs, float accTs, float magTs) {
    (void)params; (void)gyrTs; (void)accTs; (void)magTs;
    return calloc(1, sizeof(vqf_handle_t));
}

static inline void vqf_update(vqf_handle_t *handle, const float gyr[3], const float acc[3], const float mag[3]) {
    (void)handle; (void)gyr; (void)acc; (void)mag;
}

static inline void vqf_get_quat9D(vqf_handle_t *handle, float out[4]) {
    (void)handle;
    out[0] = 1.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;
    out[3] = 0.0f;
}

static inline void vqf_delete(vqf_handle_t *handle) { free(handle); }

// ============================================================================
// BMP390 driver stubs (replaces the real esp-idf-bmp390 library)
// ============================================================================
typedef void *bmp390_handle_t;

typedef struct { uint8_t i2c_address; uint32_t i2c_clock_speed; int power_mode;
    int iir_filter; int pressure_oversampling; int temperature_oversampling; int output_data_rate;
} bmp390_config_t;

typedef struct { uint8_t reg; } bmp390_power_control_register_t;
typedef struct { uint8_t reg; } bmp390_configuration_register_t;
typedef struct { uint8_t reg; } bmp390_oversampling_register_t;
typedef struct { uint8_t reg; } bmp390_output_data_rate_register_t;
typedef struct { uint8_t reg; } bmp390_interrupt_control_register_t;

#define BMP390_I2C_ADDRESS                0x77
#define I2C_SPEED                         400000
#define BMP390_POWER_MODE_NORMAL          0
#define BMP390_IIR_FILTER_1               1
#define BMP390_PRESSURE_OVERSAMPLING_4X   4
#define BMP390_TEMPERATURE_OVERSAMPLING_2X 2
#define BMP390_ODR_20MS                   20

static inline esp_err_t bmp390_init(void *bus, bmp390_config_t *cfg, bmp390_handle_t *hdl)
    { (void)bus; (void)cfg; (void)hdl; return ESP_OK; }
static inline esp_err_t bmp390_set_power_mode(bmp390_handle_t h, int mode)
    { (void)h; (void)mode; return ESP_OK; }
static inline esp_err_t bmp390_get_measurements(bmp390_handle_t h, float *temp, float *pres)
    { (void)h; (void)temp; (void)pres; return ESP_OK; }
static inline esp_err_t bmp390_delete(bmp390_handle_t h) { (void)h; return ESP_OK; }
static inline void bmp390_get_configuration_register(bmp390_handle_t h, bmp390_configuration_register_t *r) { (void)h; (void)r; }
static inline void bmp390_get_oversampling_register(bmp390_handle_t h, bmp390_oversampling_register_t *r) { (void)h; (void)r; }
static inline void bmp390_get_power_control_register(bmp390_handle_t h, bmp390_power_control_register_t *r) { (void)h; (void)r; }
static inline void bmp390_get_output_data_rate_register(bmp390_handle_t h, bmp390_output_data_rate_register_t *r) { (void)h; (void)r; }
static inline void bmp390_get_interrupt_control_register(bmp390_handle_t h, bmp390_interrupt_control_register_t *r) { (void)h; (void)r; }

static void *bus_handle = NULL;

// ============================================================================
// Hardware constants (override via compile flags or keep defaults here)
// ============================================================================
#ifndef KNOWN_TEMPERATURE
#define KNOWN_TEMPERATURE 25.0f  // °C
#endif
#ifndef KNOWN_ALTITUDE
#define KNOWN_ALTITUDE    0.0f   // metres
#endif
#ifndef BMP_SAMPLE_RATE_MS
#define BMP_SAMPLE_RATE_MS 20
#endif

// ============================================================================
// Utility stubs
// ============================================================================
static inline const char *uint32_to_binary(uint32_t val) {
    (void)val;
    return "";
}
static inline const char *uint8_to_binary(uint8_t val) {
    (void)val;
    return "";
}

// ============================================================================
// Global variables (static for test isolation)
// ============================================================================
static data_t            data_g;
static bmp390_sample_t   bmp_sample_g;
static icm20948_sample_t icm_sample_g;
static gps_sample_t      gps_sample_g;
static uint8_t           battery_voltage_g;
static _Atomic uint8_t   sys_flags_g;
static atomic_bool       lfs_full;

static pyro_channel_t pyro_drogue;
static pyro_channel_t pyro_main;

static QueueHandle_t     xB4LaunchQueue, xSDQueue, xLittleFSQueue, xLoraQueue;
static portMUX_TYPE      xDATALock, xBMPMutex, xICMMutex, xGPSMutex, xADCMutex;
static TaskHandle_t      xTaskFlightControl, xTaskAcquire, xTaskLora;

// ============================================================================
// Function declarations for code under test
// ============================================================================
void boost_status_check(data_t *data);

#endif // GLOBAL_H
