#ifndef SAVE_SEND_H
#define SAVE_SEND_H

#include "common.h"

#include "sys/stat.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "unistd.h"
#include "sdmmc_cmd.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "esp_littlefs.h"
#include "driver/uart.h"

#define LORA_BAUD_RATE 115200
#define LORA_UART_NUM UART_NUM_2
#define LORA_TX_RETRIES 2
#define CONFIG_E220_TX_DONE_TIMEOUT_MS 1000 
#define CONFIG_E220_AUX_TIMEOUT_MS 3000 

#define MAX_USED 0.8 // Maximum percentage of flash to be used by littlefs
#define FILENAME_LENGTH 32
#define E220_BAUD_RATE 115200
#define SD_MOUNT "/sdcard"
#define SD_TRANSF_SIZE 4000
#define SD_MAX_FILES 5
#define SD_UNIT_SIZE 16 * 1024

#endif

void task_sd(void *pvParameters);
void task_littlefs(void *pvParameters);
void task_lora(void *pvParameters);