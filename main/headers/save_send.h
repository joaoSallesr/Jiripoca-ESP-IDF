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

#define MAX_USED 0.8 // Maximum percentage of flash to be used by littlefs
#define E220_BAUD_RATE 115200
#define SD_MOUNT "/sdcard"
#define SD_TRANSF_SIZE 4000
#define SD_MAX_FILES 5
#define SD_UNIT_SIZE 16 * 1024


#endif

void task_sd(void *pvParameters);
void task_littlefs(void *pvParameters);