#include "global.h"

static const char *TAG_SD  = "SD";
static const char *TAG_LFS = "LittleFS";
static const char *TAG_NVS = "NVS";

void task_sd(void *pvParameters) {
    const file_counter_t counter = *(file_counter_t *)pvParameters;
    esp_err_t            errSD;
    sdmmc_card_t        *card;

    // Settings for mounting FAT filesystem
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files              = MAX_SD_FILES,
        .allocation_unit_size   = SD_UNIT_SIZE,
    };
    // Set CONFIG_SD_FORMAT_IF_MOUNT_FAILED to TRUE or FALSE
    // When format_if_mount_failed is set to true, SD card will be partitioned and formatted

    ESP_LOGI(TAG_SD, "Initializing SD card");

    // Settings for initializing SPI bus
    spi_bus_config_t bus_config = {
        .mosi_io_num     = MOSI,
        .miso_io_num     = MISO,
        .sclk_io_num     = SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = SD_BUFFER_SIZE,
    };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    // SPI initializer
    ESP_LOGD(TAG_SD, "Using SPI peripheral");
    errSD = spi_bus_initialize(host.slot, &bus_config, SDSPI_DEFAULT_DMA);
    if (errSD != ESP_OK) {
        ESP_LOGE(TAG_SD, "Failed to initialize SPI bus: %s", esp_err_to_name(errSD));
        vTaskDelete(NULL);
    }
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs               = SS;
    slot_config.host_id               = host.slot;
    ESP_LOGD(TAG_SD, "SPI bus initialized");

    // Mount filesystem
    ESP_LOGD(TAG_SD, "Mounting filesystem");
    errSD = esp_vfs_fat_sdspi_mount(SD_MOUNT, &host, &slot_config, &mount_config, &card);
    if (errSD != ESP_OK) {
        if (errSD == ESP_FAIL)
            ESP_LOGE(
                TAG_SD,
                "Failed to mount filesystem. "
                "If you want the card to be formatted, set the CONFIG_SD_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        else
            ESP_LOGE(TAG_SD, "Failed to initialize the card: %s. ", esp_err_to_name(errSD));
        spi_bus_free(host.slot);
        ESP_LOGD(TAG_SD, "SPI bus freed");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG_SD, "Filesystem mounted");

    // Format mode
    if (counter.format == pdTRUE) {
        ESP_LOGW(TAG_SD, "Format mode enabled, formatting SD card");
        errSD = esp_vfs_fat_sdcard_format(SD_MOUNT, card);
        if (errSD != ESP_OK)
            ESP_LOGE(TAG_SD, "Failed to format FATFS: %s", esp_err_to_name(errSD));
        else
            ESP_LOGI(TAG_SD, "Format Successful");

        esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
        ESP_LOGI(TAG_SD, "Card unmounted");
        spi_bus_free(host.slot);
        ESP_LOGI(TAG_SD, "SPI bus freed");
        xEventGroupSetBits(xFormatEventGroup, EVT_SD_DONE);
        vTaskDelete(NULL);
    }

    // Print sd card info
    sdmmc_card_print_info(stdout, card);

    // Create log file
    char log_name[FILENAME_LENGTH];
    snprintf(log_name, FILENAME_LENGTH, "%s/flight%ld.bin", SD_MOUNT, counter.sd_files);
    ESP_LOGI(TAG_SD, "Creating file %s", log_name);

    FILE *f = fopen(log_name, "wb");
    if (!f) {
        ESP_LOGE(TAG_SD, "Failed to open file for writing");
        esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
        ESP_LOGI(TAG_SD, "Card unmounted");
        spi_bus_free(host.slot);
        ESP_LOGI(TAG_SD, "SPI bus freed");
        vTaskDelete(NULL);
    }

    static uint8_t write_buffer[SD_BUFFER_SIZE];
    uint16_t       buffer_offset = 0;
    save_t         save_data;
    TickType_t     last_sync = xTaskGetTickCount();

    while (true) {
        // Read data from queue
        if (xQueueReceive(xSDQueue, &save_data, portMAX_DELAY) == pdTRUE) {
            // If buffer is full, write to file
            if (buffer_offset + sizeof(save_t) > SD_BUFFER_SIZE) {
                size_t w = fwrite(write_buffer, 1, SD_BUFFER_SIZE, f);
                if (w != SD_BUFFER_SIZE)
                    ESP_LOGE(TAG_SD, "Failed to write data to file");
                else
                    ESP_LOGD(TAG_SD, "Data written to SD card");
                buffer_offset = 0; // Reset buffer index for next batch
                taskYIELD();       // Yield to allow other tasks to run

                if (xTaskGetTickCount() - last_sync >= pdMS_TO_TICKS(5000)) // Flushes every 5s
                {
                    fflush(f);
                    fsync(fileno(f));
                    last_sync = xTaskGetTickCount();
                    ESP_LOGD(TAG_SD, "File flushed");
                    taskYIELD(); // Yield to allow other tasks to run
                }
            }
            // Copy remaining data to buffer
            memcpy(&write_buffer[buffer_offset], &save_data, sizeof(save_t));
            buffer_offset += sizeof(save_t);
        }

        // Check if landing
        portENTER_CRITICAL(&xDATAMutex);
        bool landing = (data_g.status & LANDING);
        portEXIT_CRITICAL(&xDATAMutex);
        if (landing)
            break;
    }

    if (buffer_offset > 0) // Write remaining data to file
    {
        size_t w = fwrite(write_buffer, 1, buffer_offset, f);
        if (w != buffer_offset)
            ESP_LOGE(TAG_SD, "Failed to write remaining data to file");
        else
            ESP_LOGD(TAG_SD, "Remaining data written to SD card");
    }

    while (xQueueReceive(xB4LaunchQueue, &save_data, 0) == pdTRUE) // Write queue data to file
    {
        size_t w = fwrite(&save_data, 1, sizeof(save_t), f);
        if (w != sizeof(save_t))
            ESP_LOGE(TAG_SD, "Failed to write before launch data to file");
        else
            ESP_LOGD(TAG_SD, "Before launch data written to SD card");
    }

    ESP_LOGW(TAG_SD, "Landed, closing file and unmounting SD card");
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    taskYIELD();                   // Yield to allow other tasks to run
    vTaskDelay(pdMS_TO_TICKS(20)); // Short delay to ensure SPI driver is done
    ESP_LOGI(TAG_SD, "File closed");
    esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
    ESP_LOGI(TAG_SD, "Card unmounted");
    spi_bus_free(host.slot);
    ESP_LOGI(TAG_SD, "SPI bus freed");
    xEventGroupSetBits(xNVSCounterEventGroup, EVT_SD_DONE); // Signal that SD task is done

    vTaskDelete(NULL);
}

void task_lfs(void *pvParameters) {
    const file_counter_t counter = *(file_counter_t *)pvParameters;
    esp_err_t            errFS;

    // Settings for initializing LittleFS
    esp_vfs_littlefs_conf_t littlefs_config = {
        .base_path              = "/littlefs",
        .partition_label        = "littlefs",
        .format_if_mount_failed = true,
        .dont_mount             = false,
    };

    ESP_LOGI(TAG_LFS, "Initializing LittleFS");
    errFS = esp_vfs_littlefs_register(&littlefs_config);
    if (errFS != ESP_OK) {
        if (errFS == ESP_FAIL)
            ESP_LOGE(TAG_LFS, "Failed to mount or format filesystem");
        else if (errFS == ESP_ERR_NOT_FOUND)
            ESP_LOGE(TAG_LFS, "Failed to find LittleFS partition");
        else
            ESP_LOGE(TAG_LFS, "Failed to initialize LittleFS: %s", esp_err_to_name(errFS));

        vTaskDelete(NULL);
    }

    // Format mode
    if (counter.format == pdTRUE) {
        ESP_LOGW(TAG_LFS, "Format mode enabled, formatting LittleFS");
        errFS = esp_littlefs_format(littlefs_config.partition_label);
        if (errFS != ESP_OK)
            ESP_LOGE(TAG_LFS, "Failed to format LittleFS: %s", esp_err_to_name(errFS));
        else
            ESP_LOGI(TAG_LFS, "Format Successful");

        xEventGroupSetBits(xFormatEventGroup, EVT_LFS_DONE);
        vTaskDelete(NULL);
    }

    // Create log file
    char log_name[FILENAME_LENGTH];
    snprintf(log_name, FILENAME_LENGTH, "%s/flight%ld.bin", littlefs_config.base_path, counter.lfs_files);
    ESP_LOGI(TAG_LFS, "Created file %s", log_name);

    FILE *f = fopen(log_name, "wb");
    if (!f) {
        ESP_LOGE(TAG_LFS, "Failed to open file for writing");
        esp_vfs_littlefs_unregister(littlefs_config.partition_label);
        ESP_LOGI(TAG_LFS, "LittleFS unmounted");
        vTaskDelete(NULL);
    }

    static uint8_t buffer[LITTLEFS_BUFFER_SIZE];
    uint16_t       buffer_offset = 0;
    save_t         save_data;
    bool           _lfs_full;

    size_t total = 0, used = 0;
    errFS = esp_littlefs_info(littlefs_config.partition_label, &total, &used);
    if (errFS != ESP_OK)
        ESP_LOGE(TAG_LFS, "Failed to get LittleFS partition information: %s", esp_err_to_name(errFS));
    else
        ESP_LOGI(TAG_LFS, "Partition size: %d/%d (%.2f%%)", used, total, (float)(used / total * 100.0f));

    while (true) {
        _lfs_full = atomic_load_explicit(&lfs_full, memory_order_relaxed);
        // Read data from queue
        if (xQueueReceive(xLittleFSQueue, &save_data, portMAX_DELAY) == pdTRUE) {
            // If buffer is full, write to file
            if (buffer_offset + sizeof(save_t) > LITTLEFS_BUFFER_SIZE) {
                if (!_lfs_full) {
                    if (used + buffer_offset > MAX_FLASH_SIZE_USED * total) // Check if there's space before writing
                    {
                        ESP_LOGW(TAG_LFS, "Flash memory almost full.");
                        _lfs_full = true;
                        atomic_store_explicit(&lfs_full, true, memory_order_relaxed);
                    } else {
                        size_t w = fwrite(buffer, 1, LITTLEFS_BUFFER_SIZE, f);
                        if (w != LITTLEFS_BUFFER_SIZE)
                            ESP_LOGE(TAG_LFS, "Failed to write data to file");
                        else
                            ESP_LOGD(TAG_LFS, "Data written to LittleFS");
                        used += sizeof(buffer); // Update used space tracker
                        taskYIELD();            // Yield to allow other tasks to run
                    }
                }
                buffer_offset = 0; // Reset buffer index for next batch (or drop if full)
            }
            // Copy remaining data to buffer if we still have space available
            if (!_lfs_full) {
                memcpy(&buffer[buffer_offset], &save_data, sizeof(save_t));
                buffer_offset += sizeof(save_t);
            }
        }

        // Check if landed
        portENTER_CRITICAL(&xDATAMutex);
        bool landed = (data_g.status & LANDED);
        portEXIT_CRITICAL(&xDATAMutex);
        if (landed)
            break;
    }

    if (buffer_offset > 0) // Write remaining data to file
    {
        if (_lfs_full || used + buffer_offset > MAX_FLASH_SIZE_USED * total) // Check if there's space before writing
            ESP_LOGW(TAG_LFS, "Flash memory almost full. Remaining data not written.");
        else {
            size_t w = fwrite(buffer, 1, buffer_offset, f);
            if (w != buffer_offset)
                ESP_LOGE(TAG_LFS, "Failed to write remaining data to file");
            else
                ESP_LOGD(TAG_LFS, "Remaining data written to LittleFS");
            used += buffer_offset; // Update used space tracker
        }
    }

    ESP_LOGW(TAG_LFS, "Landed, closing file and unmounting LittleFS");
    fclose(f);
    ESP_LOGI(TAG_LFS, "File closed");
    vTaskDelay(pdMS_TO_TICKS(20));
    esp_vfs_littlefs_unregister(littlefs_config.partition_label);
    ESP_LOGI(TAG_LFS, "LittleFS unmounted");
    xEventGroupSetBits(xNVSCounterEventGroup, EVT_LFS_DONE); // Signal that LFS task is done

    vTaskDelete(NULL);
}

void task_nvs(void *pvParameters) {
    nvs_handle_t nvs_handle;

    /* Wait for SD and LFS to finish */
    const EventBits_t save_bits = EVT_SD_DONE | EVT_LFS_DONE;
    xEventGroupWaitBits(xNVSCounterEventGroup, save_bits, pdTRUE, pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG_NVS, "Starting NVS file counter update");
    nvs_open("storage", NVS_READWRITE, &nvs_handle);

    /* Increment file counter */
    file_counter_g.sd_files += 1;
    file_counter_g.lfs_files += 1;

    /* Update NVS */
    nvs_set_u32(nvs_handle, "sd_counter", file_counter_g.sd_files);
    nvs_set_u32(nvs_handle, "lfs_counter", file_counter_g.lfs_files);
    nvs_commit(nvs_handle);

    nvs_close(nvs_handle);

    ESP_LOGI(TAG_NVS, "NVS file counter updated");

    xEventGroupSetBits(xNVSCounterEventGroup, EVT_NVS_DONE);
    vTaskDelete(NULL);
}