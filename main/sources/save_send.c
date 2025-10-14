#include "save_send.h"

// TAGS
static const char *TAG_LITTLEFS = "LittleFS";
static const char *TAG_SD = "SD Card";
static const char *TAG_LORA = "LoRa";

// task_sd reads data from queue and writes it to SD card
void task_sd(void *pvParameters)
{
    esp_err_t errSD;
    sdmmc_card_t *card;
    file_counter_t counterSD = *(file_counter_t *)pvParameters;

    ESP_LOGI(TAG_SD, "Initializing SD card");

    // Settings for initializing SD card
    spi_bus_config_t bus_config = {
        .mosi_io_num = SD_MOSI,
        .miso_io_num = SD_MISO,
        .sclk_io_num = SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SD_TRANSF_SIZE,
    };

    // Settings for mounting FAT filesystem
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = CONFIG_SD_FORMAT_IF_MOUNT_FAILED, // TESTAR 00 (simplificação do FORMAT_IF_MOUNT_FAILED)
        .max_files = SD_MAX_FILES,
        .allocation_unit_size = SD_UNIT_SIZE,
    };
    // Set CONFIG_SD_FORMAT_IF_MOUNT_FAILED to TRUE or FALSE
    // When format_if_mount_failed is set to true, SD card will be partitioned and formatted

    // SPI initializer
    ESP_LOGI(TAG_SD, "Using SPI peripheral");
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS;
    slot_config.host_id = host.slot;

    errSD = spi_bus_initialize(host.slot, &bus_config, SDSPI_DEFAULT_DMA);
    if (errSD != ESP_OK)
    {
        ESP_LOGE(TAG_SD, "Failed to initialize SPI bus.");
        return;
    }
    ESP_LOGI(TAG_SD, "SPI bus initialized");

    // Mount filesystem
    ESP_LOGI(TAG_SD, "Mounting filesystem");
    errSD = esp_vfs_fat_sdspi_mount(SD_MOUNT, &host, &slot_config, &mount_config, &card);
    if (errSD != ESP_OK)
    {
        if (errSD == ESP_FAIL)
        {
            ESP_LOGE(TAG_SD, "Failed to mount filesystem. "
                             "If you want the card to be formatted, set the CONFIG_SD_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            ESP_LOGE(TAG_SD, "Failed to initialize the card (%s). ",
                     esp_err_to_name(errSD));
        }
        spi_bus_free(host.slot);
        ESP_LOGI(TAG_SD, "SPI bus freed");
        return;
    }
    ESP_LOGI(TAG_SD, "Filesystem mounted");

    // Format mode
    if (counterSD.format == pdTRUE)
    {
        errSD = esp_vfs_fat_sdcard_format(SD_MOUNT, card);
        if (errSD != ESP_OK)
        {
            ESP_LOGE(TAG_SD, "Failed to format FATFS (%s)", esp_err_to_name(errSD));
        }
        else
        {
            ESP_LOGI(TAG_SD, "Format Successful");
        }
        
        esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
        ESP_LOGI(TAG_SD, "Card unmounted");
        spi_bus_free(host.slot);
        ESP_LOGI(TAG_SD, "SPI bus freed");        
        vTaskDelete(NULL);
    }

    // Print sd card info
    sdmmc_card_print_info(stdout, card);

    // Create log file
    char log_name[FILENAME_LENGTH];
    snprintf(log_name, FILENAME_LENGTH, "%s/flight%ld.bin", SD_MOUNT, counterSD.file_num);
    ESP_LOGI(TAG_SD, "Creating file %s", log_name);

    FILE *f = fopen(log_name, "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG_SD, "Failed to open file for writing");
        esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
        ESP_LOGI(TAG_SD, "Card unmounted");
        spi_bus_free(host.slot);
        ESP_LOGI(TAG_SD, "SPI bus freed");
        return;
    }
    fclose(f);

    while (true)
    {
        //data_t data;                                                  // 01
        data_t buffer[SD_BUFFER_COUNT]; 

        // Read data from queue
        for (int i = 0; i < SD_BUFFER_COUNT; ++i)
        {
            xQueueReceive(xSDQueue, &buffer[i], portMAX_DELAY); // TESTAR 01 (testar se realmente é necessário criar o data_t data)
            //xQueueReceive(xSDQueue, &data, portMAX_DELAY);            // 01
            //buffer[i] = data;                                         // 01
        }
        // Write buffer to file
        f = fopen(log_name, "a");
        if (f == NULL)
        {
            ESP_LOGE(TAG_SD, "Failed to open file for writing");
            esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
            ESP_LOGI(TAG_SD, "Card unmounted");
            spi_bus_free(host.slot);
            ESP_LOGI(TAG_SD, "SPI bus freed");
            return;
        }

        fwrite(buffer, sizeof(data_t), SD_BUFFER_COUNT, f);
        fflush(f);                                              // TESTAR 02 (necessidade do flush e se resolve o problema)
        fclose(f);

        ESP_LOGI(TAG_SD, "Data written to SD card");

        // Check if landed
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        if (STATUS & LANDED)
        {
            xSemaphoreGive(xStatusMutex);

            ESP_LOGW(TAG_SD, "Landed, unmounting SD card");
            esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
            ESP_LOGI(TAG_SD, "Card unmounted");
            spi_bus_free(host.slot);
            ESP_LOGI(TAG_SD, "SPI bus freed");

            vTaskDelete(NULL);
        }
        else
            xSemaphoreGive(xStatusMutex);
    }
}

// task_littlefs reads data from queue and writes it to LittleFS
void task_littlefs(void *pvParameters)
{
    esp_err_t errFS;
    file_counter_t counterFS = *(file_counter_t *)pvParameters;

    ESP_LOGW(TAG_LITTLEFS, "Initializing LittleFS");

    // Settings for initializing LittleFS
    esp_vfs_littlefs_conf_t littlefs_config = {
        .base_path = "/littlefs",
        .partition_label = "littlefs",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    errFS = esp_vfs_littlefs_register(&littlefs_config);
    if (errFS != ESP_OK)
    {
        if (errFS == ESP_FAIL)
        {
            ESP_LOGE(TAG_LITTLEFS, "Failed to mount or format filesystem");
        }
        else if (errFS == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG_LITTLEFS, "Failed to find LittleFS partition");
        }
        else
        {
            ESP_LOGE(TAG_LITTLEFS, "Failed to initialize LittleFS (%s)", esp_err_to_name(errFS));
        }
        return;
    }

    size_t total = 0, used = 0;
    errFS = esp_littlefs_info(littlefs_config.partition_label, &total, &used);
    if (errFS != ESP_OK)
    {
        ESP_LOGE(TAG_LITTLEFS, "Failed to get LittleFS partition information (%s)", esp_err_to_name(errFS));
    }
    else
    {
        ESP_LOGW(TAG_LITTLEFS, "Partition size: total: %d, used: %d", total, used);
    }

    // Format mode
    if (counterFS.format == pdTRUE)
    {
        errFS = esp_littlefs_format(littlefs_config.partition_label);
        if (errFS != ESP_OK)
        {
            ESP_LOGE(TAG_LITTLEFS, "Failed to format LittleFS (%s)", esp_err_to_name(errFS));
        }
        else
        {
            ESP_LOGI(TAG_LITTLEFS, "Format Successful");
        }
        vTaskDelete(NULL);
    }

    // Create log file
    char log_name[FILENAME_LENGTH];
    snprintf(log_name, FILENAME_LENGTH, "%s/flight%ld.bin", littlefs_config.base_path, counterFS.file_num);
    ESP_LOGI(TAG_LITTLEFS, "Creating file %s", log_name);

    FILE *f = fopen(log_name, "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG_LITTLEFS, "Failed to open file for writing");
        esp_vfs_littlefs_unregister(littlefs_config.partition_label);
        ESP_LOGI(TAG_LITTLEFS, "LittleFS unmounted");
        return;
    }
    uint32_t oldest_file_num = counterFS.file_num;
    fclose(f);

    while (true)
    {
        //data_t data;                                                  // 03
        data_t buffer[FS_BUFFER_COUNT];

        // Read data from queue
        for (int i = 0; i < FS_BUFFER_COUNT; ++i)
        {
            xQueueReceive(xLittleFSQueue, &buffer[i], portMAX_DELAY);   // TESTAR 03 (mesmo caso do SD 01)
            //xQueueReceive(xLittleFSQueue, &data, portMAX_DELAY);      // 03
            //buffer[i] = data;                                         // 03
        }

        // Delete oldest file if disk space is full
        while (used + sizeof(buffer) > MAX_USED * total)
        {
            oldest_file_num++;
            if (oldest_file_num > CONFIG_MAX_LFS_FILES)
            {
                oldest_file_num = 0;
            }
            char oldest_file_name[FILENAME_LENGTH];
            snprintf(oldest_file_name, FILENAME_LENGTH, "%s/flight%ld.bin", littlefs_config.base_path, oldest_file_num);

            struct stat st;
            if (stat(oldest_file_name, &st) == 0) // If file exists
            {
                ESP_LOGW(TAG_LITTLEFS, "Deleting file %s", oldest_file_name);
                unlink(oldest_file_name); // Delete file
            }

            esp_littlefs_info(littlefs_config.partition_label, &total, &used); // Update used space

            if (oldest_file_num == counterFS.file_num) // If oldest file is current file
            {
                ESP_LOGE(TAG_LITTLEFS, "No more disk space, unmounting LittleFS");

                esp_vfs_littlefs_unregister(littlefs_config.partition_label);
                ESP_LOGI(TAG_LITTLEFS, "LittleFS unmounted");

                xSemaphoreTake(xStatusMutex, portMAX_DELAY);
                STATUS |= LFS_FULL;
                xSemaphoreGive(xStatusMutex);

                vTaskDelete(NULL);
            }
        }

        // Write buffer to file
        f = fopen(log_name, "a");
        if (f == NULL)
        {
            ESP_LOGE(TAG_LITTLEFS, "Failed to open file for writing");
            esp_vfs_littlefs_unregister(littlefs_config.partition_label);
            ESP_LOGI(TAG_LITTLEFS, "LittleFS unmounted");
            return;
        }
            
        fwrite(buffer, sizeof(data_t), FS_BUFFER_COUNT, f);

        fflush(f);                                                          // TESTAR 04 (mesmo caso do SD 02)
        fclose(f);                             

        // Update used space tracker
        esp_littlefs_info(littlefs_config.partition_label, &total, &used);  // TESTAR 05 (se é melhor que o used+=)
        //used += sizeof(buffer);  

        ESP_LOGI(TAG_LITTLEFS, "Data written to LittleFS.");

        // Check if landed
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        if (STATUS & LANDED)
        {
            xSemaphoreGive(xStatusMutex);
            ESP_LOGW(TAG_LITTLEFS, "Landed, unmounting LittleFS");
            esp_vfs_littlefs_unregister(littlefs_config.partition_label);
            ESP_LOGI(TAG_LITTLEFS, "LittleFS unmounted");
            vTaskDelete(NULL);
        }
        else
            xSemaphoreGive(xStatusMutex);
    }
}