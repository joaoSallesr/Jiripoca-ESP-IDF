#include "global.h"

/* ==================================================================== *
 * Configuration                                                         *
 * ==================================================================== */

#define TAG_SD  "SD"
#define TAG_LFS "LittleFS"
#define TAG_NVS "NVS"

#define SD_MOUNT          "/sdcard"
#define SD_MAX_FILES      5
#define SD_BUFFER_SIZE    4096
#define SD_UNIT_SIZE      (32 * 1024)
#define SD_SYNC_PERIOD_MS 5000 // how often the SD file is fsync'd while logging

#define LFS_MOUNT       "/littlefs"
#define LFS_BUFFER_SIZE 512
#define LFS_MAX_FLASH   0.9 // max fraction of the LittleFS partition we're allowed to fill

#define FILENAME_LENGTH 32
#define NVS_NAMESPACE   "storage"

/* ==================================================================== *
 * Types                                                                 *
 * ==================================================================== */

typedef struct {
    uint16_t offset;
    size_t   size;
    uint8_t *array;
} buffer_t;

typedef struct {
    sdmmc_card_t *card;
    TickType_t    last_sync; // only SD needs periodic fsync; LFS has no equivalent
} sd_state_t;

typedef struct {
    esp_vfs_littlefs_conf_t cfg;      // partition/mount settings
    size_t                  lfs_size; // total partition size in bytes
    size_t                  lfs_used; // bytes used so far — checked against LFS_MAX_FLASH before each write
} lfs_state_t;

struct storage_device_t;

typedef esp_err_t (*storage_mount_fn)(struct storage_device_t *self);
typedef esp_err_t (*storage_format_fn)(struct storage_device_t *self);
typedef bool (*storage_can_write_fn)(struct storage_device_t *self, const buffer_t *buf);
typedef esp_err_t (*storage_write_fn)(struct storage_device_t *self, const buffer_t *buf, FILE *f);
typedef bool (*storage_should_stop_fn)(void);
typedef void (*storage_finalize_fn)(FILE *f, save_t last_sample);
typedef esp_err_t (*storage_unmount_fn)(struct storage_device_t *self);

/* One "backend" (SD or LittleFS) — log_task() drives any backend through
 * this interface, so the sampling/logging logic only has to be written once. */
typedef struct storage_device_t {
    void         *config; // backend-specific state (sd_state_t* or lfs_state_t*)
    const char   *tag;
    QueueHandle_t queue;
    EventBits_t   done_bit;
    const size_t  buffer_size;

    const char *mount_path;   // e.g. SD_MOUNT or LFS_MOUNT
    const char *file_prefix;  // e.g. "test" or "flight"
    uint32_t   *file_counter; // this backend's own counter in file_counter_g

    storage_mount_fn       mount;
    storage_format_fn      format;
    storage_can_write_fn   can_write;
    storage_write_fn       write;
    storage_should_stop_fn should_stop;
    storage_finalize_fn    finalize;
    storage_unmount_fn     unmount;
} storage_device_t;

/* ==================================================================== *
 * Buffer helpers                                                        *
 * ==================================================================== */

static inline bool buffer_has_room(const buffer_t *buf, size_t needed) { return buf->offset + needed <= buf->size; }

static inline void buffer_reset(buffer_t *buf) {
    buf->offset = 0;
    memset(buf->array, 0, buf->size);
    taskYIELD();
}

static inline void buffer_push(buffer_t *buf, const save_t *sample) {
    memcpy(&buf->array[buf->offset], sample, sizeof(save_t));
    buf->offset += sizeof(save_t);
}

/* ==================================================================== *
 * Shared helpers                                                        *
 * ==================================================================== */

bool should_stop(void) {
    portENTER_CRITICAL(&xDATAMutex);
    bool is_landing = (data_g.status & LANDING);
    portEXIT_CRITICAL(&xDATAMutex);
    return is_landing;
}

static esp_err_t write_to_file(const uint8_t *array, size_t size, FILE *f, const char *tag) {
    size_t written = fwrite(array, 1, size, f);
    if (written != size) {
        ESP_LOGE(tag, "Failed to write data to file");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t create_log_file(const storage_device_t *device, FILE **out_file) {
    char name[FILENAME_LENGTH];
    snprintf(name, sizeof(name), "%s/%s%lu.bin", device->mount_path, device->file_prefix, *device->file_counter);

    ESP_LOGI(device->tag, "Creating file %s", name);
    FILE *f = fopen(name, "wb");
    if (!f) {
        ESP_LOGE(device->tag, "Failed to open file for writing");
        return ESP_FAIL;
    }

    *out_file = f;
    return ESP_OK;
}

/* ==================================================================== *
 * SD backend                                                            *
 * ==================================================================== */

static esp_err_t sd_mount(storage_device_t *self) {
    sd_state_t *state = (sd_state_t *)self->config;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    sdspi_device_config_t slot_config = {
        .host_id          = SPI2_HOST,
        .gpio_cs          = SD_CS,
        .gpio_cd          = SDSPI_SLOT_NO_CD,
        .gpio_wp          = SDSPI_SLOT_NO_WP,
        .gpio_int         = GPIO_NUM_NC,
        .gpio_wp_polarity = SDSPI_IO_ACTIVE_LOW,
        .duty_cycle_pos   = 0,
        .wait_for_miso    = 0,
    };

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files              = SD_MAX_FILES,
        .allocation_unit_size   = SD_UNIT_SIZE,
    };

    esp_err_t err = esp_vfs_fat_sdspi_mount(SD_MOUNT, &host, &slot_config, &mount_config, &state->card);
    if (err != ESP_OK)
        return err;

    sdmmc_card_print_info(stdout, state->card);
    return ESP_OK;
}

static esp_err_t sd_format(storage_device_t *self) {
    sd_state_t *state = (sd_state_t *)self->config;
    return esp_vfs_fat_sdcard_format(SD_MOUNT, state->card);
}

static void sd_flush(FILE *f) {
    fflush(f);
    fsync(fileno(f));
}

static bool sd_can_write(storage_device_t *self, const buffer_t *buf) {
    return true; // SD card capacity isn't tracked/limited the way LittleFS flash is
}

static esp_err_t sd_write(storage_device_t *self, const buffer_t *buf, FILE *f) {
    sd_state_t *state = (sd_state_t *)self->config;

    esp_err_t err = write_to_file(buf->array, buf->offset, f, self->tag);
    taskYIELD();
    if (err != ESP_OK)
        return err;

    TickType_t now = xTaskGetTickCount();
    if (now - state->last_sync >= pdMS_TO_TICKS(SD_SYNC_PERIOD_MS)) {
        sd_flush(f);
        state->last_sync = now;
        ESP_LOGD(TAG_SD, "File flushed");
        taskYIELD();
    }
    return ESP_OK;
}

static void sd_finalize(FILE *f, save_t last_sample) {
    // Drain any samples buffered before launch and append them to the log.
    while (xQueueReceive(xB4LaunchQueue, &last_sample, 0) == pdTRUE) {
        size_t written = fwrite(&last_sample, 1, sizeof(save_t), f);
        if (written != sizeof(save_t))
            ESP_LOGE(TAG_SD, "Failed to write pre-launch data to file");
        else
            ESP_LOGD(TAG_SD, "Pre-launch data written to SD card");
    }
    sd_flush(f);
}

static esp_err_t sd_unmount(storage_device_t *self) {
    sd_state_t *state = (sd_state_t *)self->config;
    return esp_vfs_fat_sdcard_unmount(SD_MOUNT, state->card);
}

static sd_state_t       sd_state   = {0};
static storage_device_t sd_backend = {
    .config       = &sd_state,
    .tag          = TAG_SD,
    .queue        = NULL, // Assigned dynamically in task_sd()
    .done_bit     = EVT_SD_DONE,
    .buffer_size  = SD_BUFFER_SIZE,
    .mount_path   = SD_MOUNT,
    .file_prefix  = "test",
    .file_counter = &file_counter_g.sd_files,
    .mount        = sd_mount,
    .format       = sd_format,
    .can_write    = sd_can_write,
    .write        = sd_write,
    .should_stop  = should_stop,
    .finalize     = sd_finalize,
    .unmount      = sd_unmount,
};

/* ==================================================================== *
 * LittleFS backend                                                       *
 * ==================================================================== */

static esp_err_t lfs_mount(storage_device_t *self) {
    lfs_state_t *state = (lfs_state_t *)self->config;

    state->cfg = (esp_vfs_littlefs_conf_t){
        .base_path              = LFS_MOUNT,
        .partition_label        = "littlefs",
        .format_if_mount_failed = true,
        .dont_mount             = false,
    };

    esp_err_t err = esp_vfs_littlefs_register(&state->cfg);
    if (err != ESP_OK)
        return err;

    ESP_LOGI(TAG_LFS, "Filesystem mounted");

    err = esp_littlefs_info(state->cfg.partition_label, &state->lfs_size, &state->lfs_used);
    if (err != ESP_OK)
        return err;

    ESP_LOGI(TAG_LFS, "Partition size: %u/%u (%.2f%%)", (unsigned)state->lfs_used, (unsigned)state->lfs_size,
             (float)state->lfs_used / state->lfs_size * 100.0f);
    return ESP_OK;
}

static esp_err_t lfs_format(storage_device_t *self) {
    lfs_state_t *state = (lfs_state_t *)self->config;
    return esp_littlefs_format(state->cfg.partition_label);
}

static bool lfs_can_write(storage_device_t *self, const buffer_t *buf) {
    lfs_state_t *state          = (lfs_state_t *)self->config;
    size_t       incoming_usage = state->lfs_used + buf->offset;
    size_t       flash_budget   = (size_t)(LFS_MAX_FLASH * state->lfs_size);
    return incoming_usage <= flash_budget;
}

static esp_err_t lfs_write(storage_device_t *self, const buffer_t *buf, FILE *f) {
    lfs_state_t *state = (lfs_state_t *)self->config;

    esp_err_t err = write_to_file(buf->array, buf->offset, f, self->tag);
    taskYIELD();
    if (err != ESP_OK)
        return err;

    state->lfs_used += buf->offset;
    taskYIELD();
    return ESP_OK;
}

static void lfs_finalize(FILE *f, save_t last_sample) {
    (void)f;
    (void)last_sample;
    // No pre-launch queue or fsync equivalent for LittleFS — nothing to do.
}

static esp_err_t lfs_unmount(storage_device_t *self) {
    lfs_state_t *state = (lfs_state_t *)self->config;
    return esp_vfs_littlefs_unregister(state->cfg.partition_label);
}

static lfs_state_t      lfs_state   = {0};
static storage_device_t lfs_backend = {
    .config       = &lfs_state,
    .tag          = TAG_LFS,
    .queue        = NULL, // Assigned dynamically in task_lfs()
    .done_bit     = EVT_LFS_DONE,
    .buffer_size  = LFS_BUFFER_SIZE,
    .mount_path   = LFS_MOUNT,
    .file_prefix  = "flight",
    .file_counter = &file_counter_g.lfs_files,
    .mount        = lfs_mount,
    .format       = lfs_format,
    .can_write    = lfs_can_write,
    .write        = lfs_write,
    .should_stop  = should_stop,
    .finalize     = lfs_finalize,
    .unmount      = lfs_unmount,
};

/* ==================================================================== *
 * Generic logging task — drives any storage_device_t                    *
 * ==================================================================== */

static void task_finished(const storage_device_t *device, esp_err_t err) {
    xEventGroupSetBits(xNVSCounterEventGroup, device->done_bit);

    if (err != ESP_OK) {
        ESP_LOGE(device->tag, "Failed: %s", esp_err_to_name(err));
        status_event_t evt = EVT_SETUP_FAILED;
        xQueueSend(xEventQueue, &evt, portMAX_DELAY);
    }

    vTaskDelete(NULL);
}

/* Mounts the backend, formats it if requested, and opens the log file.
 * Returns ESP_OK with *out_file populated, or an error (caller must bail). */
static esp_err_t log_task_setup(storage_device_t *device, FILE **out_file) {
    esp_err_t err = device->mount(device);
    if (err != ESP_OK) {
        ESP_LOGE(device->tag, "Failed to mount");
        return err;
    }
    ESP_LOGI(device->tag, "Filesystem mounted");

    if (file_counter_g.format) {
        ESP_LOGW(device->tag, "Format mode enabled");
        err = device->format(device);
        if (err != ESP_OK) {
            ESP_LOGE(device->tag, "Failed to format");
            return err;
        }
    }

    return create_log_file(device, out_file);
}

/* Pulls one sample off the queue and buffers it, flushing (or dropping, if
 * the backend has no room) whatever's already buffered first when needed. */
static esp_err_t log_task_handle_sample(storage_device_t *device, buffer_t *buf, FILE *f, save_t *out_sample) {
    if (xQueueReceive(device->queue, out_sample, portMAX_DELAY) != pdTRUE)
        return ESP_OK;

    if (!buffer_has_room(buf, sizeof(save_t))) {
        if (device->can_write(device, buf)) {
            esp_err_t err = device->write(device, buf, f);
            buffer_reset(buf);
            if (err != ESP_OK)
                return err;
        } else {
            ESP_LOGW(device->tag, "Storage full, dropping buffered sample(s)");
            buffer_reset(buf);
        }
    }

    buffer_push(buf, out_sample);
    return ESP_OK;
}

/* Flushes whatever's left in the buffer, runs backend-specific finalization,
 * and closes/unmounts the log file. */
static void log_task_teardown(storage_device_t *device, buffer_t *buf, FILE *f, save_t last_sample) {
    if (buf->offset > 0) {
        esp_err_t err = write_to_file(buf->array, buf->offset, f, device->tag);
        if (err != ESP_OK)
            ESP_LOGE(device->tag, "Failed to write pending buffer to file");
    }

    device->finalize(f, last_sample);
    fclose(f);

    taskYIELD();
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(device->tag, "File closed");

    esp_err_t err = device->unmount(device);
    if (err != ESP_OK)
        ESP_LOGE(device->tag, "Failed to unmount");
}

void log_task(storage_device_t *device) {
    ESP_LOGI(device->tag, "Initializing");

    FILE     *f;
    esp_err_t err = log_task_setup(device, &f);
    if (err != ESP_OK) {
        task_finished(device, err);
        return;
    }

    uint8_t  storage[device->buffer_size];
    buffer_t buffer = {.offset = 0, .size = device->buffer_size, .array = storage};
    save_t   sample;

    while (!device->should_stop()) {
        err = log_task_handle_sample(device, &buffer, f, &sample);
        if (err != ESP_OK)
            ESP_LOGE(device->tag, "Failed to write data to file");
    }

    log_task_teardown(device, &buffer, f, sample);
    task_finished(device, ESP_OK);
}

/* Thin wrappers so each backend can be started with plain xTaskCreate(). */
void task_sd(void *pvParameters) {
    (void)pvParameters;
    sd_backend.queue = xSDQueue;
    log_task(&sd_backend);
}

void task_lfs(void *pvParameters) {
    (void)pvParameters;
    lfs_backend.queue = xLittleFSQueue;
    log_task(&lfs_backend);
}

/* ==================================================================== *
 * NVS file counter task                                                  *
 * ==================================================================== */

void task_nvs(void *pvParameters) {
    const EventBits_t save_bits = EVT_SD_DONE | EVT_LFS_DONE;
    xEventGroupWaitBits(xNVSCounterEventGroup, save_bits, pdTRUE, pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG_NVS, "Starting NVS file counter update");

    nvs_handle_t nvs_handle;
    nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);

    file_counter_g.sd_files += 1;
    file_counter_g.lfs_files += 1;

    nvs_set_u32(nvs_handle, "sd_counter", file_counter_g.sd_files);
    nvs_set_u32(nvs_handle, "lfs_counter", file_counter_g.lfs_files);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    ESP_LOGI(TAG_NVS, "NVS file counter updated");

    xEventGroupSetBits(xNVSCounterEventGroup, EVT_NVS_DONE);
    vTaskDelete(NULL);
}