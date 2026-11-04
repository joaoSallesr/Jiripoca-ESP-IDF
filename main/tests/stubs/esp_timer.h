// tests/stubs/esp_timer.h — Stub for native-host testing
#pragma once

#include <stdint.h>

typedef void *esp_timer_handle_t;

typedef int esp_err_t;

static inline int64_t esp_timer_get_time(void) { return 0; }

static inline esp_err_t esp_timer_start_once(esp_timer_handle_t timer, uint64_t timeout_us) {
    (void)timer;
    (void)timeout_us;
    return 0;
}

static inline esp_err_t esp_timer_stop(esp_timer_handle_t timer) {
    (void)timer;
    return 0;
}

static inline esp_err_t esp_timer_delete(esp_timer_handle_t timer) {
    (void)timer;
    return 0;
}