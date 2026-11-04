// tests/stubs/esp_check.h — Stub for native-host testing
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifndef ESP_OK
#define ESP_OK   0
#define ESP_FAIL (-1)
#endif

#define ESP_RETURN_ON_ERROR(x, tag, fmt, ...) \
    do {                                      \
        if ((x) != 0)                         \
            return (x);                       \
    } while (0)

#define ESP_RETURN_ON_FALSE(a, err, tag, fmt, ...) \
    do {                                           \
        if (!(a))                                  \
            return (err);                          \
    } while (0)

#define ESP_GOTO_ON_ERROR(x, goto_tag, tag, fmt, ...) \
    do {                                              \
        if ((x) != 0)                                 \
            goto goto_tag;                            \
    } while (0)
