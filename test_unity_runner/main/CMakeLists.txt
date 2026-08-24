idf_component_register(
    SRCS "test_dummy.c" "test_main.c"
    INCLUDE_DIRS "." "../../main"
    REQUIRES unity main esp_timer driver esp_adc esp_ringbuf nvs_flash eskf
)



