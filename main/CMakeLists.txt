idf_component_register(SRCS "air_mouse_main.c" "esp_hid_gap.c"
    INCLUDE_DIRS "."
    REQUIRES bt esp_hid nvs_flash)
