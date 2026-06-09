idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        freertos
        driver
        esp_adc
        esp_driver_spi
        esp_driver_gpio
)