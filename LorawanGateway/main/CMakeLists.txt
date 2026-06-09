idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_spi
        esp_driver_gpio
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        wpa_supplicant
        driver
        mbedtls
        mqtt
        ttn-esp32
)

idf_component_get_property(ttn_lib ttn-esp32 COMPONENT_LIB)
target_compile_options(${ttn_lib} PRIVATE -Wno-maybe-uninitialized)