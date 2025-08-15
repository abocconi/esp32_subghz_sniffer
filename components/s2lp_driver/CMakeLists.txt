# S2LP Driver Component for ESP32-S3
# Sub-GHz Transceiver Driver

idf_component_register(
    SRCS 
        "src/s2lp_driver.c"
    
    INCLUDE_DIRS 
        "include"
    
    REQUIRES 
        "driver"
        "esp_common"
        "freertos"
        "log"
        "spi_flash"
    
    PRIV_REQUIRES
        "esp_timer"
)