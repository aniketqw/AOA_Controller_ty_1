idf_component_register(
    SRCS
        "src/main.c"
        "src/hal_input.c"
        "src/validator.c"
        "src/estimator.c"
        "src/fsm.c"
        "src/thresholds.c"
        "src/logger.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        "freertos"
        "driver"
        "esp_timer"
        "log"
    PRIV_REQUIRES
        "esp_driver_uart"
        "esp_driver_gpio"
)

set(THRESHOLDS_CSV "${CMAKE_CURRENT_SOURCE_DIR}/data/thresholds.csv")
add_definitions(-DTHRESHOLDS_CSV_PATH=\"${THRESHOLDS_CSV}\")
