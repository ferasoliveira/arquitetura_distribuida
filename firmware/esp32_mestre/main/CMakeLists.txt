idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        esp_timer
        driver
        freertos
        esp_hw_support
        log
        esp_wifi
        esp_eth
        esp_http_server
        nvs_flash
        lwip
        esp_event
)

# Propaga a flag EB15_QEMU=1 como define de compilador quando passada no build
if(EB15_QEMU)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC EB15_QEMU=1)
    message(STATUS "EB15: modo QEMU ativo (redirecionamento de portas TCP ativo)")
endif()
