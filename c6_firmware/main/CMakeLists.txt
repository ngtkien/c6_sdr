if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/common")
    set(common_dir "${CMAKE_CURRENT_SOURCE_DIR}/common")
else()
    set(common_dir "../../common")
endif()

set(COMPONENT_SRCS "slave_control.c" "${common_dir}/proto/esp_hosted_rpc.pb-c.c" "protocomm_pserial.c" "app_main.c" "slave_bt.c" "mempool.c" "stats.c"  "mempool_ll.c"
    "sdr/sdr_engine.c" "sdr/ring_capture.c" "sdr/spectrum.c" "sdr/spectrum_stats.c")
set(COMPONENT_ADD_INCLUDEDIRS "." "sdr" "${common_dir}" "${common_dir}/log" "${common_dir}/proto" "${common_dir}/rpc" "${common_dir}/transport")

# Select BT UART code based on IDF Target
if(CONFIG_IDF_TARGET_ESP32)
    list(APPEND COMPONENT_SRCS slave_bt_uart_esp32.c)
elseif(CONFIG_IDF_TARGET_ESP32C3 OR CONFIG_IDF_TARGET_ESP32S3)
    list(APPEND COMPONENT_SRCS slave_bt_uart_esp32c3_s3.c)
else()
    list(APPEND COMPONENT_SRCS slave_bt_uart_esp32xx.c)
endif()

if(CONFIG_ESP_SDIO_HOST_INTERFACE)
    list(APPEND COMPONENT_SRCS sdio_slave_api.c)
elseif(CONFIG_ESP_SPI_HOST_INTERFACE)
    list(APPEND COMPONENT_SRCS spi_slave_api.c)
elseif(CONFIG_ESP_SPI_HD_MODE)
    list(APPEND COMPONENT_SRCS spi_hd_slave_api.c)
else(CONFIG_ESP_UART_HOST_INTERFACE)
    list(APPEND COMPONENT_SRCS uart_slave_api.c)
endif()

register_component()

# Add directory of protocomm_priv.h to include paths
idf_component_get_property(protocomm_dir protocomm COMPONENT_DIR)
target_include_directories(${COMPONENT_LIB} PRIVATE "${protocomm_dir}/src/common")

# c6_sdr: keep .iram/.data/.bss clear of the RF dump SRAM aperture
target_link_options(${COMPONENT_LIB} INTERFACE
    "-T${CMAKE_CURRENT_LIST_DIR}/sdr/sram_guard.ld")
