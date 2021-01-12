idf_component_register(SRCS "main.c" "usb_descriptors.c"
                    INCLUDE_DIRS "."
                    REQUIRES freertos soc i2c_bus es8311 i2s_bus)

target_compile_options(${COMPONENT_TARGET} PUBLIC
  "-DCFG_TUSB_MCU=OPT_MCU_ESP32S2"
)

idf_component_get_property( FREERTOS_ORIG_INCLUDE_PATH freertos ORIG_INCLUDE_PATH)
target_include_directories(${COMPONENT_TARGET} PUBLIC
  "${FREERTOS_ORIG_INCLUDE_PATH}"
  "${TOP}/hw"
  "${TOP}/src"
)

target_sources(${COMPONENT_TARGET} PUBLIC
  "${TOP}/src/tusb.c"
  "${TOP}/src/common/tusb_fifo.c"
  "${TOP}/src/device/usbd.c"
  "${TOP}/src/device/usbd_control.c"
  "${TOP}/src/class/audio/audio_device.c"
  "${TOP}/src/portable/espressif/esp32s2/dcd_esp32s2.c"
)
