# The following four lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Hexnet IO Module — aktif geliştirme: IOModule-v0-60
project(mqtt_tcp_custom_outbox VERSION "0.60.0")

# custom_outbox.cpp (std::pmr) PSRAM'siz kartta panic — CONFIG_MQTT_CUSTOM_OUTBOX kapali.
# Espressif ornek outbox yeniden acilacaksa asagidakileri geri ekle:
#   idf_component_get_property(mqtt mqtt COMPONENT_LIB)
#   target_sources(${mqtt} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/main/custom_outbox.cpp)
#   idf_component_get_property(pthread pthread COMPONENT_LIB)
#   target_link_libraries(${mqtt} ${pthread})

# Derleme sonrasi platform OTA adi: build.cmd / flash_usb.cmd -> vango_medium_v0_60.bin
set(HEXNET_HW_SLUG "vango_medium")
string(REPLACE "." ";" _hexnet_ver_parts "${PROJECT_VERSION}")
list(GET _hexnet_ver_parts 0 _hexnet_ver_maj)
list(GET _hexnet_ver_parts 1 _hexnet_ver_min)
set(HEXNET_RELEASE_BIN "${HEXNET_HW_SLUG}_v${_hexnet_ver_maj}_${_hexnet_ver_min}.bin")
