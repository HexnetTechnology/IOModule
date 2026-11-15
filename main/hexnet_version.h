#pragma once

/** Geliştirme hattı (klasör adı ile uyumlu). */
#define HEXNET_IO_DEV_LINE "v0-60"

/**
 * Makro sürüm (çizgi). VERSION.json makro ile uyumlu tutun.
 * CMake project(VERSION "0.60.0") → esp_app_desc.version
 */
#define HEXNET_IO_RELEASE "v0.60"

/** Mikro sürüm — her anlamlı derleme/değişiklik (takip). */
#define HEXNET_IO_MICRO "v0.60-0004"

/** Telemetri / UI — çalışan partition veya RELEASE. */
const char *hexnet_firmware_version_string(void);
const char *hexnet_firmware_micro_string(void);
