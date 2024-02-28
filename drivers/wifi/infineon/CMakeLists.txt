# Copyright (c) 2023 Cypress Semiconductor Corporation (an Infineon company) or
# an affiliate of Cypress Semiconductor Corporation
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(./)

zephyr_library_sources_ifdef(
  CONFIG_WIFI_AIROC 
  airoc_wifi.c
  airoc_whd_hal_common.c)
zephyr_library_sources_ifdef(CONFIG_AIROC_WIFI_BUS_SDIO airoc_whd_hal_sdio.c)
zephyr_library_sources_ifdef(CONFIG_AIROC_WIFI_BUS_SPI airoc_whd_hal_spi.c)

zephyr_compile_definitions(CYBSP_WIFI_CAPABLE)
zephyr_compile_definitions(CY_RTOS_AWARE)
zephyr_compile_definitions(WHD_USE_CUSTOM_MALLOC_IMPL)
zephyr_compile_definitions(WHD_USE_CUSTOM_HAL_IMPL)
# TODO(yiding) debug stuff to remove.
zephyr_compile_definitions(WPRINT_ENABLE_ERROR WPRINT_ENABLE_WHD_INFO WPRINT_ENABLE_WHD_DEBUG WPRINT_ENABLE_WHD_DATA_LOG)
