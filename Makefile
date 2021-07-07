#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := LastLink

COMPONENTS := main
COMPONENTS += app_trace
COMPONENTS += app_update
COMPONENTS += bootloader
COMPONENTS += console
COMPONENTS += dht
COMPONENTS += driver
COMPONENTS += efuse
COMPONENTS += esp32
# COMPONENTS += espcoredump
COMPONENTS += esptool_py
COMPONENTS += esp_adc_cal
COMPONENTS += esp_common
COMPONENTS += esp_eth
COMPONENTS += esp_event
COMPONENTS += esp_http_server
COMPONENTS += esp_https_server
COMPONENTS += esp_hw_support
COMPONENTS += esp_ipc
COMPONENTS += esp_netif
COMPONENTS += esp_phy
COMPONENTS += esp_pm
COMPONENTS += esp_ringbuf
COMPONENTS += esp_rom
COMPONENTS += esp_system
COMPONENTS += esp_timer
COMPONENTS += esp-tls
COMPONENTS += esp_wifi
COMPONENTS += freertos
COMPONENTS += hal
COMPONENTS += heap
COMPONENTS += idf_test
COMPONENTS += json
COMPONENTS += log
COMPONENTS += lwip
COMPONENTS += mbedtls
COMPONENTS += mdns
COMPONENTS += micro-ecc
COMPONENTS += newlib
COMPONENTS += nghttp
COMPONENTS += nvs_flash
COMPONENTS += partition_table
COMPONENTS += pthread
COMPONENTS += ssd1306-esp-idf-i2c
COMPONENTS += soc
COMPONENTS += spiffs
COMPONENTS += spi_flash
COMPONENTS += spi_flash_types
COMPONENTS += tcpip_adapter
COMPONENTS += ulp
COMPONENTS += unity
COMPONENTS += wpa_supplicant
COMPONENTS += xtensa-debug-module
COMPONENTS += bootloader_support
COMPONENTS += vfs
COMPONENTS += xtensa

EXTRA_CFLAGS = -s

include $(IDF_PATH)/make/project.mk

SPIFFS_IMAGE_FLASH_IN_PROJECT := 1
$(eval $(call spiffs_create_partition_image,storage,filesystem))

# Create self-signed certificates for HTTPS server
certs:
	mkdir -p main/certs
	openssl req -newkey rsa:2048 -nodes -keyout main/certs/prvtkey.pem -x509 -days 3650 -out main/certs/cacert.pem -subj "/CN=ESP32 HTTPS LastLink certificate"
