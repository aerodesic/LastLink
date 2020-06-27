#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := LastLink

EXTRA_CFLAGS = -s

include $(IDF_PATH)/make/project.mk

SPIFFS_IMAGE_FLASH_IN_PROJECT := 1
$(eval $(call spiffs_create_partition_image,storage,filesystem))

# Create self-signed certificates for HTTPS server
certs:
	mkdir -p main/certs
	openssl req -newkey rsa:2048 -nodes -keyout main/certs/prvtkey.pem -x509 -days 3650 -out main/certs/cacert.pem -subj "/CN=ESP32 HTTPS LastLink certificate"
