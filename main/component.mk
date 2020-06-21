#
# Main component makefile.
#
# This Makefile can be left empty. By default, it will take the sources in the 
# src/ directory, compile them and link them into lib(subdirectory_name).a 
# in the build directory. This behaviour is entirely configurable,
# please read the ESP-IDF documents if you need to do this.
#

COMPONENT_EXTRA_CLEAN := $(COMPONENT_PATH)/*_table.h

%_table.h:
	# echo "Buildint channel table for $(notdir $(patsubst %_table.h,%,$(@)))"
	${COMPONENT_PATH}/generate_channel_table.py ${COMPONENT_PATH}/${CONFIG_LASTLINK_CHANNEL_TABLE}.channels -d $(notdir $(patsubst %_table.h,%,$(@))) -o $@


sx126x_driver.o:	$(COMPONENT_PATH)/sx126x_driver.c $(COMPONENT_PATH)/sx126x_table.h

sx127x_driver.o:	$(COMPONENT_PATH)/sx127x_driver.c $(COMPONENT_PATH)/sx127x_table.h

commands.o:         $(COMPONENT_PATH)/commands.c

configdata.o:       $(COMPONENT_PATH)/configdata.c

lastlink_main.o:    $(COMPONENT_PATH)/lastlink_main.c

linklayer.o:        $(COMPONENT_PATH)/linklayer.c

linklayer_io.o:     $(COMPONENT_PATH)/linklayer_io.c

lsocket.o:          $(COMPONENT_PATH)/lsocket.c

nvs_suppert.o:      $(COMPONENT_PATH)/nvs_support.c

os_freertos.o:      $(COMPONENT_PATH)/os_freertos.c

packets.o:          $(COMPONENT_PATH)/packets.c

routes.o:           $(COMPONENT_PATH)/routes.c

spiffs.o:           $(COMPONENT_PATH)/spiffs.c

vfs_lastlink.o:     $(COMPONENT_PATH)/vfs_lastlink.c

wifi_init.o:        $(COMPONENT_PATH)/wifi_init.c



#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

COMPONENT_EMBED_TXTFILES := certs/cacert.pem
COMPONENT_EMBED_TXTFILES += certs/prvtkey.pem

