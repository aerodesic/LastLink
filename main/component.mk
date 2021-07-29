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
	echo ${COMPONENT_PATH}/generate_channel_table.py ${COMPONENT_PATH}/${CONFIG_LASTLINK_CHANNEL_TABLE}.channels -d $(notdir $(patsubst %_table.h,%,$(@))) -o $@
	${COMPONENT_PATH}/generate_channel_table.py ${COMPONENT_PATH}/${CONFIG_LASTLINK_CHANNEL_TABLE}.channels -d $(notdir $(patsubst %_table.h,%,$(@))) -o $@


commands.o:           $(COMPONENT_PATH)/commands.c

configdata.o:         $(COMPONENT_PATH)/configdata.c

crc16.o:              $(COMPONENT_PATH)/crc16.c

duplicate_sequence.o: $(COMPONENT_PATH)/duplicate_sequence.c

expressions.o:        $(COMPONENT_PATH)/expressions.c

gps.o:                $(COMPONENT_PATH)/gps.c

httpd_server.o:       $(COMPONENT_PATH)/httpd_server.c

http_template.o:      $(COMPONENT_PATH)/http_templates.c

lastlink_main.o:      $(COMPONENT_PATH)/lastlink_main.c

linklayer.o:          $(COMPONENT_PATH)/linklayer.c

linklayer_io.o:       $(COMPONENT_PATH)/linklayer_io.c

lsocket.o:            $(COMPONENT_PATH)/lsocket.c

mdns_config.o:        $(COMPONENT_PATH)/mdns_config.c

network_connect.o:    $(COMPONENT_PATH)/network_connect.c

nmea_parser.o:        $(COMPONENT_PATH)/nmea_parser.c

nvs_support.o:        $(COMPONENT_PATH)/nvs_support.c

os_specific.o:        $(COMPONENT_PATH)/os_specific.c

packets.o:            $(COMPONENT_PATH)/packets.c

packet_window.o:      $(COMPONENT_PATH)/packet_window.c

power_manager.o:      $(COMPONENT_PATH)/power_manager.c

routes.o:             $(COMPONENT_PATH)/routes.c

sensors.o:            $(COMPONENT_PATH)/sensors.c

service_names.o:      $(COMPONENT_PATH)/service_names.c

spiffs.o:             $(COMPONENT_PATH)/spiffs.c

sx126x_driver.o:	  $(COMPONENT_PATH)/sx126x_driver.c $(COMPONENT_PATH)/sx126x_table.h

sx127x_driver.o:	  $(COMPONENT_PATH)/sx127x_driver.c $(COMPONENT_PATH)/sx127x_table.h

tokenize.o:           $(COMPONENT_PATH)/tokenize.c

uuid.o:               $(COMPONENT_PATH)/uuid.c

varlist.o:            $(COMPONENT_PATH)/varlist.c

vfs_lastlink.o:       $(COMPONENT_PATH)/vfs_lastlink.c


#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

COMPONENT_EMBED_TXTFILES := certs/cacert.pem
COMPONENT_EMBED_TXTFILES += certs/prvtkey.pem

