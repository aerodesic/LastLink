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


sx127x_driver.o:	$(COMPONENT_PATH)/sx127x_table.h

sx126x_driver.o:	$(COMPONENT_PATH)/sx126x_table.h


