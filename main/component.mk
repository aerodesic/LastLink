#
# Main component makefile.
#
# This Makefile can be left empty. By default, it will take the sources in the 
# src/ directory, compile them and link them into lib(subdirectory_name).a 
# in the build directory. This behaviour is entirely configurable,
# please read the ESP-IDF documents if you need to do this.
#

# Depedency here just to test things out.
linklayer.o: sx126x_table.h

# Eventually:
# sx127x_driver.c:	sx127x_table.h
# sx126x_driver.c:	sx126x_table.h


# sx127x_table.h: $(COMPONENT_PATH)/$(CONFIG_LASTLINK_CHANNEL_TABLE).channels
sx127x_table.h: $(shell echo $(COMPONENT_PATH)/$(CONFIG_LASTLINK_CHANNEL_TABLE).channels)
	echo Generating $@
	$(COMPONENT_PATH)/generate_channel_table.py $< -d sx127x -o $@
	
sx126x_table.h: $(shell echo $(COMPONENT_PATH)/$(CONFIG_LASTLINK_CHANNEL_TABLE).channels)
	echo Generating $@
	$(COMPONENT_PATH)/generate_channel_table.py $< -d sx126x -o $@
	

