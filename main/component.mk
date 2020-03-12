#
# Main component makefile.
#
# This Makefile can be left empty. By default, it will take the sources in the 
# src/ directory, compile them and link them into lib(subdirectory_name).a 
# in the build directory. This behaviour is entirely configurable,
# please read the ESP-IDF documents if you need to do this.
#

# Enumerate the drivers
PACKET_DRIVERS = $(wildcard $(COMPONENT_PATH)/*_driver.c)

# Create dependency on table.h for each driver
PACKET_TABLES = $(patsubst %_driver.c,%_table.h, $(PACKET_DRIVERS))

# The drivers depend on the table being created
%_table.hc: $(shell echo $(COMPONENT_PATH)/$(CONFIG_LASTLINK_CHANNEL_TABLE).channels)
	echo Generating $@ from $< for $(subst _table.h,,$(notdir $@))
	$(COMPONENT_PATH)/generate_channel_table.py $< -d $(subst _table.h,,$(notdir $@)) -o $@

$(PACKET_DRIVERS): $(PACKET_TABLES)


