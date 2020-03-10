#
# Main component makefile.
#
# This Makefile can be left empty. By default, it will take the sources in the 
# src/ directory, compile them and link them into lib(subdirectory_name).a 
# in the build directory. This behaviour is entirely configurable,
# please read the ESP-IDF documents if you need to do this.
#
COMPONENT_EXTRA_CLEAN := frequency_table.c

$(COMPONENT_LIBRARY): frequency_table.c

frequency_table.c:
	# run generate_frequency_table on frequency_table.py
	echo "/*Frequency Table*/" > $(COMPONENT_PATH)/frequency_table.c



