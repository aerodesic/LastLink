#
# Main component makefile.
#
# This Makefile can be left empty. By default, it will take the sources in the 
# src/ directory, compile them and link them into lib(subdirectory_name).a 
# in the build directory. This behaviour is entirely configurable,
# please read the ESP-IDF documents if you need to do this.
#

CHANNEL_FILES   = $(wildcard $(COMPONENT_PATH)/*.channels)
CHANNEL_FILES_H = $(foreach f, $(CHANNEL_FILES), $(subst .channels,.h,$f))

COMPONENT_EXTRA_CLEAN := $(CHANNEL_FILES_H)

# Depedency here just to test things out.
linklayer.o: frequency_tables

# Eventually we will do
# sx127x.o: frequency_tables
# sx126x.o: frequency_tables

.PHONY: frequence_tables

$(COMPONENT_LIBRARY): frequency_tables

frequency_tables: $(CHANNEL_FILES_H)
#	echo CHANNEL_FILES = $(CHANNEL_FILES)
#	echo CHANNEL_FILES_H = $(CHANNEL_FILES_H)

%.h: %.channels 
	echo Creating $@
	$(COMPONENT_PATH)/generate_channel_table.py $< >$@
