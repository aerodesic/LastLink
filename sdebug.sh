#!/bin/bash
# Flash all devices seen
pattern="/dev/ttyUSB*"
devices=`echo $pattern`
if [ "$pattern" = "$devices" ]; then
    echo no devices found;
else
    # Rebuild the image before flashing
    if ! make -j8; then
        echo Compile errors
    else
        for dev in $devices; do
            echo Starting flash on $dev
            gnome-terminal --window-with-profile=default --geometry 230x14 --hide-menubar  -- bash -c "
                echo -ne '\033]0;Debug $dev\007'
                ESPPORT=$dev make monitor
            "
        done
    fi
fi
