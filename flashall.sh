#!/bin/bash
# Flash all devices seen
pattern="/dev/ttyUSB*"
devices=`echo $pattern`
echo pattern "$pattern" devices "$devices"
if [ "$pattern" = "$devices" ]; then
    echo no devices found;
else
    # Rebuild the image before flashing
    if ! make -j8; then
        echo Compile errors
    else
        for dev in $devices; do
            echo Starting flash on $dev
            gnome-terminal --window-with-profile=default --geometry 50x10 --hide-menubar -- bash -c "
                echo -ne '\033]0;Flashing to $dev\007'
                if ! ESPPORT=$dev make flash; then
                    read -n1 -r -p 'Press key to continue...' key;
                fi
            "
        done
    fi
fi
