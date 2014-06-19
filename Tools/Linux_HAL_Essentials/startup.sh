if [ "`echo $1`" = "load" ]; then
    echo "Loading Test_Capes..."
    cp BB-SPI0-SWP-01-00A0.dtbo /lib/firmware/
    cp BB-SPI1-SWP-01-00A0.dtbo /lib/firmware/
    cp BB-BONE-PRU-05-00A0.dtbo /lib/firmware/
    cp testpru0 /lib/firmware
    cp testpru1 /lib/firmware
    echo BB-BONE-PRU-05 > /sys/devices/bone_capemgr.*/slots
    echo BB-SPI0-SWP-01 > /sys/devices/bone_capemgr.*/slots
    echo BB-SPI1-SWP-01 > /sys/devices/bone_capemgr.*/slots
    dmesg | grep "SPI"
    dmesg | grep "PRU"
else
    echo "Usage:"
    echo "      ./startup.sh load  : to load the capes"
fi
