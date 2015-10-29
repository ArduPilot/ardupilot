if [ "`echo $1`" = "load" ]; then
    echo "Loading Capes..."
    echo BB-BBBMINI > /sys/devices/bone_capemgr.9/slots
    echo am33xx_pwm > /sys/devices/bone_capemgr.9/slots
    echo bone_pwm_P8_36 > /sys/devices/bone_capemgr.9/slots
    dmesg | grep "SPI"
    dmesg | grep "PRU"
    cat /sys/devices/bone_capemgr.9/slots
else
    echo "Usage:"
    echo "      ./startup.sh load  : to load the capes"
fi
