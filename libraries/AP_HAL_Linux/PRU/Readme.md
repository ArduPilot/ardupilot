Steps to manually test PRU PWM:

    1.  cp test_overlays/BB-BONE-PRU-05-00A0.dtbo /lib/firmware/
    2.  cp firmware/testpru0 firmware/testpru1 /lib/firmware/
    3.  echo BB-BONE-PRU-05 > /sys/devices/bone_capemgr.#/slots
    4.  cd /sys/class/pwm/
    5.  echo # > export       e.g. "echo 1 > export" to activate PRU_R30 pin1 as pwm
    6.  cd pwm1/
    7.  echo 1100000 > duty_ms
    8.  echo 20000000 > period_ms
    9.  echo 1 > run
    
Note: The above example will produce pwm signal with on time 1100ms and frequency 50Hz
