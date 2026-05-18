# sitl_periph_PPP

SITL AP_Periph build with the PPP networking backend enabled.

Intended for testing PPP between an ArduPlane SITL and an AP_Periph SITL over a
loopback TCP connection (the SITL UART backend's `tcp:`/`tcpclient:` modes carry
the PPP frames in place of a real serial link). Pair with the `quadplane-PPP`
frame in `Tools/autotest/pysim/vehicleinfo.json` and launch via:

```sh
sim_vehicle.py -v Plane -f quadplane-PPP --enable-ppp --enable-networking-tests
```
