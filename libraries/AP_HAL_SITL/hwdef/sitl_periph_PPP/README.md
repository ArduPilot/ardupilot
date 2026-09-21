# sitl_periph_PPP

SITL AP_Periph build with the PPP networking backend enabled.

Intended for testing PPP between an ArduPlane SITL and an AP_Periph SITL over a
loopback TCP connection (the SITL UART backend's `tcp:`/`tcpclient:` modes carry
the PPP frames in place of a real serial link). Pair with the `quadplane-PPP`
frame in `Tools/autotest/pysim/vehicleinfo.json` and launch via:

```sh
sim_vehicle.py -v Plane -f quadplane-PPP
```

The frame's `configure_args` in `vehicleinfo.json` adds `--enable-PPP` and
`--enable-networking-tests` to the waf configure step automatically; this
periph board's hwdef already defines `AP_NETWORKING_BACKEND_PPP=1` itself.
