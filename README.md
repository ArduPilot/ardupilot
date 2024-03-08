### Instalacja

```
git clone --recurse-submodules git@github.com:Sentinel-Eagle/ardupilot.git
cd ardupilot
docker build . -t ardupilot
```

### Uruchomienie - X11

```
docker run \
    -e DISPLAY=unix$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --privileged --rm -it \
    -v `pwd`:/ardupilot \
    ardupilot:latest bash
```

Żeby uruchomić symulator i mapę:
```
./Tools/autotest/sim_vehicle.py --console --map -v ArduPlane
```

Żeby uruchomić FlightGear 3D View:
```
cd /ardupilot/Tools/autotest/
./fg_plane_view.sh
```
potem:
```
sim_vehicle.py -L KSFO
```

### Uruchomienie - Wayland

```
xhost +SI:localuser:$(id -un)
docker run \
    -e DISPLAY=$DISPLAY \
    -e XDG_RUNTIME_DIR=/tmp \
    -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    -e GDK_BACKEND=wayland \
    -v $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY \
    --privileged --rm -it \
    -v `pwd`:/ardupilot \
    --user=$(id -u):$(id -g) \
    --network=host \
    ardupilot:latest bash
```

Reszta tak samo jak X11.
