#!/bin/sh

AUTOTESTDIR=$(dirname $0)

fgfs \
    --native-fdm=socket,in,10,,5503,udp \
    --fdm=external \
    --aircraft=arducopter \
    --control=mouse \
    --fg-aircraft="$AUTOTESTDIR/aircraft" \
    --disable-intro-music \
    --airport=YKRY \
    --geometry=650x550 \
    --bpp=32 \
    --disable-anti-alias-hud \
    --disable-hud-3d \
    --disable-horizon-effect \
    --timeofday=noon \
    --disable-sound \
    --disable-fullscreen \
    --disable-random-objects \
    --disable-ai-models \
    --fog-disable \
    --disable-specular-highlight \
    --disable-skyblend \
    --disable-anti-alias-hud \
    --wind=0@0 \
    $*
