# Shared by the multi-vehicle Webots scripts: source it, do not run it.
#
#   launch_sitl TITLE MODEL PORT sim_vehicle.py-args...
#
# Starts one ArduCopter instance in its own xterm, attached to the Webots
# controller listening on 127.0.0.1:PORT.
#
# Why this is more than a bare "xterm -e sim_vehicle.py":
#  - sim_vehicle.py only cleans up leftovers from earlier runs for instance 0.
#    Closing the xterm kills sim_vehicle.py but not the arducopter it started
#    in a separate, minimised window, so old instances piled up, kept their
#    SERIAL ports bound, and every later launch died with "bind failed ...
#    Address already in use" before it ever connected to Webots.  We kill any
#    arducopter still driving this controller port before starting, and again
#    when this xterm exits or is closed.
#  - every instance gets its own working directory, build/sitl_webots_PORT,
#    so they do not share (and wipe) the same eeprom.bin and logs.

ROOTDIR=${ROOTDIR:-$PWD}

launch_sitl() {
    local title=$1 model=$2 port=$3
    shift 3

    # arducopter processes (and the minimised xterms holding them) that talk
    # to this Webots controller port -- and nothing else
    local stale="^(xterm .* -e )?[^ ]*/arducopter .*--model webots-[a-z]*:127\.0\.0\.1:$port( |\$)"
    if pkill -f "$stale"; then
        echo "$title: stopped a leftover SITL instance on port $port"
        sleep 1
    fi

    local inner
    inner="trap $(printf %q "pkill -f $(printf %q "$stale")") EXIT HUP; "
    inner+=$(printf '%q ' "$ROOTDIR/Tools/autotest/sim_vehicle.py" "$@" \
        --use-dir="$ROOTDIR/build/sitl_webots_$port" --model "$model:127.0.0.1:$port")
    xterm -title "$title" -e bash -c "$inner" &
}
