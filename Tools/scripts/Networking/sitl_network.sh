#!/usr/bin/env bash
#
# Set up a host-side TAP interface and NAT so SITL builds with
# AP_NETWORKING_BACKEND_SITL_TUN can be reached from the host and so the
# SITL processes (e.g. the sitl_periph_PPP periph and the ArduPlane
# behind it on the PPP link) can in turn reach the wider host network /
# internet via NAT masquerade.
#
# Typical use:
#
#   Tools/scripts/Networking/sitl_network.sh up     # create TAP + host route + NAT
#   ./sim_vehicle.py -v Plane -f quadplane-PPP      # periph attaches to TAP
#   curl http://10.77.193.20/                       # browse the PPPGW web UI
#   Tools/scripts/Networking/sitl_network.sh down   # tear it back down
#
# See Tools/scripts/Networking/README.md for details.
#
# Defaults match the 10.77.193.x/24 plumbing used by quadplane-PPP:
#   DEV     - tap device name              (default: sitltap)
#   HOST_IP - address given to the host    (default: 10.77.193.1/24)
#   SUBNET  - subnet that gets NAT'd       (default: 10.77.193.0/24)
#   PEER_IP - the SITL periph's IP, used   (default: 10.77.193.20)
#             only for the friendly print
#
# The TAP device is created with `user $USER` so the SITL process does NOT
# need to run as root - it just opens the pre-existing /dev/net/tun and
# does TUNSETIFF on the named device.
#
# A small state file (/tmp/sitl_network.state) records the outbound
# interface we MASQUERADE through so `down` can undo exactly what `up`
# put in place.

set -e

DEV="${DEV:-sitltap}"
HOST_IP="${HOST_IP:-10.77.193.1/24}"
SUBNET="${SUBNET:-10.77.193.0/24}"
PEER_IP="${PEER_IP:-10.77.193.20}"
STATE_FILE="${STATE_FILE:-/tmp/sitl_network.state}"
ACTION="${1:-up}"

SUDO="sudo"
if [ "$(id -u)" = "0" ]; then
    SUDO=""
fi

detect_out_dev() {
    ip route show default 2>/dev/null | awk '/^default/{print $5; exit}'
}

ip_forward_was() {
    cat /proc/sys/net/ipv4/ip_forward 2>/dev/null || echo 0
}

case "$ACTION" in
    up)
        if ! ip link show "$DEV" >/dev/null 2>&1; then
            echo "Creating TAP device $DEV (owner: $USER)"
            $SUDO ip tuntap add dev "$DEV" mode tap user "$USER"
        fi
        $SUDO ip link set "$DEV" up
        if ! ip addr show "$DEV" | grep -q "${HOST_IP%/*}"; then
            echo "Assigning $HOST_IP to $DEV"
            $SUDO ip addr add "$HOST_IP" dev "$DEV"
        fi

        # NAT setup: detect outbound interface, remember the previous
        # ip_forward setting, then masquerade traffic from $SUBNET out
        # through it. The matching FORWARD rules are needed because most
        # default firewall configs DROP forwarded traffic.
        OUT_DEV="$(detect_out_dev)"
        if [ -z "$OUT_DEV" ]; then
            echo "warning: no default route found; skipping NAT setup" >&2
        else
            PREV_IP_FORWARD="$(ip_forward_was)"
            echo "Enabling NAT: $SUBNET -> $OUT_DEV (MASQUERADE)"
            if [ "$PREV_IP_FORWARD" != "1" ]; then
                $SUDO sysctl -w net.ipv4.ip_forward=1 >/dev/null
            fi
            # idempotent: skip if rule already present
            if ! $SUDO iptables -t nat -C POSTROUTING -s "$SUBNET" -o "$OUT_DEV" -j MASQUERADE 2>/dev/null; then
                $SUDO iptables -t nat -A POSTROUTING -s "$SUBNET" -o "$OUT_DEV" -j MASQUERADE
            fi
            if ! $SUDO iptables -C FORWARD -i "$DEV" -o "$OUT_DEV" -j ACCEPT 2>/dev/null; then
                $SUDO iptables -A FORWARD -i "$DEV" -o "$OUT_DEV" -j ACCEPT
            fi
            if ! $SUDO iptables -C FORWARD -i "$OUT_DEV" -o "$DEV" -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT 2>/dev/null; then
                $SUDO iptables -A FORWARD -i "$OUT_DEV" -o "$DEV" -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
            fi
            # state file lets `down` undo exactly what we added
            {
                echo "DEV=$DEV"
                echo "SUBNET=$SUBNET"
                echo "OUT_DEV=$OUT_DEV"
                echo "PREV_IP_FORWARD=$PREV_IP_FORWARD"
            } > "$STATE_FILE"
        fi

        echo "TAP $DEV is up."
        echo "Once SITL is running, the periph should be reachable at: $PEER_IP"
        echo "Example: curl http://$PEER_IP/"
        ;;
    down)
        # tear down NAT first (in reverse order)
        if [ -f "$STATE_FILE" ]; then
            # shellcheck disable=SC1090
            . "$STATE_FILE"
            echo "Removing NAT rules for $SUBNET via $OUT_DEV"
            $SUDO iptables -t nat -D POSTROUTING -s "$SUBNET" -o "$OUT_DEV" -j MASQUERADE 2>/dev/null || true
            $SUDO iptables -D FORWARD -i "$DEV" -o "$OUT_DEV" -j ACCEPT 2>/dev/null || true
            $SUDO iptables -D FORWARD -i "$OUT_DEV" -o "$DEV" -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true
            if [ "$PREV_IP_FORWARD" = "0" ]; then
                $SUDO sysctl -w net.ipv4.ip_forward=0 >/dev/null
            fi
            rm -f "$STATE_FILE"
        fi
        if ip link show "$DEV" >/dev/null 2>&1; then
            echo "Removing TAP device $DEV"
            $SUDO ip link del "$DEV"
        else
            echo "TAP $DEV does not exist"
        fi
        ;;
    *)
        echo "Usage: $0 {up|down}" >&2
        exit 2
        ;;
esac
