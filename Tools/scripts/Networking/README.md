# Tools/scripts/Networking

Host-side helpers for ArduPilot's SITL networking. The scripts here
manage the host plumbing that lets SITL binaries with the optional
`AP_NETWORKING_BACKEND_SITL_TUN` backend bridge their lwIP stack to a
real TAP device and reach (or be reached from) the wider host network.

## `sitl_network.sh`

Brings up a host-side TAP interface, assigns the host an address on
it, and installs iptables rules so the SITL subnet has NAT'd
outbound reachability through the host's default outbound interface.

### Usage

```sh
Tools/scripts/Networking/sitl_network.sh up     # create TAP + NAT
sim_vehicle.py -v Plane -f quadplane-PPP        # SITL processes start
curl http://10.77.193.20/                       # browse the PPPGW web UI
Tools/scripts/Networking/sitl_network.sh down   # tear it back down
```

Both subcommands require `sudo` only for the privileged steps (TAP
creation, `iptables -t nat`, `sysctl net.ipv4.ip_forward`). The TAP
device itself is created with `user $USER` so the SITL binary opens
`/dev/net/tun` without any extra privileges.

The script writes a small `/tmp/sitl_network.state` file on `up` so
that `down` undoes exactly what was put in place (the outbound
interface used for MASQUERADE, and the previous
`net.ipv4.ip_forward` value).

### What `up` does

1. Creates `sitltap` (TAP, persistent, owned by `$USER`) if it doesn't
   already exist, sets it `up`, and assigns the host `10.77.193.1/24`.
2. Detects the default outbound interface from `ip route`.
3. Enables `net.ipv4.ip_forward` (if it wasn't on already).
4. Adds an `iptables -t nat -A POSTROUTING ... -j MASQUERADE` rule for
   `10.77.193.0/24` leaving via the outbound interface, plus matching
   `FORWARD` ACCEPTs so the kernel actually forwards.

### What `down` does

Reverses everything `up` set up, in the opposite order. Safe to call
even if nothing is up (it just prints "TAP does not exist").

### Customising

A few environment variables let you change the defaults:

| Variable  | Default        | Meaning |
|-----------|----------------|---------|
| `DEV`     | `sitltap`      | TAP interface name |
| `HOST_IP` | `10.77.193.1/24` | Address assigned to the host on the TAP |
| `SUBNET`  | `10.77.193.0/24` | Subnet the MASQUERADE rule covers |
| `PEER_IP` | `10.77.193.20` | Just used in the friendly "where to curl" message |

### Companion SITL build

This script is paired with the
`libraries/AP_HAL_SITL/hwdef/sitl_periph_PPP` build target, which has
`AP_NETWORKING_BACKEND_SITL_TUN` enabled and a periph-side
`NET_GWADDR=10.77.193.1` default so the periph's lwIP routes
out-of-subnet traffic via the host TAP. With the script's NAT rules in
place, both the periph and the ArduPlane behind it on the PPP link can
reach the wider host network and the public internet; with NAT off,
only the developer-side reachability (curl the PPPGW web UI on
`10.77.193.20`) still works.

If `sitl_network.sh up` has not been run, the SITL_TUN backend falls
back to a "PPPGW-only, no host bridge" mode: PPP and IPCP still work,
the periph is just not reachable from the host.
