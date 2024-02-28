# UDP Proxy

This is a tool to do UDP proxying, particularly for MAVLink
connections. It is useful when operating both a ground station and
aircraft on network links that don't have a public IP address.

# Functionality

udpproxy opens two listening UDP ports. When it has a connection on
both ports then it will forward packets between the ports. This allows
your GCS to connect to one of the ports and your aircraft to connect
to the other port. The GCS and aircraft will be able to communicate,
despite both not having public IP addresses.

# Why not a VPN?

udpproxy is an alternative to using a VPN for communication between
the aircraft and the GCS. The reason for not using a VPN in flight is
VPNs typically have a high reconnect time, and often add significant
latency. This poses an issue for aircraft control as you may lose the
ability to control the aircraft for minutes if there is a short
network outage. Using udpproxy minimises the time for the link to
re-establish after a network outage.

# Disadvantages

The main disadvantage of udpproxy is that it offers no security. If
someone knows that UDP ports and host you are using then they could
connect to your aircraft and control it. The risk can be reduced by
enabling MAVLink2 signing which allows you to ensure that nobody can
control the aircraft without knowing the signing key.

You can also reduce the risk by using firewall rules on the computer
to run the proxy on to only allow connections from the IP ranges you
known you will be using.

# Building

Just run 'make' command

# Usage

Basic usage is:

  udpproxy PORT1 PORT2

this will listen on both PORT1 and PORT2. You should then make an
outgoing UDP connection from both GCS and aircraft to those ports, one
to each port.

Adding the -v option tells udpproxy to display information about new
connections and shows transfer rates which are useful for diagnostics.

You should run udpproxy on a computer with a public IP address.

# Keeping it running

You will typically want to keep udpproxy running for long periods
without having to keep a shell open on the computer running the
proxy. An example script which starts it under GNU screen and thus
allows you to monitor the connections and automatically restart them
is provided in this directory.

# Connecting

To connect from mavproxy to your proxy just add this to the
mavproxy.py command line:

  --out AA.BB.CC.DD:PORT1

where AA.BB.CC.DD is the IPv4 network address of your proxy.

To connect from MissionPlanner use the "UDPCL" option, and enter the
IP address and port number of the proxy.
