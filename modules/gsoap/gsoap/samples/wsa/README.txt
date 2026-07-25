
This application demonstrates server-side and client-side logic for services
based on WS-Addressing. At the server side, WS-Addressing enables
forwarding/relaying of service responses and faults to other services. At the
client side, a relayed response or fault will not be received and an HTTP
ACCEPTED (code 202) is delivered instead, assuming that the relay was
successful.

For more details, please see wsademo.h. Instructions about the demo are
included therein.
