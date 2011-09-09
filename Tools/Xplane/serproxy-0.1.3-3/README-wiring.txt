Using Wiring with Flash (via serproxy)
--------------------------------------
Connect the Wiring board to a USB port.  Take note of which USB port you use, as different ones have different identifiers.

Launch Wiring.  Look in the Tools | Serial Port menu and take note of the identifier corresponding to the serial device for the USB port to which the Wiring board is connected (look for something WITHOUT "modem" in its name).

Open serproxy.cfg in TextEdit.

The serproxy can handle multiple serial devices (each of which corresponds to a USB or serial port).  For example:

comm_ports=1,2

tells serproxy to use two comm ports, numbered 1 and 2.  To tell serproxy which serial device (from the Wiring menu) corresponds to each comm port, use a line like this:

serial_device1=/dev/tty.usbserial-1B1

This tells serproxy that comm port 1 uses serial device /dev/tty.usbserial-1B1.  Change this to the identifier from the Wiring menu.

Then, you assign a network port the comm port:

net_port1=5331

This tells serproxy to listen for connections on port 5331.  Set this to whatever port you wish to connect to from Flash (on host "localhost").

To configure another port, repeat the above instructions, plugging the Wiring board into a different USB port (you'll have to quit Wiring, plug into the new USB port, and then relaunch Wiring).  Then add the appropriate lines to the serproxy.cfg.  For example:

serial_device2=/dev/tty.usbserial-3B1
net_port2=5332

Will configure comm port 2 to use serial device /dev/tty.usbserial-3B1 and listen for connections from Flash on port 5332.

Each serial device will always correspond to the same network port, so if you plug the Wiring board into a different USB port, you'll need to connect to a different network port from Flash.

If you need more than two comm ports, you'll need to change the comm_ports line.  For example:

comm_ports=1,2,3,4

allocates four comm ports numbered 1 through 4.

You need to make sure that the baud rate specified by your Wiring code:

beginSerial(19200);

matches the baud rate specified in serproxy.cfg:

comm_baud=19200

Finally, Flash will not process any incoming data (i.e. call the onData() function of your XMLSocket) until it receives a nil (ASCII code 0, '\0').  If your Wiring code uses newlines (ASCII code 10, '\n') to delimit data (so that, for example, it's easier to read in the Wiring serial monitor, HyperTerminal, zterm, Terminal, etc), you can tell serproxy to convert them into nils:

newlines_to_nils=true

To launch serproxy, simply double-click it.  (This version does not require a .command file.)
--
David A. Mellis
7 June 2005
