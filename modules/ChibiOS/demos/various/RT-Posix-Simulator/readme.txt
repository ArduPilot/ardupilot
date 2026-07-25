*****************************************************************************
** ChibiOS/RT port for x86 into a Posix process                            **
*****************************************************************************

** TARGET **

The demo runs under any Posix IA32 system as an application program. The serial
I/O is simulated over TCP/IP sockets.

** The Demo **

The demo listens on the two serial ports, when a connection is detected a
thread is started that serves a small command shell.
The demo shows how to create/terminate threads at runtime, how to listen to
events, how to work with serial ports, how to use the messages.
You can develop your ChibiOS/RT application using this demo as a simulator
then you can recompile it for a different architecture.
See demo.c for details.

** Build Procedure **

The demo was built using GCC.

** Connect to the demo **

In order to connect to the demo a telnet client is required.

Host Name: 127.0.0.1
Port: 29001 and/or 29002
Connection Type: Raw

