MAVLink Micro Air Vehicle Message Marshalling Library

This is a library for lightweight communication between
Micro Air Vehicles (swarm) and/or ground control stations.

It serializes C-structs for serial channels and can be used with
any type of radio modem.

To generate/update packets, select mavlink_standard_message.xml
in the QGroundControl station settings view, select mavlink/include as
the output directory and click on "Save and Generate".
You will find the newly generated/updated message_xx.h files in
the mavlink/include/generated folder.

To use MAVLink, #include the <mavlink.h> file, not the individual
message files. In some cases you will have to add the main folder to the include search path as well. To be safe, we recommend these flags:

gcc -I mavlink/include -I mavlink/include/<your message set, e.g. common>

For more information, please visit:

http://pixhawk.ethz.ch/wiki/software/mavlink/

(c) 2009-2011 Lorenz Meier <pixhawk@switched.com> / PIXHAWK Team

