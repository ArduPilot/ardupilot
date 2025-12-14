"export PWD=/mnt",
"export HOME=/",

"netmanager",

"mkdir -s /mnt",
"mount -t vfat /dev/mmc0 /mnt",
"cd /mnt",

//"arducopter -S -w --model copter --speedup 8 --defaults /copter.parm --slave 0 --sim-address=10.0.2.10 -I0",
"arducopter --serial0=udp:10.0.2.10:14550",

"tish",
