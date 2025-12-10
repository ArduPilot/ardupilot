"export PWD=/mnt",
"export HOME=/",

"mkdir -s /mnt",
"mount -t vfat /dev/mmc0 /mnt",
"cd /mnt",

"arducopter --serial0=/dev/ttyS1",
