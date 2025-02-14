define CH_CFG_USE_EVENTS 1
undef CH_CFG_USE_SEMAPHORES
define CH_CFG_USE_SEMAPHORES 1
undef CH_CFG_USE_MUTEXES
define CH_CFG_USE_MUTEXES 1
undef CH_CFG_USE_HEAP
define CH_CFG_USE_MEMCORE 1
define CH_CFG_USE_HEAP 1
define CH_CFG_USE_REGISTRY 1
define CH_CFG_USE_DYNAMIC 1
define CH_CFG_USE_WAITEXIT 1

define AP_NETWORKING_ENABLED 1
define AP_NETWORKING_CAN_MCAST_ENABLED 1

ROMFS_WILDCARD Tools/AP_Bootloader/Web/*.html
env ROMFS_UNCOMPRESSED True

# we don't want to use external flash with network bootloader
EXT_FLASH_SIZE_MB 0

# don't combine sdcard bl and network bl
undef AP_BOOTLOADER_FLASH_FROM_SD_ENABLED

define CH_CFG_THREAD_EXTRA_FIELDS struct ch_thread *delete_next;
