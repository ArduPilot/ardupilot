/*
  application -> bootloader communication structure This is put into
  the start of RAM by AP_Periph to facilitate firmware upload with
  UAVCAN
 */

#define APP_BOOTLOADER_COMMS_MAGIC 0xc544ad9a

struct app_bootloader_comms {
    uint32_t magic;
    uint32_t reserved[4];
    uint8_t server_node_id;
    uint8_t my_node_id;
    uint8_t path[201];
};
