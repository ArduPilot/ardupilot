/*
  support checking board ID and firmware CRC in the bootloader
 */
#include "AP_CheckFirmware.h"
#include <AP_HAL/HAL.h>

#if AP_CHECK_FIRMWARE_ENABLED && AP_SIGNED_FIRMWARE && !defined(HAL_BOOTLOADER_BUILD)

#include "monocypher.h"
#include <AP_Math/crc.h>

extern const AP_HAL::HAL &hal;

uint8_t AP_CheckFirmware::session_key[8];

/*
  make a session key
 */
static void make_session_key(uint8_t key[8])
{
    struct {
        uint32_t time_us;
        uint8_t unique_id[12];
        uint16_t rand1;
        uint16_t rand2;
    } data {};
    static_assert(sizeof(data) % 4 == 0, "data must be multiple of 4 bytes");

    // get data which will not apply on a different board, and includes some randomness
    uint8_t uid_len = 12;
    hal.util->get_system_id_unformatted(data.unique_id, uid_len);
    data.time_us = AP_HAL::micros();
    data.rand1 = get_random16();
    data.rand2 = get_random16();
    const uint64_t c64 = crc_crc64((const uint32_t *)&data, sizeof(data)/sizeof(uint32_t));
    memcpy(key, (uint8_t *)&c64, 8);
}

/*
  find public keys in bootloader, or return NULL if signature not found

  this assumes the public keys are in the first sector
 */
const struct ap_secure_data *AP_CheckFirmware::find_public_keys(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    const uint32_t page_size = hal.flash->getpagesize(0);
    const uint32_t flash_addr = hal.flash->getpageaddr(0);
    const uint8_t *flash = (const uint8_t *)flash_addr;
    const uint8_t key[] = AP_PUBLIC_KEY_SIGNATURE;
    return (const struct ap_secure_data *)memmem(flash, page_size, key, sizeof(key));
#else
    return nullptr;
#endif
}

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
/*
  return true if 1k of data is all 0xff (empty flash)
 */
static bool empty_1k(const uint8_t *data)
{
    for (uint32_t i=0; i<1024; i++) {
        if (data[i] != 0xFFU) {
            return false;
        }
    }
    return true;
}
#endif

/*
  read bootloader into memory. This is complicated by the potential presence
  of persistent data from temperature calibration at the end of the sector

  Also note this assumes the public keys are in the first sector if
  the bootloader covers more than one sector. This is a reasonable
  assumption given the linker file
 */
AP_CheckFirmware::bl_data *AP_CheckFirmware::read_bootloader(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    struct bl_data *bld = new bl_data;
    if (bld == nullptr) {
        return nullptr;
    }
    const uint32_t page_size = hal.flash->getpagesize(0);
    const uint32_t flash_addr = hal.flash->getpageaddr(0);
    const uint8_t *flash = (uint8_t *)flash_addr;
    const uint16_t block_size = 1024;
    uint16_t num_blocks = page_size / block_size;
    /*
      find first empty block
     */
    for (uint16_t i=0; i<num_blocks; i++) {
        if (empty_1k(&flash[block_size*i])) {
            break;
        }
        bld->length1 += block_size;
    }
    bld->data1 = new uint8_t[bld->length1];
    if (bld->data1 == nullptr) {
        delete bld;
        return nullptr;
    }
    memcpy(bld->data1, flash, bld->length1);
    flash += bld->length1;
    num_blocks -= bld->length1 / block_size;

    /*
      find first non-empty block, which should be the persistent data if-any
     */
    bld->offset2 = bld->length1;
    while (num_blocks > 0) {
        if (!empty_1k(&flash[bld->offset2])) {
            break;
        }
        num_blocks--;
        bld->offset2 += block_size;
    }
    if (num_blocks > 0) {
        // we have persistent data to save
        bld->length2 = num_blocks * block_size;
        bld->data2 = new uint8_t[bld->length2];
        if (bld->data2 == nullptr) {
            delete bld;
            return nullptr;
        }
        memcpy(bld->data2, &flash[bld->offset2], bld->length2);
    }
    return bld;
#else
    return nullptr;
#endif
}

/*
  write bootloader from memory
 */
bool AP_CheckFirmware::write_bootloader(const struct bl_data *bld)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    const uint32_t flash_addr = hal.flash->getpageaddr(0);
    EXPECT_DELAY_MS(3000);
    if (!hal.flash->erasepage(0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Bootloader erase failed");
        return false;
    }
    EXPECT_DELAY_MS(3000);
    if (!hal.flash->write(flash_addr, bld->data1, bld->length1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Bootloader write1 failed");
        return false;
    }
    EXPECT_DELAY_MS(3000);
    if (bld->length2 != 0 &&
        !hal.flash->write(flash_addr+bld->offset2, bld->data2, bld->length2)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Bootloader write1 failed");
        return false;
    }
    return true;
#else
    return false;
#endif
}

/*
  return true if all keys are zeros
 */
bool AP_CheckFirmware::all_zero_keys(const struct ap_secure_data *sec_data)
{
    const uint8_t zero_key[AP_PUBLIC_KEY_LEN] {};
    /*
      look over all public keys, if one matches then we are OK
     */
    for (const auto &public_key : sec_data->public_key) {
        if (memcmp(public_key.key, zero_key, AP_PUBLIC_KEY_LEN) != 0) {
            return false;
        }
    }
    return true;
}

/*
  check signature in a command against bootloader public keys
 */
bool AP_CheckFirmware::check_signature(const mavlink_secure_command_t &pkt)
{
    const struct ap_secure_data *sec_data = find_public_keys();
    if (sec_data == nullptr) {
        return false;
    }
    if (all_zero_keys(sec_data)) {
        // allow through if no keys are setup
        return true;
    }
    if (pkt.sig_length != 64) {
        // monocypher signatures are 64 bytes
        return false;
    }
    /*
      look over all public keys, if one matches then we are OK
     */
    for (const auto &public_key : sec_data->public_key) {
        crypto_check_ctx ctx {};
        crypto_check_ctx_abstract *actx = (crypto_check_ctx_abstract*)&ctx;
        crypto_check_init(actx, &pkt.data[pkt.data_length], public_key.key);

        crypto_check_update(actx, (const uint8_t*)&pkt.sequence, sizeof(pkt.sequence));
        crypto_check_update(actx, (const uint8_t*)&pkt.operation, sizeof(pkt.operation));
        crypto_check_update(actx, pkt.data, pkt.data_length);
        if (pkt.operation != SECURE_COMMAND_GET_SESSION_KEY) {
            crypto_check_update(actx, session_key, sizeof(session_key));
        }
        if (crypto_check_final(actx) == 0) {
            // good signature
            return true;
        }
    }
    return false;
}

/*
  set public keys in bootloader
 */
bool AP_CheckFirmware::set_public_keys(uint8_t key_idx, uint8_t num_keys, const uint8_t *key_data)
{
    auto *bld = read_bootloader();
    if (bld == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Failed to load bootloader into memory");
        return false;
    }
    const uint8_t key[] = AP_PUBLIC_KEY_SIGNATURE;
    struct ap_secure_data *sec_data = (struct ap_secure_data *)memmem(bld->data1, bld->length1, key, sizeof(key));
    if (sec_data == nullptr) {
        delete bld;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Failed to find key signature");
        return false;
    }
    memcpy(sec_data->public_key[key_idx].key, key_data, num_keys*AP_PUBLIC_KEY_LEN);

    /*
      pack so non-zero keys are at the start
     */
    const uint8_t zero_key[AP_PUBLIC_KEY_LEN] {};
    uint8_t max_keys = AP_PUBLIC_KEY_MAX_KEYS;
    for (uint8_t i=0; max_keys>1 && i<max_keys-1; i++) {
        if (memcmp(zero_key, sec_data->public_key[i].key, AP_PUBLIC_KEY_LEN) == 0) {
            memmove(sec_data->public_key[i].key, sec_data->public_key[i+1].key, AP_PUBLIC_KEY_LEN*(max_keys-(i+1)));
            max_keys--;
            i--;
        }
    }
    memset(sec_data->public_key[max_keys-1].key, 0, AP_PUBLIC_KEY_LEN*(AP_PUBLIC_KEY_MAX_KEYS-max_keys));

    bool ret = write_bootloader(bld);
    delete bld;
    return ret;
}

/*
  handle a SECURE_COMMAND
 */
void AP_CheckFirmware::handle_secure_command(mavlink_channel_t chan, const mavlink_secure_command_t &pkt)
{
    mavlink_secure_command_reply_t reply {};
    reply.result = MAV_RESULT_UNSUPPORTED;
    reply.sequence = pkt.sequence;
    reply.operation = pkt.operation;

    if (uint16_t(pkt.data_length) + uint16_t(pkt.sig_length) > sizeof(pkt.data)) {
        reply.result = MAV_RESULT_DENIED;
        goto send_reply;
    }
    if (!check_signature(pkt)) {
        reply.result = MAV_RESULT_DENIED;
        goto send_reply;
    }

    switch (pkt.operation) {

    case SECURE_COMMAND_GET_SESSION_KEY: {
        make_session_key(session_key);
        reply.data_length = sizeof(session_key);
        memcpy(reply.data, session_key, reply.data_length);
        reply.result = MAV_RESULT_ACCEPTED;
        break;
    }

    case SECURE_COMMAND_GET_PUBLIC_KEYS: {
        const struct ap_secure_data *sec_data = find_public_keys();
        if (pkt.data_length != 2) {
            reply.result = MAV_RESULT_UNSUPPORTED;
            goto send_reply;
        }
        const uint8_t key_idx = pkt.data[0];
        uint8_t num_keys = pkt.data[1];
        const uint8_t max_fetch = (sizeof(reply.data)-1) / AP_PUBLIC_KEY_LEN;
        if (key_idx >= AP_PUBLIC_KEY_MAX_KEYS ||
            num_keys > max_fetch ||
            key_idx+num_keys > AP_PUBLIC_KEY_MAX_KEYS ||
            sec_data == nullptr) {
            reply.result = MAV_RESULT_FAILED;
            goto send_reply;
        }

        // remove zero keys
        const uint8_t zero_key[AP_PUBLIC_KEY_LEN] {};
        while (num_keys > 0 &&
               memcmp(zero_key, &sec_data->public_key[key_idx+num_keys-1], AP_PUBLIC_KEY_LEN) == 0) {
            num_keys--;
        }

        reply.data_length = 1+num_keys*AP_PUBLIC_KEY_LEN;
        reply.data[0] = key_idx;
        memcpy(&reply.data[1], &sec_data->public_key[key_idx], reply.data_length-1);
        reply.result = MAV_RESULT_ACCEPTED;
        break;
    }

    case SECURE_COMMAND_SET_PUBLIC_KEYS: {
        if (pkt.data_length < AP_PUBLIC_KEY_LEN+1) {
            reply.result = MAV_RESULT_FAILED;
            goto send_reply;
        }
        const uint8_t key_idx = pkt.data[0];
        const uint8_t num_keys = (pkt.data_length-1) / AP_PUBLIC_KEY_LEN;
        if (num_keys == 0) {
            reply.result = MAV_RESULT_FAILED;
            goto send_reply;
        }
        if (key_idx >= AP_PUBLIC_KEY_MAX_KEYS ||
            key_idx+num_keys > AP_PUBLIC_KEY_MAX_KEYS) {
            reply.result = MAV_RESULT_FAILED;
            goto send_reply;
        }
        if (set_public_keys(key_idx, num_keys, &pkt.data[1])) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Bootloader update OK");
            reply.result = MAV_RESULT_ACCEPTED;
        } else {
            reply.result = MAV_RESULT_FAILED;
        }
        break;
    }

    case SECURE_COMMAND_REMOVE_PUBLIC_KEYS: {
        if (pkt.data_length != 2) {
            reply.result = MAV_RESULT_FAILED;
            goto send_reply;
        }
        const uint8_t key_idx = pkt.data[0];
        const uint8_t num_keys = pkt.data[1];
        if (num_keys == 0) {
            reply.result = MAV_RESULT_FAILED;
            goto send_reply;
        }
        if (key_idx >= AP_PUBLIC_KEY_MAX_KEYS ||
            key_idx+num_keys > AP_PUBLIC_KEY_MAX_KEYS) {
            reply.result = MAV_RESULT_FAILED;
            goto send_reply;
        }
        uint8_t *data = new uint8_t[num_keys*AP_PUBLIC_KEY_LEN];
        if (data == nullptr) {
            reply.result = MAV_RESULT_FAILED;
            goto send_reply;
        }
        if (set_public_keys(key_idx, num_keys, data)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Bootloader update OK");
            reply.result = MAV_RESULT_ACCEPTED;
        } else {
            reply.result = MAV_RESULT_FAILED;
        }
        delete[] data;
        break;
    }
    }

send_reply:
    // send reply
    mavlink_msg_secure_command_reply_send_struct(chan, &reply);
}

/*
  implement secure command operations for updating public keys
 */
void AP_CheckFirmware::handle_msg(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_SECURE_COMMAND: {
        mavlink_secure_command_t pkt;
        mavlink_msg_secure_command_decode(&msg, &pkt);
        handle_secure_command(chan, pkt);
        break;
    }
    }
}

/*
  check that a bootloader is OK to flash. We don't want to allow
  flashing of a bootloader unless we either have no public keys setup
  or the bootloader has public keys embedded. This prevents an easy
  mistake of including an insecure bootloader in ROMFS with a secure build
 */
bool AP_CheckFirmware::check_signed_bootloader(const uint8_t *fw, uint32_t fw_size)
{
    const struct ap_secure_data *sec_data = find_public_keys();
    if (sec_data == nullptr || all_zero_keys(sec_data)) {
        // current bootloader doesn't have public keys, so OK to load any bootloader
        return true;
    }
    const uint8_t key[] = AP_PUBLIC_KEY_SIGNATURE;
    sec_data = (const struct ap_secure_data *)memmem(fw, fw_size, key, sizeof(key));
    if (sec_data == nullptr || all_zero_keys(sec_data)) {
        // new bootloader doesn't have any public keys, not allowed
        return false;
    }
    return true;
}

#endif // AP_CHECK_FIRMWARE_ENABLED
