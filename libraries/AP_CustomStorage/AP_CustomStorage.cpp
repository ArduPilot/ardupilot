#if defined(AP_ENABLE_CUSTOM_STORAGE) && AP_ENABLE_CUSTOM_STORAGE==1
#include "AP_CustomStorage.h"
#include <stdio.h>
#include <string.h>

// Note: StorageManager handles flash memory access

StorageAccess AP_CustomStorage::_storage(StorageManager::StorageCustom);
AP_CustomStorage g_custom_storage;  // Global singleton instance

AP_CustomStorage::AP_CustomStorage()
{
    // Safety: Clear buffer to prevent stale data leaks
    memset(_data, 0, sizeof(_data));
}

void AP_CustomStorage::init()
{
    if (_initialized) {
        return;
    }
    // Critical check - must match hwdef.dat storage allocation
    if (_storage.size() < sizeof(_data)) {
        _initialized = true;  // Prevent repeated initialization attempts
        return;
    }

    if (load_from_flash()) {
        StorageHeader header;
        memcpy(&header, _data, sizeof(header));

        // Initialize header if missing/corrupt (first boot)
        if (header.magic != HEADER_MAGIC) {
            header.magic = HEADER_MAGIC;
            header.version = 1;
            memset(_data, 0, CUSTOM_PARAM_DATA_SIZE);
            memcpy(_data, &header.magic, sizeof(header.magic));
            save_to_flash();
            load_from_flash();
            memcpy(&header, _data, sizeof(header));
        }
    } else {
        memset(_data, 0, sizeof(_data));
        save_to_flash();
    }

    _initialized = true;
}

//--------------------------------------------------
// Safety-Critical Methods (all include validation)
//--------------------------------------------------

bool AP_CustomStorage::set_uuid(const char *uuid)
{
    if (!_initialized || uuid == nullptr) {
        return false;  // Prevents operations on uninitialized storage
    }

    if (strlen(uuid) > _layout.uuid_length) {
        return false;  // Length enforcement
    }

    strncpy(&_data[_layout.uuid_offset], uuid, _layout.uuid_length);
    return save_to_flash();  // Auto-persist
}

bool AP_CustomStorage::get_uuid(char *buf, uint8_t len) const
{
    if (!_initialized || buf == nullptr || len <= _layout.uuid_length) {
        return false;  // Buffer safety check
    }

    strncpy(buf, &_data[_layout.uuid_offset], _layout.uuid_length);
    buf[_layout.uuid_length] = '\0';  // Guaranteed termination
    return true;
}

bool AP_CustomStorage::set_password(const char *pass)
{
    if (!_initialized || pass == nullptr) {
        return false;
    }

    if (strlen(pass) > _layout.pass_length) {
        return false;
    }

    strncpy(&_data[_layout.pass_offset], pass, _layout.pass_length);
    return save_to_flash();
}

bool AP_CustomStorage::get_password(char *buf, uint8_t len) const
{
    if (!_initialized || buf == nullptr || len <= _layout.pass_length) {
        return false;
    }

    strncpy(buf, &_data[_layout.pass_offset], _layout.pass_length);
    buf[_layout.pass_length] = '\0';
    return true;
}

//--------------------------------------------------
// Flash Operations (atomic read/write)
//--------------------------------------------------

bool AP_CustomStorage::load_from_flash()
{
    // Note: Atomic read prevents partial state loading
    return _storage.read_block(_data, 0, sizeof(_data));
}

bool AP_CustomStorage::save_to_flash()
{
    if (_storage.write_block(0, _data, sizeof(_data))) {
        return true;
    } else {
        gcs().send_text(MAV_SEVERITY_ERROR,"CustomStorage: Failed to save data to flash!");  // Critical error
        return false;
    }
}

//--------------------------------------------------
// Utility Methods
//--------------------------------------------------

const char *AP_CustomStorage::get_storage() const
{
    // Direct access - caller manages buffer size
    return _data;
}

void AP_CustomStorage::set_storage(const char *new_data)
{
    if (!_initialized) {
        gcs().send_text(MAV_SEVERITY_ERROR,"CustomStorage: Not initialized, cannot set string.");
        return;
    }

    strncpy(_data, new_data, DATA_BUFFER_SIZE);
    save_to_flash();
}

bool AP_CustomStorage::is_initialized()
{
    return _initialized;
}
#endif