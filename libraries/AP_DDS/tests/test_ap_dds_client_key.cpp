#include <AP_gtest.h>
#include <AP_DDS/AP_DDS_Client_Key.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

TEST(AP_DDS_CLIENT_KEY, legacy_ids)
{
    for (uint32_t sysid = 1; sysid <= 255; sysid++) {
        EXPECT_EQ(AP_DDS::client_key_from_sysid(sysid), 0xAD000000U | sysid);
    }
}

TEST(AP_DDS_CLIENT_KEY, wide_ids)
{
    // Include IDs which collided under OR, both signed boundaries, and the
    // ID which a plain XOR would turn into the reserved zero client key.
    const uint32_t ids[] = {1, 0x01000001, 0xAD000001, 0x7FFFFFFF,
                            0x80000000, 0xAD000000, 0xFFFFFFFF};
    for (const uint32_t sysid : ids) {
        const uint32_t key = AP_DDS::client_key_from_sysid(sysid);
        EXPECT_NE(key, 0U);
        EXPECT_EQ(AP_DDS::client_key_from_sysid(key), sysid);
        for (const uint32_t other : ids) {
            if (other != sysid) {
                EXPECT_NE(key, AP_DDS::client_key_from_sysid(other));
            }
        }
    }
}

AP_GTEST_MAIN()
