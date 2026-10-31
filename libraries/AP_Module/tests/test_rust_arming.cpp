// SPDX-License-Identifier: GPL-3.0-or-later
#include <AP_gtest.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_ARMING_AUX_AUTH_ENABLED
TEST(RustArming, ModuleOwnedAuxiliaryAuthorisers)
{
    AP_Arming arming;
    uint8_t owner_a_storage;
    uint8_t owner_b_storage;
    uint8_t owner_c_storage;
    const void *owner_a = &owner_a_storage;
    const void *owner_b = &owner_b_storage;
    const void *owner_c = &owner_c_storage;

    uint8_t a;
    uint8_t b;
    uint8_t lua;
    ASSERT_TRUE(arming.get_aux_auth_id(a, owner_a));
    ASSERT_TRUE(arming.get_aux_auth_id(b, owner_b));
    ASSERT_TRUE(arming.get_aux_auth_id(lua));
    EXPECT_EQ(a, 0U);
    EXPECT_EQ(b, 1U);
    EXPECT_EQ(lua, 2U);
    EXPECT_FALSE(arming.set_aux_auth_passed(a, owner_b));
    EXPECT_FALSE(arming.set_aux_auth_failed(b, "wrong owner", owner_a));

    // Releasing one module creates a reusable hole without renumbering the
    // remaining module or legacy/Lua authoriser.
    ASSERT_TRUE(arming.set_aux_auth_failed(a, "first failure", owner_a));
    ASSERT_TRUE(arming.set_aux_auth_failed(b, "second failure", owner_b));
    arming.release_aux_auths(owner_a);
    uint8_t c;
    ASSERT_TRUE(arming.get_aux_auth_id(c, owner_c));
    EXPECT_EQ(c, a);
    EXPECT_TRUE(arming.set_aux_auth_passed(b, owner_b));
    EXPECT_TRUE(arming.set_aux_auth_passed(lua, nullptr));

    // Lua restart cleanup does not invalidate either Native module. It removes
    // only the null-owner slot, which may then be reused by Lua.
    arming.release_aux_auths(nullptr);
    EXPECT_TRUE(arming.set_aux_auth_failed(c, "replacement failure", owner_c));
    EXPECT_TRUE(arming.set_aux_auth_passed(b, owner_b));
    EXPECT_FALSE(arming.set_aux_auth_passed(lua, nullptr));
    uint8_t replacement_lua;
    ASSERT_TRUE(arming.get_aux_auth_id(replacement_lua));
    EXPECT_EQ(replacement_lua, lua);

    // Cleanup of the current failure source leaves other owners valid and the
    // released ID unusable by either its former owner or another module.
    arming.release_aux_auths(owner_c);
    EXPECT_FALSE(arming.set_aux_auth_passed(c, owner_c));
    EXPECT_FALSE(arming.set_aux_auth_passed(c, owner_b));
    EXPECT_TRUE(arming.set_aux_auth_failed(b, "remaining failure", owner_b));
    arming.release_aux_auths(owner_b);

    // A system-wide reset retains its original meaning and invalidates every
    // owner, unlike the Lua-specific cleanup above.
    ASSERT_TRUE(arming.get_aux_auth_id(a, owner_a));
    ASSERT_TRUE(arming.get_aux_auth_id(b, owner_b));
    arming.reset_all_aux_auths();
    EXPECT_FALSE(arming.set_aux_auth_passed(a, owner_a));
    EXPECT_FALSE(arming.set_aux_auth_passed(b, owner_b));
}
#endif

AP_GTEST_MAIN()
