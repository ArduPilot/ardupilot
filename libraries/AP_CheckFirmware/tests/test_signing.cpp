#include <AP_gtest.h>

#include "../AP_CheckFirmware_Signing.h"

#include <array>

static void set_key(struct ap_secure_data &keys, uint8_t index, uint8_t value)
{
    memset(keys.public_key[index].key, value, AP_PUBLIC_KEY_LEN);
}

TEST(AP_CheckFirmware, ZeroKeyForgeryIsSkipped)
{
    // SECURE_COMMAND signs both sequence and operation as little-endian
    // uint32_t values. Sequence 14 makes this low-order Ed25519 forgery
    // validate against an all-zero public key.
    const std::array<uint8_t, 8> transcript {
        0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    const std::array<uint8_t, 64> forged_signature {
        0x58, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66,
        0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66,
        0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66,
        0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66,
        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    const uint8_t zero_key[AP_PUBLIC_KEY_LEN] {};
    ASSERT_EQ(0, crypto_check(forged_signature.data(), zero_key,
                              transcript.data(), transcript.size()));

    struct ap_secure_data keys {};
    set_key(keys, 0, 0x42);
    const struct ap_signature_buffer buffer { transcript.data(), transcript.size() };
    EXPECT_FALSE(ap_check_signature(keys, forged_signature.data(), &buffer, 1));
}

TEST(AP_CheckFirmware, ValidSignatureIsAccepted)
{
    const std::array<uint8_t, 32> secret_key {
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
        0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
        0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
    };
    const std::array<uint8_t, 6> message { 1, 2, 3, 4, 5, 6 };
    struct ap_secure_data keys {};
    crypto_sign_public_key(keys.public_key[0].key, secret_key.data());
    std::array<uint8_t, 64> signature {};
    crypto_sign(signature.data(), secret_key.data(), keys.public_key[0].key,
                message.data(), message.size());
    const struct ap_signature_buffer buffer { message.data(), message.size() };
    EXPECT_TRUE(ap_check_signature(keys, signature.data(), &buffer, 1));
}

TEST(AP_CheckFirmware, KeyCompactionClearsTail)
{
    struct ap_secure_data keys {};
    set_key(keys, 0, 0x11);
    set_key(keys, 2, 0x22);
    set_key(keys, 9, 0x33);
    ap_public_keys_compact(keys);
    EXPECT_EQ(0x11, keys.public_key[0].key[0]);
    EXPECT_EQ(0x22, keys.public_key[1].key[0]);
    EXPECT_EQ(0x33, keys.public_key[2].key[0]);
    for (uint8_t i = 3; i < AP_PUBLIC_KEY_MAX_KEYS; i++) {
        EXPECT_TRUE(ap_public_key_is_zero(keys.public_key[i].key));
    }
}

TEST(AP_CheckFirmware, CandidateMustPreserveExactKeyMultiset)
{
    struct ap_secure_data current {};
    struct ap_secure_data candidate {};
    set_key(current, 0, 0x11);
    set_key(current, 1, 0x22);
    set_key(candidate, 0, 0x22);
    set_key(candidate, 1, 0x11);
    EXPECT_TRUE(ap_public_key_sets_equal(current, candidate));

    set_key(candidate, 1, 0x33);
    EXPECT_FALSE(ap_public_key_sets_equal(current, candidate));
    set_key(candidate, 1, 0x11);
    set_key(candidate, 2, 0x33);
    EXPECT_FALSE(ap_public_key_sets_equal(current, candidate));
}

TEST(AP_CheckFirmware, SecureCommandSequenceRejectsReplay)
{
    ap_secure_command_sequence sequence;
    EXPECT_TRUE(sequence.accept(100));
    EXPECT_FALSE(sequence.accept(100));
    EXPECT_FALSE(sequence.accept(99));
    EXPECT_FALSE(sequence.accept(100U + (1U << 31)));
    EXPECT_TRUE(sequence.accept(101));

    sequence.reset();
    EXPECT_TRUE(sequence.accept(UINT32_MAX));
    EXPECT_TRUE(sequence.accept(0));
    EXPECT_FALSE(sequence.accept(UINT32_MAX));

    sequence.reset();
    EXPECT_TRUE(sequence.accept(99));
}

static bool check_extf_signature(const uint8_t signature[64],
                                 const uint8_t public_key[AP_PUBLIC_KEY_LEN],
                                 const uint8_t *flash1, uint32_t len1,
                                 const uint8_t *flash2, uint32_t len2,
                                 const uint8_t *extf, uint32_t extf_length)
{
    crypto_check_ctx ctx {};
    crypto_check_ctx_abstract *actx = (crypto_check_ctx_abstract *)&ctx;
    ap_extf_signature_check_init(actx, signature, public_key, len1, len2, extf_length);
    crypto_check_update(actx, flash1, len1);
    crypto_check_update(actx, flash2, len2);
    crypto_check_update(actx, extf, extf_length);
    return crypto_check_final(actx) == 0;
}

TEST(AP_CheckFirmware, ExternalSignatureBindsImageBoundaries)
{
    const std::array<uint8_t, 32> secret_key {
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
        0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
        0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
    };
    const std::array<uint8_t, 4> flash1 { 1, 2, 3, 4 };
    const std::array<uint8_t, 5> flash2 { 5, 6, 7, 8, 9 };
    const std::array<uint8_t, 3> extf { 10, 11, 12 };
    const uint32_t lengths[] { flash1.size(), flash2.size(), extf.size() };
    static const uint8_t domain[] = "ArduPilotFwSig2";
    std::array<uint8_t, sizeof(domain)-1+sizeof(lengths)+flash1.size()+flash2.size()+extf.size()> transcript {};
    uint8_t *out = transcript.data();
    memcpy(out, domain, sizeof(domain)-1);
    out += sizeof(domain)-1;
    memcpy(out, lengths, sizeof(lengths));
    out += sizeof(lengths);
    memcpy(out, flash1.data(), flash1.size());
    out += flash1.size();
    memcpy(out, flash2.data(), flash2.size());
    out += flash2.size();
    memcpy(out, extf.data(), extf.size());

    std::array<uint8_t, 32> public_key {};
    crypto_sign_public_key(public_key.data(), secret_key.data());
    std::array<uint8_t, 64> signature {};
    crypto_sign(signature.data(), secret_key.data(), public_key.data(),
                transcript.data(), transcript.size());

    EXPECT_TRUE(check_extf_signature(signature.data(), public_key.data(),
                                     flash1.data(), flash1.size(),
                                     flash2.data(), flash2.size(),
                                     extf.data(), extf.size()));

    // Keep flash1||flash2 byte-for-byte identical while moving its boundary.
    std::array<uint8_t, flash1.size()+flash2.size()> internal {};
    memcpy(internal.data(), flash1.data(), flash1.size());
    memcpy(internal.data()+flash1.size(), flash2.data(), flash2.size());
    EXPECT_FALSE(check_extf_signature(signature.data(), public_key.data(),
                                      internal.data(), flash1.size()-1,
                                      internal.data()+flash1.size()-1, flash2.size()+1,
                                      extf.data(), extf.size()));

    auto mutated_extf = extf;
    mutated_extf[0] ^= 1;
    EXPECT_FALSE(check_extf_signature(signature.data(), public_key.data(),
                                      flash1.data(), flash1.size(),
                                      flash2.data(), flash2.size(),
                                      mutated_extf.data(), mutated_extf.size()));
}

AP_GTEST_MAIN()
