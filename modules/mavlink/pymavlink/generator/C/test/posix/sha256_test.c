/*
  simple test of mavlink sha256 code. 
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include <mavlink_sha256.h>

// the sha256 result is just the first six bytes of a regular sha256 output,
// e.g. `printf '' | sha256sum | cut -c1-12`. then for these tables we prefix
// 0x to that result

void validate(mavlink_sha256_ctx *ctx, uint64_t expected)
{
    uint8_t result_arr[6];
    mavlink_sha256_final_48(ctx, result_arr);

    uint64_t result = 0;
    result |= ((uint64_t)result_arr[0]) << 40;
    result |= ((uint64_t)result_arr[1]) << 32;
    result |= ((uint64_t)result_arr[2]) << 24;
    result |= ((uint64_t)result_arr[3]) << 16;
    result |= ((uint64_t)result_arr[4]) << 8;
    result |= ((uint64_t)result_arr[5]) << 0;

    if (result != expected) {
        printf("FAILED: expected %012" PRIx64 ", got %012" PRIx64 "\n",
            expected, result);
        exit(1);
    }
}

void run_test(const char *input, size_t length, uint64_t expected)
{
    mavlink_sha256_ctx ctx;

    mavlink_sha256_init(&ctx);
    mavlink_sha256_update(&ctx, input, length);
    validate(&ctx, expected);
}

void run_multipiece_test(const char *input, size_t length, int bits, uint64_t expected)
{
    mavlink_sha256_ctx ctx;

    mavlink_sha256_init(&ctx);

    // hash the input over several calls with different lengths
    uint64_t mask = (1 << bits) - 1;
    uint64_t order = expected;
    while (length > 0 && order > 0) {
        // use n lowest bits of the expected value as the number of bytes to put
        // in this time; that should be a pretty random sequence
        int this_len = order & mask;
        if (this_len > length) {
            this_len = length;
        }

        mavlink_sha256_update(&ctx, input, this_len);

        input += this_len;
        length -= this_len;
        order >>= bits;
    }

    mavlink_sha256_update(&ctx, input, length); // last bits

    validate(&ctx, expected);
}

typedef struct {
    const char *input;
    uint64_t result; // first six bytes
} string_test_t;

static const string_test_t string_tests[] = {
    {"", 0xe3b0c44298fc},
    {"Hello, world!", 0x315f5bdb76d0},
    {"\xfd\x23\x01\x01\x01", 0xc56152f79cb7},
};

void run_string_tests(void)
{
    for (int i=0; i<(sizeof(string_tests)/sizeof(string_tests[0])); i++) {
        const string_test_t *test = &string_tests[i];

        run_test(test->input, strlen(test->input), test->result);
    }
}

typedef struct {
    char input[16];
    uint64_t result; // first six bytes
} bin_test_t;

static const bin_test_t bin_tests[] = {
    // these are padded with zeros to 16 bytes
    {"", 0x374708fff771},
    {"Hello, world!", 0xd87443b8fc88},
    {"\xfd\x23\x01\x01\x01", 0x0e40dd48bb95},
    {"26\xcb\xcb&\x12[|\xfez?\xb3\x8e\x03\xe4", 0x33ffd10eceae},
    {"\xb3;\xc3\rq\n\xd4\xab}\x97\xcev\x8f+Z", 0x3127873b8ce5},
    {"\n !+\xcd%X#\xa1\xa4^.O\x9d#\xb8", 0x87444c012c7a},
};

void run_bin_tests(void)
{
    for (int i=0; i<(sizeof(bin_tests)/sizeof(bin_tests[0])); i++) {
        const bin_test_t *test = &bin_tests[i];

        run_test(test->input, sizeof(test->input), test->result);
    }
}

// block size is 64 bytes, so this contains from 0 to 64 bytes from a bad
// sequence (which hopefully would pick up a byte or block being repeated):
// python3 -c 'import hashlib; print(",\n    ".join(map(lambda n: "0x"+hashlib.sha256(bytes(map(lambda v: (v*13397//3)&255, range(n)))).hexdigest()[:12], range(65))))'
static uint64_t block_tests[] = {
    0xe3b0c44298fc, 0x6e340b9cffb3, 0x0c85b39d1274, 0x960cb62040bd,
    0x7452e89f2ec3, 0xcb42995e3549, 0x1cf487cf16f4, 0x38d5dc594edc,
    0x97c0bdae93b6, 0xb0d6bcf265c7, 0x5e64777ddf31, 0x69328676115a,
    0x9bb27cdbcca4, 0xe0b2396d7ad0, 0x0e1930714d33, 0xe5877a87e254,
    0x8f78be9d7814, 0xd5d291326b17, 0xf32ab48ef5b2, 0xc865c0660961,
    0x6f0491403382, 0xca89fc9a3e5f, 0xbb97c3258f9e, 0x07e74032378c,
    0xd9ef24816cdc, 0x25edd25739be, 0x4f0608480691, 0x60c6de9db660,
    0x23f047a47ada, 0xb2b957b87b43, 0x4d7eb57e04e5, 0x50ab65b8d166,
    0x341a886b09cf, 0x829b670eb2ab, 0x9235cf421427, 0x77a17d80c423,
    0xbde4d067e625, 0x9058c6f537c8, 0x38452777fbc6, 0x2f845bb8ee1b,
    0xb54d97a24fed, 0x43a202588d02, 0x2e1e5465131c, 0xca7819f66afb,
    0x081042ffc665, 0xf7604aef37ef, 0xe9c38cae9044, 0x080832597990,
    0x2ea640641f2c, 0x663c346ec6b2, 0xd497a09295ed, 0x9207b378bbb2,
    0xcef7cb2ca376, 0x8b81496acd8c, 0x24e88aaf30e4, 0xf5119b6b5bbb,
    0x7e7b46b2390b, 0x48d59c3cc341, 0xd9025fb7c53e, 0x2c092c28f99e,
    0xb7a5d5bebce4, 0x85683a46d052, 0x258b912f3ced, 0x78499eacfa7d,
    0xdbed48bde530 };

// as above, but in multiples of 17 to span multiple blocks
// python3 -c 'import hashlib; print(",\n    ".join(map(lambda n: "0x"+hashlib.sha256(bytes(map(lambda v: (v*13397//3)&255, range(n*17)))).hexdigest()[:12], range(65))))'
static uint64_t block_tests_17[] = {
    0xe3b0c44298fc, 0xd5d291326b17, 0x9235cf421427, 0x9207b378bbb2,
    0x1f393cf78335, 0xad827c8fa206, 0xab1e836c26bd, 0x14492ee0572f,
    0x05999423ae8c, 0x62363df6d287, 0x3ae95d199f68, 0x1cfdb91a14f6,
    0x061420ef628c, 0x1d11f7886c89, 0x10d184c5a5af, 0xfdd306160e9d,
    0xae8980049af8, 0x3f3b08fc547b, 0xb367d1179cb9, 0x1376135e5cfe,
    0x53541932ee1e, 0x7c6275dc6e12, 0x891e97f3bfb1, 0x8bd0f41c0c47,
    0xffdee5266463, 0xda2faa759b3a, 0x588902063ddb, 0x12aee813d087,
    0x995eac8ee67f, 0xc1793d72d96e, 0x38892859af4c, 0xb487af5fc201,
    0xda4080846c3a, 0xd774246c4d7e, 0x125000302600, 0xa467c7c152a9,
    0xa996272b348f, 0xf7805004973e, 0xf5eb332207cb, 0x00fb202ac899,
    0xa4b62eb07534, 0x72accc5cc195, 0xeeb8077ca6e3, 0xd45570b32e3e,
    0x864443ce69e9, 0x3b7d4560fa6e, 0xe7f199844788, 0xe5414e24a0dd,
    0xd97fd2c57b62, 0xef24a5e279cb, 0x16767d8a4df7, 0xe8176ec5d6ac,
    0xc3d3c62e1840, 0x40404b683480, 0x9454091cdf93, 0x81ab0db75152,
    0x5008b535466b, 0x20a3496c51e1, 0xef9ea3a86305, 0x76c93f988709,
    0xfd24437cfab4, 0xd6ecbb13639c, 0xeffdc5a9a8aa, 0xc63c0a11c4c7,
    0x1bc7ad39edc4 };

void run_block_tests(void)
{
    size_t num_tests = (sizeof(block_tests)/sizeof(block_tests[0]));
    size_t max_chars = 17*num_tests;
    char input[max_chars];

    for (int i=0; i<max_chars; i++) {
        input[i] = (i*13397/3) & 255;
    }

    for (int i=0; i<num_tests; i++) {
        run_test(input, i, block_tests[i]);
        run_test(input, 17*i, block_tests_17[i]);

        run_multipiece_test(input, i, 4, block_tests[i]);
        run_multipiece_test(input, 17*i, 8, block_tests_17[i]);
    }
}

void run_tests(void)
{
    run_string_tests();
    run_bin_tests();
    run_block_tests();
}

int main(int argc, const char *argv[])
{
    mavlink_sha256_ctx ctx;
    uint8_t result[6];
    uint8_t i;

    if (argc == 1) {
        run_tests(); // failure causes call to exit()
        printf("Tests passed\n");
        return 0;
    }

    mavlink_sha256_init(&ctx);
    mavlink_sha256_update(&ctx, argv[1], strlen(argv[1]));
    mavlink_sha256_final_48(&ctx, result);
    for (i=0; i<6; i++) {
        printf("%02x ", (unsigned)result[i]);
    }
    printf("\n");
    return 0;
}
