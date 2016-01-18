/*
 * Simple tool to dump the AP_Param contents from an EEPROM dump
 * Andrew Tridgell February 2012
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

uint8_t eeprom[0x1000];

struct PACKED EEPROM_header {
    uint8_t magic[2];
    uint8_t revision;
    uint8_t spare;
};

static const uint16_t k_EEPROM_magic0     = 0x50;
static const uint16_t k_EEPROM_magic1     = 0x41;
static const uint16_t k_EEPROM_revision   = 6;

enum ap_var_type {
    AP_PARAM_NONE    = 0,
    AP_PARAM_INT8,
    AP_PARAM_INT16,
    AP_PARAM_INT32,
    AP_PARAM_FLOAT,
    AP_PARAM_VECTOR3F,
    AP_PARAM_VECTOR6F,
    AP_PARAM_MATRIX3F,
    AP_PARAM_GROUP
};

static const char *type_names[8] = {
    "NONE", "INT8", "INT16", "INT32", "FLOAT", "VECTOR3F", "MATRIX6F", "GROUP"
};

struct Param_header {
    uint32_t key : 8;
    uint32_t type : 6;
    uint32_t group_element : 18;
};


static const uint8_t _sentinal_key   = 0xFF;
static const uint8_t _sentinal_type  = 0xFF;
static const uint8_t _sentinal_group = 0xFF;

static uint8_t type_size(enum ap_var_type type)
{
    switch (type) {
    case AP_PARAM_NONE:
    case AP_PARAM_GROUP:
        return 0;
    case AP_PARAM_INT8:
        return 1;
    case AP_PARAM_INT16:
        return 2;
    case AP_PARAM_INT32:
        return 4;
    case AP_PARAM_FLOAT:
        return 4;
    case AP_PARAM_VECTOR3F:
        return 3*4;
    case AP_PARAM_VECTOR6F:
        return 6*4;
    case AP_PARAM_MATRIX3F:
        return 3*3*4;
    }
    printf("unknown type %u\n", (unsigned int)type);
    return 0;
}

static void
fail(const char *why)
{
    fprintf(stderr, "ERROR: %s\n", why);
    exit(1);
}

int
main(int argc, char *argv[])
{
    FILE                    *fp;
    struct EEPROM_header    *header;
    struct Param_header     *var;
    unsigned index;
    unsigned i;

    if (argc != 2) {
        fail("missing EEPROM file name");
    }
    if (NULL == (fp = fopen(argv[1], "rb"))) {
        fail("can't open EEPROM file");
    }
    if (1 != fread(eeprom, sizeof(eeprom), 1, fp)) {
        fail("can't read EEPROM file");
    }
    fclose(fp);

    header = (struct EEPROM_header *)&eeprom[0];
    if (header->magic[0] != k_EEPROM_magic0 ||
        header->magic[1] != k_EEPROM_magic1) {
        fail("bad magic in EEPROM file");
    }
    if (header->revision != k_EEPROM_revision) {
        fail("unsupported EEPROM format revision");
    }
    printf("Header OK\n");

    index = sizeof(*header);
    for (;; ) {
        uint8_t size;
        var = (struct Param_header *)&eeprom[index];
        if (var->key == _sentinal_key ||
            var->group_element == _sentinal_group ||
            var->type == _sentinal_type) {
            printf("end sentinel at %u\n", index);
            break;
        }
        size = type_size(var->type);
        printf("%04x: type %u (%s) key %u group_element %u size %d value ",
               index, var->type, type_names[var->type], var->key, var->group_element, size);
        index += sizeof(*var);
        switch (var->type) {
        case AP_PARAM_INT8:
            printf("%d\n", (int)*(int8_t *)&eeprom[index]);
            break;
        case AP_PARAM_INT16:
            printf("%d\n", (int)*(int16_t *)&eeprom[index]);
            break;
        case AP_PARAM_INT32:
            printf("%d\n", (int)*(int32_t *)&eeprom[index]);
            break;
        case AP_PARAM_FLOAT:
            printf("%f\n", *(float *)&eeprom[index]);
            break;
        case AP_PARAM_VECTOR3F:
            printf("%f %f %f\n",
                   *(float *)&eeprom[index],
                   *(float *)&eeprom[index+4],
                   *(float *)&eeprom[index+8]);
            break;
        case AP_PARAM_VECTOR6F:
            printf("%f %f %f %f %f %f\n",
                   *(float *)&eeprom[index],
                   *(float *)&eeprom[index+4],
                   *(float *)&eeprom[index+8],
                   *(float *)&eeprom[index+12],
                   *(float *)&eeprom[index+16],
                   *(float *)&eeprom[index+20]);
            break;
        case AP_PARAM_MATRIX3F:
            printf("%f %f %f %f %f %f %f %f %f\n",
                   *(float *)&eeprom[index],
                   *(float *)&eeprom[index+4],
                   *(float *)&eeprom[index+8],
                   *(float *)&eeprom[index+12],
                   *(float *)&eeprom[index+16],
                   *(float *)&eeprom[index+20],
                   *(float *)&eeprom[index+24],
                   *(float *)&eeprom[index+28],
                   *(float *)&eeprom[index+32]);
            break;
        default:
            printf("NONE\n");
            break;
        }
        for (i = 0; i < size; i++) {
            printf(" %02x", eeprom[index + i]);
        }
        printf("\n");
        index += size;
        if (index >= sizeof(eeprom)) {
            fflush(stdout);
            fail("missing end sentinel");
        }
    }
    return 0;
}
