/*
 * Simple tool to dump the AP_Var contents from an EEPROM dump
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

uint8_t eeprom[0x1000];

struct PACKED EEPROM_header {
    uint16_t magic;
    uint8_t revision;
    uint8_t spare;
};

static const uint16_t k_EEPROM_magic      = 0x5041;
static const uint16_t k_EEPROM_revision   = 2;

struct PACKED Var_header {
    uint8_t size : 6;
    uint8_t spare : 2;
    uint8_t key;
};

static const uint8_t k_key_sentinel = 0xff;

void
fail(const char *why)
{
    fprintf(stderr, "ERROR: %s\n", why);
    exit(1);
}

int
main(int argc, char *argv[])
{
    FILE                  *fp;
    struct EEPROM_header  *header;
    struct Var_header     *var;
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
    if (header->magic != k_EEPROM_magic) {
        fail("bad magic in EEPROM file");
    }
    if (header->revision != 2) {
        fail("unsupported EEPROM format revision");
    }
    printf("Header OK\n");

    index = sizeof(*header);
    for (;; ) {
        var = (struct Var_header *)&eeprom[index];
        if (var->key == k_key_sentinel) {
            printf("end sentinel at %u\n", index);
            break;
        }
        printf("%04x: key %u size %d\n ", index, var->key, var->size + 1);
        index += sizeof(*var);
        for (i = 0; i <= var->size; i++) {
            printf(" %02x", eeprom[index + i]);
        }
        printf("\n");
        index += var->size + 1;
        if (index >= sizeof(eeprom)) {
            fflush(stdout);
            fail("missing end sentinel");
        }
    }
}
