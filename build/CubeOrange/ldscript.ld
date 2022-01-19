/* generated ldscript.ld */
MEMORY
{
    flash : org = 0x08020000, len = 1920K
    ram0  : org = 0x30000000, len = 262144
}

INCLUDE common.ld
