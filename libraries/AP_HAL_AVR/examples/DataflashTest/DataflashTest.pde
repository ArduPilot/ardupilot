
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

bool dbg = true;

void erase() {
    hal.console->println("Erasing dataflash");
    hal.dataflash->erase_all();
}

void readback_page(int i);
void single_page_readwrite() {
    
    hal.console->println("Writing simple sequence to page 1");
    hal.dataflash->start_write(1);
    hal.dataflash->write_byte(1);
    hal.dataflash->write_byte(2);
    hal.dataflash->write_byte(3);
    hal.dataflash->write_byte(4);
    hal.dataflash->write_byte(5);
    /* Fill up the rest of the page with garbage */
    for (int i = 0; i < 600; i++) {
        hal.dataflash->write_byte(0x77);
    }
    hal.dataflash->finish_write();
    hal.scheduler->delay(100);
    readback_page(1);
}

void readback_page(int i) {
    hal.console->printf_P(PSTR("Reading back sequence from page %d\r\n"), i);
    hal.dataflash->start_read(i);
    uint8_t v1 = hal.dataflash->read_byte();
    uint8_t v2 = hal.dataflash->read_byte();
    uint8_t v3 = hal.dataflash->read_byte();
    uint8_t v4 = hal.dataflash->read_byte();
    uint8_t v5 = hal.dataflash->read_byte();
    uint8_t v6 = hal.dataflash->read_byte();
    uint8_t v7 = hal.dataflash->read_byte();
    hal.console->printf_P(PSTR("vals on page %d: %d %d %d %d %d 0x%x 0x%x\r\n"),
            i, (int) v1, (int) v2, (int) v3, (int) v4, (int) v5,
            (int) v6, (int) v7);
}
void two_page_readwrite() {
    hal.console->println("Writing simple sequence to page 1");
    hal.dataflash->start_write(1);
    hal.dataflash->write_byte(1);
    hal.dataflash->write_byte(2);
    hal.dataflash->write_byte(3);
    hal.dataflash->write_byte(4);
    hal.dataflash->write_byte(5);
    /* Fill up the rest of the page with garbage */
    for (int i = 0; i < 6; i++) {
        hal.dataflash->write_byte(0x77);
    }
    hal.dataflash->finish_write();

    hal.scheduler->delay(100);
    readback_page(1);

    hal.console->println("Writing simple sequence to page 2");
    hal.dataflash->start_write(2);
    hal.dataflash->write_byte(21);
    hal.dataflash->write_byte(22);
    hal.dataflash->write_byte(23);
    hal.dataflash->write_byte(24);
    hal.dataflash->write_byte(25);
    /* Fill up the rest of the page with garbage */
    for (int i = 0; i < 6; i++) {
        hal.dataflash->write_byte(0x77);
    }
    hal.dataflash->finish_write();

    hal.scheduler->delay(100);
    readback_page(1);
    readback_page(2);
    hal.dataflash->start_write(3);
    hal.dataflash->write_byte(99);
    hal.dataflash->write_byte(99);
    hal.dataflash->write_byte(99);
    hal.dataflash->finish_write();
    /* NOTE: getting rid of the above finish_write
    fixup the upcoming reads: fetching page 1, 2 will work! and three is a
    faithful read (255s if erased) */
    hal.scheduler->delay(100);
    readback_page(1);
    readback_page(2);
    readback_page(3);
}



void longtest_write() { 
    // We start to write some info (sequentialy) starting from page 1
    // This is similar to what we will do...
    hal.console->println("After testing perform erase before using hal.dataflash->for logging!");
    hal.console->println("");
    hal.console->println("Writing to flash... wait...");
    hal.dataflash->start_write(1);
    for (int i = 0; i < 40; i++) {
        // Write 1000 packets...
        // We write packets of binary data... (without worry about nothing more)
        hal.dataflash->write_byte(0xA3);
        hal.dataflash->write_byte(0x95);
        hal.dataflash->write_word(2000 + i);
        hal.dataflash->write_word(2001 + i);
        hal.dataflash->write_word(2002 + i);
        hal.dataflash->write_word(2003 + i);
        hal.dataflash->write_dword((int32_t)i * 5000);
        hal.dataflash->write_dword((int32_t)i * 16268);
        hal.dataflash->write_byte(0xA2);// 2 bytes of checksum (example)
        hal.dataflash->write_byte(0x4E);
        hal.scheduler->delay(10);
    }
    hal.scheduler->delay(100);
}

void longtest_readback()
{
    uint8_t tmp_byte1, tmp_byte2;
    long tmp_long;

    hal.console->println("Start reading page 1...");

    hal.dataflash->start_read(1);      // We start reading from page 1
    for (int i = 0; i < 40; i++) {          // Read 200 packets...

        uint8_t sync1 = hal.dataflash->read_byte();
        uint8_t sync2 = hal.dataflash->read_byte();

        // Read 4 ints...
        int16_t w1 = hal.dataflash->read_word();
        int16_t w2 = hal.dataflash->read_word();
        int16_t w3 = hal.dataflash->read_word();
        int16_t w4 = hal.dataflash->read_word();

        // Read 2 longs...
        int32_t l1 = hal.dataflash->read_dword();
        int32_t l2 = hal.dataflash->read_dword();

        // Read the checksum...
        int8_t cs1 = hal.dataflash->read_byte();
        int8_t cs2 = hal.dataflash->read_byte();

        hal.console->printf_P(PSTR("sync 0x%x 0x%x ints %d %d %d %d, "
                "longs %d %d, cksm %d %d\r\n"),
                (int) sync1, (int) sync2,
                w1, w2, w3, w4,
                l1, l2,
                cs1, cs2);

    }
}


void setup()
{
    hal.dataflash->init(NULL);

    hal.console->println("Dataflash Log Test 1.0");

    hal.scheduler->delay(20);
    hal.console->print("Manufacturer:");
    hal.console->print((int)hal.dataflash->mfg_id());
    hal.console->print(",");
    hal.console->print((int)hal.dataflash->device_id());
    hal.console->println();

    erase();

   // two_page_readwrite();
   dbg = false;
   longtest_write();
   longtest_readback();

}
void loop () {}

extern "C" {
int main (void) {
    hal.init(NULL);
    setup();
    for(;;) loop();
    return 0;
}
}
