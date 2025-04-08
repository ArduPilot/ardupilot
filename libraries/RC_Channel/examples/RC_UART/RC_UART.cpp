/*
  take RC channels in from UART and put out as PWM
 */

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define NUM_CHANNELS 4
#define ESC_MAGIC 0xF7
#define RC_SPEED 490
#define UART serial(4)

class RC_UART : public AP_HAL::HAL::Callbacks {
public:
    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;

private:
    uint8_t read_wait(void);
    uint8_t enable_mask;
    const uint32_t baudrate = 115200;
    uint32_t counter;
};

void RC_UART::setup()
{
    hal.scheduler->delay(1000);
    hal.console->printf("RC_UART starting\n");
    hal.UART->begin(baudrate, 512, 512);
    hal.rcout->set_freq(0xFF, RC_SPEED);
}

uint8_t RC_UART::read_wait(void)
{
    while (true) {
        int16_t c = hal.UART->read();
        if (c != -1) {
            // hal.console->printf("c=0x%02x\n", (unsigned)c);
            return c;
        }
        hal.scheduler->delay_microseconds(100);
    }
}

void RC_UART::loop()
{
    union {
        uint16_t period[NUM_CHANNELS];
        uint8_t bytes[NUM_CHANNELS*2];
    } u;

    // wait for magic
    while (true) {
        uint8_t c = read_wait();
        if (c == ESC_MAGIC) break;
        // hal.console->printf("c=0x%02x\n", (unsigned)c);
    }

    uint8_t nbytes=0;
    // wait for periods
    while (nbytes < NUM_CHANNELS*2) {
        u.bytes[nbytes++] = read_wait();
    }

    // and CRC
    union {
        uint8_t crc[2];
        uint16_t crc16;
    } u2;
    u2.crc[0] = read_wait();
    u2.crc[1] = read_wait();
    uint16_t crc2 = crc_calculate(u.bytes, NUM_CHANNELS*2);
    if (crc2 != u2.crc16) {
        hal.console->printf("bad CRC 0x%04x should be 0x%04x\n", (unsigned)crc2, (unsigned)u2.crc16);
        return;
    }

    // and output
    for (uint8_t i=0; i<NUM_CHANNELS; i++) {
        if (u.period[i] == 0) {
            continue;
        }
        if (!(enable_mask & 1U<<i)) {
            if (enable_mask == 0) {
                hal.rcout->force_safety_off();
            }
            hal.rcout->enable_ch(i);
            enable_mask |= 1U<<i;
        }
        hal.rcout->write(i, u.period[i]);
    }

    // report periods to console for debug
    counter++;
    if (counter % 100 == 0) {
        hal.console->printf("%4u %4u %4u %4u\n",
                            (unsigned)u.period[0],
                            (unsigned)u.period[1],
                            (unsigned)u.period[2],
                            (unsigned)u.period[3]);
    }

    // every 10th frame give an RCInput frame if possible
    if (counter % 10 == 0) {
        struct PACKED {
            uint8_t magic = 0xf6;
            uint16_t rcin[8];
            uint16_t crc;
        } rcin;
        uint16_t rcval[8];
        if (hal.rcin->new_input() && hal.rcin->read(rcval, 8) == 8) {
            memcpy(rcin.rcin, rcval, sizeof(rcval));
            rcin.crc = crc_calculate((uint8_t*)&rcin.rcin[0], 16);
            hal.UART->write((uint8_t*)&rcin, sizeof(rcin));
        }
    }
}

RC_UART rc_uart;

AP_HAL_MAIN_CALLBACKS(&rc_uart);

