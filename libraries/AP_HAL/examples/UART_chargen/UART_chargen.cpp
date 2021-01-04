/*
  simple test of UART interfaces

  on an UP2's hardware serial:
  DEVICE=/dev/ttyS5
  socat -,escape=0x0f $DEVICE,rawer,crnl,b4000000 | dd status=progress >/dev/null
  n.b. socat corrupts the data; screen(1) does not


  @ 4Mbaud:

  unbuffered:
  21526016 bytes (22 MB, 21 MiB) copied, 55.5364 s, 388 kB/s

  buffered:
  1702912 bytes (1.7 MB, 1.6 MiB) copied, 36.3764 s, 46.8 kB/s

 */

#include <AP_HAL/AP_HAL.h>

#include <AP_BoardConfig/AP_BoardConfig.h>

#define UNBUFFERED_WRITES 1
#define FLOW_CONTROL_ENABLED 1

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static const uint32_t baud = 4000000;

AP_HAL::UARTDriver *uart;

uint8_t buffer[16384];
uint16_t buflen = 0;

void setup(void)
{
    hal.scheduler->delay(1000); //Ensure that the uartA can be initialized

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    uart = hal.serial(0);  //  console
#endif
    // uart = hal.serial(3);   // 1st GPS
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    uart = hal.serial(1);  // telemetry 1
#endif
    // uart = hal.serial(2);  // telemetry 2
    // uart = hal.serial(4);  // 2nd GPS


    if (uart == nullptr) {
        AP_HAL::panic("bad uart");
    }

#if UNBUFFERED_WRITES
    uart->set_unbuffered_writes(true);
#endif

    uart->begin(baud);

#if FLOW_CONTROL_ENABLED
    uart->set_flow_control(AP_HAL::UARTDriver::flow_control::FLOW_CONTROL_ENABLE);
#endif

    // populate chargen buffer
    for (uint8_t line=0; line<95; line++) {
        for (uint8_t i=0; i<72; i++) {
            uint8_t c = ' ' + line + i;
            if (c >= ' ' + 72) {
                c -= 72;
            }
            buffer[buflen++] = c;
        }
        buffer[buflen++] = '\r';
        buffer[buflen++] = '\n';
    }
}

void loop(void)
{
    uint16_t offset = 0;
    while (offset < buflen) {
        uint16_t to_write = buflen-offset;
        const ssize_t ret = uart->write(&buffer[offset], to_write);
        if (ret > 0) {
            offset += ret;
        }
#if !UNBUFFERED_WRITES
        // give the uart thread time to move data from buffer to uart:
        hal.scheduler->delay(1);
#endif
    }
}

AP_HAL_MAIN();
