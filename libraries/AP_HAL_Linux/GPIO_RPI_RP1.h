#pragma once

#include <cstdint>
#include "GPIO_RPI_HAL.h"

namespace Linux {

/**
 * @brief Class for Raspberry PI 5 GPIO control
 *
 *  For more information:
 *    - RP1 datasheet: https://datasheets.raspberrypi.com/rp1/rp1-peripherals.pdf
 *    - gpiomem0: https://github.com/raspberrypi/linux/blob/1e53604087930e7cf42eee3d42572d0d6f54c86a/arch/arm/boot/dts/broadcom/bcm2712-rpi.dtsi#L178
 *        - Address: 0x400d'0000, Size: 0x3'0000
 *
 */
class GPIO_RPI_RP1 : public GPIO_RPI_HAL {
public:
    GPIO_RPI_RP1();
    void init() override;
    void pinMode(uint8_t pin, uint8_t mode) override;
    void pinMode(uint8_t pin, uint8_t mode, uint8_t alt) override;

    uint8_t read(uint8_t pin) override;
    void write(uint8_t pin, uint8_t value) override;
    void toggle(uint8_t pin) override;

    enum class PadsPull : uint8_t {
        Off = 0,
        Down = 1,
        Up = 2,
    };

    void set_pull(uint8_t pin, PadsPull mode);

private:
    // gpiomem0 already maps the 0x400d'0000 address for us
    static constexpr const char* PATH_DEV_GPIOMEM = "/dev/gpiomem0";
    static constexpr uint32_t MEM_SIZE = 0x30000;
    static constexpr uint32_t REG_SIZE = sizeof(uint32_t);

    // Register offsets from RP1 datasheet 'Table 2. Peripheral Address Map'
    // E.g:
    //   - IO_BANK0_OFFSET: 0x0'0000 result in 0x400d'0000
    //   - SYS_RIO0_OFFSET: 0x1'0000 result in 0x400e'0000
    //   ...
    static constexpr uint32_t IO_BANK0_OFFSET = 0x00000;
    static constexpr uint32_t SYS_RIO0_OFFSET = 0x10000;
    static constexpr uint32_t PADS_BANK0_OFFSET = 0x20000;

    // GPIO control from https://github.com/raspberrypi/linux/blob/21012295fe87a7ccc1c356d1e268fd289aafbad1/drivers/pinctrl/pinctrl-rp1.c
    static constexpr uint32_t RIO_OUT = 0x00;
    static constexpr uint32_t RIO_OE = 0x04;
    static constexpr uint32_t RIO_IN = 0x08;

    // GPIO control from '2.4. Atomic register access'
    static constexpr uint32_t RW_OFFSET = 0x0000;
    static constexpr uint32_t XOR_OFFSET = 0x1000;
    static constexpr uint32_t SET_OFFSET = 0x2000;
    static constexpr uint32_t CLR_OFFSET = 0x3000;

    static constexpr uint32_t GPIO_CTRL = 0x0004;
    static constexpr uint32_t GPIO_OFFSET = 8;

    static constexpr uint32_t PADS_GPIO = 0x04;
    static constexpr uint32_t PADS_OFFSET = 4;

    /**
     * GPIO control from 'Table 8. GPI0_CTRL, GPI1_CTRL, ...'
     *
     * 0b0000'0000'0000'0000'0000'0000'0001'1101
     *   ├┘│├─┘├┘├─┘├┘├─┘├┘├─┘├┘│ ├──────┘└────┴─ Bits 4:0   FUNCSEL: Function select
     *   │ ││  │ │  │ │  │ │  │ │ └────────────── Bits 11:5  F_M: Filter/debounce time constant M
     *   │ ││  │ │  │ │  │ │  │ └──────────────── Bits 13:12 OUTOVER: Output control
     *   │ ││  │ │  │ │  │ │  └────────────────── Bits 15:14 OEOVER: Output enable control
     *   │ ││  │ │  │ │  │ └───────────────────── Bits 17:16 INOVER: Input control
     *   │ ││  │ │  │ │  └─────────────────────── Bits 19:18 Reserved
     *   │ ││  │ │  │ └────────────────────────── Bits 20:21 IRQMASK_EDGE_LOW/HIGH
     *   │ ││  │ │  └──────────────────────────── Bits 22:23 IRQMASK_LEVEL_LOW/HIGH
     *   │ ││  │ └─────────────────────────────── Bits 24:25 IRQMASK_F_EDGE_LOW/HIGH
     *   │ ││  └───────────────────────────────── Bits 26:27 IRQMASK_DB_LEVEL_LOW/HIGH
     *   │ │└──────────────────────────────────── Bit 28     IRQRESET: Interrupt edge detector reset
     *   │ └───────────────────────────────────── Bit 29     Reserved
     *   └─────────────────────────────────────── Bits 31:30 IRQOVER: Interrupt control
     */
    static constexpr uint32_t CTRL_FUNCSEL_MASK = 0x001f;
    static constexpr uint32_t CTRL_FUNCSEL_LSB = 0;
    static constexpr uint32_t CTRL_OUTOVER_MASK = 0x3000;
    static constexpr uint32_t CTRL_OUTOVER_LSB = 12;
    static constexpr uint32_t CTRL_OEOVER_MASK = 0xc000;
    static constexpr uint32_t CTRL_OEOVER_LSB = 14;
    static constexpr uint32_t CTRL_INOVER_MASK = 0x30000;
    static constexpr uint32_t CTRL_INOVER_LSB = 16;
    static constexpr uint32_t CTRL_IRQOVER_MASK = 0xc0000000;
    static constexpr uint32_t CTRL_IRQOVER_LSB = 30;

    /**
     * Mask for PADS_BANK control from 'Table 21.'
     *
     * 0b0...001'1101
     *   ├──┘││├─┘││└─ Bit 1      SLEWFAST: Slew rate control. 1 = Fast, 0 = Slow
     *   │   │││  │└── Bit 2      SCHMITT: Enable schmitt trigger
     *   │   │││  └─── Bit 3      PDE: Pull down enable
     *   │   ││└────── Bits 4:5   DRIVE: Drive strength
     *   │   │└─────── Bit 6      IE: Input enable
     *   │   └──────── Bit 7      OD: Output disable. Has priority over output enable from
     *   └──────────── Bits 31:8  Reserved
     */
    static constexpr uint32_t PADS_IN_ENABLE_MASK = 0x40;
    static constexpr uint32_t PADS_OUT_DISABLE_MASK = 0x80;
    static constexpr uint32_t PADS_PULL_MASK = 0x0c;
    static constexpr uint32_t PADS_PULL_LSB = 2;

    enum class FunctionSelect : uint8_t {
        Alt0 = 0,
        Alt1 = 1,
        Alt2 = 2,
        Alt3 = 3,
        Alt4 = 4,
        Alt5 = 5,
        Alt6 = 6,
        Alt7 = 7,
        Alt8 = 8,
        Null = 31
    };

    enum Mode {
        Input,
        Output,
        Alt0,
        Alt1,
        Alt2,
        Alt3,
        Alt4,
        Alt5,
        Alt6,
        Alt7,
        Alt8,
        Null
    };

    enum Bias {
        Off,
        PullDown,
        PullUp
    };

    volatile uint32_t* _gpio;
    int _system_memory_device;
    uint32_t _gpio_output_port_status = 0;

    bool openMemoryDevice();
    void closeMemoryDevice();
    volatile uint32_t* get_memory_pointer(uint32_t address, uint32_t range) const;

    uint32_t read_register(uint32_t offset) const;
    void write_register(uint32_t offset, uint32_t value);

    Mode direction(uint8_t pin) const;
    void set_direction(uint8_t pin, Mode mode);
    void input_enable(uint8_t pin);
    void input_disable(uint8_t pin);
    void output_enable(uint8_t pin);
    void output_disable(uint8_t pin);

    void set_mode(uint8_t pin, Mode mode);
};

}
