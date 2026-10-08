/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/* Boot-time I2C bus address scan: reports which declared devices actually answer. */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#ifdef CONFIG_AP_I2C_PROBE_DIAG

/* Addresses worth naming when they answer, so the log is readable without a
 * datasheet to hand. */
static const char *addr_hint(uint8_t addr)
{
	switch (addr) {
	case 0x0c: return "BMM150 (alt)";
	case 0x10: return "BMM150";
	case 0x13: return "BMM150 (alt)";
	case 0x14: return "BMM350";
	case 0x1e: return "HMC5883/LIS3MDL";
	case 0x28: return "IST8310 (alt)";
	case 0x0e: return "IST8310";
	case 0x40: return "INA226/PM";
	case 0x48: return "SE050 secure element";
	case 0x50: return "sensor EEPROM";
	case 0x76: return "BMP388/BMP280 (SDO low)";
	case 0x77: return "BMP388/BMP390 (SDO high)";
	default: return NULL;
	}
}

static void scan_bus(const char *label, const struct device *bus)
{
	if (bus == NULL) {
		printk("I2C_SCAN: %-7s device is NULL\n", label);
		return;
	}
	if (!device_is_ready(bus)) {
		printk("I2C_SCAN: %-7s bus '%s' NOT READY\n", label, bus->name);
		return;
	}

	printk("I2C_SCAN: %-7s bus '%s' ready\n", label, bus->name);

	unsigned int found = 0U;

	/* 0x08..0x77 is the usable 7-bit range; below/above are reserved.
	 * A zero-length write is the standard presence test - it addresses the
	 * device and looks for the ACK without reading or altering anything. */
	for (uint8_t addr = 0x08U; addr <= 0x77U; addr++) {
		if (i2c_write(bus, NULL, 0, addr) == 0) {
			const char *hint = addr_hint(addr);

			printk("I2C_SCAN: %-7s   ACK 0x%02x%s%s\n", label, addr,
			       hint ? "  <-- " : "", hint ? hint : "");
			found++;

			/* Power-monitor identification: 16-bit big-endian register reads. */
			if (addr >= 0x40U && addr <= 0x4FU) {
				static const uint8_t regs[] = {
					0x00, 0x02, 0x05, 0x3E, 0x3F,
					0xFE, 0xFF
				};
				for (unsigned int i = 0; i < ARRAY_SIZE(regs); i++) {
					uint8_t out[2];
					if (i2c_write_read(bus, addr, &regs[i], 1,
							   out, 2) == 0) {
						printk("I2C_SCAN: %-7s     reg 0x%02x = 0x%02x%02x\n",
						       label, regs[i], out[0], out[1]);
					}
				}
			}
		}
	}

	if (found == 0U) {
		printk("I2C_SCAN: %-7s   *** nothing ACKed - check power, wiring, pull-ups ***\n",
		       label);
	} else {
		printk("I2C_SCAN: %-7s   %u device(s)\n", label, found);
	}
}

static int i2c_probe_diag(void)
{
	printk("I2C_SCAN: ---- I2C bus address scan ----\n");

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(lpi2c1))
	scan_bus("lpi2c1", DEVICE_DT_GET(DT_NODELABEL(lpi2c1)));
#else
	printk("I2C_SCAN: lpi2c1  disabled in DTS\n");
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(lpi2c2))
	scan_bus("lpi2c2", DEVICE_DT_GET(DT_NODELABEL(lpi2c2)));
#else
	printk("I2C_SCAN: lpi2c2  disabled in DTS\n");
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(lpi2c3))
	scan_bus("lpi2c3", DEVICE_DT_GET(DT_NODELABEL(lpi2c3)));
#else
	printk("I2C_SCAN: lpi2c3  disabled in DTS\n");
#endif

	printk("I2C_SCAN: ---- scan complete ----\n");
	return 0;
}

/* APPLICATION/51: just after the SPI scan, before ArduPilot starts probing. */
SYS_INIT(i2c_probe_diag, APPLICATION, 51);

#endif /* CONFIG_AP_I2C_PROBE_DIAG */
