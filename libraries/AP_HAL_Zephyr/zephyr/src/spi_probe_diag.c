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
/* Boot-time SPI sensor identification scan: reports which parts actually answer
 * on each bus, because this board's documented fit has been wrong three times. */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>

#ifdef CONFIG_AP_SPI_PROBE_DIAG

/* Identity registers across the IMU families this board may carry. All of
 * these parts use "MSB set == read" for register access, so one read helper
 * covers every candidate. */
struct id_reg {
	uint8_t reg;
	const char *what;
};

static const struct id_reg id_regs[] = {
	{ 0x75, "WHO_AM_I(0x75)" },   /* ICM-206xx / ICM-426xx / ICM-209xx  */
	{ 0x72, "WHO_AM_I(0x72)" },   /* ICM-456xx (e.g. ICM-45686)         */
	{ 0x00, "CHIP_ID(0x00)" },    /* BMI088 accel 0x1e, gyro 0x0f       */
	{ 0x0f, "WHO_AM_I(0x0f)" },   /* ST ISM330DHCX 0x6b                 */
};

/* Known identity values, so the log names the part rather than leaving a bare
 * hex byte for a human to look up. */
static const char *id_to_part(uint8_t reg, uint8_t val)
{
	if (reg == 0x75) {
		switch (val) {
		case 0x12: return "ICM-20602";
		case 0x20: return "ICM-20948";
		case 0xe1: return "ICM-20649";
		case 0x47: return "ICM-42688-P";
		case 0x67: return "ICM-42670-P";
		case 0x6f: return "ICM-42605";
		case 0x71: return "MPU-9250";
		case 0x68: return "MPU-6000/6500";
		default: break;
		}
	} else if (reg == 0x72) {
		if (val == 0xe9) { return "ICM-45686"; }
	} else if (reg == 0x00) {
		if (val == 0x1e) { return "BMI088-accel"; }
		if (val == 0x1f) { return "BMI085-accel"; }
		if (val == 0x0f) { return "BMI088/BMI160-gyro"; }
		if (val == 0x24) { return "BMI270"; }
		if (val == 0xd1) { return "BMI160"; }
	} else if (reg == 0x0f) {
		/* ST 6-DoF / accelerometer WHO_AM_I values, taken from ST's datasheets. */
		switch (val) {
		case 0x6a: return "ST ISM330DLC / LSM6DSL / LSM6DSM / LSM6DS3TR-C *** NO AP DRIVER ***";
		case 0x6b: return "ST ISM330DHCX / ASM330LHH / LSM6DSR *** NO AP DRIVER ***";
		case 0x6c: return "ST LSM6DSO / LSM6DSOX / LSM6DSO32 *** NO AP DRIVER ***";
		case 0x70: return "ST LSM6DSV / LSM6DSV16X / LSM6DSV32X *** NO AP DRIVER ***";
		case 0x71: return "ST ISM330BX / LSM6DSV16B *** NO AP DRIVER ***";
		case 0x73: return "ST ISM6HG256X / LSM6DSV80X *** NO AP DRIVER ***";
		case 0x22: return "ST ISM330IS / LSM6DSO16IS *** NO AP DRIVER ***";
		case 0x68: return "ST LSM9DS1 (AP driver EXISTS: AP_InertialSensor_LSM9DS1)";
		default: break;
		}
	}
	return NULL;
}

/* Read `n` bytes after the address byte; n==1 is the plain register read. */
static int read_regs(const struct spi_dt_spec *spec, uint8_t reg,
		     uint8_t *out, size_t n)
{
	uint8_t tx[4] = { (uint8_t)(reg | 0x80U), 0U, 0U, 0U };
	uint8_t rx[4] = { 0U, 0U, 0U, 0U };
	size_t len = n + 1U;

	if (len > sizeof(tx)) {
		return -EINVAL;
	}

	const struct spi_buf tx_buf = { .buf = tx, .len = len };
	const struct spi_buf rx_buf = { .buf = rx, .len = len };
	const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

	int rc = spi_transceive_dt(spec, &tx_set, &rx_set);
	if (rc == 0) {
		for (size_t i = 0; i < n; i++) {
			out[i] = rx[i + 1U];
		}
	}
	return rc;
}

static int read_reg(const struct spi_dt_spec *spec, uint8_t reg, uint8_t *out)
{
	return read_regs(spec, reg, out, 1U);
}

/* Same register, read both ways, so a Bosch part cannot masquerade as an
 * Invensense one. Returns the "skip-dummy" byte in *bosch. */
static int read_reg_both(const struct spi_dt_spec *spec, uint8_t reg,
			 uint8_t *direct, uint8_t *bosch)
{
	uint8_t two[2] = { 0U, 0U };
	int rc = read_reg(spec, reg, direct);

	if (rc != 0) {
		return rc;
	}
	rc = read_regs(spec, reg, two, 2U);
	if (rc == 0) {
		*bosch = two[1];
	}
	return rc;
}

/* Dump a register range in both forms. A pattern across 16 registers
 * identifies a part far more reliably than one WHO_AM_I byte, and immediately
 * shows the tell-tale one-byte shift when a dummy byte is in play. */
static void dump_range(const char *label, const struct spi_dt_spec *spec,
		       uint8_t first, uint8_t count)
{
	printk("SPI_SCAN: %-7s regs 0x%02x..0x%02x\n", label, first,
	       (unsigned)(first + count - 1U));

	printk("SPI_SCAN: %-7s   direct:", label);
	for (uint8_t i = 0; i < count; i++) {
		uint8_t v = 0U;

		if (read_reg(spec, (uint8_t)(first + i), &v) != 0) {
			printk(" --");
			continue;
		}
		printk(" %02x", v);
	}
	printk("\n");

	printk("SPI_SCAN: %-7s   skipdum:", label);
	for (uint8_t i = 0; i < count; i++) {
		uint8_t two[2] = { 0U, 0U };

		if (read_regs(spec, (uint8_t)(first + i), two, 2U) != 0) {
			printk(" --");
			continue;
		}
		printk(" %02x", two[1]);
	}
	printk("\n");
}

static void scan_bus(const char *label, const struct spi_dt_spec *spec)
{
	if (!device_is_ready(spec->bus)) {
		printk("SPI_SCAN: %-7s bus '%s' NOT READY\n", label, spec->bus->name);
		return;
	}

	printk("SPI_SCAN: %-7s bus '%s' ready, cs=%s pin %u\n", label,
	       spec->bus->name, spec->config.cs.gpio.port->name,
	       spec->config.cs.gpio.pin);

	/* Bosch BMI08x accel needs a throwaway read before its SPI interface
	 * answers at all - AP_InertialSensor_BMI088.cpp does the same thing
	 * ("dummy read on accel ChipID to init accel, see section 3 of
	 * datasheet"). Harmless on every other candidate. */
	uint8_t discard = 0U;
	(void)read_reg(spec, 0x00U, &discard);

	for (size_t i = 0; i < ARRAY_SIZE(id_regs); i++) {
		uint8_t direct = 0U;
		uint8_t bosch = 0U;
		int rc = read_reg_both(spec, id_regs[i].reg, &direct, &bosch);

		if (rc != 0) {
			printk("SPI_SCAN: %-7s   %-15s transceive rc=%d\n",
			       label, id_regs[i].what, rc);
			continue;
		}

		const char *pd = id_to_part(id_regs[i].reg, direct);
		const char *pb = id_to_part(id_regs[i].reg, bosch);

		printk("SPI_SCAN: %-7s   %-15s direct=0x%02x skipdum=0x%02x%s%s%s%s\n",
		       label, id_regs[i].what, direct, bosch,
		       pd ? "  <-- " : "", pd ? pd : "",
		       pb ? "  <-- (skipdum) " : "", pb ? pb : "");
	}

	/* Identity registers alone were not enough to name the parts on lpspi1
	 * (0x44) and lpspi3 (0x25/0x1e), and those may well have been dummy
	 * bytes. Dump the low and high register blocks so the part can be
	 * recognised by pattern. */
	dump_range(label, spec, 0x00U, 16U);
	dump_range(label, spec, 0x70U, 16U);
}

/* One spec per enabled sensor bus. Frequency is deliberately low (1 MHz) and
 * mode 3 — every candidate part supports that for register reads, so a
 * non-answer here is a wiring or bus fault, not a speed/mode mismatch. */
#define SCAN_SPEC(nodelabel)                                                  \
	{                                                                     \
		.bus = DEVICE_DT_GET(DT_NODELABEL(nodelabel)),                \
		.config = {                                                   \
			.frequency = 1000000U,                                \
			.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |   \
				     SPI_TRANSFER_MSB | SPI_MODE_CPOL |       \
				     SPI_MODE_CPHA,                           \
			.slave = 0,                                           \
			.cs = {                                               \
				.gpio = GPIO_DT_SPEC_GET_BY_IDX(              \
					DT_NODELABEL(nodelabel), cs_gpios, 0),\
				.delay = 2,                                   \
				.cs_is_gpio = true,                           \
			},                                                    \
		},                                                            \
	}

/* Switched-rail voltage readout. */
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(lpadc1))

#define RAIL_ADC_NODE DT_NODELABEL(lpadc1)

struct rail_mon {
	const char *name;
	uint8_t channel_id;      /* which CMD register */
	uint8_t input_positive;  /* low bits = channel num, bit5 = side B */
};

static const struct rail_mon rails[] = {
	{ "VDD_3V3_SENSORS1", 0, 2        },  /* AD_10 ADC1_CH2A */
	{ "VDD_3V3_SENSORS2", 1, 2 | 0x20 },  /* AD_11 ADC1_CH2B */
	{ "VDD_3V3_SENSORS3", 2, 3        },  /* AD_12 ADC1_CH3A */
	{ "VDD_3V3_SENSORS4", 3, 5 | 0x20 },  /* AD_17 ADC1_CH5B */
};

static void scan_rails(void)
{
	const struct device *adc = DEVICE_DT_GET(RAIL_ADC_NODE);
	int16_t sample = 0;

	if (!device_is_ready(adc)) {
		printk("SPI_SCAN: rail ADC not ready - cannot verify rails\n");
		return;
	}

	for (size_t i = 0; i < ARRAY_SIZE(rails); i++) {
		struct adc_channel_cfg ccfg = {
			.gain             = ADC_GAIN_1,
			/* adc_mcux_lpadc accepts only EXTERNAL0/EXTERNAL1 channel identifiers. */
			.reference        = ADC_REF_EXTERNAL0,
			.acquisition_time = ADC_ACQ_TIME_DEFAULT,
			.channel_id       = rails[i].channel_id,
			.input_positive   = rails[i].input_positive,
			.differential     = false,
		};

		int rc = adc_channel_setup(adc, &ccfg);
		if (rc != 0) {
			printk("SPI_SCAN: rail %-18s setup rc=%d\n",
			       rails[i].name, rc);
			continue;
		}

		struct adc_sequence seq = {
			.channels    = BIT(rails[i].channel_id),
			.buffer      = &sample,
			.buffer_size = sizeof(sample),
			.resolution  = 12,
		};

		rc = adc_read(adc, &seq);
		if (rc != 0) {
			printk("SPI_SCAN: rail %-18s read rc=%d\n",
			       rails[i].name, rc);
			continue;
		}

		/* Raw counts are enough to answer "is it up?" - a rail that
		 * never came up reads near zero. Absolute volts would need the
		 * divider ratio, which is not needed for a go/no-go. */
		printk("SPI_SCAN: rail %-18s raw=%d  %s\n", rails[i].name,
		       (int)sample, sample > 200 ? "UP" : "*** DOWN ***");
	}
}
#else
static void scan_rails(void)
{
	printk("SPI_SCAN: lpadc1 disabled - cannot verify rail voltages\n");
}
#endif

/* Defined in rt1176_gpio_mux_fixup.c. Re-applied here, immediately before the
 * scan, and reported - a GPIO_MUX bit left selecting CM7_GPIOn silently steals
 * the pad from the gpio node Zephyr drives, so a chip select can read as
 * configured-and-driven while the pin never actually moves. */
extern void rt1176_gpio_mux_apply(uint32_t *before, uint32_t *after);

static void scan_gpio_mux(void)
{
	static const char *const names[4] = {
		"GPR40 GPIO_MUX2_SEL_LOW ", "GPR41 GPIO_MUX2_SEL_HIGH",
		"GPR42 GPIO_MUX3_SEL_LOW ", "GPR43 GPIO_MUX3_SEL_HIGH",
	};
	uint32_t before[4] = { 0 };
	uint32_t after[4] = { 0 };

	rt1176_gpio_mux_apply(before, after);

	for (int i = 0; i < 4; i++) {
		printk("SPI_SCAN: %s before=0x%08x after=0x%08x%s\n", names[i],
		       before[i], after[i],
		       before[i] != 0U ? "   <-- pads were stolen by CM7_GPIO" : "");
	}
}

static int spi_probe_diag(void)
{
	printk("SPI_SCAN: ---- sensor bus identification scan ----\n");
	scan_gpio_mux();
	scan_rails();

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(lpspi1))
	static const struct spi_dt_spec s1 = SCAN_SPEC(lpspi1);
	scan_bus("lpspi1", &s1);
#else
	printk("SPI_SCAN: lpspi1  disabled in DTS\n");
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(lpspi2))
	static const struct spi_dt_spec s2 = SCAN_SPEC(lpspi2);
	scan_bus("lpspi2", &s2);
#else
	printk("SPI_SCAN: lpspi2  disabled in DTS\n");
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(lpspi3))
	static const struct spi_dt_spec s3 = SCAN_SPEC(lpspi3);
	scan_bus("lpspi3", &s3);
#else
	printk("SPI_SCAN: lpspi3  disabled in DTS\n");
#endif

	printk("SPI_SCAN: ---- scan complete ----\n");
	return 0;
}

/* APPLICATION/50: after all SPI and GPIO drivers are initialised, and before
 * ArduPilot's own probe order starts producing INS messages. */
SYS_INIT(spi_probe_diag, APPLICATION, 50);

#endif  /* CONFIG_AP_SPI_PROBE_DIAG */
