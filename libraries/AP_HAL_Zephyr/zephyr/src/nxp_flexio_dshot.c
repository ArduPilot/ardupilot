/* VENDORED verbatim from CogniPilot cerebri: modules/cerebri is reference
 * material, not a build dependency. */
/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#define LOG_MODULE_NAME nxp_flexio_dshot
#include <fsl_clock.h>
#include <fsl_flexio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_NXP_FLEXIO_DSHOT_LOG_LEVEL);

#ifdef CONFIG_SOC_MIMX9596_M7
#include <zephyr/drivers/firmware/scmi/clk.h>
#include <zephyr/dt-bindings/clock/imx95_clock.h>
#endif

#include <zephyr/drivers/misc/nxp_flexio/nxp_flexio.h>

#define DT_DRV_COMPAT nxp_flexio_dshot

#define DSHOT_THROTTLE_POSITION  5u
#define DSHOT_TELEMETRY_POSITION 4u
#define NIBBLES_SIZE             4u
#define DSHOT_NUMBER_OF_NIBBLES  3u

static const uint32_t gcr_decode[32] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x9, 0xA,
					0xB, 0x0, 0xD, 0xE, 0xF, 0x0, 0x0, 0x2, 0x3, 0x0, 0x5,
					0x6, 0x7, 0x0, 0x0, 0x8, 0x1, 0x0, 0x4, 0xC, 0x0};

typedef enum {
	DSHOT_START = 0,
	DSHOT_12BIT_FIFO,
	DSHOT_12BIT_TRANSFERRED,
	DSHOT_TRANSMIT_COMPLETE,
	BDSHOT_RECEIVE,
	BDSHOT_RECEIVE_COMPLETE,
} dshot_state;

struct nxp_flexio_dshot_channel {
	uint8_t dshot_channel_count;
	struct nxp_flexio_dshot_channel_config *dshot_info;
};

struct nxp_flexio_dshot_config {
	const struct device *flexio_dev;
	FLEXIO_Type *flexio_base;
	const struct pinctrl_dev_config *pincfg;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct nxp_flexio_dshot_channel *channel;
	const struct nxp_flexio_child *child;
	const uint32_t speed;
};

struct nxp_flexio_dshot_data {
	uint32_t flexio_clk;
	uint32_t dshot_tcmp;
	uint32_t bdshot_tcmp;
	uint32_t dshot_mask;
	uint32_t dshot_timer_mask;
	uint32_t bdshot_recv_mask;
	uint32_t bdshot_parsed_recv_mask;
};

struct nxp_flexio_dshot_channel_config {
	/** Flexio used pin index */
	uint8_t pin_id;
	bool init;
	uint32_t data_seg1;
	uint32_t irq_data;
	dshot_state state;
	bool bdshot;
	uint32_t raw_response;
	uint16_t erpm;
	uint32_t crc_error_cnt;
	uint32_t frame_error_cnt;
	uint32_t no_response_cnt;
	uint32_t last_no_response_cnt;
};

static void nxp_flexio_dshot_output(const struct device *dev, uint32_t channel)
{
	const struct nxp_flexio_dshot_config *config = dev->config;
	struct nxp_flexio_dshot_data *data = dev->data;
	FLEXIO_Type *flexio_base = (FLEXIO_Type *)(config->flexio_base);
	struct nxp_flexio_child *child = (struct nxp_flexio_child *)(config->child);
	struct nxp_flexio_dshot_channel_config *dshot_info = &config->channel->dshot_info[channel];

	flexio_timer_config_t timerConfig;
	flexio_shifter_config_t shifterConfig;

	/* Disable Shifter */
	(void)memset(&shifterConfig, 0, sizeof(shifterConfig));
	FLEXIO_SetShifterConfig(flexio_base, child->res.shifter_index[channel], &shifterConfig);

	/* No start bit, stop bit low */
	shifterConfig.inputSource = kFLEXIO_ShifterInputFromPin;
	shifterConfig.shifterStop = kFLEXIO_ShifterStopBitLow;
	shifterConfig.shifterStart = kFLEXIO_ShifterStartBitDisabledLoadDataOnEnable;

	/* Transmit mode, output to FXIO pin, inverted output for bdshot */
	shifterConfig.timerSelect = child->res.timer_index[channel];
	shifterConfig.timerPolarity = kFLEXIO_ShifterTimerPolarityOnPositive;
	shifterConfig.pinConfig = kFLEXIO_PinConfigOutput;
	shifterConfig.pinSelect = dshot_info->pin_id;
	shifterConfig.pinPolarity = dshot_info->bdshot;
	shifterConfig.shifterMode = kFLEXIO_ShifterModeTransmit;

	FLEXIO_SetShifterConfig(flexio_base, child->res.shifter_index[channel], &shifterConfig);

	(void)memset(&timerConfig, 0, sizeof(timerConfig));

	/* Start transmitting on trigger, disable on compare */
	timerConfig.timerOutput = kFLEXIO_TimerOutputOneNotAffectedByReset;
	timerConfig.timerDecrement = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
	timerConfig.timerReset = kFLEXIO_TimerResetNever;
	timerConfig.timerDisable = kFLEXIO_TimerDisableOnTimerCompare;
	timerConfig.timerEnable = kFLEXIO_TimerEnableOnTriggerHigh;
	timerConfig.timerStop = kFLEXIO_TimerStopBitDisabled;
	timerConfig.timerStart = kFLEXIO_TimerStartBitDisabled;

	timerConfig.timerCompare = data->dshot_tcmp;

	/* Baud mode, Trigger on shifter write */
	timerConfig.triggerSelect =
		FLEXIO_TIMER_TRIGGER_SEL_SHIFTnSTAT(child->res.shifter_index[channel]);
	timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveLow;
	timerConfig.triggerSource = kFLEXIO_TimerTriggerSourceInternal;
	timerConfig.pinConfig = kFLEXIO_PinConfigOutputDisabled;
	timerConfig.pinSelect = 0;
	timerConfig.pinPolarity = kFLEXIO_PinActiveLow;
	timerConfig.timerMode = kFLEXIO_TimerModeDual8BitBaudBit;

	FLEXIO_SetTimerConfig(flexio_base, child->res.timer_index[channel], &timerConfig);
}

static void nxp_flexio_bdshot_input(const struct device *dev, uint32_t channel)
{
	const struct nxp_flexio_dshot_config *config = dev->config;
	struct nxp_flexio_dshot_data *data = dev->data;
	FLEXIO_Type *flexio_base = (FLEXIO_Type *)(config->flexio_base);
	struct nxp_flexio_child *child = (struct nxp_flexio_child *)(config->child);
	struct nxp_flexio_dshot_channel_config *dshot_info = &config->channel->dshot_info[channel];

	flexio_timer_config_t timerConfig;
	flexio_shifter_config_t shifterConfig;

	(void)memset(&timerConfig, 0, sizeof(timerConfig));
	(void)memset(&shifterConfig, 0, sizeof(shifterConfig));

	/* Input data from pin, no start/stop bit*/
	shifterConfig.inputSource = kFLEXIO_ShifterInputFromPin;
	shifterConfig.shifterStop = kFLEXIO_ShifterStopBitDisable;
	shifterConfig.shifterStart = kFLEXIO_ShifterStartBitDisabledLoadDataOnShift;

	/* Shifter receive mode, on FXIO pin input */
	shifterConfig.timerSelect = child->res.timer_index[channel];
	shifterConfig.timerPolarity = kFLEXIO_ShifterTimerPolarityOnPositive;
	shifterConfig.pinConfig = kFLEXIO_PinConfigOutputDisabled;
	shifterConfig.pinSelect = dshot_info->pin_id;
	shifterConfig.pinPolarity = kFLEXIO_PinActiveLow;
	shifterConfig.shifterMode = kFLEXIO_ShifterModeReceive;

	FLEXIO_SetShifterConfig(flexio_base, child->res.shifter_index[channel], &shifterConfig);

	/* Make sure there no shifter flags high from transmission */
	FLEXIO_ClearShifterStatusFlags(flexio_base, 1 << child->res.shifter_index[channel]);

	/* Enable on pin transition, resychronize through reset on rising
	 * edge */
	timerConfig.timerOutput = kFLEXIO_TimerOutputOneAffectedByReset;
	timerConfig.timerDecrement = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
	timerConfig.timerReset = kFLEXIO_TimerResetOnTimerPinRisingEdge;
	timerConfig.timerDisable = kFLEXIO_TimerDisableOnTimerCompare;
	timerConfig.timerEnable = kFLEXIO_TimerEnableOnTriggerBothEdge;
	timerConfig.timerStop = kFLEXIO_TimerStopBitEnableOnTimerDisable;
	timerConfig.timerStart = kFLEXIO_TimerStartBitEnabled;

	timerConfig.timerCompare = data->bdshot_tcmp;

	/* Baud mode, Trigger on shifter write */
	timerConfig.triggerSelect = FLEXIO_TIMER_TRIGGER_SEL_PININPUT(dshot_info->pin_id);
	timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveHigh;
	timerConfig.triggerSource = kFLEXIO_TimerTriggerSourceInternal;
	timerConfig.pinConfig = kFLEXIO_PinConfigOutputDisabled;
	timerConfig.pinSelect = 0;
	timerConfig.pinPolarity = kFLEXIO_PinActiveLow;
	timerConfig.timerMode = kFLEXIO_TimerModeDual8BitBaudBit;

	FLEXIO_SetTimerConfig(flexio_base, child->res.timer_index[channel], &timerConfig);
}

static int nxp_flexio_dshot_set_clock(const struct nxp_flexio_dshot_config *cfg)
{
	int ret = 0;
#ifdef CONFIG_SOC_MIMXRT1176
	/* Init System Pll2 pfd3 to 432Mhz */
	CLOCK_InitPfd(kCLOCK_PllSys2, kCLOCK_Pfd3, 22);

	clock_root_config_t rootCfg = {0};

	/* Configure FLEXIO1 using SysPLL3Div2 @ 108MHz */
	rootCfg.mux = kCLOCK_FLEXIO1_ClockRoot_MuxSysPll2Pfd3;
	rootCfg.div = 4;
	CLOCK_SetRootClock(kCLOCK_Root_Flexio1, &rootCfg);
#endif

#ifdef CONFIG_SOC_MIMX9596_M7
	uint64_t flexio_clk = 133333333;
	struct scmi_protocol *proto = cfg->clock_dev->data;
	struct scmi_clock_rate_config clk_cfg = {0};
	ret = scmi_clock_parent_set(proto, IMX95_CLK_FLEXIO1, IMX95_CLK_SYSPLL1_PFD1_DIV2);
	if (ret) {
		return ret;
	}

	clk_cfg.flags = SCMI_CLK_RATE_SET_FLAGS_ROUNDS_AUTO;
	clk_cfg.clk_id = IMX95_CLK_FLEXIO1;
	clk_cfg.rate[0] = flexio_clk & 0xffffffff;
	clk_cfg.rate[1] = (flexio_clk >> 32) & 0xffffffff;

	ret = scmi_clock_rate_set(proto, &clk_cfg);
	if (ret) {
		return ret;
	}
#endif

#ifdef CONFIG_SOC_MIMXRT1064
	/* 108Mhz clock for FlexIO using PLL3 PFD2 @ 520 */
	CLOCK_InitUsb1Pfd(kCLOCK_Pfd2, 16U);

	CLOCK_SetMux(kCLOCK_Flexio1Mux, 1); // PPL3 PFD2
	CLOCK_SetDiv(kCLOCK_Flexio1Div, 0);
	CLOCK_SetDiv(kCLOCK_Flexio1PreDiv, 4);
#endif

	return ret;
}

static int nxp_flexio_dshot_init(const struct device *dev)
{
	const struct nxp_flexio_dshot_config *config = dev->config;
	struct nxp_flexio_dshot_data *data = dev->data;
	uint8_t channel = 0;
	int err;
	struct nxp_flexio_child *child = (struct nxp_flexio_child *)(config->child);

	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	if (nxp_flexio_dshot_set_clock(config)) {
		return -EINVAL;
	}

	if (clock_control_get_rate(config->clock_dev, config->clock_subsys, &data->flexio_clk)) {
		return -EINVAL;
	}

	const int dshot_pwm_freq = config->speed * 1000;

	/* Calculate dshot timings based on dshot_pwm_freq */
	data->dshot_tcmp = 0x2F00 | (((data->flexio_clk / (dshot_pwm_freq * 3) / 2) - 1) & 0xFF);
	data->bdshot_tcmp =
		0x2900 | (((data->flexio_clk / (dshot_pwm_freq * 5 / 4) / 2) - 3) & 0xFF);

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		LOG_ERR("Failed to configure pins");
		return err;
	}

	err = nxp_flexio_child_attach(config->flexio_dev, child);
	if (err < 0) {
		LOG_ERR("Failed to attach child");
		return err;
	}

	data->dshot_mask = 0;
	data->dshot_timer_mask = 0;

	for (channel = 0; channel < config->channel->dshot_channel_count; channel++) {
		nxp_flexio_dshot_output(dev, channel);
		config->channel->dshot_info[channel].init = true;
		data->dshot_mask |= (1 << child->res.shifter_index[channel]);
		data->dshot_timer_mask |= (1 << child->res.timer_index[channel]);
	}

	return 0;
}

void nxp_flexio_dshot_trigger(const struct device *dev)
{
	const struct nxp_flexio_dshot_config *config = dev->config;
	struct nxp_flexio_dshot_data *data = dev->data;
	FLEXIO_Type *flexio_base = (FLEXIO_Type *)(config->flexio_base);
	struct nxp_flexio_dshot_channel_config *dshot_info;

	FLEXIO_ClearTimerStatusFlags(flexio_base, data->dshot_timer_mask);

	for (uint8_t channel = 0; (channel < config->channel->dshot_channel_count); channel++) {
		dshot_info = &config->channel->dshot_info[channel];

		if (dshot_info->bdshot && (data->bdshot_recv_mask & (1 << channel)) == 0) {
			dshot_info->no_response_cnt++;
		}

		if (dshot_info->init && dshot_info->data_seg1 != 0) {
			flexio_base->SHIFTBUF[channel] = dshot_info->data_seg1;
		}
	}

	data->bdshot_recv_mask = 0x0;

	FLEXIO_ClearTimerStatusFlags(flexio_base, data->dshot_timer_mask);
	FLEXIO_EnableShifterStatusInterrupts(flexio_base, data->dshot_mask);
	FLEXIO_EnableTimerStatusInterrupts(flexio_base, data->dshot_timer_mask);
}

uint8_t nxp_flexio_dshot_channel_count(const struct device *dev)
{
	const struct nxp_flexio_dshot_config *config = dev->config;

	return config->channel->dshot_channel_count;
}

/* Expand packet from 16 bits 48 to get T0H and T1H timing */
uint64_t nxp_flexio_dshot_expand_data(uint16_t packet)
{
	unsigned int mask;
	unsigned int index = 0;
	uint64_t expanded = 0x0;

	for (mask = 0x8000; mask != 0; mask >>= 1) {
		if (packet & mask) {
			expanded = expanded | ((uint64_t)0x3 << index);

		} else {
			expanded = expanded | ((uint64_t)0x1 << index);
		}

		index = index + 3;
	}

	return expanded;
}

/**
 * bits 	1-11	- throttle value (0-47 are reserved, 48-2047 give 2000
 *steps of throttle resolution) bit 	12		- dshot telemetry
 *enable/disable bits 	13-16	- XOR checksum
 **/
void nxp_flexio_dshot_data_set(const struct device *dev, unsigned channel, uint16_t throttle,
			       bool telemetry)
{
	const struct nxp_flexio_dshot_config *config = dev->config;
	struct nxp_flexio_dshot_data *data = dev->data;
	struct nxp_flexio_dshot_channel_config *dshot_info = &config->channel->dshot_info[channel];
	FLEXIO_Type *flexio_base = (FLEXIO_Type *)(config->flexio_base);

	if (channel < config->channel->dshot_channel_count && dshot_info->init) {
		uint16_t csum_data;
		uint16_t packet = 0;
		uint16_t checksum = 0;

		packet |= throttle << DSHOT_THROTTLE_POSITION;
		packet |= ((uint16_t)telemetry & 0x01) << DSHOT_TELEMETRY_POSITION;

		if (dshot_info->bdshot) {
			csum_data = ~packet;

		} else {
			csum_data = packet;
		}

		/* XOR checksum calculation */
		csum_data >>= NIBBLES_SIZE;

		for (unsigned i = 0; i < DSHOT_NUMBER_OF_NIBBLES; i++) {
			checksum ^= (csum_data & 0x0F); // XOR data by nibbles
			csum_data >>= NIBBLES_SIZE;
		}

		packet |= (checksum & 0x0F);

		uint64_t dshot_expanded = nxp_flexio_dshot_expand_data(packet);

		dshot_info->data_seg1 = (uint32_t)(dshot_expanded & 0xFFFFFF);
		dshot_info->irq_data = (uint32_t)(dshot_expanded >> 24);
		dshot_info->state = DSHOT_START;

		if (dshot_info->bdshot) {
			flexio_base->TIMCTL[config->child->res.timer_index[channel]] = 0;
			FLEXIO_DisableShifterStatusInterrupts(flexio_base, data->dshot_mask);

			nxp_flexio_dshot_output(dev, channel);

			FLEXIO_ClearTimerStatusFlags(flexio_base, data->dshot_timer_mask);
		}
	}
}

int up_bdshot_decode_erpm(const struct device *dev)
{
	const struct nxp_flexio_dshot_config *config = dev->config;
	struct nxp_flexio_dshot_data *data = dev->data;
	struct nxp_flexio_child *child = (struct nxp_flexio_child *)(config->child);
	uint32_t value;
	uint32_t decode_data;
	uint32_t csum_data;
	uint8_t exponent;
	uint16_t period;
	uint16_t erpm;
	uint32_t shifter_flag;

	data->bdshot_parsed_recv_mask = 0;

	// Decode each individual channel
	for (uint8_t channel = 0; (channel < config->channel->dshot_channel_count); channel++) {
		shifter_flag = 1 << child->res.shifter_index[channel];
		if (data->bdshot_recv_mask & shifter_flag) {
			value = ~config->channel->dshot_info[channel].raw_response & 0xFFFFF;

			/* if lowest significant isn't 1 we've got a framing error */
			if (value & 0x1) {
				/* Decode RLL */
				value = (value ^ (value >> 1));

				/* Decode GCR */
				decode_data = gcr_decode[value & 0x1fU];
				decode_data |= gcr_decode[(value >> 5U) & 0x1fU] << 4U;
				decode_data |= gcr_decode[(value >> 10U) & 0x1fU] << 8U;
				decode_data |= gcr_decode[(value >> 15U) & 0x1fU] << 12U;

				/* Calculate checksum */
				csum_data = decode_data;
				csum_data = csum_data ^ (csum_data >> 8U);
				csum_data = csum_data ^ (csum_data >> NIBBLES_SIZE);

				if ((csum_data & 0xFU) != 0xFU) {
					config->channel->dshot_info[channel].crc_error_cnt++;

				} else {
					decode_data = (decode_data >> 4) & 0xFFF;

					if (decode_data == 0xFFF) {
						erpm = 0;

					} else {
						/* 3 bit: exponent */
						exponent = ((decode_data >> 9U) & 0x7U);
						/* 9 bit: period base */
						period = (decode_data & 0x1ffU);
						period = period << exponent; /* Period in usec */
						erpm = ((1000000U * 60U / 100U + period / 2U) /
							period);
					}

					config->channel->dshot_info[channel].erpm = erpm;
					data->bdshot_parsed_recv_mask |= (1 << channel);
					config->channel->dshot_info[channel].last_no_response_cnt =
						config->channel->dshot_info[channel]
							.no_response_cnt;
				}

			} else {
				config->channel->dshot_info[channel].frame_error_cnt++;
			}
		}
	}

	return data->bdshot_parsed_recv_mask != 0;
}

// TEMP helper function before moving over to sensor api
int up_bdshot_get_erpm(const struct device *dev, uint8_t channel, int *erpm)
{
	const struct nxp_flexio_dshot_config *config = dev->config;
	struct nxp_flexio_dshot_data *data = dev->data;

	if (data->bdshot_parsed_recv_mask & (1 << channel)) {
		*erpm = (int)config->channel->dshot_info[channel].erpm;
		return 0;
	}

	return -1;
}

static int nxp_flexio_dshot_isr(void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	const struct nxp_flexio_dshot_config *config = dev->config;
	struct nxp_flexio_dshot_data *data = dev->data;
	FLEXIO_Type *flexio_base = (FLEXIO_Type *)(config->flexio_base);
	struct nxp_flexio_child *child = (struct nxp_flexio_child *)(config->child);

	uint32_t flags = FLEXIO_GetShifterStatusFlags(flexio_base);
	uint32_t channel;
	uint32_t shifter_flag;
	uint32_t timer_flag;
	struct nxp_flexio_dshot_channel_config *dshot_info;

	for (channel = 0; flags && channel < config->channel->dshot_channel_count; channel++) {
		shifter_flag = 1 << child->res.shifter_index[channel];

		if (flags & shifter_flag) {
			FLEXIO_DisableShifterStatusInterrupts(flexio_base, shifter_flag);
			dshot_info = &config->channel->dshot_info[channel];

			if (dshot_info->state == DSHOT_START) {
				dshot_info->state = DSHOT_12BIT_FIFO;
				flexio_base->SHIFTBUF[child->res.shifter_index[channel]] =
					dshot_info->irq_data;
			} else if (dshot_info->state == BDSHOT_RECEIVE) {
				dshot_info->state = BDSHOT_RECEIVE_COMPLETE;
				dshot_info->raw_response = flexio_base->SHIFTBUFBIS[channel];

				data->bdshot_recv_mask |= shifter_flag;

				if (data->bdshot_recv_mask == data->dshot_mask) {
					// Received telemetry on all channels
					// Schedule workqueue?
					// up_bdshot_erpm(dev);

					// Trigger RTIO stream?
				}
			}
		}
	}

	flags = FLEXIO_GetTimerStatusFlags(flexio_base);

	for (channel = 0; (flags & data->dshot_mask);
	     (channel = (channel + 1) % config->channel->dshot_channel_count)) {
		flags = FLEXIO_GetTimerStatusFlags(flexio_base);
		timer_flag = 1 << child->res.timer_index[channel];

		if (flags & timer_flag) {
			FLEXIO_ClearTimerStatusFlags(flexio_base, timer_flag);
			dshot_info = &config->channel->dshot_info[channel];

			if (dshot_info->state == DSHOT_12BIT_FIFO) {
				dshot_info->state = DSHOT_12BIT_TRANSFERRED;

			} else if (!dshot_info->bdshot &&
				   dshot_info->state == DSHOT_12BIT_TRANSFERRED) {
				dshot_info->state = DSHOT_TRANSMIT_COMPLETE;

			} else if (dshot_info->bdshot &&
				   dshot_info->state == DSHOT_12BIT_TRANSFERRED) {
				shifter_flag = 1 << child->res.shifter_index[channel];

				FLEXIO_DisableShifterStatusInterrupts(flexio_base, shifter_flag);
				dshot_info->state = BDSHOT_RECEIVE;

				/* Transmit done, disable timer and reconfigure to receive*/
				flexio_base->TIMCTL[config->child->res.timer_index[channel]] = 0;

				/* Configure shifter and timer to receive data */
				nxp_flexio_bdshot_input(dev, channel);

				/* Enable shifter interrupt for receiving data */
				FLEXIO_EnableShifterStatusInterrupts(flexio_base, shifter_flag);
			}
		}
	}
	return 0;
}

static int nxp_flexio_dshot_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	up_bdshot_decode_erpm(dev);

	return 0;
}

static int nxp_flexio_dshot_channel_get(const struct device *dev, enum sensor_channel chan,
					struct sensor_value *val)
{
	const struct nxp_flexio_dshot_config *config = dev->config;

	switch (chan) {
	case SENSOR_CHAN_RPM:
		for (uint32_t i = 0; i < config->channel->dshot_channel_count; i++) {
			val[i].val1 = (int32_t)config->channel->dshot_info[i].erpm;
			val[i].val2 = 0;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct sensor_driver_api nxp_flexio_dshot_api_funcs = {
	.sample_fetch = nxp_flexio_dshot_sample_fetch,
	.channel_get = nxp_flexio_dshot_channel_get,
};

#define _FLEXIO_DSHOT_GEN_CONFIG(n)                                                                \
	{                                                                                          \
		.pin_id = DT_PROP(n, pin_id),                                                      \
		.bdshot = DT_PROP(n, bidirectional_dshot),                                         \
	},

#define FLEXIO_DSHOT_GEN_CONFIG(n)                                                                 \
	static struct nxp_flexio_dshot_channel_config flexio_dshot_##n##_init[] = {                \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(n, _FLEXIO_DSHOT_GEN_CONFIG)};                   \
	static const struct nxp_flexio_dshot_channel flexio_dshot_##n##_info = {                   \
		.dshot_channel_count = ARRAY_SIZE(flexio_dshot_##n##_init),                        \
		.dshot_info = flexio_dshot_##n##_init,                                             \
	};

#define FLEXIO_DSHOT_FLEXIO_INDEX_INIT(n)                                                          \
	static uint8_t flexio_dshot_##n##_timer_index[ARRAY_SIZE(flexio_dshot_##n##_init)];        \
	static uint8_t flexio_dshot_##n##_shifter_index[ARRAY_SIZE(flexio_dshot_##n##_init)];

#define FLEXIO_DSHOT_CHILD_CONFIG(n)                                                               \
	static const struct nxp_flexio_child mcux_flexio_dshot_child_##n = {                       \
		.isr = nxp_flexio_dshot_isr,                                                       \
		.user_data = (void *)DEVICE_DT_INST_GET(n),                                        \
		.res = {.shifter_index = (uint8_t *)flexio_dshot_##n##_shifter_index,              \
			.shifter_count = ARRAY_SIZE(flexio_dshot_##n##_init),                      \
			.timer_index = (uint8_t *)flexio_dshot_##n##_timer_index,                  \
			.timer_count = ARRAY_SIZE(flexio_dshot_##n##_init)}};

#define FLEXIO_DSHOT_GEN_GET_CONFIG(n) .channel = &flexio_dshot_##n##_info,

#define NXP_FLEXIO_DSHOT_INIT(n)                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	FLEXIO_DSHOT_GEN_CONFIG(n)                                                                 \
	FLEXIO_DSHOT_FLEXIO_INDEX_INIT(n)                                                          \
	FLEXIO_DSHOT_CHILD_CONFIG(n)                                                               \
	static const struct nxp_flexio_dshot_config nxp_flexio_dshot_config_##n = {                \
		.flexio_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),                                    \
		.flexio_base = (FLEXIO_Type *)DT_REG_ADDR(DT_INST_PARENT(n)),                      \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(n))),                     \
		.clock_subsys = (clock_control_subsys_t)DT_CLOCKS_CELL(DT_INST_PARENT(n), name),   \
		.child = &mcux_flexio_dshot_child_##n,                                             \
		.speed = DT_INST_PROP(n, speed),                                                   \
		FLEXIO_DSHOT_GEN_GET_CONFIG(n)};                                                   \
                                                                                                   \
	static struct nxp_flexio_dshot_data nxp_flexio_dshot_data_##n;                             \
	SENSOR_DEVICE_DT_INST_DEFINE(n, &nxp_flexio_dshot_init, NULL, &nxp_flexio_dshot_data_##n,  \
				     &nxp_flexio_dshot_config_##n, POST_KERNEL,                    \
				     CONFIG_NXP_FLEXIO_DSHOT_INIT_PRIORITY,                        \
				     &nxp_flexio_dshot_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(NXP_FLEXIO_DSHOT_INIT)
