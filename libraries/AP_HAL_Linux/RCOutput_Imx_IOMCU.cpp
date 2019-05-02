#include "RCOutput_Imx_IOMCU.h"

namespace Linux
{
	RCOutput_Imx_IOMCU::RCOutput_Imx_IOMCU(Imx_IOMCU *m)
	{
		mcu = m;
	}

	RCOutput_Imx_IOMCU::~RCOutput_Imx_IOMCU()
	{
	}

	void RCOutput_Imx_IOMCU::init()
	{
		set_freq(0xFF, 50);
	}

	void RCOutput_Imx_IOMCU::set_freq(uint32_t chmask, uint16_t freq_hz)
	{
		if (freq_hz > 400)
			freq_hz = 400;
		for (int i = 0; i < 12; i++)
		{
			if (chmask & (1u << i))
			{
				all_freq[i] = freq_hz;
			}
		}
		mcu->set_freq(chmask, all_freq);
	}

	uint16_t RCOutput_Imx_IOMCU::get_freq(uint8_t ch)
	{
		return all_freq[ch];
	}

	void RCOutput_Imx_IOMCU::enable_ch(uint8_t ch)
	{
	}

	void RCOutput_Imx_IOMCU::disable_ch(uint8_t ch)
	{
		write(ch, 1);
	}

	void RCOutput_Imx_IOMCU::write(uint8_t ch, uint16_t period_us)
	{
		if (ch >= 12)
			return;

		_pulses_buffer[ch] = period_us;
		_pending_write_mask |= (1U << ch);

		if (!_corking) {
			_corking = true;
			push();
		}
	}

	void RCOutput_Imx_IOMCU::cork()
	{
		_corking = true;
	}

	void RCOutput_Imx_IOMCU::push()
	{
		if (!_corking) {
			return;
		}
		_corking = false;

		if (_pending_write_mask == 0)
			return;

		mcu->set_duty(_pending_write_mask, _pulses_buffer);

		_pending_write_mask = 0;
	}

	uint16_t RCOutput_Imx_IOMCU::read(uint8_t ch)
	{
		return _pulses_buffer[ch];
	}

	void RCOutput_Imx_IOMCU::read(uint16_t *period_us, uint8_t len)
	{
		for (int i = 0; i < len; i++)
		{
			period_us[i] = read(i);
		}
	}
}