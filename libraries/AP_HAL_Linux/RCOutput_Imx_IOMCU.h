#pragma once

#include "AP_HAL_Linux.h"
#include "Imx_IOMCU.h"

namespace Linux
{
	class RCOutput_Imx_IOMCU : public AP_HAL::RCOutput
	{
	public:
		RCOutput_Imx_IOMCU(Imx_IOMCU *m);
		~RCOutput_Imx_IOMCU();

		void init();
		void set_freq(uint32_t chmask, uint16_t freq_hz);
		uint16_t get_freq(uint8_t ch);
		void enable_ch(uint8_t ch);
		void disable_ch(uint8_t ch);
		void write(uint8_t ch, uint16_t period_us);
		void cork() override;
		void push() override;
		uint16_t read(uint8_t ch);
		void read(uint16_t *period_us, uint8_t len);

	private:
		Imx_IOMCU *mcu;
		uint32_t _pulses_buffer[12];
		uint32_t all_freq[12];
		bool _corking = false;
		uint32_t _pending_write_mask;
	};
}
