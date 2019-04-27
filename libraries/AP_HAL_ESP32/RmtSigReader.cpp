#include "RmtSigReader.h"

using namespace ESP32;

void RmtSigReader::init()
{
	printf("%s\n",__PRETTY_FUNCTION__);

	// in case the peripheral was left in a bad state, such as reporting full buffers, this can help clear it, and can be called repeatedly if need be.
	periph_module_reset(PERIPH_RMT_MODULE);


    rmt_config_t config = {
        .rmt_mode = RMT_MODE_RX,
        .channel = RMT_CHANNEL_0,
        .clk_div = 80,   //80MHZ APB clock to the 1MHZ target frequency
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_ICARUS
        .gpio_num = (gpio_num_t)36,
#endif
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_DIY
        .gpio_num = (gpio_num_t)4,
#endif
        .mem_block_num = 2 //each block could store 64 pulses
    };
    config.rx_config.filter_en = true;
    config.rx_config.filter_ticks_thresh = 8;
    config.rx_config.idle_threshold = idle_threshold;

    rmt_config(&config);
    rmt_driver_install(config.channel, max_pulses * 8, 0);
    rmt_get_ringbuf_handle(config.channel, &handle);
    rmt_rx_start(config.channel, true);
}

bool RmtSigReader::add_item(uint32_t duration, bool level)
{
	//printf("%s\n",__PRETTY_FUNCTION__);

    bool has_more = true;
    if (duration == 0) {
        has_more = false;
        duration = idle_threshold;
    }
    if (level) {
        if (last_high == 0) {
            last_high = duration;
        }
    } else {
        if (last_high != 0) {
            ready_high = last_high;
            ready_low = duration;
            pulse_ready = true;
            last_high = 0;
        }
    }
    return has_more;
}

bool RmtSigReader::read(uint32_t &width_high, uint32_t &width_low)
{
	//printf("%s\n",__PRETTY_FUNCTION__);

    if (item == nullptr) {
        item = (rmt_item32_t*) xRingbufferReceive(handle, &item_size, 0);
        item_size /= 4;
        current_item = 0;
    }
    if (item == nullptr) {
        return false;
    }


    //printf("item size ( ~number of transitions): %d\n",item_size);
	for (unsigned int i = 0; i < item_size>>2; i++)
	{
		printf("%d:%dus %d:%dus\n", (item+i)->level0, (item+i)->duration0, (item+i)->level1, (item+i)->duration1);
	}
	//vRingbufferReturnItem(rb, (void*) item);

    bool buffer_empty = (current_item == item_size);
    buffer_empty = buffer_empty || !add_item(item[current_item].duration0, item[current_item].level0);
    buffer_empty = buffer_empty || !add_item(item[current_item].duration1, item[current_item].level1);
    current_item++;
    if (buffer_empty) {
        vRingbufferReturnItem(handle, (void*) item);
        item = nullptr;
    }
    if (pulse_ready) {
        width_high = ready_high;
        width_low = ready_low;
        pulse_ready = false;
        return true;
    }
    return false;
}
