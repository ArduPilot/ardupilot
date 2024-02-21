/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AP_CodevEsc.h"
#include <AP_Arming/AP_Arming.h>
#include <AP_BattMonitor/AP_BattMonitor.h>



extern const AP_HAL::HAL &hal;

AP_CodevEsc::AP_CodevEsc(/* args */)
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Fence must be singleton");
    }

    _singleton = this;
}

AP_CodevEsc::~AP_CodevEsc()
{
    channels_count = 0;
}

void AP_CodevEsc::init()
{
    channels_count = HAL_ESC_NUM;
    const AP_SerialManager &serial_manager = AP::serialmanager();
    
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_CoDevESC, 0);
    if (uart != nullptr) {
        baudrate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_CoDevESC, 0);
        uart->begin(baudrate);

        // configure and initialise the esc
        configure_esc();
    }
}


int AP_CodevEsc::configure_esc()
{

    unsigned long _esc_start_time_us;

    _esc_start_time_us = AP_HAL::micros64();

    if (_esc_start_time_us < MAX_BOOT_TIME_MS * 1000) {
        hal.scheduler->delay_microseconds((MAX_BOOT_TIME_MS * 1000) - _esc_start_time_us);
    }

    /* Issue Basic Config */
    EscPacket packet = {PACKET_HEAD, sizeof(ConfigInfoBasicRequest), ESCBUS_MSG_ID_CONFIG_BASIC};
    ConfigInfoBasicRequest   &config = packet.d.reqConfigInfoBasic;
    memset(&config, 0, sizeof(ConfigInfoBasicRequest));
    config.maxChannelInUse = channels_count;
    /* Enable closed-loop control if supported by the board */
    config.controlMode = BOARD_TAP_ESC_MODE;

    /* Asign the id's to the ESCs to match the mux */
	for (uint8_t phy_chan_index = 0; phy_chan_index < channels_count; phy_chan_index++) {
		config.channelMapTable[phy_chan_index] = _device_mux_map[phy_chan_index] &
				ESC_MASK_MAP_CHANNEL;
		config.channelMapTable[phy_chan_index] |= (_device_dir_map[phy_chan_index] << 4) &
				ESC_MASK_MAP_RUNNING_DIRECTION;
	}

    config.maxChannelValue = RPMMAX;
	config.minChannelValue = RPMMIN - 5;

    select_responder(0);
    int packet_len =  crc_packet(packet);
    uart->write(&packet.head,packet_len);

	/* set wait time for tap esc configurate and write flash (0.02696s measure by Saleae logic Analyzer) */
	hal.scheduler->delay_microseconds(3000);


    /* To Unlock the ESC from the Power up state we need to issue 10
    * ESCBUS_MSG_ID_RUN request with all the values 0;
    */
    EscPacket unlock_packet = {PACKET_HEAD, channels_count, ESCBUS_MSG_ID_RUN};
	unlock_packet.len *= sizeof(unlock_packet.d.reqRun.rpm_flags[0]);
	memset(unlock_packet.d.bytes, 0, sizeof(unlock_packet.d.bytes));

    uint16_t rpm[TAP_ESC_MAX_MOTOR_NUM] = {};

    for (uint8_t i = 0; i < channels_count; i++) {
        rpm[i] = RPMSTOPPED;
        rpm[i] |= RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
    }

    int unlock_times = 5;

	while (unlock_times--) {

        hal.scheduler->delay_microseconds(30000);

        rpm[_device_mux_map[responding_esc]] |= RUN_FEEDBACK_ENABLE_MASK;

        for (uint8_t i = 0; i < channels_count; i++) {
		    unlock_packet.d.reqRun.rpm_flags[i] = rpm[i];
	    }

        if (responding_esc >= 0) {
            select_responder(responding_esc);
        }

        packet_len = crc_packet(unlock_packet);
        uart->write(&unlock_packet.head,packet_len);

        if (++responding_esc >= channels_count) {
            responding_esc = 0;
        }

		/* Min Packet to Packet time is 1 Ms so use 2 */
		hal.scheduler->delay_microseconds(100);
	}
    return 0;
}

void AP_CodevEsc::execute_codev_esc()
{
    if (uart != nullptr) {
        send_esc_outputs();
    }
}

void AP_CodevEsc::receive_esc_status()
{
    ESC_UART_BUF 	_uartbuf = {};
	EscPacket 	_packet = {};

    read_data_from_uart(&_uartbuf);
    if (!parse_tap_esc_feedback(&_uartbuf, &_packet)) {
        if (_packet.msg_id == ESCBUS_MSG_ID_RUN_INFO) {
            RunInfoRepsonse &feed_back_data = _packet.d.rspRunInfo;

                if (feed_back_data.channelID < channels_count) {
                    _esc_status[feed_back_data.channelID].id = feed_back_data.channelID;
                    _esc_status[feed_back_data.channelID].state = feed_back_data.ESCStatus;
                    _esc_status[feed_back_data.channelID].current = feed_back_data.current;
                    _esc_status[feed_back_data.channelID].rpm = feed_back_data.speed;
                    _esc_status[feed_back_data.channelID].esc_set = motor_out[feed_back_data.channelID];
                    _esc_status[feed_back_data.channelID].temperature = feed_back_data.temperature;
                }

        }
    }
}


void AP_CodevEsc::read_data_from_uart(ESC_UART_BUF *const uart_buf)
{
    if (uart == nullptr) {
        return;
    }

    uint8_t tmp_serial_buf[UART_BUFFER_SIZE] = {};
    uint8_t buffer_count = 0;

    // read any available characters
    int16_t nbytes = uart->available();

    int16_t len = nbytes;

    while(nbytes-- > 0 && len < UART_BUFFER_SIZE) {
        uint8_t c = uart->read();
        tmp_serial_buf[buffer_count++] = c;

        if (buffer_count >= UART_BUFFER_SIZE -1) {
            buffer_count = 0;
        }
    }

    if (len > 0 && (uart_buf->dat_cnt + len < UART_BUFFER_SIZE)) {
		for (int i = 0; i < len; i++) {
			uart_buf->esc_feedback_buf[uart_buf->tail++] = tmp_serial_buf[i];
			uart_buf->dat_cnt++;

			if (uart_buf->tail >= UART_BUFFER_SIZE) {
				uart_buf->tail = 0;
			}
		}

	}
}


int AP_CodevEsc::parse_tap_esc_feedback(ESC_UART_BUF *const serial_buf, EscPacket *const packetdata)
{
	static PARSR_ESC_STATE state = HEAD;
	static uint8_t data_index = 0;
	static uint8_t crc_data_cal;

	if (serial_buf->dat_cnt > 0) {
		int count = serial_buf->dat_cnt;

		for (int i = 0; i < count; i++) {
			switch (state) {
			case HEAD:
				if (serial_buf->esc_feedback_buf[serial_buf->head] == PACKET_HEAD) {
					packetdata->head = PACKET_HEAD; //just_keep the format
					state = LEN;
				}

				break;

			case LEN:
				if (serial_buf->esc_feedback_buf[serial_buf->head] < sizeof(packetdata->d)) {
					packetdata->len = serial_buf->esc_feedback_buf[serial_buf->head];
					state = ID;

				} else {
					state = HEAD;
				}

				break;

			case ID:
				if (serial_buf->esc_feedback_buf[serial_buf->head] < ESCBUS_MSG_ID_MAX_NUM) {
					packetdata->msg_id = serial_buf->esc_feedback_buf[serial_buf->head];
					data_index = 0;
					state = DATA;

				} else {
					state = HEAD;
				}

				break;

			case DATA:
				packetdata->d.bytes[data_index++] = serial_buf->esc_feedback_buf[serial_buf->head];

				if (data_index >= packetdata->len) {

					crc_data_cal = crc8_esc((uint8_t *)(&packetdata->len), packetdata->len + 2);
					state = CRC1;
				}

				break;

			case CRC1:
				if (crc_data_cal == serial_buf->esc_feedback_buf[serial_buf->head]) {
					packetdata->crc_data = serial_buf->esc_feedback_buf[serial_buf->head];

					if (++serial_buf->head >= UART_BUFFER_SIZE) {
						serial_buf->head = 0;
					}

					serial_buf->dat_cnt--;
					state = HEAD;
					return 0;
				}

				state = HEAD;
				break;

			default:
				state = HEAD;
				break;

			}

			if (++serial_buf->head >= UART_BUFFER_SIZE) {
				serial_buf->head = 0;
			}

			serial_buf->dat_cnt--;
		}
	}

	return -1;
}

void AP_CodevEsc::send_esc_outputs()
{
    uint16_t rpm[TAP_ESC_MAX_MOTOR_NUM] = {};
    for (uint8_t i = 0;i < channels_count; i++) {

        AP_Motors *motors = AP_Motors::get_singleton();
        if (motors->armed()) {
            motor_out[i]= constrain_int16(motor_out[i],RPMMIN,RPMMAX);
        }

        rpm[i] = motor_out[i];

        if ((rpm[i] & RUN_CHANNEL_VALUE_MASK) > RPMMAX) {
			rpm[i] = (rpm[i] & ~RUN_CHANNEL_VALUE_MASK) | RPMMAX;

		} else if ((rpm[i] & RUN_CHANNEL_VALUE_MASK) < RPMSTOPPED) {
			rpm[i] = (rpm[i] & ~RUN_CHANNEL_VALUE_MASK) | RPMSTOPPED;
		}

        // apply the led color
        if (i < HAL_ESC_NUM) {
            set_led_status(i,control_mode,rpm[i]);
		}
    }

    rpm[_device_mux_map[responding_esc]] |= RUN_FEEDBACK_ENABLE_MASK;

    EscPacket packet = {PACKET_HEAD, channels_count, ESCBUS_MSG_ID_RUN};
	packet.len *= sizeof(packet.d.reqRun.rpm_flags[0]);

	for (uint8_t i = 0; i < channels_count; i++) {
		packet.d.reqRun.rpm_flags[i] = rpm[i];
	}

    if (responding_esc >= 0) {
        select_responder(responding_esc);
    }

    int packet_len = crc_packet(packet);

    uart->write(&packet.head,packet_len);
    if (++responding_esc >= channels_count) {
        responding_esc = 0;
    }
}

void AP_CodevEsc::set_led_status(uint8_t id,uint8_t mode,uint16_t& led_status)
{
    unsigned long _esc_time_now_us = AP_HAL::micros64();
    uint16_t tail_left_led = 0;
    uint16_t tail_right_led = 0;
    bool arm_state = false;
    bool arm_checks_status = false;
    AP_Arming &ap_arm = AP::arming();

    switch ((int8_t)mode)
    {
    case ALT_HOLD:
        tail_left_led = RUN_BLUE_LED_ON_MASK;
        tail_right_led = RUN_BLUE_LED_ON_MASK;
        break;
    case LOITER:
        tail_left_led = RUN_RED_LED_ON_MASK;
        tail_right_led = RUN_GREEN_LED_ON_MASK;
        break;
    case RTL:
        // yellow color
        tail_left_led = RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK;
        tail_right_led = RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK;
        break;

    case AUTO:
    case GUIDED:
    case LAND:
        // purple color
        tail_left_led = RUN_RED_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
        tail_right_led = RUN_RED_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
        break;

    default:
        tail_left_led = RUN_RED_LED_ON_MASK;
        tail_right_led = RUN_RED_LED_ON_MASK;
        break;
    }

    arm_state = ap_arm.is_armed();

    if (!arm_state) {
        arm_checks_status = ap_arm.get_pre_arm_passed();

        if (!arm_checks_status) {
            tail_left_led = RUN_BLUE_LED_ON_MASK;
            tail_right_led = RUN_BLUE_LED_ON_MASK;
        }
    }

    if (_esc_led_on_time_us == 0) {
        _esc_led_on_time_us = AP_HAL::micros64();
        led_on_off = 0;
    }

    // open led if on
    if (led_on_off == 0) {
        switch (id)
        {
        case 0:
            led_status |= RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
            break;
        case 1:
            led_status |= tail_left_led;
            break;
        case 2:
            led_status |= RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
            break;
        case 3:
            led_status |= tail_right_led;
            break;

        default:
            break;
        }

        if (_esc_time_now_us - _esc_led_on_time_us > LED_ON_TIME_MS * 1000) {
            led_on_off = 1;
            _esc_led_on_time_us = _esc_time_now_us;
        }
    } else {

        // Turn off led
        switch (id)
        {
        case 0:
            led_status |= RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
            break;
        case 1:
            led_status |= RUN_RED_LED_OFF_MASK | RUN_GREEN_LED_OFF_MASK | RUN_BLUE_LED_OFF_MASK;
            break;
        case 2:
            led_status |= RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
            break;
        case 3:
            led_status |= RUN_RED_LED_OFF_MASK | RUN_GREEN_LED_OFF_MASK | RUN_BLUE_LED_OFF_MASK;
            break;

        default:
            break;
        }

        if (_esc_time_now_us - _esc_led_on_time_us > LED_OFF_TIME_MS * 1000) {
            led_on_off = 0;
            _esc_led_on_time_us = _esc_time_now_us;
        }
    }
}


void AP_CodevEsc::select_responder(uint8_t channel)
{
    #if defined(HAL_ESC_SELECT0_GPIO_PIN)
		hal.gpio->write(HAL_ESC_SELECT0_GPIO_PIN, channel & 1);
		hal.gpio->write(HAL_ESC_SELECT1_GPIO_PIN, channel & 2);
		hal.gpio->write(HAL_ESC_SELECT2_GPIO_PIN, channel & 4);
    #endif
}


uint8_t AP_CodevEsc::crc_packet(EscPacket &p)
{
	/* Calculate the crc over Len,ID,data */
	p.d.bytes[p.len] = crc8_esc(&p.len, p.len + 2);
	return p.len + offsetof(EscPacket, d) + 1;
}

uint8_t AP_CodevEsc::crc8_esc(uint8_t *p, uint8_t len)
{
	uint8_t crc = 0;

	for (uint8_t i = 0; i < len; i++) {
		crc = crc_table[crc^*p++];
	}

	return crc;
}


void AP_CodevEsc::send_esc_telemetry_mavlink(uint8_t mav_chan)
{
    if (channels_count == 0) {
        return;
    }
    uint16_t voltage[4] {};
    uint16_t current[4] {};
    uint16_t rpm[4] {};
    uint8_t temperature[4] {};
    uint16_t totalcurrent[4] {};
    uint16_t count[4] {};
    AP_BattMonitor &battery = AP::battery();
    float bat_voltage = battery.voltage();

    for (uint8_t i = 0; i < channels_count; i++) {
        uint8_t idx = i % 4;
        temperature[idx]  = _esc_status[i].temperature;
        voltage[idx]      = bat_voltage * 1000;
        current[idx]      = _esc_status[i].current * 10;
        rpm[idx]          = _esc_status[i].rpm;

    }
    if (!HAVE_PAYLOAD_SPACE((mavlink_channel_t)mav_chan, ESC_TELEMETRY_1_TO_4)) {
        return;
    }
    mavlink_msg_esc_telemetry_1_to_4_send((mavlink_channel_t)mav_chan, temperature, voltage, current, totalcurrent, rpm, count);

}


// singleton instance
AP_CodevEsc *AP_CodevEsc::_singleton;
namespace AP {
    AP_CodevEsc *codevesc()
    {
        return AP_CodevEsc::get_singleton();
    }
}
