#pragma once

#include <termios.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <poll.h>

#define PACKET_HEADER 0x11

#define PACKET_SEND_ADC_VALUE 1
#define PACEET_RECV_SET_PWM_FREQ 101
#define PACKET_RECV_SET_PWM_DUTY 102

struct adc_value_s
{
	int32_t adc_pc0_data;
	int32_t adc_pc1_data;
}__attribute__((packed));

class Imx_IOMCU
{
public:
	int fd;

	Imx_IOMCU();
	virtual ~Imx_IOMCU();
	int init(const char *uart_name, unsigned int baud = 115200);
	bool set_freq(uint32_t chmask, uint32_t freq[12]);
	bool set_duty(uint32_t chmask, uint32_t duty[12]);
	bool read_adc(int32_t *adc_pc0_data, int32_t *adc_pc1_data);
private:
	uint8_t uart_buf[4096];
	int buf_used;
	struct pollfd poll_fd;

	int set_uart_baudrate(const int _fd, unsigned int baud);
	bool uart_send_packet(int _fd, uint8_t packet_id, void *buffer, void *buffer2, int size);

	bool recv_packet(uint8_t *_uart_buf,
				int *_buf_used,
				uint8_t *packet_id,
				uint8_t *packet,
				uint8_t *packet_size);
};
