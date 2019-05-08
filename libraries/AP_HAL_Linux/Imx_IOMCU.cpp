#include "Imx_IOMCU.h"

Imx_IOMCU::Imx_IOMCU()
{
	fd = -1;
	buf_used = 0;
}

Imx_IOMCU::~Imx_IOMCU()
{
	if (fd != -1)
	{
		close(fd);
		fd = -1;
		buf_used = 0;
		poll_fd.fd = -1;
	}
}

int Imx_IOMCU::init(const char *uart_name, unsigned int baud)
{
	fd = open(uart_name, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		printf("Message : %s\n", strerror(errno));
		return fd;
	}
	if (set_uart_baudrate(fd, baud) <= 0)
	{
		printf("Message : %s\n", strerror(errno));
		close(fd);
		fd = -1;
		return -1;
	}
	poll_fd.events = POLLIN;
	poll_fd.fd = fd;
	return fd;
}

bool Imx_IOMCU::set_freq(uint32_t chmask, uint32_t freq[12])
{
	uint32_t packet[13] = { 0 };
	char packet2[128];
	packet[0] = chmask;
	for (int i = 0; i < 12; i++)
	{
		packet[i + 1] = freq[i];
	}

	return uart_send_packet(fd, PACEET_RECV_SET_PWM_FREQ, packet, packet2, 13 * 4);
}

bool Imx_IOMCU::set_duty(uint32_t chmask, uint32_t duty[12])
{
	uint32_t packet[13] = { 0 };
	char packet2[128];
	packet[0] = chmask;
	for (int i = 0; i < 12; i++)
	{
		packet[i + 1] = duty[i];
	}

	return uart_send_packet(fd, PACKET_RECV_SET_PWM_DUTY, packet, packet2, 13 * 4);
}

bool Imx_IOMCU::read_adc(int32_t *adc_pc0_data, int32_t *adc_pc1_data)
{
	int poll_ret = poll(&poll_fd, 1, 0);
	if (poll_ret <= 0)
		return false;

	int read_len = read(fd, &uart_buf[buf_used], 4096 - buf_used);
	if (read_len <= 0)
	{
		return false;
	}
	buf_used += read_len;

	uint8_t packet_id;
	uint8_t packet[4096];
	uint8_t packet_size;
	if (recv_packet(uart_buf, &buf_used, &packet_id, packet, &packet_size) == false)
	{
		return false;
	}
	if (PACKET_SEND_ADC_VALUE != packet_id)
		return false;

	adc_value_s *padc_value = (adc_value_s *)packet;
	*adc_pc0_data = padc_value->adc_pc0_data;
	*adc_pc1_data = padc_value->adc_pc1_data;

	return true;
}

int Imx_IOMCU::set_uart_baudrate(const int _fd, unsigned int baud)
{
	int speed;
	switch (baud) {
	case 9600:   speed = B9600;   break;
	case 19200:  speed = B19200;  break;
	case 38400:  speed = B38400;  break;
	case 57600:  speed = B57600;  break;
	case 115200: speed = B115200; break;
	case 230400: speed = B230400; break;
	default:
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	tcgetattr(_fd, &uart_config);

	uart_config.c_cflag |= (CLOCAL | CREAD);
	uart_config.c_cflag &= ~PARENB;
	uart_config.c_cflag &= ~CSTOPB;
	uart_config.c_cflag &= ~CSIZE;
	uart_config.c_cflag |= CS8;
	uart_config.c_cflag &= ~CRTSCTS;

	uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	uart_config.c_iflag = 0;

	uart_config.c_oflag = 0;

	uart_config.c_cc[VTIME] = 0;
	uart_config.c_cc[VMIN] = 1;

	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		return 0;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		return 0;
	}

	if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
		return 0;
	}

	return 1;
}

bool Imx_IOMCU::uart_send_packet(int _fd, uint8_t packet_id, void *buffer, void *buffer2, int size)
{
	if (size <= 0)
		return false;

	int newbuf_len = size * 2 + 2 + 2 + 2 + 2;
	uint8_t *oldbuf = (uint8_t *)buffer;
	uint8_t *newbuf = (uint8_t *)buffer2;
	newbuf[0] = PACKET_HEADER;
	newbuf[1] = PACKET_HEADER;
	newbuf[2] = packet_id;
	newbuf[3] = 0;
	newbuf[4] = (uint8_t)size;
	newbuf[5] = 0;
	int i;
	uint8_t mask = oldbuf[0];
	for (i = 1; i < size; i++)
	{
		mask = mask ^ oldbuf[i];
	}
	newbuf[6] = mask;
	newbuf[7] = 0;

	for (i = 0; i < size; i++)
	{
		newbuf[i * 2 + 8] = oldbuf[i];
		newbuf[i * 2 + 9] = 0;
	}

	int write_bytes = write(_fd, newbuf, newbuf_len);
	if (write_bytes != newbuf_len)
		return false;

	return true;
}

bool Imx_IOMCU::recv_packet(uint8_t *_uart_buf,
				int *_buf_used,
				uint8_t *packet_id,
				uint8_t *packet,
				uint8_t *packet_size)
{
	int i;
	bool first_hit = false;
	bool found_header = false;
	for (i = 0; i < *_buf_used; i++)
	{
		if (_uart_buf[i] == PACKET_HEADER)
		{
			if (first_hit == false)
			{
				first_hit = true;
				continue;
			}
			else
			{
				found_header = true;
				i--;
				break;
			}
		}
		else
		{
			if (first_hit == true)
				first_hit = false;
		}
	}

	if (found_header)
	{
		memmove(_uart_buf, _uart_buf + i, *_buf_used - i);
		*_buf_used = *_buf_used - i;
	}
	else
	{
		printf("ACM data miss header!\n");
		return false;
	}

	*packet_id = _uart_buf[2];
	*packet_size = _uart_buf[4];
	if (*_buf_used < *packet_size * 2 + 2 + 2 + 2 + 2)
	{
		return false;
	}

	uint8_t mask = _uart_buf[6];
	for (i = 0; i < *packet_size; i++)
	{
		packet[i] = _uart_buf[8 + i * 2];
	}

	uint8_t mask1 = packet[0];
	for (i = 1; i < *packet_size; i++)
	{
		mask1 = mask1 ^ packet[i];
	}

	memmove(_uart_buf, _uart_buf + *packet_size * 2 + 8, *_buf_used - (*packet_size * 2 + 8));
	*_buf_used = *_buf_used - (*packet_size * 2 + 8);

	if (mask == mask1)
		return true;

	printf("xor check failed!\n");
	return false;
}
