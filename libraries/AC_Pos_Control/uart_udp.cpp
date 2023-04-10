#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <mavlink/common/mavlink.h>

#define BUFFER_SIZE 1024

using namespace std;

int setup_serial_port(const char* device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        cerr << "Failed to open " << device << endl;
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        cerr << "Failed to get attributes for " << device << endl;
        return -1;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        cerr << "Failed to set attributes for " << device << endl;
        return -1;
    }

    return fd;
}

int setup_udp_socket(const char* address, int port) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        cerr << "Failed to create UDP socket" << endl;
        return -1;
    }

    int optval = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval , sizeof(int));

    struct sockaddr_in serveraddr;
    memset(&serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_addr.s_addr = inet_addr(address);
    serveraddr.sin_port = htons(port);

    if (bind(sockfd, (struct sockaddr*)&serveraddr, sizeof(serveraddr)) < 0) {
        cerr << "Failed to bind UDP socket to " << address << ":" << port << endl;
        return -1;
    }

    return sockfd;
}

int main(int argc, char** argv) {
    if (argc != 4) {
        cerr << "Usage: " << argv[0] << " <serial port> <udp address> <udp port>" << endl;
        return -1;
    }

    const char* serial_port = argv[1];
    const char* udp_address = argv[2];
    int udp_port = atoi(argv[3]);

    SSL_library_init();
    SSL_CTX *ctx = SSL_CTX_new(TLSv1_2_client_method());
    if (ctx == NULL) {
        cerr << "Failed to create SSL context" << endl;
        return -1;
    }


    SSL *ssl = SSL_new(ctx);
    if (ssl == NULL) {
        cerr << "Failed to create SSL object" << endl;
    return -1;
}

int serial_fd = setup_serial_port(serial_port, B57600);
if (serial_fd < 0) {
    cerr << "Failed to setup serial port" << endl;
    return -1;
}

int udp_fd = setup_udp_socket(udp_address, udp_port);
if (udp_fd < 0) {
    cerr << "Failed to setup UDP socket" << endl;
    return -1;
}

SSL_set_fd(ssl, udp_fd);
if (SSL_connect(ssl) < 0) {
    cerr << "Failed to connect to UDP server" << endl;
    return -1;
}

mavlink_message_t msg;
mavlink_status_t status;
uint8_t buffer[BUFFER_SIZE];

while (true) {
    ssize_t n = read(serial_fd, buffer, BUFFER_SIZE);
    if (n < 0) {
        cerr << "Failed to read from serial port" << endl;
        continue;
    }

    for (int i = 0; i < n; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
            n = mavlink_msg_to_send_buffer(buffer, &msg);
            SSL_write(ssl, buffer, n);
        }
    }
}

SSL_shutdown(ssl);
SSL_free(ssl);
SSL_CTX_free(ctx);
close(serial_fd);
close(udp_fd);

return 0;
}
