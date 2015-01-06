// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: -*- nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "UARTDriver.h"

#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <assert.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <string.h>
#include <arpa/inet.h>
#include "../AP_HAL/utility/RingBuffer.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

LinuxUARTDriver::LinuxUARTDriver(bool default_console) :
    device_path(NULL),
    _rd_fd(-1),
    _wr_fd(-1),
    _packetise(false),
    _flow_control(FLOW_CONTROL_DISABLE)
{
    if (default_console) {
        _rd_fd = 0;
        _wr_fd = 1;
        _console = true;
    }
}

/*
  set the tty device to use for this UART
 */
void LinuxUARTDriver::set_device_path(char *path)
{
    device_path = path;
}

/*
  open the tty
 */
void LinuxUARTDriver::begin(uint32_t b) 
{
    begin(b, 0, 0);
}

void LinuxUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) 
{
    if (device_path == NULL && _console) {
        _rd_fd = 0;
        _wr_fd = 1;
        fcntl(_rd_fd, F_SETFL, fcntl(_rd_fd, F_GETFL, 0) | O_NONBLOCK);
        fcntl(_wr_fd, F_SETFL, fcntl(_wr_fd, F_GETFL, 0) | O_NONBLOCK);
    } else if (!_initialised) {
        if (device_path == NULL) {
            return;
        }
        
        switch (_parseDevicePath(device_path)){        
        case DEVICE_TCP:
        {
            _connected = false;
            if (_flag != NULL){
                if (!strcmp(_flag, "wait")){    
                    _tcp_start_connection(true);    
                } else {
                    _tcp_start_connection(false);    
                }
            } else {
                _tcp_start_connection(false);    
            }
            
            if (!_connected) {
                ::printf("LinuxUARTDriver TCP connection not stablished\n");
                exit(1);
            }
            _flow_control = FLOW_CONTROL_ENABLE;
            break;
        }   

        case DEVICE_UDP:
        {
            _udp_start_connection();
            _flow_control = FLOW_CONTROL_ENABLE;
            break;
        }   

        case DEVICE_SERIAL:            
        {
            _rd_fd = open(device_path, O_RDWR);
            _wr_fd = _rd_fd;
            if (_rd_fd == -1) {
                ::fprintf(stdout, "Failed to open UART device %s - %s\n",
                          device_path, strerror(errno));
                return;
            }
            
            // always run the file descriptor non-blocking, and deal with
            // blocking IO in the higher level calls
            fcntl(_rd_fd, F_SETFL, fcntl(_rd_fd, F_GETFL, 0) | O_NONBLOCK);

            // TODO: add proper flow control support
            _flow_control = FLOW_CONTROL_DISABLE;
            break;
        }
        default:
        {
            // Notify that the option is not valid and select standart input and output
            ::printf("LinuxUARTDriver parsing failed, using default\n");

            _rd_fd = 0;
            _wr_fd = 1;
            fcntl(_rd_fd, F_SETFL, fcntl(_rd_fd, F_GETFL, 0) | O_NONBLOCK);
            fcntl(_wr_fd, F_SETFL, fcntl(_wr_fd, F_GETFL, 0) | O_NONBLOCK);
            break;
        }
        }
    }

    // we have enough memory to have a larger transmit buffer for
    // all ports. This means we don't get delays while waiting to
    // write GPS config packets
    if (rxS < 1024) {
        rxS = 8192;
    }            
    if (txS < 8192) {
        txS = 8192;
    }
    
    _initialised = false;
    while (_in_timer) hal.scheduler->delay(1);

    if (b != 0 && _rd_fd == _wr_fd) {
        // set the baud rate
        struct termios t;
        memset(&t, 0, sizeof(t));
        tcgetattr(_rd_fd, &t);
        cfsetspeed(&t, b);
        // disable LF -> CR/LF
        t.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL | IXON | IXOFF);
        t.c_oflag &= ~(OPOST | ONLCR);
        t.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE);
        t.c_cc[VMIN] = 0;
        tcsetattr(_rd_fd, TCSANOW, &t);
    }

    /*
      allocate the read buffer
    */
    if (rxS != 0 && rxS != _readbuf_size) {
        _readbuf_size = rxS;
        if (_readbuf != NULL) {
            free(_readbuf);
        }
        _readbuf = (uint8_t *)malloc(_readbuf_size);
        _readbuf_head = 0;
        _readbuf_tail = 0;
    }

    /*
      allocate the write buffer
    */
    if (txS != 0 && txS != _writebuf_size) {
        _writebuf_size = txS;
        if (_writebuf != NULL) {
            free(_writebuf);
        }
        _writebuf = (uint8_t *)malloc(_writebuf_size);
        _writebuf_head = 0;
        _writebuf_tail = 0;
    }

    if (_writebuf_size != 0 && _readbuf_size != 0) {
        _initialised = true;
    }
}

/*
    Device path accepts the following syntaxes:
        - /dev/ttyO1
        - tcp:*:1243:wait
        - udp:192.168.2.15:1243
*/
LinuxUARTDriver::device_type LinuxUARTDriver::_parseDevicePath(const char *arg)
{
    struct stat st;
    _flag = NULL; // init flag


    char *devstr = strdup(arg);
    if (devstr == NULL) {
        return DEVICE_UNKNOWN;
    }

    if (stat(devstr, &st) == 0 && S_ISCHR(st.st_mode)) {
        free(devstr);
        return DEVICE_SERIAL;
    } else if (strncmp(devstr, "tcp:", 4) == 0 ||
               strncmp(devstr, "udp:", 4) == 0) {
        char *saveptr = NULL;
        // Parse the string        
        char *protocol, *ip, *port, *flag;
        protocol = strtok_r(devstr, ":", &saveptr);
        ip = strtok_r(NULL, ":", &saveptr);
        port = strtok_r(NULL, ":", &saveptr);
        flag = strtok_r(NULL, ":", &saveptr);        
        
        _base_port = (uint16_t) atoi(port);
        if (_ip) free(_ip);
        _ip = NULL;
        if (ip) {
            _ip = strdup(ip);        
        }
        if (_flag) free(_flag);
        _flag = NULL;
        if (flag) {
            _flag = strdup(flag);
        }
        if (strcmp(protocol, "udp") == 0) {
            free(devstr);
            return DEVICE_UDP;
        }
        free(devstr);
        return DEVICE_TCP;
    }
    free(devstr);
    return DEVICE_UNKNOWN;
}

/*
  start a TCP connection for the serial port. If wait_for_connection
  is true then block until a client connects
 */
void LinuxUARTDriver::_tcp_start_connection(bool wait_for_connection)
{
    int one=1;
    struct sockaddr_in sockaddr;
    int ret;    
    int listen_fd = -1;  // socket we are listening on    
    int net_fd = -1; // network file descriptor, will be linked to wr_fd and rd_fd
    uint8_t portNumber = 0; // connecto to _base_port + portNumber

    if (net_fd != -1) {
        close(net_fd);
    }

    if (listen_fd == -1) {
        memset(&sockaddr,0,sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
        sockaddr.sin_len = sizeof(sockaddr);
#endif
        sockaddr.sin_port = htons(_base_port + portNumber);
        sockaddr.sin_family = AF_INET;
        if (strcmp(_ip, "*") == 0) {
            // Bind to all interfaces
            sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
        } else {
            sockaddr.sin_addr.s_addr = inet_addr(_ip);
        }

        listen_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd == -1) {
            ::printf("socket failed - %s\n", strerror(errno));
            exit(1);
        }

        /* we want to be able to re-use ports quickly */
        setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        ::printf("bind port %u for %u\n", 
                 (unsigned)ntohs(sockaddr.sin_port),
                 (unsigned)portNumber);

        ret = bind(listen_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
        if (ret == -1) {
            ::printf("bind failed on port %u - %s\n",
                     (unsigned)ntohs(sockaddr.sin_port),
                     strerror(errno));
            exit(1);
        }

        ret = listen(listen_fd, 5);
        if (ret == -1) {
            ::printf("listen failed - %s\n", strerror(errno));
            exit(1);
        }

        ::printf("Serial port %u on TCP port %u\n", portNumber, 
                 _base_port + portNumber);
        fflush(stdout);
    }

    if (wait_for_connection) {
        ::printf("Waiting for connection ....\n");
        ::fflush(stdout);
        net_fd = accept(listen_fd, NULL, NULL);
        if (net_fd == -1) {
            ::printf("accept() error - %s", strerror(errno));
            exit(1);
        }
        setsockopt(net_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        setsockopt(net_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        // always run the file descriptor non-blocking, and deal with                                         |
        // blocking IO in the higher level calls
        fcntl(net_fd, F_SETFL, fcntl(net_fd, F_GETFL, 0) | O_NONBLOCK);

        _connected = true;
        _rd_fd = net_fd;
        _wr_fd = net_fd;
    }
}


/*
  start a UDP connection for the serial port
 */
void LinuxUARTDriver::_udp_start_connection(void)
{
    struct sockaddr_in sockaddr;
    int ret;    
    
    memset(&sockaddr,0,sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
    sockaddr.sin_len = sizeof(sockaddr);
#endif
    sockaddr.sin_port = htons(_base_port);
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(_ip);

    _rd_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (_rd_fd == -1) {
        ::printf("socket failed - %s\n", strerror(errno));
        exit(1);
    }
        
    ret = connect(_rd_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (ret == -1) {
        ::printf("connect failed to %s:%u - %s\n",
                 _ip, (unsigned)_base_port,
                 strerror(errno));
        exit(1);
    }

    // always run the file descriptor non-blocking, and deal with                                         |
    // blocking IO in the higher level calls
    fcntl(_rd_fd, F_SETFL, fcntl(_rd_fd, F_GETFL, 0) | O_NONBLOCK);
    _wr_fd = _rd_fd;

    // try to write on MAVLink packet boundaries if possible
    _packetise = true;
}

/*
  shutdown a UART
 */
void LinuxUARTDriver::end() 
{
    _initialised = false;
    _connected = false;
    while (_in_timer) hal.scheduler->delay(1);
    if (_rd_fd == _wr_fd && _rd_fd != -1) {
        close(_rd_fd);
    }
    _rd_fd = -1;
    _wr_fd = -1;
    if (_readbuf) {
        free(_readbuf);
        _readbuf = NULL;
    }
    if (_writebuf) {
        free(_writebuf);
        _writebuf = NULL;
    }
    _readbuf_size = _writebuf_size = 0;
    _writebuf_head = 0;
    _writebuf_tail = 0;
    _readbuf_head = 0;
    _readbuf_tail = 0;
}


void LinuxUARTDriver::flush() 
{
    // we are not doing any buffering, so flush is a no-op
}


/*
  return true if the UART is initialised
 */
bool LinuxUARTDriver::is_initialized() 
{
    return _initialised;
}


/*
  enable or disable blocking writes
 */
void LinuxUARTDriver::set_blocking_writes(bool blocking) 
{
    _nonblocking_writes = !blocking;
}


/*
  do we have any bytes pending transmission?
 */
bool LinuxUARTDriver::tx_pending() 
{ 
    return !BUF_EMPTY(_writebuf);
}

/*
  return the number of bytes available to be read
 */
int16_t LinuxUARTDriver::available() 
{ 
    if (!_initialised) {
        return 0;
    }
    uint16_t _tail;
    return BUF_AVAILABLE(_readbuf);
}

/*
  how many bytes are available in the output buffer?
 */
int16_t LinuxUARTDriver::txspace() 
{ 
    if (!_initialised) {
        return 0;
    }
    uint16_t _head;
    return BUF_SPACE(_writebuf);
}

int16_t LinuxUARTDriver::read() 
{ 
    uint8_t c;
    if (!_initialised || _readbuf == NULL) {
        return -1;
    }
    if (BUF_EMPTY(_readbuf)) {
        return -1;
    }
    c = _readbuf[_readbuf_head];
    BUF_ADVANCEHEAD(_readbuf, 1);
    return c;
}

/* Linux implementations of Print virtual methods */
size_t LinuxUARTDriver::write(uint8_t c) 
{ 
    if (!_initialised) {
        return 0;
    }
    uint16_t _head;

    while (BUF_SPACE(_writebuf) == 0) {
        if (_nonblocking_writes) {
            return 0;
        }
        hal.scheduler->delay(1);
    }
    _writebuf[_writebuf_tail] = c;
    BUF_ADVANCETAIL(_writebuf, 1);
    return 1;
}

/*
  write size bytes to the write buffer
 */
size_t LinuxUARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_initialised) {
        return 0;
    }
    if (!_nonblocking_writes) {
        /*
          use the per-byte delay loop in write() above for blocking writes
         */
        size_t ret = 0;
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
    }

    uint16_t _head, space;
    space = BUF_SPACE(_writebuf);
    if (space == 0) {
        return 0;
    }
    if (size > space) {
        size = space;
    }
    if (_writebuf_tail < _head) {
        // perform as single memcpy
        assert(_writebuf_tail+size <= _writebuf_size);
        memcpy(&_writebuf[_writebuf_tail], buffer, size);
        BUF_ADVANCETAIL(_writebuf, size);
        return size;
    }

    // perform as two memcpy calls
    uint16_t n = _writebuf_size - _writebuf_tail;
    if (n > size) n = size;
    assert(_writebuf_tail+n <= _writebuf_size);
    memcpy(&_writebuf[_writebuf_tail], buffer, n);
    BUF_ADVANCETAIL(_writebuf, n);
    buffer += n;
    n = size - n;
    if (n > 0) {
        assert(_writebuf_tail+n <= _writebuf_size);
        memcpy(&_writebuf[_writebuf_tail], buffer, n);
        BUF_ADVANCETAIL(_writebuf, n);
    }        
    return size;
}

/*
  try writing n bytes, handling an unresponsive port
 */
int LinuxUARTDriver::_write_fd(const uint8_t *buf, uint16_t n)
{
    int ret = 0;

    struct pollfd fds;
    fds.fd = _wr_fd;
    fds.events = POLLOUT;
    fds.revents = 0;

    if (poll(&fds, 1, 0) == 1) {
        ret = ::write(_wr_fd, buf, n);
    }

    if (ret > 0) {
        BUF_ADVANCEHEAD(_writebuf, ret);
        return ret;
    }

    return ret;
}

/*
  try reading n bytes, handling an unresponsive port
 */
int LinuxUARTDriver::_read_fd(uint8_t *buf, uint16_t n)
{
    int ret;
    ret = ::read(_rd_fd, buf, n);
    if (ret > 0) {
        BUF_ADVANCETAIL(_readbuf, ret);
    }
    return ret;
}


/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread. Doing it this way reduces the system call
  overhead in the main task enormously. 
 */
void LinuxUARTDriver::_timer_tick(void)
{
    uint16_t n;

    if (!_initialised) return;

    _in_timer = true;

    // write any pending bytes
    uint16_t _tail;
    n = BUF_AVAILABLE(_writebuf);
    if (_packetise && n > 0 && _writebuf[_writebuf_head] == 254) {
        // this looks like a MAVLink packet - try to write on
        // packet boundaries when possible
        if (n < 8) {
            n = 0;
        } else {
            // the length of the packet is the 2nd byte, and mavlink
            // packets have a 6 byte header plus 2 byte checksum,
            // giving len+8 bytes
            uint16_t ofs = (_writebuf_head + 1) % _writebuf_size;
            uint8_t len = _writebuf[ofs];
            if (n < len+8) {
                // we don't have a full packet yet
                n = 0;
            } else if (n > len+8) {
                // send just 1 packet at a time (so MAVLink packets
                // are aligned on UDP boundaries)
                n = len+8;
            }
        }        
    }

    if (n > 0) {
        uint16_t n1 = _writebuf_size - _writebuf_head;
        if (n1 >= n) {
            // do as a single write
            _write_fd(&_writebuf[_writebuf_head], n);
        } else {
            // split into two writes
            if (_packetise) {
                // keep as a single UDP packet
                uint8_t tmpbuf[n];
                memcpy(tmpbuf, &_writebuf[_writebuf_head], n1);
                if (n > n1) {
                    memcpy(&tmpbuf[n1], &_writebuf[0], n-n1);
                }
                _write_fd(tmpbuf, n);
            } else {
                int ret = _write_fd(&_writebuf[_writebuf_head], n1);
                if (ret == n1 && n > n1) {
                    _write_fd(&_writebuf[_writebuf_head], n - n1);                
                }
            }
        }
    }

    // try to fill the read buffer
    uint16_t _head;
    n = BUF_SPACE(_readbuf);
    if (n > 0) {
        uint16_t n1 = _readbuf_size - _readbuf_tail;
        if (n1 >= n) {
            // one read will do
            assert(_readbuf_tail+n <= _readbuf_size);
            _read_fd(&_readbuf[_readbuf_tail], n);
        } else {
            assert(_readbuf_tail+n1 <= _readbuf_size);
            int ret = _read_fd(&_readbuf[_readbuf_tail], n1);
            if (ret == n1 && n > n1) {
                assert(_readbuf_tail+(n-n1) <= _readbuf_size);
                _read_fd(&_readbuf[_readbuf_tail], n - n1);                
            }
        }
    }

    _in_timer = false;
}

#endif // CONFIG_HAL_BOARD
