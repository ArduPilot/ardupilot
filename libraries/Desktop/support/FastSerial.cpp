// -*-  tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// Interrupt-driven serial transmit/receive library.
//
//      Copyright (c) 2010 Michael Smith. All rights reserved.
//
// Receive and baudrate calculations derived from the Arduino
// HardwareSerial driver:
//
//      Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
//
// Transmit algorithm inspired by work:
//
//      Code Jose Julio and Jordi Munoz. DIYDrones.com
//
//      This library is free software; you can redistribute it and/or
//      modify it under the terms of the GNU Lesser General Public
//      License as published by the Free Software Foundation; either
//      version 2.1 of the License, or (at your option) any later version.
//
//      This library is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//      Lesser General Public License for more details.
//
//      You should have received a copy of the GNU Lesser General Public
//      License along with this library; if not, write to the Free Software
//      Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//


//#include "../AP_Common/AP_Common.h"
#include "FastSerial.h"
#include "WProgram.h"
#include <unistd.h>
#include <pty.h>
#include <fcntl.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>



    int       list_s;                /*  listening socket          */
    int       conn_s;                /*  connection socket         */
    short int port = 5760;                  /*  port number               */
    struct    sockaddr_in servaddr, cli_addr;  /*  socket address structure  */
    char      buffer[256];      /*  character buffer          */
    socklen_t clilen;

    
int myread(void)
{
    return read(conn_s,buffer,1);
}

int mywrite(uint8_t c)
{
    return write(conn_s,(char *) &c,1);
}


#if   defined(UDR3)
# define FS_MAX_PORTS   4
#elif defined(UDR2)
# define FS_MAX_PORTS   3
#elif defined(UDR1)
# define FS_MAX_PORTS   2
#else
# define FS_MAX_PORTS   1
#endif

FastSerial::Buffer __FastSerial__rxBuffer[FS_MAX_PORTS];
FastSerial::Buffer __FastSerial__txBuffer[FS_MAX_PORTS];

// Constructor /////////////////////////////////////////////////////////////////

FastSerial::FastSerial(const uint8_t portNumber, volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
					   volatile uint8_t *ucsra, volatile uint8_t *ucsrb, const uint8_t u2x,
					   const uint8_t portEnableBits, const uint8_t portTxBits) :
					   _ubrrh(ubrrh),
					   _ubrrl(ubrrl),
					   _ucsra(ucsra),
					   _ucsrb(ucsrb),
					   _u2x(portNumber),
					   _portEnableBits(portEnableBits),
					   _portTxBits(portTxBits),
					   _rxBuffer(&__FastSerial__rxBuffer[portNumber]),
					   _txBuffer(&__FastSerial__txBuffer[portNumber])
{
}

// Public Methods //////////////////////////////////////////////////////////////

void FastSerial::begin(long baud)
{
	if (_u2x == 0) {
		unsigned v;
		v = fcntl(0, F_GETFL, 0);
		fcntl(0, F_SETFL, v | O_NONBLOCK);
		v = fcntl(1, F_GETFL, 0);
		fcntl(1, F_SETFL, v | O_NONBLOCK);
    
        /*  Create the listening socket  */

        if ( (list_s = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
        fprintf(stderr, "ECHOSERV: Error creating listening socket.\n");
        exit(EXIT_FAILURE);
        }

        int val = 1;
        
        setsockopt(list_s, SOL_SOCKET, SO_REUSEADDR, &val, sizeof val);

        /*  Set all bytes in socket address structure to
            zero, and fill in the relevant data members   */

        memset(&servaddr, 0, sizeof(servaddr));
       
        servaddr.sin_family      = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port        = htons(port + _u2x);


        /*  Bind our socket addresss to the 
        listening socket, and call listen()  */

        if ( bind(list_s, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0 ) {
        fprintf(stderr, "ECHOSERV: Error calling bind()\n");
        exit(EXIT_FAILURE);
        }

        if ( listen(list_s, 5) < 0 ) {
        fprintf(stderr, "ECHOSERV: Error calling listen()\n");
        exit(EXIT_FAILURE);
        }
        clilen = sizeof(cli_addr);
        
        fprintf(stdout, "Listerning on port %i\n",port + _u2x);
        fflush(stdout);

        if ( (conn_s = accept(list_s, NULL, NULL) ) < 0 ) {
            fprintf(stderr, "ECHOSERV: Error calling accept()\n");
            exit(EXIT_FAILURE);
        }
        
        fcntl(conn_s, F_SETFL, O_NONBLOCK);
    }

}

void FastSerial::begin(long baud, unsigned int rxSpace, unsigned int txSpace)
{
	begin(baud);
}

void FastSerial::end()
{
}

int FastSerial::available(void)
{
	if (_u2x != 0) return 0;
	int num_ready;

 fd_set socks;
 struct timeval t;
 FD_ZERO(&socks);
 FD_SET(conn_s, &socks);
 t.tv_sec = 0;
 t.tv_usec = 500;
 if (int ans = select(conn_s + 1, &socks, NULL, NULL,&t))
 {

  //FD_ZERO(&socks);
 //FD_SET(conn_s, &socks);
 //if (FD_ISSET(conn_s, &socks)) {
        return 1;
//    }
 }
	return 0;
}

int FastSerial::txspace(void)
{
	return 128;
}

int FastSerial::read(void)
{
	if (_u2x != 0) {
		return -1;
	}
	char c;
    int n = myread();
    if (n == 0) {
    
        if ( (conn_s = accept(list_s, NULL, NULL) ) < 0 ) {
            fprintf(stderr, "ECHOSERV: Error calling accept()\n");
            exit(EXIT_FAILURE);
        }
        
        fcntl(conn_s, F_SETFL, O_NONBLOCK);
    }
	return (int)buffer[0];
}

int FastSerial::peek(void)
{
	return -1;
}

void FastSerial::flush(void)
{
}

void FastSerial::write(uint8_t c)
{
	if (_u2x != 0) {
		return;
	}
    mywrite(c);
    if (c >= '\n' && c <= 128) {
        fwrite(&c, 1, 1, stdout);
        fflush(stdout);
    }
}

// Buffer management ///////////////////////////////////////////////////////////

bool FastSerial::_allocBuffer(Buffer *buffer, unsigned int size)
{
	return false;
}

void FastSerial::_freeBuffer(Buffer *buffer)
{
}

