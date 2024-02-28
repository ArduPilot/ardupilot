/*
  UDP proxy code for connecting two UDP endpoints
  Released under GNU GPLv3
  Author: Andrew Tridgell
 */
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <netinet/in.h>

static bool verbose;
static int listen_port1, listen_port2;

static double timestamp()
{
    struct timeval tval;
    gettimeofday(&tval,NULL);
    return tval.tv_sec + (tval.tv_usec*1.0e-6);
}

/*
  open a socket of the specified type, port and address for incoming data
*/
int open_socket_in(int port)
{
	struct sockaddr_in sock;
	int res;
	int one=1;

	memset(&sock,0,sizeof(sock));

#ifdef HAVE_SOCK_SIN_LEN
	sock.sin_len = sizeof(sock);
#endif
	sock.sin_port = htons(port);
	sock.sin_family = AF_INET;

	res = socket(AF_INET, SOCK_DGRAM, 0);
	if (res == -1) { 
		fprintf(stderr, "socket failed\n"); return -1; 
		return -1;
	}

	setsockopt(res,SOL_SOCKET,SO_REUSEADDR,(char *)&one,sizeof(one));

	if (bind(res, (struct sockaddr *)&sock, sizeof(sock)) < 0) { 
		return(-1); 
	}

	return res;
}

static void main_loop(int sock1, int sock2)
{
	unsigned char buf[10240];
        bool have_conn1=false;
        bool have_conn2=false;
        double last_pkt1=0;
        double last_pkt2=0;
        int fdmax = (sock1>sock2?sock1:sock2)+1;
        double last_stats = timestamp();
        uint32_t bytes_in1=0;
        uint32_t bytes_in2=0;
        
	while (1) {
            fd_set fds;
            int ret;
            struct timeval tval;
            double now = timestamp();

            if (verbose && now - last_stats > 1) {
                double dt = now - last_stats;
                printf("%u: %u bytes/sec  %u: %u bytes/sec\n",
                       (unsigned)listen_port1, (unsigned)(bytes_in1/dt),
                       (unsigned)listen_port2, (unsigned)(bytes_in2/dt));
                bytes_in1 = bytes_in2 = 0;
                last_stats = now;
            }
            
            if (have_conn1 && now - last_pkt1 > 10) {
                break;
            }
            if (have_conn2 && now - last_pkt2 > 10) {
                break;
            }
            
            FD_ZERO(&fds);
            FD_SET(sock1, &fds);
            FD_SET(sock2, &fds);

            tval.tv_sec = 10;
            tval.tv_usec = 0;

            ret = select(fdmax, &fds, NULL, NULL, &tval);
            if (ret == -1 && errno == EINTR) continue;
            if (ret <= 0) break;

            now = timestamp();
                
            if (FD_ISSET(sock1, &fds)) {
                struct sockaddr_in from;
                socklen_t fromlen = sizeof(from);
                int n = recvfrom(sock1, buf, sizeof(buf), 0, 
                                 (struct sockaddr *)&from, &fromlen);
                if (n <= 0) break;

                bytes_in1 += n;
                
                last_pkt1 = now;
                if (!have_conn1) {
                    if (connect(sock1, (struct sockaddr *)&from, fromlen) != 0) {
                        break;
                    }
                    have_conn1 = true;
                    printf("have conn1\n");
                }
                if (have_conn2) {
                    if (send(sock2, buf, n, 0) != n) {
                        break;
                    }
                }
            }

            if (FD_ISSET(sock2, &fds)) {
                struct sockaddr_in from;
                socklen_t fromlen = sizeof(from);
                int n = recvfrom(sock2, buf, sizeof(buf), 0, 
                                 (struct sockaddr *)&from, &fromlen);
                if (n <= 0) break;

                bytes_in2 += n;

                last_pkt2 = now;
                if (!have_conn2) {
                    if (connect(sock2, (struct sockaddr *)&from, fromlen) != 0) {
                        break;
                    }
                    have_conn2 = true;
                    printf("have conn2\n");
                }
                if (have_conn1) {
                    if (send(sock1, buf, n, 0) != n) {
                        break;
                    }
                }
            }
	}
}

int main(int argc, char *argv[])
{
    int sock_in1, sock_in2;

    if (argc < 3) {
        printf("Usage: udpproxy <port1> <port2>\n");
        exit(1);
    }
    if (argc > 3) {
        verbose = strcmp(argv[3],"-v") == 0;
        printf("verbose=%u\n", (unsigned)verbose);
    }
    
    while (true) {
        listen_port1 = atoi(argv[1]);
        listen_port2 = atoi(argv[2]);

        printf("Opening sockets %u %u\n", listen_port1, listen_port2);
        sock_in1 = open_socket_in(listen_port1);
        sock_in2 = open_socket_in(listen_port2);
        if (sock_in1 == -1 || sock_in2 == -1) {
            fprintf(stderr,"sock on ports %d or %d failed - %s\n", 
                    listen_port1, listen_port2, strerror(errno));
            exit(1);
        }
        
        main_loop(sock_in1, sock_in2);
        close(sock_in1);
        close(sock_in2);
    }

    return 0;
}
