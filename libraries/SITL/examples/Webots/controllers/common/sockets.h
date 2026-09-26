/*
  socket helpers shared by the ArduPilot Webots controllers
*/

#ifndef ARDUPILOT_SITL_SOCKETS_H
#define ARDUPILOT_SITL_SOCKETS_H

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>


#ifdef _WIN32
#include <winsock.h>
#else
#include <arpa/inet.h> /* definition of inet_ntoa */
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>      /* definition of gethostbyname */
#include <netinet/in.h> /* definition of struct sockaddr_in */
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h> /* definition of close */
#endif

int create_socket_server(int port);
bool socket_cleanup(void);
int socket_accept(int server_fd);
bool socket_close(int fd);
ssize_t socket_send(int fd, const void *buf, size_t len);
bool socket_send_all(int fd, const void *buf, size_t len);

#endif  /* ARDUPILOT_SITL_SOCKETS_H */
