/*
 * File: ardupilot_SITL_ROV.c
 * Date: 03 Aug 2019
 * Description: integration with ardupilot SITL simulation.
 * Author: M.S.Hefny (HefnySco)
 * Modifications:
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <webots/supervisor.h>
#include "sockets.h"
#include "sensors.h"

bool socket_init() {
#ifdef _WIN32 /* initialize the socket API */
  WSADATA info;
  if (WSAStartup(MAKEWORD(1, 1), &info) != 0) {
    fprintf(stderr, "Cannot initialize Winsock.\n");
    return false;
  }
#endif
  return true;
}

bool socket_set_non_blocking(int fd) {
  if (fd < 0)
    return false;
#ifdef _WIN32
  unsigned long mode = 1;
  return (ioctlsocket(fd, FIONBIO, &mode) == 0) ? true : false;
#else
  int flags = fcntl(fd, F_GETFL, 0) | O_NONBLOCK;
  return (fcntl(fd, F_SETFL, flags) == 0) ? true : false;
#endif
}

int socket_accept(int server_fd) {
  int cfd;
  struct sockaddr_in client;
  struct hostent *client_info;
#ifndef _WIN32
  socklen_t asize;
#else
  int asize;
#endif
  asize = sizeof(struct sockaddr_in);
  cfd = accept(server_fd, (struct sockaddr *)&client, &asize);
  if (cfd == -1) {
#ifdef _WIN32
    int e = WSAGetLastError();
    if (e == WSAEWOULDBLOCK)
      return 0;
    fprintf(stderr, "Accept error: %d.\n", e);
#else
    if (errno == EWOULDBLOCK)
      return 0;
    fprintf(stderr, "Accept error: %d.\n", errno);
#endif
    return -1;
  }
  client_info = gethostbyname((char *)inet_ntoa(client.sin_addr));
  printf("Accepted connection from: %s.\n", client_info->h_name);
  return cfd;
}

bool socket_close(int fd) {
#ifdef _WIN32
  return (closesocket(fd) == 0) ? true : false;
#else
  return (close(fd) == 0) ? true : false;
#endif
}

bool socket_cleanup() {
#ifdef _WIN32
  return (WSACleanup() == 0) ? true : false;
#else
  return true;
#endif
}




/*
  Creates a socket and bind it to port.
 */
int create_socket_server(int port) {
  int sfd, rc;
  struct sockaddr_in address;
  if (!socket_init())
  {
    fprintf (stderr, "socket_init failed");
    return -1;
  }
  sfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sfd == -1) {
    fprintf(stderr, "Cannot create socket.\n");
    return -1;
  }
  int one = 1;
  setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons((unsigned short)port);
  address.sin_addr.s_addr = INADDR_ANY;
  rc = bind(sfd, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1) {
    fprintf(stderr, "Cannot bind port %d.\n", port);
    socket_close(sfd);
    return -1;
  }
  if (listen(sfd, 1) == -1) {
    fprintf(stderr, "Cannot listen for connections.\n");
    socket_close(sfd);
    return -1;
  }
  
  printf ("socket initialized at port %d.\n", port);
  return sfd;
}