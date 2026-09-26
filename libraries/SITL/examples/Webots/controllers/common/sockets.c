
#include "sockets.h"


static bool socket_init(void) {
#ifdef _WIN32 /* initialize the socket API */
  WSADATA info;
  if (WSAStartup(MAKEWORD(1, 1), &info) != 0) {
    fprintf(stderr, "Cannot initialize Winsock.\n");
    return false;
  }
#endif
  return true;
}

int socket_accept(int server_fd) {
  int cfd;
  struct sockaddr_in client;
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
  /* no reverse lookup: gethostbyname() can return NULL, and did not need to
     be called just to print an address we already have */
  printf("Accepted connection from: %s.\n", inet_ntoa(client.sin_addr));
  return cfd;
}

/*
  send() that reports a vanished peer as an error instead of raising SIGPIPE,
  whose default action would kill the controller the moment SITL exits.
*/
ssize_t socket_send(int fd, const void *buf, size_t len) {
#if defined(MSG_NOSIGNAL)
  return send(fd, buf, len, MSG_NOSIGNAL);
#else
#if defined(SO_NOSIGPIPE)
  int one = 1;
  setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
  return send(fd, buf, len, 0);
#endif
}

/*
  send the whole buffer: a stream socket's send() may write only part of it,
  and a frame cut short would run into the next one
*/
bool socket_send_all(int fd, const void *buf, size_t len) {
  const char *p = (const char *)buf;
  while (len > 0) {
    const ssize_t n = socket_send(fd, p, len);
    if (n <= 0) {
#ifndef _WIN32
      if (n < 0 && errno == EINTR) {
        continue;
      }
#endif
      return false;
    }
    p += n;
    len -= (size_t)n;
  }
  return true;
}

bool socket_close(int fd) {
#ifdef _WIN32
  return (closesocket(fd) == 0) ? true : false;
#else
  return (close(fd) == 0) ? true : false;
#endif
}

bool socket_cleanup(void) {
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