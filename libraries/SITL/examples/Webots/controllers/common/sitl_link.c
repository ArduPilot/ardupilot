#include <webots/robot.h>
#include <webots/supervisor.h>

#include "sitl_link.h"
#include "sockets.h"

static int server_fd = -1;
static int client_fd = -1;

/* bytes received from SITL that do not yet end in a newline */
static char rx_buf[4096];
static size_t rx_len;

/* memrchr is a GNU extension and absent on Windows */
static const char *last_char(const char *buf, size_t len, char c)
{
  for (size_t i = len; i > 0; --i) {
    if (buf[i - 1] == c) {
      return &buf[i - 1];
    }
  }
  return NULL;
}

bool sitl_link_open(int port)
{
  server_fd = create_socket_server(port);
  return server_fd >= 0;
}

bool sitl_link_connected(void)
{
  return client_fd >= 0;
}

bool sitl_link_accept(void)
{
  printf("Waiting for ArduPilot SITL to connect...\n");
  while (true) {
    const int fd = socket_accept(server_fd);
    if (fd > 0) {
      client_fd = fd;
      rx_len = 0;   /* never mix a partial line from the previous session in */
      return true;
    }
#ifndef _WIN32
    if (fd < 0 && errno != EINTR) {
      return false;
    }
#else
    if (fd < 0) {
      return false;
    }
#endif
  }
}

static void drop_client(void)
{
  if (client_fd >= 0) {
    socket_close(client_fd);
    client_fd = -1;
  }
  printf("ArduPilot SITL disconnected.\n");
}

/*
  Read whatever is pending and, if that completes at least one line, parse the
  newest complete line.  TCP gives no message boundaries, and older lines are
  stale servo frames.
*/
static enum sitl_link_status receive_controls(sitl_parse_fn parse)
{
  if (rx_len >= sizeof(rx_buf) - 1) {
    /* no newline in a full buffer: the peer is not speaking our protocol */
    fprintf(stderr, "Receive buffer overflow, dropping %zu bytes\n", rx_len);
    rx_len = 0;
  }

  const ssize_t n = recv(client_fd, rx_buf + rx_len, sizeof(rx_buf) - 1 - rx_len, 0);
  if (n == 0) {
    return SITL_LINK_CLOSED;
  }
  if (n < 0) {
#ifdef _WIN32
    const int e = WSAGetLastError();
    if (e == WSAEWOULDBLOCK) {
      return SITL_LINK_NO_CONTROLS;
    }
    fprintf(stderr, "Error reading from socket: %d.\n", e);
#else
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return SITL_LINK_NO_CONTROLS;
    }
    fprintf(stderr, "Error reading from socket: %d.\n", errno);
#endif
    return SITL_LINK_CLOSED;
  }

  rx_len += (size_t)n;
  rx_buf[rx_len] = 0;

  char *last_nl = (char *)last_char(rx_buf, rx_len, '\n');
  if (last_nl == NULL) {
    return SITL_LINK_NO_CONTROLS;
  }

  *last_nl = 0;
  char *start = (char *)last_char(rx_buf, (size_t)(last_nl - rx_buf), '\n');
  start = (start == NULL) ? rx_buf : start + 1;

  const bool parsed = parse(start);

  /* keep the partial line that follows the last newline */
  const size_t consumed = (size_t)(last_nl - rx_buf) + 1;
  memmove(rx_buf, rx_buf + consumed, rx_len - consumed);
  rx_len -= consumed;

  return parsed ? SITL_LINK_CONTROLS : SITL_LINK_NO_CONTROLS;
}

enum sitl_link_status sitl_link_exchange(const char *frame, sitl_parse_fn parse)
{
  if (client_fd < 0) {
    return SITL_LINK_CLOSED;
  }

  if (!socket_send_all(client_fd, frame, strlen(frame))) {
    drop_client();
    return SITL_LINK_CLOSED;
  }

  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(client_fd, &rfds);

  /* a real timeout keeps the lockstep but lets us re-send if a frame is lost */
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 50000;   /* 50 ms */

  const int ready = select(client_fd + 1, &rfds, NULL, NULL, &tv);
  if (ready < 0) {
#ifndef _WIN32
    if (errno == EINTR) {
      return SITL_LINK_NO_CONTROLS;
    }
#endif
    fprintf(stderr, "select error\n");
    drop_client();
    return SITL_LINK_CLOSED;
  }
  if (ready == 0) {
    /* timed out: re-send the same frame without advancing simulation time */
    return SITL_LINK_NO_CONTROLS;
  }

  const enum sitl_link_status status = receive_controls(parse);
  if (status == SITL_LINK_CLOSED) {
    drop_client();
  }
  return status;
}

void sitl_link_close(void)
{
  if (client_fd >= 0) {
    socket_close(client_fd);
    client_fd = -1;
  }
  if (server_fd >= 0) {
    socket_close(server_fd);
    server_fd = -1;
  }
  socket_cleanup();
}

static bool pose_saved;
static double start_translation[3];
static double start_rotation[4];

void vehicle_pose_save(void)
{
  if (!wb_robot_get_supervisor()) {
    return;
  }
  WbNodeRef self = wb_supervisor_node_get_self();
  memcpy(start_translation,
         wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(self, "translation")),
         sizeof(start_translation));
  memcpy(start_rotation,
         wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(self, "rotation")),
         sizeof(start_rotation));
  pose_saved = true;
}

void vehicle_pose_restore(void)
{
  if (!pose_saved) {
    printf("Robot is not a supervisor: leaving the vehicle where it is.\n");
    return;
  }
  WbNodeRef self = wb_supervisor_node_get_self();
  wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(self, "translation"), start_translation);
  wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(self, "rotation"), start_rotation);
  /* zero the velocity of the vehicle and everything attached to it */
  wb_supervisor_node_reset_physics(self);
  printf("Vehicle reset to its start pose.\n");
}
