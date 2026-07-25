/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * This file is a modified version of the lwIP web server demo. The original
 * author is unknown because the file didn't contain any license information.
 *
 * The HTTPS version is Copyright (C) 2017 - WolfSSL Inc. and is based on the 
 * demo HTTP code of ChibiOS.
 */


/**
 * @file web.c
 * @brief HTTPS server wrapper thread code.
 * @addtogroup WEB_THREAD
 * @{
 */

#include <ctype.h>

#include "ch.h"

#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"

#include "wolfssl_chibios.h"
#include "web.h"

#if LWIP_NETCONN

static char url_buffer[WEB_MAX_PATH_SIZE];
extern unsigned char server_cert[];
extern unsigned int server_cert_len;
extern unsigned char server_key[];
extern unsigned int server_key_len;

#define HEXTOI(x) (isdigit(x) ? (x) - '0' : (x) - 'a' + 10)

/**
 * @brief   Decodes an URL sting.
 * @note    The string is terminated by a zero or a separator.
 *
 * @param[in] url       encoded URL string
 * @param[out] buf      buffer for the processed string
 * @param[in] max       max number of chars to copy into the buffer
 * @return              The conversion status.
 * @retval false        string converted.
 * @retval true         the string was not valid or the buffer overflowed
 *
 * @notapi
 */
static bool decode_url(const char *url, char *buf, size_t max) {

  while (true) {
    int h, l;
    unsigned c = *url++;

    switch (c) {
    case 0:
    case '\r':
    case '\n':
    case '\t':
    case ' ':
    case '?':
      *buf = 0;
      return false;
    case '.':
      if (max <= 1)
        return true;

      h = *(url + 1);
      if (h == '.')
        return true;

      break;
    case '%':
      if (max <= 1)
        return true;

      h = tolower((int)*url++);
      if (h == 0)
        return true;
      if (!isxdigit(h))
        return true;

      l = tolower((int)*url++);
      if (l == 0)
        return true;
      if (!isxdigit(l))
        return true;

      c = (char)((HEXTOI(h) << 4) | HEXTOI(l));
      break;
    default:
      if (max <= 1)
        return true;

      if (!isalnum(c) && (c != '_') && (c != '-') && (c != '+') &&
          (c != '/'))
        return true;

      break;
    }

    *buf++ = c;
    max--;
  }
}


#define MAX_HTTPREQ_SIZE 256
static const char http_html_hdr[] = "HTTP/1.1 200 OK\r\nContent-type: text/html\r\n\r\n";
static const char http_index_html[] = "<html><head><title>Congrats!</title></head><body><h1>Welcome to chibiOS HTTPS server!</h1><p>Powered by LwIP + WolfSSL</body></html>";

static char inbuf[MAX_HTTPREQ_SIZE];
static void https_server_serve(sslconn *sc) 
{
  int ret;

  /* Read the data from the port, blocking if nothing yet there.
     We assume the request (the part we care about) is in one netbuf.*/
  ret = wolfSSL_read(sc->ssl, inbuf, MAX_HTTPREQ_SIZE);
  if (ret >= 5 &&
          inbuf[0] == 'G' &&
          inbuf[1] == 'E' &&
          inbuf[2] == 'T' &&
          inbuf[3] == ' ' &&
          inbuf[4] == '/') {

      if (decode_url(inbuf + 4, url_buffer, WEB_MAX_PATH_SIZE)) {
          /* Invalid URL handling.*/
          return;
      }

      /* Send the HTML header
       * subtract 1 from the size, since we dont send the \0 in the string
       * NETCONN_NOCOPY: our data is const static, so no need to copy it
       */
      wolfSSL_write(sc->ssl, http_html_hdr, sizeof(http_html_hdr)-1);

      /* Send our HTML page */
      wolfSSL_write(sc->ssl, http_index_html, sizeof(http_index_html)-1);
  }
}

/**
 * @brief   Stack area for the http thread.
 */
THD_WORKING_AREA(wa_https_server, WEB_THREAD_STACK_SIZE);

/**
 * @brssl   HTTPS server thread.
 */
THD_FUNCTION(https_server, p) {
  sslconn *sc, *newsc;
  (void)p;
  chRegSetThreadName("https");

  /* Initialize wolfSSL */
  wolfSSL_Init();

  /* Create a new SSL connection handle */
  sc = sslconn_new(NETCONN_TCP, wolfTLSv1_2_server_method());
  if (!sc) {
      while(1) {}
  }

  /* Load certificate file for the HTTPS server */
  if (wolfSSL_CTX_use_certificate_buffer(sc->ctx, server_cert,
              server_cert_len, SSL_FILETYPE_ASN1 ) != SSL_SUCCESS)
      while(1) {}

  /* Load the private key */
  if (wolfSSL_CTX_use_PrivateKey_buffer(sc->ctx, server_key, 
              server_key_len, SSL_FILETYPE_ASN1 ) != SSL_SUCCESS)
      while(1) {}
  
  /* Bind to port 443 (HTTPS) with default IP address */
  netconn_bind(sc->conn, NULL, WEB_THREAD_PORT);

  /* Put the connection into LISTEN state */
  netconn_listen(sc->conn);
  
  /* Goes to the final priority after initialization.*/
  chThdSetPriority(WEB_THREAD_PRIORITY);

  /* Listening loop */
  while (true) {
    newsc = sslconn_accept(sc);
    if (!newsc) {
        chThdSleepMilliseconds(500);
        continue;
    }
    /* New connection: a new SSL connector is spawned */
    https_server_serve(newsc);
    sslconn_close(newsc);
  }
}

#endif /* LWIP_NETCONN */

/** @} */
