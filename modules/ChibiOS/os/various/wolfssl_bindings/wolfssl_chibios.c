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
 * **** This file incorporates work covered by the following copyright and ****
 * **** permission notice:                                                 ****
 *
 * Copyright (C) 2006-2017 wolfSSL Inc.
 *
 * This file is part of wolfSSL.
 *
 * wolfSSL is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * wolfSSL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA
 *
 */

#include "ch.h"
#include "wolfssl_chibios.h"
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/mem.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"
#include <string.h>
static int wolfssl_is_initialized = 0;

sslconn *sslconn_accept(sslconn *sk)
{
  sslconn *new;
  struct netconn *newconn = NULL;
  err_t err;
  err = netconn_accept(sk->conn, &newconn);
  if (err != ERR_OK) {
      return NULL;
  }
  new = chHeapAlloc(NULL, sizeof(sslconn));
  if (!new)
      return NULL;
  new->conn = newconn;
  new->ctx = sk->ctx;
  new->ssl = wolfSSL_new(new->ctx);
  wolfSSL_SetIOReadCtx(new->ssl, new);
  wolfSSL_SetIOWriteCtx(new->ssl, new);

  if (wolfSSL_accept(new->ssl) == SSL_SUCCESS) {
    wolfSSL_set_using_nonblock(new->ssl, 1);
    newconn->pcb.tcp->mss = 1480;
    return new;
  } else {
    wolfSSL_free(new->ssl);
    chHeapFree(new);
    return NULL;
  }
}

sslconn *sslconn_new(enum netconn_type t, WOLFSSL_METHOD* method)
{
    sslconn *sk;
    if (!wolfssl_is_initialized) {
        wolfSSL_Init();
        wolfssl_is_initialized++;
    }

    sk = chHeapAlloc(NULL, sizeof(sslconn));
    if (!sk)
        return NULL;
    memset(sk, 0, sizeof(sslconn));
    sk->ctx = wolfSSL_CTX_new(method);
    if (!sk->ctx)
        goto error;
    sk->conn = netconn_new(t);
    if (!sk->conn)
        goto error;
    wolfSSL_SetIORecv(sk->ctx, wolfssl_recv_cb);
    wolfSSL_SetIOSend(sk->ctx, wolfssl_send_cb);
    return sk;

error:
    if (sk->ctx)
        wolfSSL_CTX_free(sk->ctx);
    chHeapFree(sk);
    return NULL;
}

void sslconn_close(sslconn *sk)
{
    netconn_delete(sk->conn);
    wolfSSL_free(sk->ssl);
    chHeapFree(sk);
}


/* IO Callbacks */
int wolfssl_send_cb(WOLFSSL* ssl, char *buf, int sz, void *ctx)
{
  sslconn *sk = (sslconn *)ctx;
  int err;
  (void)ssl;
  err = netconn_write(sk->conn, buf, sz, NETCONN_COPY);
  if (err == ERR_OK)
    return sz;
  else if (err == ERR_CLSD || err == ERR_ABRT || err == ERR_CONN)
    return WOLFSSL_CBIO_ERR_CONN_CLOSE;
  else if (err == ERR_RST)
    return WOLFSSL_CBIO_ERR_CONN_RST;
  else if (err == ERR_MEM)
    return WOLFSSL_CBIO_ERR_GENERAL;
  else if (err == ERR_TIMEOUT)
    return WOLFSSL_CBIO_ERR_TIMEOUT;
  else
    return WOLFSSL_CBIO_ERR_WANT_WRITE;
}


#define MAX_SSL_BUF 1460
static uint8_t ssl_recv_buffer[MAX_SSL_BUF];
static int ssl_rb_len = 0;
static int ssl_rb_off = 0;

int wolfssl_recv_cb(WOLFSSL *ssl, char *buf, int sz, void *ctx)
{
    sslconn *sk = (sslconn *)ctx;
    struct netbuf *inbuf = NULL;
    uint8_t *net_buf;
    uint16_t buflen;
    (void)ssl;
    err_t err;

    if (ssl_rb_len > 0) {
        if (sz > ssl_rb_len - ssl_rb_off)
            sz = ssl_rb_len - ssl_rb_off;
        memcpy(buf, ssl_recv_buffer + ssl_rb_off, sz);
        ssl_rb_off += sz;
        if (ssl_rb_off >= ssl_rb_len) {
            ssl_rb_len = 0;
            ssl_rb_off = 0;
        }
        return sz;
    }


    err = netconn_recv(sk->conn, &inbuf);
    if (err == ERR_OK) {
        netbuf_data(inbuf, (void **)&net_buf, &buflen);
        ssl_rb_len = buflen;
        if (ssl_rb_len > MAX_SSL_BUF)
            ssl_rb_len = MAX_SSL_BUF;
        memcpy(ssl_recv_buffer, net_buf, ssl_rb_len);
        ssl_rb_off = 0;
        if (sz > ssl_rb_len)
            sz = ssl_rb_len;
        memcpy(buf, ssl_recv_buffer, sz);
        ssl_rb_off += sz;
        if (ssl_rb_off >= ssl_rb_len) {
            ssl_rb_len = 0;
            ssl_rb_off = 0;
        }
        netbuf_delete(inbuf);
        return sz;
    }
    else if (err == ERR_CLSD || err == ERR_ABRT || err == ERR_CONN)
        return WOLFSSL_CBIO_ERR_CONN_CLOSE;
    else if (err == ERR_RST)
        return WOLFSSL_CBIO_ERR_CONN_RST;
    else if (err == ERR_MEM)
        return WOLFSSL_CBIO_ERR_GENERAL;
    else if (err == ERR_TIMEOUT)
        return WOLFSSL_CBIO_ERR_TIMEOUT;
    else
        return 0;
    //return WOLFSSL_CBIO_ERR_WANT_READ;
}

#ifndef ST2S
#   define ST2S(n) (((n) + CH_CFG_ST_FREQUENCY - 1UL) / CH_CFG_ST_FREQUENCY)
#endif

#ifndef ST2MS
#define ST2MS(n) (((n) * 1000UL + CH_CFG_ST_FREQUENCY - 1UL) / CH_CFG_ST_FREQUENCY)
#endif


word32 LowResTimer(void)
{
    systime_t t = chVTGetSystemTimeX();
    return ST2S(t);
}

uint32_t TimeNowInMilliseconds(void)
{
    systime_t t = chVTGetSystemTimeX();
    return ST2MS(t);
}

void *chHeapRealloc (void *addr, uint32_t size)
{
    union heap_header *hp;
    uint32_t prev_size, new_size;

    void *ptr;

    if(addr == NULL) {
        return chHeapAlloc(NULL, size);
    }

    /* previous allocated segment is preceded by an heap_header */
    hp = addr - sizeof(union heap_header);
    prev_size = hp->used.size; /* size is always multiple of 8 */

    /* check new size memory alignment */
    if(size % 8 == 0) {
        new_size = size;
    }
    else {
        new_size = ((int) (size / 8)) * 8 + 8;
    }

    if(prev_size >= new_size) {
        return addr;
    }

    ptr = chHeapAlloc(NULL, size);
    if(ptr == NULL) {
        return NULL;
    }

    memcpy(ptr, addr, prev_size);

    chHeapFree(addr);

    return ptr;
}

void *chibios_alloc(void *heap, int size)
{
    return chHeapAlloc(heap, size);
}

void chibios_free(void *ptr)
{
    if (ptr)
        chHeapFree(ptr);
}

