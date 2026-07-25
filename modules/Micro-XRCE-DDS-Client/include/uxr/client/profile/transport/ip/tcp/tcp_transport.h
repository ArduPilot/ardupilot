// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file
 */

#ifndef UXR_CLIENT_TCP_TRANSPORT_H_
#define UXR_CLIENT_TCP_TRANSPORT_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/profile/transport/ip/ip.h>
#include <uxr/client/core/communication/communication.h>
#include <uxr/client/config.h>
#include <uxr/client/visibility.h>
#include <uxr/client/transport.h>
typedef enum uxrTCPInputBufferState
{
    UXR_TCP_BUFFER_EMPTY,
    UXR_TCP_SIZE_INCOMPLETE,
    UXR_TCP_SIZE_READ,
    UXR_TCP_MESSAGE_INCOMPLETE,
    UXR_TCP_MESSAGE_AVAILABLE

} uxrTCPInputBufferState;

typedef struct uxrTCPInputBuffer
{
    uint8_t buffer[UXR_CONFIG_TCP_TRANSPORT_MTU];
    size_t position;
    uxrTCPInputBufferState state;
    size_t msg_size;

} uxrTCPInputBuffer;

typedef struct uxrTCPTransport
{
    uxrTCPInputBuffer input_buffer;
    uxrCommunication comm;
    struct uxrTCPPlatform platform;

} uxrTCPTransport;

/** \addtogroup transport Transport
 *  These functions are platform-dependent. The declaration of these functions can be found in the uxr/client/profile/transport/ folder. The common init transport functions follow the nomenclature below.
 *  @{
 */

/**
 * @brief Initializes a TCP transport.
 * @param transport     The uninitialized transport structure used for managing the transport.
 *                      This structure must be accesible during the connection.
 * @param ip_protocol   The IP protocol, it could be UXR_IPv4 or UXR_IPv6.
 * @param ip            The IP address of the Agent.
 * @param port          The port of the Agent.
 * @return `true` in case of successful initialization. `false` in other case.
 */
UXRDLLAPI bool uxr_init_tcp_transport(
        uxrTCPTransport* transport,
        uxrIpProtocol ip_protocol,
        const char* ip,
        const char* port);

/**
 * @brief Closes a TCP transport.
 * @param transport The transport structure.
 * @return `true` in case of successful closing. `false` in other case.
 */
UXRDLLAPI bool uxr_close_tcp_transport(
        uxrTCPTransport* transport);

/** @}*/

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // UXR_CLIENT_TCP_TRANSPORT_H_
