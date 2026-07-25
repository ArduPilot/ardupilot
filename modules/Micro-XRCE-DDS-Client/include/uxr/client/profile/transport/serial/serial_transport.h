// Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef UXR_CLIENT_SERIAL_TRANSPORT_H_
#define UXR_CLIENT_SERIAL_TRANSPORT_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/core/communication/communication.h>
#include <uxr/client/profile/transport/stream_framing/stream_framing_protocol.h>
#include <uxr/client/config.h>
#include <uxr/client/visibility.h>
#include <uxr/client/transport.h>

typedef struct uxrSerialTransport
{
    uint8_t buffer[UXR_CONFIG_SERIAL_TRANSPORT_MTU];
    uxrFramingIO framing_io;
    uint8_t remote_addr;
    uxrCommunication comm;
    struct uxrSerialPlatform platform;

} uxrSerialTransport;

/** \addtogroup transport Transport
 *  These functions are platform-dependent. The declaration of these functions can be found in the uxr/client/profile/transport/ folder. The common init transport functions follow the nomenclature below.
 *  @{
 */

/**
 * @brief Initializes a UDP transport.
 * @param transport     The uninitialized transport structure used for managing the transport.
 *                      This structure must be accesible during the connection.
 * @param fd            The file descriptor of the serial connection.
 *                      The fd usually comes from the `open` OS function.
 * @param remote_addr   The addresss of the Agent in the serial connection.
 *                      By default, the Agent address' in a serial transport is 0.
 * @param local_addr    The address of the Client in the serial connection.
 * @return `true` in case of successful initialization. `false` in other case.
 */
UXRDLLAPI bool uxr_init_serial_transport(
        uxrSerialTransport* transport,
        const int fd,
        uint8_t remote_addr,
        uint8_t local_addr);

/**
 * @brief Closes a Serial transport.
 * @param transport The transport structure.
 * @return `true` in case of successful closing. `false` in other case.
 */
UXRDLLAPI bool uxr_close_serial_transport(
        uxrSerialTransport* transport);

/** @}*/

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // UXR_CLIENT_SERIAL_TRANSPORT_H_
