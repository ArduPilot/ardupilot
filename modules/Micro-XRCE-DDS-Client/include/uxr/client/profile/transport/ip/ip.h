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

#ifndef UXR_CLIENT_PROFILE_TRANSPORT_IP_IP_H_
#define UXR_CLIENT_PROFILE_TRANSPORT_IP_IP_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/core/type/xrce_types.h>
#include <uxr/client/visibility.h>

typedef enum uxrIpProtocol
{
    UXR_IPv4,
    UXR_IPv6

} uxrIpProtocol;

/** \addtogroup transport Transport
 *  These functions are platform-dependent. The declaration of these functions can be found in the uxr/client/profile/transport/ folder. The common init transport functions follow the nomenclature below.
 *  @{
 */

/**
 * @brief Converts IPv4/IPv6 address + port to a TransportLocator.
 *
 * @param ip            The IP address to convert.
 *                      It could be IPv4 or IPv6 address.
 * @param port          The port to convert.
 * @param ip_protocol   The IP protocol of the IP address.
 * @param locator       The TransportLocator resulted from the conversion.
 *                      In case of error it will be NULL.
 * @return true         In case of successful conversion.
 * @return false        In other case.
 */
UXRDLLAPI bool uxr_ip_to_locator(
        char const* ip,
        uint16_t port,
        uxrIpProtocol ip_protocol,
        TransportLocator* locator);

/**
 * @brief Converts a TrasnportLocator to an IPv4/IPv6 address + port.
 *
 * @param locator       The TransportLocator resulted from the conversion.
 *                      In case of error it will be NULL.
 * @param ip            A char buffer there the address will be copied.
 * @param size          The size of the IP buffer.
 * @param port          The resulted port.
 * @param ip_protocol   The resulted IP protocol of the IP address.
 * @return true         In case of successful conversion.
 * @return false        In other case.
 */
UXRDLLAPI bool uxr_locator_to_ip(
        TransportLocator const* locator,
        char* ip,
        size_t size,
        uint16_t* port,
        uxrIpProtocol* ip_protocol);

/** @}*/

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // UXR_CLIENT_PROFILE_TRANSPORT_IP_IP_H_
