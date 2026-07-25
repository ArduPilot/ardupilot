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

#ifndef _UXR_CLIENT_CLIENT_H_
#define _UXR_CLIENT_CLIENT_H_

#include <uxr/client/config.h>

#ifdef UCLIENT_PROFILE_DISCOVERY
#include <uxr/client/profile/discovery/discovery.h>
#endif //UCLIENT_PROFILE_DISCOVERY

#include <uxr/client/core/session/session.h>
#include <uxr/client/core/session/write_access.h>
#include <uxr/client/core/session/read_access.h>
#include <uxr/client/core/session/create_entities_ref.h>
#include <uxr/client/core/session/create_entities_xml.h>
#include <uxr/client/core/session/create_entities_bin.h>

#include <uxr/client/transport.h>
#include <uxr/client/profile/multithread/multithread.h>

#endif // _UXR_CLIENT_CLIENT_H_
