// Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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


#ifndef _UXR_CLIENT_VISIBILITY_H_
#define _UXR_CLIENT_VISIBILITY_H_

#if defined(_WIN32)
#if defined(microxrcedds_client_SHARED)
#if defined(microxrcedds_client_EXPORTS)
#define UXRDLLAPI __declspec( dllexport )
#else
#define UXRDLLAPI __declspec( dllimport )
#endif // micro_xrce_client_dds_EXPORTS
#else
#define UXRDLLAPI
#endif // BUILDING_SHARED_LIBS
#else
#define UXRDLLAPI
#endif // _WIN32

#endif // _UXR_CLIENT_VISIBILITY_H_
