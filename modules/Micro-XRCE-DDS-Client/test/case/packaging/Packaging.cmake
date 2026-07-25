# Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

execute_process(
    COMMAND
        ${CMAKE_COMMAND} ${ORIGINAL_DIR}
            -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
            -DREQUIRED_VERSION=${REQUIRED_VERSION}
            -DCMAKE_GENERATOR_TOOLSET=${CMAKE_GENERATOR_TOOLSET}
            -DCMAKE_GENERATOR_PLATFORM=${CMAKE_GENERATOR_PLATFORM}
    RESULT_VARIABLE _result
    )

if(_result)
    message(FATAL_ERROR "Error in find_package.")
endif()

execute_process(
    COMMAND
        ${CMAKE_COMMAND} --build .
    RESULT_VARIABLE _result
    )

if(_result)
    message(FATAL_ERROR "Error compiling example.")
endif()