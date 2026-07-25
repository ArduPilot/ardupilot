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

# Check directories.
set(_directories "lib" "include" "share")
foreach(_dir ${_directories})
    if(NOT EXISTS ${INSTALL_PATH}/${_dir})
        message(FATAL_ERROR "Directory ${_dir} not found.")
    endif()
endforeach()

# Check library.
if(NOT EXISTS ${INSTALL_PATH}/lib/${LIBRARY_NAME})
    message(FATAL_ERROR "Library lib/${LIBRARY_NAME} not found.")
endif()