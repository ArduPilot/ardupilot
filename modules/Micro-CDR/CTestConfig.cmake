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

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # MemoryCheck configuration.
    find_program(MEMORYCHECK_COMMAND NAMES valgrind)
    set(MEMORYCHECK_COMMAND_OPTIONS "${MEMORYCHECK_COMMAND_OPTIONS} --quiet --tool=memcheck --leak-check=yes --show-reachable=yes --num-callers=50 --xml=yes --xml-file=test_%p_memcheck.xml \"--suppressions=${CMAKE_CURRENT_SOURCE_DIR}/ci/valgrind.supp\"")

    # Coverage configuration.
    find_program(COVERAGE_COMMAND NAMES gcov)
endif()