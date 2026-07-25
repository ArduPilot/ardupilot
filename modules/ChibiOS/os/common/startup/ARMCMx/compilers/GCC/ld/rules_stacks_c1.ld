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

SECTIONS
{
    /* TODO */
    __c1_main_stack_size__ = __main_stack_size__;
    __c1_process_stack_size__ = __process_stack_size__;

    /* Special section for exceptions stack.*/
    .c1_mstack (NOLOAD) :
    {
        . = ALIGN(8);
        __c1_main_stack_base__ = .;
        . += __c1_main_stack_size__;
        . = ALIGN(8);
        __c1_main_stack_end__ = .;
    } > C1_MAIN_STACK_RAM

    /* Special section for process stack.*/
    .c1_pstack (NOLOAD) :
    {
        . = ALIGN(8);
        __c1_process_stack_base__ = .;
        __c1_main_thread_stack_base__ = .;
        . += __c1_process_stack_size__;
        . = ALIGN(8);
        __c1_process_stack_end__ = .;
        __c1_main_thread_stack_end__ = .;
    } > C1_PROCESS_STACK_RAM
}
