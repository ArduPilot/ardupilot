[#ftl]
[#--
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  --]
[@pp.dropOutputFile /]
[#import "/@lib/libutils.ftl" as utils /]
[@pp.changeOutputFile name="board.mk" /]
[#if doc1.board.configuration_settings.board_files_path[0]??]
  [#assign path = doc1.board.configuration_settings.board_files_path[0]?string?trim /]
  [#if !path?ends_with("/")]
    [#assign path = path + "/"]
  [/#if]
[#else]
  [#assign path = "$(CHIBIOS)/os/hal/boards/" /]
[/#if]
# List of all the board related files.
BOARDSRC = ${path}${doc1.board.board_id[0]}/board.c

# Required include directories
BOARDINC = ${path}${doc1.board.board_id[0]}

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
