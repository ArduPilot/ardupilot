/*
  AP_InternalError holds information about "should not happen" errors
  that have occured within ArduPilot.  This covers things like code
  paths that should not be crossed or pointers being null when they
  really, really shouldn't be.  It does NOT cover things like losing
  GPS lock at inopportune times - that's just bad luck, not bad
  programming.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

class AP_InternalError {
public:
    // internal error counters.  Do not set these unless it is a
    // *true* internal error - a thread locking up, a codepath which
    // should never be taken, a code-sanity-check failing, that sort
    // of thing.  Examples of what NOT to put in here - sd card
    // filling up, bad input received from GCS, GPS unit was working
    // and now is not.
    enum class error_t {                           // Hex      Decimal
        logger_mapfailure           = (1U <<  0),  // 0x00001  1
        logger_missing_logstructure = (1U <<  1),  // 0x00002  2
        logger_logwrite_missingfmt  = (1U <<  2),  // 0x00004  4
        logger_too_many_deletions   = (1U <<  3),  // 0x00008  8
        logger_bad_getfilename      = (1U <<  4),  // 0x00010  16
        unused1                     = (1U <<  5),  // 0x00020  32
        logger_flushing_without_sem = (1U <<  6),  // 0x00040  64
        logger_bad_current_block    = (1U <<  7),  // 0x00080  128
        logger_blockcount_mismatch  = (1U <<  8),  // 0x00100  256
        logger_dequeue_failure      = (1U <<  9),  // 0x00200  512
        constraining_nan            = (1U << 10),  // 0x00400  1024
        watchdog_reset              = (1U << 11),  // 0x00800  2048
        iomcu_reset                 = (1U << 12),  // 0x01000  4096
        iomcu_fail                  = (1U << 13),  // 0x02000  8192
        spi_fail                    = (1U << 14),  // 0x04000  16384
        main_loop_stuck             = (1U << 15),  // 0x08000  32768
        gcs_bad_missionprotocol_link= (1U << 16),  // 0x10000  65536
        bitmask_range               = (1U << 17),  // 0x20000  131072
        gcs_offset                  = (1U << 18),  // 0x40000  262144
        i2c_isr                     = (1U << 19),  // 0x80000  524288
        flow_of_control             = (1U << 20), // for generic we-should-never-get-here situations
    };

    void error(const AP_InternalError::error_t error);
    uint32_t count() const { return total_error_count; }

    // internal_errors - return mask of internal errors seen
    uint32_t errors() const {
        return internal_errors;
    }

private:

    // bitmask holding errors from internal_error_t
    uint32_t internal_errors;

    uint32_t total_error_count;
};

namespace AP {
    AP_InternalError &internalerror();
};
