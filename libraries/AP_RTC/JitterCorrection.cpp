/*
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
/*
  class to correct an offboard timestamp in microseconds into a local
  timestamp, removing timing jitter caused by the transport.

  It takes the offboard_usec timestamp which is a timestamp provided
  in a protocol from a remote device, and the time of arrival of the
  message in local microseconds. It returns an improved estimate of
  the time that the message was generated on the remote system in the
  local time domain

  The algorithm assumes two things:

   1) the data did not come from the future in our local time-domain
   2) the data is not older than max_lag_ms in our local time-domain

  It works by estimating the transport lag by looking for the incoming
  packet that had the least lag, and converging on the offset that is
  associated with that lag
 */

#include <AP_HAL/AP_HAL.h>
#include "JitterCorrection.h"

// constructor
JitterCorrection::JitterCorrection(uint16_t _max_lag_ms, uint16_t _convergence_loops) :
    max_lag_ms(_max_lag_ms),
    convergence_loops(_convergence_loops)
{}

/*
  correct an offboard timestamp in microseconds into a local
  timestamp, removing timing jitter caused by the transport. 

  Return a value in microseconds since boot in the local time domain
 */
uint64_t JitterCorrection::correct_offboard_timestamp_usec(uint64_t offboard_usec, uint64_t local_usec)
{
    int64_t diff_us = int64_t(local_usec) - int64_t(offboard_usec);

    if (!initialised ||
        diff_us < link_offset_usec) {
        // this message arrived from the remote system with a
        // timestamp that would imply the message was from the
        // future. We know that isn't possible, so we adjust down the
        // correction value
        link_offset_usec = diff_us;
        initialised = true;
    }

    int64_t estimate_us = offboard_usec + link_offset_usec;

    if (estimate_us + (max_lag_ms*1000U) < int64_t(local_usec)) {
        // this implies the message came from too far in the past. clamp the lag estimate
        // to assume the message had maximum lag
        estimate_us = local_usec - (max_lag_ms*1000U);
        link_offset_usec = estimate_us - offboard_usec;
    }

    if (min_sample_counter == 0) {
        min_sample_us = diff_us;
    }
    min_sample_counter++;
    if (diff_us < min_sample_us) {
        min_sample_us = diff_us;
    }
    if (min_sample_counter == convergence_loops) {
        // we have the requested number of samples of the transport
        // lag for convergence. To account for long term clock drift
        // we set the diff we will use in future to this value
        link_offset_usec = min_sample_us;
        min_sample_counter = 0;
    }
    
    return uint64_t(estimate_us);
}

/*
  correct an offboard timestamp in microseconds into a local
  timestamp, removing timing jitter caused by the transport. 

  Return a value in milliseconds since boot in the local time domain
 */
uint32_t JitterCorrection::correct_offboard_timestamp_msec(uint32_t offboard_ms, uint32_t local_ms)
{
    return correct_offboard_timestamp_usec(offboard_ms*1000ULL, local_ms*1000ULL) / 1000ULL;
}
