#include "AP_InternalError.h"

#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL &hal;

// actually create the instance:
static AP_InternalError instance;

void AP_InternalError::error(const AP_InternalError::error_t e, uint16_t line) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    switch (e) {
    case AP_InternalError::error_t::watchdog_reset:
    case AP_InternalError::error_t::main_loop_stuck:
        // don't panic on these to facilitate watchdog testing
        break;
    default:
        AP_HAL::panic("internal error %u", unsigned(e));
    }
#endif
    internal_errors |= uint32_t(e);
    total_error_count++;
    last_line = line;

    hal.util->persistent_data.internal_errors = internal_errors;
    hal.util->persistent_data.internal_error_count = total_error_count;
    hal.util->persistent_data.internal_error_last_line = line;
}

void AP_InternalError::errors_as_string(uint8_t *buffer, const uint16_t len) const
{
    static const char * const error_bit_descriptions[] {
        "mapfailure",  // logger_mapfailure
        "miss_struct",  // logger_missing_logstructure
        "write_mssfmt",  // logger_logwrite_missingfmt
        "many_deletes",  // logger_too_many_deletions
        "bad_getfile",  // logger_bad_getfilename
        "unused1",
        "flush_no_sem",  // logger_flushing_without_sem
        "bad_curr_blk",  // logger_bad_current_block
        "blkcnt_bad",  // logger_blockcount_mismatch
        "dq_failure",  // logger_dequeue_failure
        "cnstring_nan",  // constraining_nan
        "watchdog_rst",  // watchdog_reset
        "iomcu_reset",
        "iomcu_fail",
        "spi_fail",
        "main_loop_stk",  // main_loop_stuck
        "gcs_bad_link",  // gcs_bad_missionprotocol_link
        "bitmask_range",
        "gcs_offset",
        "i2c_isr",
        "flow_of_ctrl",  // flow_of_control
        "sfs_recursion",  // switch_full_sector_recursion
        "bad_rotation",
        "stack_ovrflw",  // stack_overflow
    };

    static_assert((1U<<(ARRAY_SIZE(error_bit_descriptions))) == uint32_t(AP_InternalError::error_t::__LAST__), "too few descriptions for bits");

    buffer[0] = 0;
    uint32_t buffer_used = 0;
    for (uint8_t i=0; i<ARRAY_SIZE(error_bit_descriptions); i++) {
        if (buffer_used >= len) {
            break;
        }
        if (internal_errors & (1U<<i)) {
            const char *format;
            if (i == 0) {
                format = "%s";
            } else {
                format = ",%s";
            }
            const size_t written = hal.util->snprintf((char*)&buffer[buffer_used],
                                                      len-buffer_used,
                                                      format,
                                                      error_bit_descriptions[i]);
            if (written <= 0) {
                break;
            }
            buffer_used += written;
        }
    }
}

namespace AP {

AP_InternalError &internalerror()
{
    return instance;
}

};

// stack overflow hook for low level RTOS code, C binding
void AP_stack_overflow(const char *thread_name)
{
    static bool done_stack_overflow;
    INTERNAL_ERROR(AP_InternalError::error_t::stack_overflow);
    if (!done_stack_overflow) {
        // we don't want to record the thread name more than once, as
        // first overflow can trigger a 2nd
        strncpy(hal.util->persistent_data.thread_name4, thread_name, 4);
        done_stack_overflow = true;
    }
    hal.util->persistent_data.fault_type = 42; // magic value
    if (!hal.util->get_soft_armed()) {
        AP_HAL::panic("stack overflow %s\n", thread_name);
    }
}
