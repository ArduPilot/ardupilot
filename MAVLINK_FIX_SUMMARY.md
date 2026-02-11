# MAVLink Packet Processing Fix Summary

## Problem Description
The MAVLink packet processing in `GCS_MAVLink::update_receive()` was causing system blocking and unresponsiveness under high load conditions. The main issues were:

1. **No time limit checks inside the main processing loop** - The function could spend excessive time processing packets without yielding to other tasks
2. **No protection against malformed packet loops** - Bad data could cause the parser to get stuck in tight loops
3. **No periodic task yielding** - The function would monopolize CPU time, preventing other critical tasks from running

## Root Cause Analysis
The `update_receive()` function processes incoming MAVLink packets in a loop without:
- Time budget enforcement within the loop
- Protection against malformed packet storms
- Periodic yielding to other tasks
- Parser state recovery mechanisms

## Solution Implemented
Added comprehensive protections to prevent blocking:

### 1. Time Limit Enforcement
```cpp
// Check time limit more frequently to prevent blocking other tasks
if (AP_HAL::micros() - tstart_us > max_time_us) {
    break;
}
```

### 2. Malformed Packet Protection
```cpp
const uint16_t max_malformed_packets = 100; // Limit malformed packets to prevent tight loops
uint16_t malformed_packet_count = 0;

// Count malformed packets to prevent tight loops
if (framing == MAVLINK_FRAMING_BAD_CRC || framing == MAVLINK_FRAMING_BAD_SIGNATURE) {
    malformed_packet_count++;
}

// Check for excessive malformed packets and reset parser state if needed
if (malformed_packet_count > max_malformed_packets) {
    // Reset the parser state to recover from malformed data
    mavlink_status_t *status_ptr = channel_status();
    if (status_ptr != nullptr) {
        // Reset parser state
        status_ptr->parse_state = MAVLINK_PARSE_STATE_IDLE;
        status_ptr->packet_idx = 0;
        status_ptr->current_rx_seq = 0;
        status_ptr->packet_rx_success_count = 0;
        status_ptr->packet_rx_drop_count += malformed_packet_count;
    }
    malformed_packet_count = 0;
}
```

### 3. Periodic Task Yielding
```cpp
const uint16_t max_bytes_per_iteration = 64; // Limit bytes processed per iteration to prevent blocking

// Yield to other tasks periodically to prevent CPU starvation
if (processed_bytes % max_bytes_per_iteration == 0) {
    // Small delay to allow other tasks to run
    hal.scheduler->delay_microseconds(10);
    
    // Check if we've exceeded our time budget
    if (AP_HAL::micros() - tstart_us > max_time_us) {
        break;
    }
}
```

## Benefits
1. **Improved System Responsiveness** - Other tasks can run even under high MAVLink load
2. **Protection Against Malformed Data** - System recovers gracefully from bad packets
3. **Predictable Performance** - Processing time is bounded and controlled
4. **Better Resource Management** - CPU time is shared fairly among tasks

## Testing
The fix has been tested and verified to:
- Maintain normal MAVLink functionality under normal conditions
- Prevent blocking under high packet load
- Recover gracefully from malformed packet storms
- Yield appropriately to other system tasks

## Files Modified
- `libraries/GCS_MAVLink/GCS_Common.cpp` - Added comprehensive protections in `update_receive()` method

## Commit
The fix has been committed with the message:
"Fix MAVLink packet processing to prevent blocking other tasks"