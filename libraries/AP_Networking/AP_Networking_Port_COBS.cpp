#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_COBS

#include "AP_Networking_Port_COBS.h"
#include "AP_Networking_Hub.h"
#include "AP_Networking_COBS_Protocol.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <string.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

using namespace AP_Networking_COBS_Protocol;

AP_Networking_Port_COBS::AP_Networking_Port_COBS(AP_Networking_Hub *hub_in,
        AP_HAL::UARTDriver *uart_in,
        uint32_t baud_rate,
        const uint8_t device_id[6]) :
    hub(hub_in)
{
    memcpy(local_device_id, device_id, 6);
    
    // Initialize first UART
    uarts[0] = {};  // Value-initialize to clear all fields
    uarts[0].uart = uart_in;
    uarts[0].baud = baud_rate;
    uarts[0].health = UARTState::Health::GOOD;
    num_uarts = 1;
}

AP_Networking_Port_COBS::~AP_Networking_Port_COBS()
{
    delete[] reorder_buffer;
}

bool AP_Networking_Port_COBS::init()
{
    if (hub == nullptr || uarts[0].uart == nullptr) {
        return false;
    }
    
    // Initialize all UARTs
    for (uint8_t i = 0; i < num_uarts; i++) {
        UARTState &u = uarts[i];
        if (u.uart == nullptr) continue;
        
        // Large TX buffer for direct writes
        u.uart->begin(u.baud, 2048, TX_BUFFER_SIZE);
        
        const uint32_t now = AP_HAL::millis();
        u.last_rx_ms = 0;  // Start with link down
        u.last_keepalive_tx_ms = now;
    }
    
    // Start dedicated thread for low-latency RX/TX
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_Port_COBS::thread, void),
                                      "cobs_port",
                                      2048, AP_HAL::Scheduler::PRIORITY_NET, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: COBS thread failed");
        return false;
    }
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: COBS port init OK, %u UARTs", num_uarts);
    return true;
}

bool AP_Networking_Port_COBS::add_uart(AP_HAL::UARTDriver *uart, uint32_t baud)
{
    if (num_uarts >= MAX_UARTS || uart == nullptr) {
        return false;
    }
    
    UARTState &u = uarts[num_uarts];
    u = {};  // Value-initialize to clear all fields
    u.uart = uart;
    u.baud = baud;
    u.health = UARTState::Health::GOOD;
    
    // Initialize UART
    u.uart->begin(baud, 2048, TX_BUFFER_SIZE);
    u.last_keepalive_tx_ms = AP_HAL::millis();
    
    num_uarts++;
    
    // Transition to ganged mode
    if (num_uarts >= 2 && state == State::SINGLE) {
        state = State::GANGED;
        tx_seq = 0;
        rx_expected_seq = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: COBS entering ganged mode (%u UARTs)", num_uarts);
    }
    
    return true;
}

bool AP_Networking_Port_COBS::extract_uart(uint8_t idx,
                                            AP_HAL::UARTDriver *&uart_out,
                                            uint32_t &baud_out)
{
    if (idx >= num_uarts) return false;
    
    UARTState &u = uarts[idx];
    uart_out = u.uart;
    baud_out = u.baud;
    
    // Remove from array (shift remaining)
    for (uint8_t i = idx; i < num_uarts - 1; i++) {
        uarts[i] = uarts[i + 1];
    }
    num_uarts--;
    
    // State transition if needed
    if (num_uarts <= 1) {
        state = State::SINGLE;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: COBS returning to single mode");
    }
    
    return true;
}

bool AP_Networking_Port_COBS::get_remote_device_id(uint8_t id_out[6]) const
{
    if (!remote_id_known) {
        return false;
    }
    memcpy(id_out, remote_device_id, 6);
    return true;
}

AP_HAL::UARTDriver *AP_Networking_Port_COBS::get_uart(uint8_t idx) const
{
    if (idx >= num_uarts) return nullptr;
    return uarts[idx].uart;
}

uint32_t AP_Networking_Port_COBS::get_baud(uint8_t idx) const
{
    if (idx >= num_uarts) return 0;
    return uarts[idx].baud;
}

void AP_Networking_Port_COBS::thread()
{
    // Wait for system to be ready
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay_microseconds(1000);
    }
    
    // Take UART ownership for this thread
    for (uint8_t i = 0; i < num_uarts; i++) {
        if (uarts[i].uart != nullptr) {
            uarts[i].uart->begin(0);
        }
    }

    while (true) {
        // Process RX on all UARTs
        for (uint8_t i = 0; i < num_uarts; i++) {
            if (uarts[i].uart != nullptr) {
                process_rx(i);
            }
        }
        
        // Send keepalives as needed
        check_keepalive_tx();
        
        // Check for UART timeouts
        check_uart_timeouts();
        
        // Update health states
        update_uart_health();
        
        // Small delay - 100us polling rate
        hal.scheduler->delay_microseconds(100);
    }
}

void AP_Networking_Port_COBS::process_rx(uint8_t uart_idx)
{
    UARTState &u = uarts[uart_idx];
    if (u.uart == nullptr) return;
    
    uint8_t chunk[256];
    const uint32_t start_us = AP_HAL::micros();
    
    while (true) {
        const uint32_t avail = u.uart->available();
        if (avail == 0) break;
        
        const uint32_t to_read = (avail < sizeof(chunk)) ? avail : sizeof(chunk);
        const auto nread = u.uart->read(chunk, to_read);
        if (nread <= 0) break;
        
        // Process each byte through decoder
        for (ssize_t i = 0; i < nread; i++) {
            if (u.decoder.process_byte(chunk[i])) {
                // Frame complete
                size_t frame_len = sizeof(rx_frame_buffer);
                if (!u.decoder.get_frame(rx_frame_buffer, &frame_len, sizeof(rx_frame_buffer))) {
                    rx_errors++;
                    u.decoder.resync();
                    continue;
                }
                u.decoder.reset();
                if (!handle_decoded_frame(uart_idx, rx_frame_buffer, frame_len)) {
                    u.decoder.resync();
                }
            }
        }
        
        // Time limit (1ms max per UART)
        if ((AP_HAL::micros() - start_us) > 1000U) break;
    }
}

bool AP_Networking_Port_COBS::handle_decoded_frame(uint8_t uart_idx, const uint8_t *data, size_t len)
{
    const uint8_t *payload;
    size_t payload_len;
    
    // Use shared frame identification
    FrameType type = identify_frame(data, len, &payload, &payload_len);
    
    switch (type) {
    case FrameType::KEEPALIVE:
        return handle_keepalive(uart_idx, payload);
    
    case FrameType::DATA_SINGLE:
        return handle_data_frame_single(uart_idx, payload, payload_len);
    
    case FrameType::DATA_GANGED: {
        uint16_t seq = get_ganged_seq(data);  // seq is at start of data, before payload
        return handle_data_frame_ganged(uart_idx, seq, payload, payload_len);
    }
    
    case FrameType::INVALID:
    default:
        crc_errors++;
        return false;
    }
}

bool AP_Networking_Port_COBS::handle_keepalive(uint8_t uart_idx, const uint8_t *data)
{
    UARTState &u = uarts[uart_idx];
    
    // Extract keepalive fields using shared helper
    uint8_t ka_device_id[6];
    uint16_t their_rx_good;
    parse_keepalive(data, ka_device_id, &their_rx_good);
    
    // Hotplug detection: device ID changed on this UART?
    if (u.device_id_valid && memcmp(ka_device_id, u.seen_device_id, 6) != 0) {
        hub->request_cobs_split(this, uart_idx, ka_device_id);
        return true;
    }
    
    // Check against port's established remote
    if (remote_id_known && memcmp(ka_device_id, remote_device_id, 6) != 0) {
        hub->request_cobs_split(this, uart_idx, ka_device_id);
        return true;
    }
    
    // Update UART state
    memcpy(u.seen_device_id, ka_device_id, 6);
    u.device_id_valid = true;
    u.last_rx_ms = AP_HAL::millis();
    
    // Establish port's remote if not yet known
    if (!remote_id_known) {
        memcpy(remote_device_id, ka_device_id, 6);
        remote_id_known = true;
    }
    
    // Calculate loss for this measurement period
    uint16_t frames_sent = u.tx_count - u.tx_count_at_last_ka;
    uint16_t frames_received = their_rx_good - u.last_remote_rx_good;
    
    if (frames_sent > 0) {
        // Calculate loss (handle wrapping)
        int16_t lost = (int16_t)(frames_sent - frames_received);
        if (lost < 0) lost = 0;
        u.lost_since_last_ka = lost;
        
        // Update rolling loss rate (exponential moving average)
        uint8_t new_loss_rate = MIN(255, (uint16_t)lost * 255 / frames_sent);
        // EMA: new = old * 0.75 + sample * 0.25
        u.loss_rate_u8 = (u.loss_rate_u8 * 3 + new_loss_rate) / 4;
    }
    
    u.tx_count_at_last_ka = u.tx_count;
    u.last_remote_rx_good = their_rx_good;
    
    ka_rx_count++;
    return true;
}

bool AP_Networking_Port_COBS::handle_data_frame_single(uint8_t uart_idx, const uint8_t *data, size_t len)
{
    uarts[uart_idx].rx_good++;
    uarts[uart_idx].last_rx_ms = AP_HAL::millis();
    hub->route_frame(this, data, len);
    rx_count++;
    return true;
}

bool AP_Networking_Port_COBS::handle_data_frame_ganged(uint8_t uart_idx, uint16_t seq,
                                                        const uint8_t *data, size_t len)
{
    // Track rx_good for keepalive stats
    uarts[uart_idx].rx_good++;
    uarts[uart_idx].last_rx_ms = AP_HAL::millis();
    
    // Lazy allocation of reorder buffer
    if (!ensure_reorder_buffer()) {
        // Fallback: deliver immediately without reordering
        hub->route_frame(this, data, len);
        rx_count++;
        return true;
    }
    
    WITH_SEMAPHORE(rx_sem);
    
    // Check if seq is in acceptable window
    int16_t diff = (int16_t)(seq - rx_expected_seq);
    if (diff < -(reorder_window/2) || diff >= reorder_window) {
        // Out of window
        if (diff > 0 && diff < reorder_window * 2) {
            // Frame is ahead - advance window
            rx_expected_seq = seq;
            reorder_timeout_count++;
        } else {
            rx_errors++;
            return false;
        }
    }
    
    uint8_t slot_idx = seq & reorder_mask;
    ReorderSlot &slot = reorder_buffer[slot_idx];
    
    if (slot.occupied && slot.seq != seq) {
        // Slot occupied by different seq - deliver old, replace
        hub->route_frame(this, slot.frame, slot.len);
        rx_count++;
        reorder_count++;
    }
    
    // Store frame
    memcpy(slot.frame, data, len);
    slot.len = len;
    slot.seq = seq;
    slot.occupied = true;
    
    // Deliver in-order frames
    deliver_reordered_frames();
    
    return true;
}

bool AP_Networking_Port_COBS::ensure_reorder_buffer()
{
    if (reorder_buffer != nullptr) {
        return true;
    }
    
    // Calculate window size: 2 * num_uarts, rounded up to power of 2, min 4
    uint8_t needed = MAX(MIN_REORDER_SLOTS, num_uarts * 2);
    
    // Round up to power of 2
    reorder_window = 4;
    while (reorder_window < needed) {
        reorder_window *= 2;
    }
    reorder_mask = reorder_window - 1;
    
    reorder_buffer = NEW_NOTHROW ReorderSlot[reorder_window];
    if (reorder_buffer == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: COBS reorder buffer alloc failed");
        reorder_window = 0;
        reorder_mask = 0;
        return false;
    }
    
    // Initialize slots
    for (uint8_t i = 0; i < reorder_window; i++) {
        reorder_buffer[i].occupied = false;
    }
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: COBS reorder buffer: %u slots", reorder_window);
    return true;
}

void AP_Networking_Port_COBS::deliver_reordered_frames()
{
    if (reorder_buffer == nullptr) return;
    
    while (true) {
        uint8_t slot_idx = rx_expected_seq & reorder_mask;
        ReorderSlot &slot = reorder_buffer[slot_idx];
        
        if (!slot.occupied || slot.seq != rx_expected_seq) {
            break;
        }
        
        hub->route_frame(this, slot.frame, slot.len);
        rx_count++;
        
        slot.occupied = false;
        rx_expected_seq++;
    }
}

void AP_Networking_Port_COBS::check_keepalive_tx()
{
    uint32_t now = AP_HAL::millis();
    
    for (uint8_t i = 0; i < num_uarts; i++) {
        UARTState &u = uarts[i];
        if (u.uart == nullptr) continue;
        
        // Flood keepalives on DISABLED UARTs for recovery stats
        uint16_t interval = (u.health == UARTState::Health::DISABLED) 
                            ? KEEPALIVE_FLOOD_INTERVAL_MS 
                            : KEEPALIVE_INTERVAL_MS;
        
        if ((now - u.last_keepalive_tx_ms) >= interval) {
            send_keepalive(i);
            u.last_keepalive_tx_ms = now;
        }
    }
}

void AP_Networking_Port_COBS::send_keepalive(uint8_t uart_idx)
{
    UARTState &u = uarts[uart_idx];
    if (u.uart == nullptr) return;
    
    // Serialize use of tx_encode_buffer
    if (!tx_sem.take(1)) return;
    
    // Build keepalive using shared protocol helper
    uint8_t ka_with_crc[KA_TOTAL_LEN];
    size_t ka_len = build_keepalive(ka_with_crc, sizeof(ka_with_crc), local_device_id, u.rx_good);
    if (ka_len == 0) {
        tx_sem.give();
        return;
    }
    
    // COBS encode with delimiter
    size_t enc_len = encode_frame(ka_with_crc, ka_len, tx_encode_buffer, sizeof(tx_encode_buffer));
    if (enc_len == 0) {
        tx_sem.give();
        return;
    }
    
    if (u.uart->txspace() >= enc_len) {
        u.uart->write(tx_encode_buffer, enc_len);
        ka_tx_count++;
    }
    
    tx_sem.give();
}

void AP_Networking_Port_COBS::update()
{
    // Thread handles all processing
}

void AP_Networking_Port_COBS::deliver_frame(const uint8_t *frame, size_t len)
{
    if (frame == nullptr || len == 0 || len > MAX_FRAME) {
        return;
    }
    
    WITH_SEMAPHORE(tx_sem);
    
    size_t input_len;
    uint8_t uart_idx;
    
    if (state == State::SINGLE) {
        // Non-ganged: frame + CRC32(frame)
        memcpy(tx_input_buffer, frame, len);
        uint32_t crc = crc_crc32(0, frame, len);
        memcpy(&tx_input_buffer[len], &crc, 4);
        input_len = len + 4;
        uart_idx = 0;
    } else {
        // Ganged: seq + frame + CRC32(seq + frame + "GANG")
        uint16_t seq = tx_seq++;
        memcpy(tx_input_buffer, &seq, 2);
        memcpy(&tx_input_buffer[2], frame, len);
        
        uint32_t crc = crc_crc32(0, tx_input_buffer, 2 + len);
        crc = crc_crc32(crc, GANG_CRC_SUFFIX, 4);
        memcpy(&tx_input_buffer[2 + len], &crc, 4);
        input_len = 2 + len + 4;
        
        // Select UART based on earliest completion time
        uart_idx = select_tx_uart(input_len + (input_len / 254U) + 2U);
    }
    
    // COBS encode with delimiter
    size_t enc_len = encode_frame(tx_input_buffer, input_len, tx_encode_buffer, sizeof(tx_encode_buffer));
    if (enc_len == 0) return;
    
    UARTState &u = uarts[uart_idx];
    
    // Send
    if (u.uart->txspace() >= enc_len) {
        u.uart->write(tx_encode_buffer, enc_len);
        u.tx_count++;
        tx_count++;
    }
}

uint8_t AP_Networking_Port_COBS::select_tx_uart(size_t frame_size)
{
    if (num_uarts == 1) {
        return 0;
    }
    
    uint8_t best_uart = 0;
    uint32_t best_completion_us = UINT32_MAX;
    bool found_any = false;
    
    for (uint8_t i = 0; i < num_uarts; i++) {
        UARTState &u = uarts[i];
        
        // Skip disabled UARTs
        if (u.health == UARTState::Health::DISABLED) {
            continue;
        }
        
        // Skip UARTs without enough space
        uint32_t space = u.uart->txspace();
        if (space < frame_size) {
            continue;
        }
        
        // Estimate bytes queued
        uint32_t bytes_queued = (space < TX_BUFFER_SIZE) ? (TX_BUFFER_SIZE - space) : 0;
        uint32_t total_bytes = bytes_queued + frame_size;
        
        // completion_time_us = total_bytes * 10 * 1000000 / baud
        uint32_t completion_us = (uint32_t)((uint64_t)total_bytes * 10000000ULL / u.baud);
        
        if (completion_us < best_completion_us) {
            best_completion_us = completion_us;
            best_uart = i;
            found_any = true;
        }
    }
    
    // If no UART had space, return first non-disabled
    if (!found_any) {
        for (uint8_t i = 0; i < num_uarts; i++) {
            if (uarts[i].health != UARTState::Health::DISABLED) {
                return i;
            }
        }
        return 0;
    }
    
    return best_uart;
}

bool AP_Networking_Port_COBS::can_receive() const
{
    // Accept if any UART has space for worst-case encoded frame
    const size_t worst = (MAX_FRAME + 4) + ((MAX_FRAME + 4) / 254U) + 3U;
    for (uint8_t i = 0; i < num_uarts; i++) {
        if (uarts[i].uart != nullptr && 
            uarts[i].health == UARTState::Health::GOOD &&
            uarts[i].uart->txspace() >= worst) {
            return true;
        }
    }
    return false;
}

bool AP_Networking_Port_COBS::is_link_up() const
{
    const uint32_t now = AP_HAL::millis();
    for (uint8_t i = 0; i < num_uarts; i++) {
        if (uarts[i].device_id_valid && 
            (now - uarts[i].last_rx_ms) <= (KEEPALIVE_INTERVAL_MS * 4U)) {
            return true;
        }
    }
    return false;
}

uint8_t AP_Networking_Port_COBS::get_best_loss_rate() const
{
    uint8_t best = 255;
    for (uint8_t i = 0; i < num_uarts; i++) {
        if (uarts[i].device_id_valid && uarts[i].loss_rate_u8 < best) {
            best = uarts[i].loss_rate_u8;
        }
    }
    return best;
}

void AP_Networking_Port_COBS::update_uart_health()
{
    if (num_uarts < 2) {
        // Single UART - always use it regardless of loss
        if (num_uarts == 1) {
            uarts[0].health = UARTState::Health::GOOD;
        }
        return;
    }
    
    uint8_t best_loss = get_best_loss_rate();
    uint8_t active_count = 0;
    
    for (uint8_t i = 0; i < num_uarts; i++) {
        UARTState &u = uarts[i];
        if (!u.device_id_valid) continue;
        
        // Need minimum samples before judging
        if (u.tx_count < MIN_SAMPLE_SIZE) continue;
        
        // Convert loss_rate_u8 to percentage (0-100)
        uint8_t loss_pct = u.loss_rate_u8 * 100 / 255;
        uint8_t best_pct = best_loss * 100 / 255;
        
        if (u.health == UARTState::Health::GOOD) {
            // Disable if significantly worse than best link
            if (loss_pct > best_pct * LOSS_RATE_DISABLE_MARGIN && 
                loss_pct > LOSS_RATE_DISABLE_PCT) {
                u.health = UARTState::Health::DISABLED;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, 
                              "NET: COBS UART %u disabled (%u%% loss, best=%u%%)", 
                              i, loss_pct, best_pct);
            }
        } else {
            // Re-enable if loss improves significantly (hysteresis)
            if (loss_pct <= LOSS_RATE_ENABLE_PCT ||
                (loss_pct <= best_pct * 2 && loss_pct <= LOSS_RATE_ENABLE_RELATIVE_PCT)) {
                u.health = UARTState::Health::GOOD;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, 
                              "NET: COBS UART %u re-enabled (%u%% loss)", i, loss_pct);
            }
        }
        
        if (u.health == UARTState::Health::GOOD) {
            active_count++;
        }
    }
    
    // State transition if needed
    if (active_count <= 1 && state == State::GANGED) {
        state = State::SINGLE;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: COBS returning to single mode");
    }
}

void AP_Networking_Port_COBS::check_uart_timeouts()
{
    uint32_t now = AP_HAL::millis();
    
    for (uint8_t i = 0; i < num_uarts; i++) {
        UARTState &u = uarts[i];
        
        if (u.device_id_valid && (now - u.last_rx_ms) > KEEPALIVE_TIMEOUT_MS) {
            u.device_id_valid = false;
            u.health = UARTState::Health::DISABLED;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NET: COBS UART %u timeout", i);
        }
    }
    
    // Check if ALL UARTs have timed out
    bool any_valid = false;
    for (uint8_t i = 0; i < num_uarts; i++) {
        if (uarts[i].device_id_valid) {
            any_valid = true;
            break;
        }
    }
    
    if (!any_valid && remote_id_known) {
        // All links dead - reset port for potential new device
        remote_id_known = false;
        state = State::SINGLE;
        tx_seq = 0;
        rx_expected_seq = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: COBS port reset, awaiting device");
    }
}

#endif // AP_NETWORKING_BACKEND_HUB_PORT_COBS
