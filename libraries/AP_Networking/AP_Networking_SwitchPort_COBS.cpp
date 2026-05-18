#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_SWITCHPORT_COBS

#include "AP_Networking_SwitchPort_COBS.h"
#include "AP_Networking_Switch.h"
#include "AP_Networking.h"
#include "AP_Networking_COBS_Protocol.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <string.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// Conditional debug macro for COBS reorder debugging
#if defined(HAL_BUILD_AP_PERIPH)
extern bool periph_debug_switch_pkt_enabled();
#define COBS_REORDER_DEBUG(fmt, args...) do { if (periph_debug_switch_pkt_enabled()) GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, fmt, ##args); } while(0)
#else
#define COBS_REORDER_DEBUG(fmt, args...) do { if (AP::network().option_is_set(AP_Networking::OPTION::DEBUG_SWITCH_PKT)) GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, fmt, ##args); } while(0)
#endif

using namespace AP_Networking_COBS_Protocol;

/*
 * Shared workspace - heap allocated on first COBS port init
 * 
 * TX buffers are shared (serialized by tx_sem).
 * RX frame buffer is shared (single-threaded in cobs_thread, used after decode).
 * 
 * Note: decoder buffer is per-port because partial frames can span multiple
 * process_rx() calls, and different ports would corrupt each other.
 */
struct AP_Networking_COBS_Link::SharedWorkspace {
    // TX state (protected by tx_sem)
    HAL_Semaphore tx_sem;
    uint8_t tx_encode_buffer[1600];
    uint8_t tx_input_buffer[MAX_FRAME + 4];  // frame + CRC(4)
    
    // RX frame buffer (single-threaded, used after decode completes)
    uint8_t rx_frame_buffer[MAX_FRAME + 4];  // frame + CRC(4)
};

// Static workspace pointer
AP_Networking_COBS_Link::SharedWorkspace *AP_Networking_COBS_Link::workspace = nullptr;

bool AP_Networking_COBS_Link::ensure_workspace()
{
    if (workspace != nullptr) {
        return true;
    }
    
    workspace = NEW_NOTHROW SharedWorkspace();
    if (workspace == nullptr) {
        return false;
    }
    
    return true;
}

AP_Networking_COBS_Link::AP_Networking_COBS_Link(AP_Networking_Switch *hub_in,
        AP_HAL::UARTDriver *uart_in,
        uint32_t baud_rate,
        const uint8_t device_id[6]) :
    hub(hub_in),
    uart(uart_in),
    baud(baud_rate)
{
    memcpy(local_device_id, device_id, 6);
}

AP_Networking_COBS_Link::~AP_Networking_COBS_Link()
{
}

bool AP_Networking_COBS_Link::init()
{
    if (hub == nullptr || uart == nullptr) {
        return false;
    }
    
    // Ensure shared workspace is allocated
    if (!ensure_workspace()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: COBS workspace alloc failed");
        return false;
    }
    
    // Set decoder to use per-port buffer (can't share - partial frames would corrupt)
    decoder.set_buffer(decoder_buffer, sizeof(decoder_buffer));
    
    // Note: uart->begin(0) is called from thread_update() to take ownership
    // from the correct thread context
    
    const uint32_t now_ms = AP_HAL::millis();
    last_rx_ms = 0;  // Start with link down
    last_keepalive_tx_ms = now_ms;
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: COBS port init OK");
    return true;
}

bool AP_Networking_COBS_Link::has_remote_device_id() const
{
    return remote_id_known;
}

bool AP_Networking_COBS_Link::get_remote_device_id(uint8_t id_out[6]) const
{
    if (!remote_id_known) {
        return false;
    }
    memcpy(id_out, remote_device_id, 6);
    return true;
}

void AP_Networking_COBS_Link::thread_update()
{
    // Take UART ownership and configure buffers from this thread
    if (!uart_initialized && uart != nullptr) {
        uart->begin(baud, 2048, TX_BUFFER_SIZE);
        uart_initialized = true;
    }
    
    // Process RX (routes frames directly to hub)
    process_rx();
    
    // Maintenance operations
    check_keepalive_tx();
    check_uart_timeout();
}

void AP_Networking_COBS_Link::process_rx()
{
    if (uart == nullptr || workspace == nullptr) return;
    
    uint8_t chunk[256];
    const uint32_t start_us = AP_HAL::micros();
    
    while (true) {
        const uint32_t avail = uart->available();
        if (avail == 0) break;
        
        const uint32_t to_read = (avail < sizeof(chunk)) ? avail : sizeof(chunk);
        const auto nread = uart->read(chunk, to_read);
        if (nread <= 0) break;
        
        // Process each byte through decoder
        for (ssize_t i = 0; i < nread; i++) {
            if (decoder.process_byte(chunk[i])) {
                // Frame complete
                size_t frame_len = sizeof(workspace->rx_frame_buffer);
                if (!decoder.get_frame(workspace->rx_frame_buffer, &frame_len, 
                                       sizeof(workspace->rx_frame_buffer))) {
                    rx_errors++;
                    decoder.resync();
                    continue;
                }
                decoder.reset();
                if (!handle_decoded_frame(workspace->rx_frame_buffer, frame_len)) {
                    decoder.resync();
                }
            }
        }
        
        // Time limit (1ms max)
        if ((AP_HAL::micros() - start_us) > 1000U) break;
    }
}

bool AP_Networking_COBS_Link::handle_decoded_frame(const uint8_t *data, size_t len)
{
    const uint8_t *payload;
    size_t payload_len;
    
    // Use shared frame identification with hint from last received type
    // This avoids computing both CRCs when frame type matches expectation
    FrameType type = identify_frame(data, len, &payload, &payload_len, last_data_type);
    
    switch (type) {
    case FrameType::KEEPALIVE:
        return handle_keepalive(payload);
    
    case FrameType::DATA_SINGLE:
        // Single mode: route through bond (direct to hub)
        last_data_type = FrameType::DATA_SINGLE;
        return handle_data_frame_single(payload, payload_len);
    
    case FrameType::DATA_BONDED:
        // Bonded mode: extract sequence and route through bond's reorder buffer
        // Sequence is in first 2 bytes of original data (before payload)
        last_data_type = FrameType::DATA_BONDED;
        {
            uint16_t seq = get_bonded_seq(data);
            return handle_data_frame_sequenced(seq, payload, payload_len);
        }
    
    case FrameType::INVALID:
    default:
        crc_errors++;
        return false;
    }
}

bool AP_Networking_COBS_Link::handle_keepalive(const uint8_t *data)
{
    // Extract keepalive fields using shared helper
    uint8_t ka_device_id[6];
    uint16_t their_rx_good;
    parse_keepalive(data, ka_device_id, &their_rx_good);
    
    // Update state
    memcpy(seen_device_id, ka_device_id, 6);
    device_id_valid = true;
    last_rx_ms = AP_HAL::millis();
    
    // Establish remote if not yet known
    if (!remote_id_known) {
        memcpy(remote_device_id, ka_device_id, 6);
        remote_id_known = true;
    }
    
    ka_rx_count++;
    return true;
}

bool AP_Networking_COBS_Link::handle_data_frame_single(const uint8_t *data, size_t len)
{
    // Single mode: route directly to hub via bond
    if (len > MAX_FRAME || bond == nullptr) {
        rx_errors++;
        return false;
    }
    
    rx_good++;
    last_rx_ms = AP_HAL::millis();
    
    // Route directly (no locks held during RX processing)
    bond->route_rx_frame(data, len);
    rx_count++;
    return true;
}

bool AP_Networking_COBS_Link::handle_data_frame_sequenced(uint16_t seq, const uint8_t *data, size_t len)
{
    // Bonded mode: route through bond's reorder buffer
    // Bond will deliver directly if seq is next expected, otherwise buffer
    if (len > MAX_FRAME || bond == nullptr) {
        rx_errors++;
        return false;
    }
    
    rx_good++;
    last_rx_ms = AP_HAL::millis();
    
    // Route through bond's reorder buffer (delivers immediately if in sequence)
    bond->route_rx_frame_sequenced(seq, data, len);
    rx_count++;
    return true;
}

void AP_Networking_COBS_Link::check_keepalive_tx()
{
    if (uart == nullptr) return;
    
    const uint32_t now_ms = AP_HAL::millis();
    
    if ((now_ms - last_keepalive_tx_ms) >= KEEPALIVE_INTERVAL_MS) {
        if (send_keepalive()) {
            last_keepalive_tx_ms = now_ms;
        }
    }
}

bool AP_Networking_COBS_Link::send_keepalive()
{
    if (uart == nullptr || workspace == nullptr) return false;
    
    // Build keepalive into local buffer first
    uint8_t ka_with_crc[KA_TOTAL_LEN];
    size_t ka_len = build_keepalive(ka_with_crc, sizeof(ka_with_crc), local_device_id, rx_good);
    if (ka_len == 0) {
        return false;
    }
    
    // Try to take semaphore (non-blocking) - skip keepalive if busy
    if (!workspace->tx_sem.take_nonblocking()) {
        return false;
    }
    
    // COBS encode with delimiter
    size_t enc_len = encode_frame(ka_with_crc, ka_len, 
                                   workspace->tx_encode_buffer, 
                                   sizeof(workspace->tx_encode_buffer));
    
    bool sent = false;
    if (enc_len > 0 && uart->txspace() >= enc_len) {
        uart->write(workspace->tx_encode_buffer, enc_len);
        ka_tx_count++;
        sent = true;
    }
    
    workspace->tx_sem.give();
    return sent;
}

void AP_Networking_COBS_Link::deliver_frame(const uint8_t *frame, size_t len)
{
    // Single mode: frame + CRC32(frame)
    // No sequence number overhead - used for single-UART bonds
    if (frame == nullptr || len == 0 || len > MAX_FRAME || workspace == nullptr) {
        return;
    }
    
    WITH_SEMAPHORE(workspace->tx_sem);
    
    memcpy(workspace->tx_input_buffer, frame, len);
    uint32_t crc = crc_crc32(0, frame, len);
    memcpy(&workspace->tx_input_buffer[len], &crc, 4);
    size_t input_len = len + 4;
    
    // COBS encode with delimiter
    size_t enc_len = encode_frame(workspace->tx_input_buffer, input_len, 
                                   workspace->tx_encode_buffer, 
                                   sizeof(workspace->tx_encode_buffer));
    if (enc_len == 0) return;
    
    // Send
    if (uart != nullptr && uart->txspace() >= enc_len) {
        uart->write(workspace->tx_encode_buffer, enc_len);
        tx_count++;
    }
}

void AP_Networking_COBS_Link::deliver_frame_sequenced(const uint8_t *frame, size_t len, uint16_t seq)
{
    // Bonded mode: seq[2] + frame + CRC32(seq + frame + "BOND")
    // Sequence numbers allow receiver to reorder frames from multiple UARTs
    if (frame == nullptr || len == 0 || len > MAX_FRAME || workspace == nullptr) {
        return;
    }
    
    WITH_SEMAPHORE(workspace->tx_sem);
    
    // Build: seq[2] + frame
    workspace->tx_input_buffer[0] = seq & 0xFF;
    workspace->tx_input_buffer[1] = (seq >> 8) & 0xFF;
    memcpy(&workspace->tx_input_buffer[2], frame, len);
    size_t data_len = 2 + len;
    
    // CRC32(seq + frame + "BOND") - the suffix distinguishes from single mode
    uint32_t crc = crc_crc32(0, workspace->tx_input_buffer, data_len);
    crc = crc_crc32(crc, AP_Networking_COBS_Protocol::BOND_CRC_SUFFIX, 4);
    memcpy(&workspace->tx_input_buffer[data_len], &crc, 4);
    size_t input_len = data_len + 4;
    
    // COBS encode with delimiter
    size_t enc_len = encode_frame(workspace->tx_input_buffer, input_len, 
                                   workspace->tx_encode_buffer, 
                                   sizeof(workspace->tx_encode_buffer));
    if (enc_len == 0) return;
    
    // Send
    if (uart != nullptr && uart->txspace() >= enc_len) {
        uart->write(workspace->tx_encode_buffer, enc_len);
        tx_count++;
    }
}

bool AP_Networking_COBS_Link::can_receive() const
{
    if (uart == nullptr) return false;
    
    // Accept if UART has space for worst-case encoded frame
    // frame + CRC(4), then COBS overhead + delimiter
    const size_t worst = (MAX_FRAME + 4) + ((MAX_FRAME + 4) / 254U) + 3U;
    return uart->txspace() >= worst;
}

bool AP_Networking_COBS_Link::is_link_up() const
{
    const uint32_t now_ms = AP_HAL::millis();
    return device_id_valid && (now_ms - last_rx_ms) <= (KEEPALIVE_INTERVAL_MS * 4U);
}

uint32_t AP_Networking_COBS_Link::get_tx_completion_us() const
{
    if (uart == nullptr || baud == 0) {
        return UINT32_MAX;
    }
    
    // Calculate bytes pending in TX buffer
    const uint32_t tx_space = uart->txspace();
    const uint32_t bytes_pending = (tx_space < TX_BUFFER_SIZE) ? (TX_BUFFER_SIZE - tx_space) : 0;
    
    // Convert to microseconds: bytes * 10 bits/byte * 1000000 us/s / baud
    // Simplified: bytes * 10000000 / baud
    // To avoid overflow, compute as: bytes * (10000000 / baud) with rounding
    return (uint32_t)((uint64_t)bytes_pending * 10000000ULL / baud);
}

void AP_Networking_COBS_Link::check_uart_timeout()
{
    const uint32_t now_ms = AP_HAL::millis();
    
    if (device_id_valid && (now_ms - last_rx_ms) > KEEPALIVE_TIMEOUT_MS) {
        device_id_valid = false;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NET: COBS timeout");
    }
    
    // Reset port if link is dead
    if (!device_id_valid && remote_id_known) {
        remote_id_known = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: COBS port reset, awaiting device");
    }
}

// ============================================================================
// AP_Networking_SwitchPort_COBS implementation
// ============================================================================

AP_Networking_SwitchPort_COBS::AP_Networking_SwitchPort_COBS(AP_Networking_Switch *hub_in, uint8_t bond_id_in) :
    hub(hub_in),
    bond_id(bond_id_in)
{
    memset(members, 0, sizeof(members));
    // reorder_buffer is allocated lazily in add_member() when num_members > 1
}

AP_Networking_SwitchPort_COBS::~AP_Networking_SwitchPort_COBS()
{
    // Delete all member ports
    for (uint8_t i = 0; i < num_members; i++) {
        delete members[i];
    }
    // Free reorder buffer if allocated
    delete[] reorder_buffer;
}

bool AP_Networking_SwitchPort_COBS::add_member(AP_Networking_COBS_Link *port)
{
    if (port == nullptr || num_members >= MAX_BOND_MEMBERS) {
        return false;
    }
    
    // Allocate reorder buffer when adding second member (enables sequenced mode)
    if (num_members == 1 && reorder_buffer == nullptr) {
        reorder_buffer = NEW_NOTHROW ReorderEntry[REORDER_BUFFER_SIZE];
        if (reorder_buffer == nullptr) {
            return false;  // Allocation failed
        }
        for (uint8_t i = 0; i < REORDER_BUFFER_SIZE; i++) {
            reorder_buffer[i].valid = false;
        }
    }
    
    // Set the port's bond pointer so it routes RX to us
    port->bond = this;
    
    members[num_members++] = port;
    return true;
}

void AP_Networking_SwitchPort_COBS::thread_update()
{
    // Update all member ports
    for (uint8_t i = 0; i < num_members; i++) {
        if (members[i] != nullptr) {
            members[i]->thread_update();
        }
    }
    
    // Check for reorder buffer timeout (only relevant for multi-member bonds)
    if (num_members > 1) {
        check_reorder_timeout();
    }
}

void AP_Networking_SwitchPort_COBS::deliver_frame(const uint8_t *frame, size_t len)
{
    if (num_members == 0) {
        return;
    }
#if AP_NETWORKING_CAPTURE_ENABLED
    capture.capture_frame(frame, len);
#endif
    
    // Single member: use simple mode without sequence numbers (no overhead)
    // Multiple members: use sequenced mode for proper reordering at receiver
    const bool use_sequencing = (num_members > 1);
    
    // Find the member with soonest TX completion time (lowest buffer depth / baud rate)
    // This naturally load-balances across UARTs with different speeds
    AP_Networking_COBS_Link *best_port = nullptr;
    uint32_t best_completion_us = UINT32_MAX;
    
    for (uint8_t i = 0; i < num_members; i++) {
        AP_Networking_COBS_Link *port = members[i];
        if (port == nullptr || !port->is_link_up() || !port->can_receive()) {
            continue;
        }
        
        uint32_t completion_us = port->get_tx_completion_us();
        if (completion_us < best_completion_us) {
            best_completion_us = completion_us;
            best_port = port;
        }
    }
    
    // If no member available, fall back to first member that's up (will buffer)
    if (best_port == nullptr) {
        for (uint8_t i = 0; i < num_members; i++) {
            if (members[i] != nullptr && members[i]->is_link_up()) {
                best_port = members[i];
                break;
            }
        }
    }
    
    if (best_port != nullptr) {
        if (use_sequencing) {
            best_port->deliver_frame_sequenced(frame, len, tx_seq++);
        } else {
            best_port->deliver_frame(frame, len);
        }
    }
}

bool AP_Networking_SwitchPort_COBS::can_receive() const
{
    // Can receive if any member is link up and can receive
    // This must match the criteria in deliver_frame() to avoid
    // silently dropping frames when link is down
    for (uint8_t i = 0; i < num_members; i++) {
        if (members[i] != nullptr && members[i]->is_link_up() && members[i]->can_receive()) {
            return true;
        }
    }
    return false;
}

bool AP_Networking_SwitchPort_COBS::is_link_up() const
{
    // Link is up if any member is up
    for (uint8_t i = 0; i < num_members; i++) {
        if (members[i] != nullptr && members[i]->is_link_up()) {
            return true;
        }
    }
    return false;
}

uint32_t AP_Networking_SwitchPort_COBS::get_rx_count() const
{
    uint32_t total = 0;
    for (uint8_t i = 0; i < num_members; i++) {
        if (members[i] != nullptr) {
            total += members[i]->get_rx_count();
        }
    }
    return total;
}

uint32_t AP_Networking_SwitchPort_COBS::get_tx_count() const
{
    uint32_t total = 0;
    for (uint8_t i = 0; i < num_members; i++) {
        if (members[i] != nullptr) {
            total += members[i]->get_tx_count();
        }
    }
    return total;
}

uint32_t AP_Networking_SwitchPort_COBS::get_crc_errors() const
{
    uint32_t total = 0;
    for (uint8_t i = 0; i < num_members; i++) {
        if (members[i] != nullptr) {
            total += members[i]->get_crc_errors();
        }
    }
    return total;
}

uint32_t AP_Networking_SwitchPort_COBS::get_keepalive_rx() const
{
    uint32_t total = 0;
    for (uint8_t i = 0; i < num_members; i++) {
        if (members[i] != nullptr) {
            total += members[i]->get_keepalive_rx();
        }
    }
    return total;
}

uint32_t AP_Networking_SwitchPort_COBS::get_keepalive_tx() const
{
    uint32_t total = 0;
    for (uint8_t i = 0; i < num_members; i++) {
        if (members[i] != nullptr) {
            total += members[i]->get_keepalive_tx();
        }
    }
    return total;
}

void AP_Networking_SwitchPort_COBS::route_rx_frame(const uint8_t *frame, size_t len)
{
    // Single mode: route directly to hub
#if AP_NETWORKING_CAPTURE_ENABLED
    capture.capture_frame(frame, len);
#endif
    hub->route_frame(this, frame, len);
}

void AP_Networking_SwitchPort_COBS::route_rx_frame_sequenced(uint16_t seq, const uint8_t *frame, size_t len)
{
    // Bonded mode: deliver in sequence, buffering out-of-order frames
    // Note: reorder_buffer is only allocated when num_members > 1
    if (reorder_buffer == nullptr) {
        // Shouldn't happen - sequenced frames only sent when num_members > 1
        // Fall back to direct routing
        hub->route_frame(this, frame, len);
        return;
    }
    
    // Fast path: if this is the next expected sequence, deliver directly
    if (seq == rx_seq_expected) {
        hub->route_frame(this, frame, len);
        rx_seq_expected++;
        reorder_stall_start_us = 0;  // Not stalled
        // Check if we can now deliver buffered frames
        deliver_reordered();
        return;
    }
    
    // Check if sequence is too old (already delivered or wrapped)
    // Use signed comparison to handle wraparound
    int16_t diff = (int16_t)(seq - rx_seq_expected);
    if (diff < 0) {
        // Check if remote has reset its sequence counter (large negative diff)
        // If so, resync to the new sequence
        if (diff < -REORDER_BUFFER_SIZE) {
            COBS_REORDER_DEBUG("COBS seq resync: got %u exp %u", seq, rx_seq_expected);
            rx_seq_expected = seq;
            // Clear reorder buffer - old frames are stale
            for (uint8_t i = 0; i < REORDER_BUFFER_SIZE; i++) {
                reorder_buffer[i].valid = false;
            }
            reorder_stall_start_us = 0;
            hub->route_frame(this, frame, len);
            rx_seq_expected++;
            return;
        }
        // Otherwise it's a duplicate, drop it
        return;
    }
    
    // Too far ahead? Drop to avoid stalling
    if (diff >= REORDER_BUFFER_SIZE) {
        // Skip ahead - we've lost frames
        rx_seq_expected = seq;
        // Clear buffer - old frames are now stale
        for (uint8_t i = 0; i < REORDER_BUFFER_SIZE; i++) {
            reorder_buffer[i].valid = false;
        }
        hub->route_frame(this, frame, len);
        rx_seq_expected++;
        reorder_stall_start_us = 0;  // Not stalled
        return;
    }
    
    // Buffer this frame for later delivery
    uint8_t slot = seq % REORDER_BUFFER_SIZE;
    if (!reorder_buffer[slot].valid && len <= MAX_FRAME) {
        memcpy(reorder_buffer[slot].frame, frame, len);
        reorder_buffer[slot].len = len;
        reorder_buffer[slot].seq = seq;
        reorder_buffer[slot].valid = true;
        
        // Track when we started waiting for the missing sequence
        if (reorder_stall_start_us == 0) {
            reorder_stall_start_us = AP_HAL::micros();
            if (reorder_stall_start_us == 0) {
                reorder_stall_start_us = 1;  // Avoid 0 meaning "not stalled"
            }
        }
    }
}

void AP_Networking_SwitchPort_COBS::deliver_reordered()
{
    // Deliver buffered frames that are now in sequence
    while (true) {
        uint8_t slot = rx_seq_expected % REORDER_BUFFER_SIZE;
        if (!reorder_buffer[slot].valid || reorder_buffer[slot].seq != rx_seq_expected) {
            break;
        }
        
#if AP_NETWORKING_CAPTURE_ENABLED
        capture.capture_frame(reorder_buffer[slot].frame, reorder_buffer[slot].len);
#endif
        hub->route_frame(this, reorder_buffer[slot].frame, reorder_buffer[slot].len);
        reorder_buffer[slot].valid = false;
        rx_seq_expected++;
    }
    
    // Check if we're still stalled
    bool still_stalled = false;
    for (uint8_t i = 0; i < REORDER_BUFFER_SIZE; i++) {
        if (reorder_buffer[i].valid) {
            still_stalled = true;
            break;
        }
    }
    if (!still_stalled) {
        reorder_stall_start_us = 0;
    }
}

void AP_Networking_SwitchPort_COBS::check_reorder_timeout()
{
    if (reorder_stall_start_us == 0 || reorder_buffer == nullptr) {
        return;  // Not stalled or no buffer
    }
    
    const uint32_t now_us = AP_HAL::micros();
    const uint32_t elapsed_us = now_us - reorder_stall_start_us;
    
    if (elapsed_us < REORDER_TIMEOUT_MS * 1000) {
        return;  // Not timed out yet
    }
    
    // Timeout - skip missing sequence(s) and deliver buffered frames
    // Find the lowest valid sequence in the buffer
    uint16_t min_seq = rx_seq_expected;
    bool found = false;
    for (uint8_t i = 0; i < REORDER_BUFFER_SIZE; i++) {
        if (reorder_buffer[i].valid) {
            int16_t diff = (int16_t)(reorder_buffer[i].seq - min_seq);
            if (!found || diff < 0) {
                min_seq = reorder_buffer[i].seq;
                found = true;
            }
        }
    }
    
    if (!found) {
        // No buffered frames, nothing to do
        reorder_stall_start_us = 0;
        return;
    }
    
    // Skip to the lowest buffered sequence and deliver
    COBS_REORDER_DEBUG("COBS reorder timeout: skip to %u", min_seq);
    rx_seq_expected = min_seq;
    reorder_stall_start_us = 0;
    deliver_reordered();
}

#endif // AP_NETWORKING_BACKEND_SWITCHPORT_COBS
