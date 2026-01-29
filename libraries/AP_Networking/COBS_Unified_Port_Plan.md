# Unified COBS Port with Automatic Ganging

## MAVLink COBS Tunneling

A separate `AP_Networking_Port_MAVLink_COBS` class provides COBS ethernet frame tunneling over MAVLink TUNNEL messages. This enables networking over long-range MAVLink links where direct UART COBS isn't available.

### Features

- COBS-encodes ethernet frames and sends as 128-byte TUNNEL payloads
- Assumes in-order MAVLink delivery (no sequencing/windowing)
- Feeds data directly into streaming COBS decoder on receive
- Two payload types distinguish frame boundaries

### Protocol

Two MAVLink TUNNEL payload types (experimental range):

| Payload Type | Value | Description |
|--------------|-------|-------------|
| `COBS_START` | 32769 | First chunk of a new frame (resets RX decoder) |
| `COBS_CONT`  | 32770 | Continuation chunk |

TX flow:
1. COBS-encode ethernet frame (adds 0x00 delimiter)
2. Split into ≤128-byte chunks
3. Send first chunk as `COBS_START`, rest as `COBS_CONT`

RX flow:
1. `COBS_START` resets streaming COBS decoder
2. All data fed into decoder byte-by-byte
3. Decoder delivers complete frames when 0x00 delimiter seen

### Configuration

Enable with `AP_NETWORKING_BACKEND_HUB_PORT_MAVLINK_COBS=1` in hwdef.

---

## Overview

A single `AP_Networking_Port_COBS` class that:
1. Starts immediately with one UART (no discovery delay)
2. Dynamically detects and absorbs additional UARTs connected to the same remote device
3. Automatically enables frame striping when multiple UARTs are present
4. Supports hotplug - detects device changes and splits/merges ports accordingly

**Replaces**: `AP_Networking_Port_COBS`, `AP_Networking_Port_COBS_Gang`, `AP_Networking_COBS_Discovery`

## Design Goals

- **Single COBS is maximally efficient** - zero overhead for the common case
- **No negotiation** - frames are self-describing via CRC technique
- **Automatic ganging** - ports dynamically combine when connected to same remote
- **Hotpluggable** - handles device replacement gracefully
- **Auto-disable bad UARTs** - uses RX stats relative to other links
- **Lazy allocation** - reorder buffer only created when needed

---

## Wire Protocol

### Keepalive

```
"KA" + device_id[6] + rx_good[2] + CRC32  →  COBS  →  + 0x00
```

**Total: 10 bytes before CRC, 14 bytes with CRC**

| Field | Offset | Size | Description |
|-------|--------|------|-------------|
| `"KA"` | 0 | 2 | Marker |
| `device_id` | 2 | 6 | Sender's MAC address |
| `rx_good` | 8 | 2 | Frames successfully received on this UART (LE, wrapping) |

**Timing**: Sent every 500ms on each UART, **regardless of other traffic**. This ensures consistent stats collection for link quality assessment.

### Data Frame - Single Mode

```
ethernet_frame + CRC32(ethernet_frame)  →  COBS  →  + 0x00
```

**Zero overhead** compared to raw ethernet.

### Data Frame - Ganged Mode

```
seq[2] + ethernet_frame + CRC32(seq + ethernet_frame + "GANG")  →  COBS  →  + 0x00
```

- `seq`: 16-bit sequence number, little-endian, wraps at 65535→0
- `"GANG"`: Included in CRC calculation but **not transmitted**

**Overhead**: 2 bytes per frame for sequence number.

---

## Frame Type Detection

Frames are self-describing. The receiver determines the frame type by trying both CRC calculations.

**Key optimization**: Calculate base CRC once, then extend it with "GANG" suffix for the second check.

```cpp
static const uint8_t GANG_CRC_SUFFIX[] = {'G', 'A', 'N', 'G'};
static const uint8_t COBS_KA_MARKER[] = {'K', 'A'};
static constexpr size_t COBS_KA_LEN = 10;       // Before CRC
static constexpr size_t MIN_ETH_FRAME = 60;     // Minimum ethernet (no FCS)

bool handle_decoded_frame(uint8_t uart_idx, const uint8_t *data, size_t len) {
    if (len < 4) return false;
    
    const size_t data_len = len - 4;
    uint32_t rx_crc;
    memcpy(&rx_crc, &data[data_len], 4);
    
    // Keepalive: exactly 10 bytes, starts with "KA"
    if (data_len == COBS_KA_LEN && memcmp(data, COBS_KA_MARKER, 2) == 0) {
        uint32_t calc_crc = crc_crc32(0, data, data_len);
        if (rx_crc == calc_crc) {
            return handle_keepalive(uart_idx, data);
        }
        return false;
    }
    
    // Data frame - calculate base CRC once
    if (data_len >= MIN_ETH_FRAME) {
        uint32_t base_crc = crc_crc32(0, data, data_len);
        
        // Try non-ganged CRC first (more common for single-UART setups)
        if (rx_crc == base_crc) {
            return handle_data_frame_single(uart_idx, data, data_len);
        }
        
        // Try ganged CRC: base_crc extended with "GANG"
        uint32_t gang_crc = crc_crc32(base_crc, GANG_CRC_SUFFIX, 4);
        if (rx_crc == gang_crc) {
            if (data_len < 2 + MIN_ETH_FRAME) return false;  // seq + min eth frame
            uint16_t seq = data[0] | (data[1] << 8);
            return handle_data_frame_ganged(uart_idx, seq, &data[2], data_len - 2);
        }
    }
    
    crc_errors++;
    return false;
}
```

**CRC collision risk**: ~1 in 2³² per frame - negligible.

---

## State Machine

Simple two-state machine based on UART count:

```
┌────────┐                              ┌────────┐
│ SINGLE │  ◄────── num_uarts == 1 ─────│ GANGED │
│        │  ─────── num_uarts >= 2 ────►│        │
└────────┘                              └────────┘
```

| State | TX Format | RX Accepts | TX Strategy |
|-------|-----------|------------|-------------|
| SINGLE | Non-ganged | Both | Single UART |
| GANGED | Ganged (seq) | Both | Earliest completion |

No negotiation or intermediate states - transitions are instant.

---

## TX UART Selection Strategy

### Goal

In ganged mode, select the UART that will **complete transmission soonest**, accounting for:
1. Bytes already queued in TX buffer
2. UART baud rate
3. Encoded frame size

This naturally load-balances across UARTs with different speeds and adapts to congestion.

### Calculation

For each eligible UART, calculate **completion time**:

```
bytes_queued = tx_buffer_size - txspace()
total_bytes = bytes_queued + encoded_frame_size
completion_time_us = total_bytes * 10 * 1000000 / baud_rate
```

(10 bits per byte for 8N1 serial)

Select UART with **minimum completion time**.

### Implementation

```cpp
uint8_t AP_Networking_Port_COBS::select_tx_uart(size_t frame_size) {
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
        
        // Estimate bytes queued (TX_BUFFER_SIZE - txspace)
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
    
    // If no UART had space, return first non-disabled (will block/drop)
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
```

### Example Scenarios

| UART | Baud | TX Queued | Frame Size | Completion Time | Selected |
|------|------|-----------|------------|-----------------|----------|
| 0 | 115200 | 1000 B | 1600 B | 226 ms | |
| 1 | 921600 | 2000 B | 1600 B | 39 ms | ✓ |
| 2 | 115200 | 0 B | 1600 B | 139 ms | |

UART 1 wins despite having more queued bytes because its 8x higher baud rate more than compensates.

| UART | Baud | TX Queued | Frame Size | Completion Time | Selected |
|------|------|-----------|------------|-----------------|----------|
| 0 | 3000000 | 8000 B | 1600 B | 32 ms | ✓ |
| 1 | 3000000 | 10000 B | 1600 B | 39 ms | |

Equal baud rates - UART 0 wins due to less queued data.

### Benefits over Round-Robin

1. **Adapts to asymmetric baud rates**: Faster UARTs naturally get more frames
2. **Adapts to congestion**: Avoids UARTs with full buffers
3. **Minimizes latency**: Frames complete transmission sooner
4. **Self-balancing**: Load distributes based on actual capacity

---

## Class Interface

```cpp
class AP_Networking_Port_COBS : public AP_Networking_HubPort
{
public:
    static constexpr uint8_t MAX_UARTS = 4;
    
    AP_Networking_Port_COBS(AP_Networking_Hub *hub,
                            AP_HAL::UARTDriver *uart,
                            uint32_t baud_rate,
                            const uint8_t local_device_id[6]);
    ~AP_Networking_Port_COBS() {
        delete[] reorder_buffer;
    }
    
    CLASS_NO_COPY(AP_Networking_Port_COBS);
    
    bool init();
    void update() override;
    
    // AP_Networking_HubPort interface
    void deliver_frame(const uint8_t *frame, size_t len) override;
    bool can_receive() const override;
    const char *get_name() const override { return "COBS"; }
    bool is_link_up() const override;
    
    // Gang management (called by hub)
    bool add_uart(AP_HAL::UARTDriver *uart, uint32_t baud);
    bool extract_uart(uint8_t idx, AP_HAL::UARTDriver *&uart_out, uint32_t &baud_out);
    
    // Discovery info (for hub to match ports)
    bool get_remote_device_id(uint8_t id_out[6]) const;
    bool has_remote_device_id() const { return remote_id_known; }
    
    // Status
    uint8_t get_num_uarts() const { return num_uarts; }
    bool is_ganged() const { return state == State::GANGED; }
    AP_HAL::UARTDriver *get_uart(uint8_t idx) const;
    uint32_t get_baud(uint8_t idx) const;
    
    // Statistics
    uint32_t get_rx_count() const { return rx_count; }
    uint32_t get_tx_count() const { return tx_count; }
    uint32_t get_rx_errors() const { return rx_errors; }
    uint32_t get_reorder_count() const { return reorder_count; }

private:
    AP_Networking_Hub *hub;
    uint8_t local_device_id[6];
    
    enum class State : uint8_t {
        SINGLE,     // 1 UART, non-sequenced frames
        GANGED      // 2+ UARTs, sequenced frames, earliest-completion TX, reorder RX
    };
    State state = State::SINGLE;
    
    // Per-UART state
    struct UARTState {
        AP_HAL::UARTDriver *uart;
        uint32_t baud;
        AP_Networking_COBS::Decoder decoder;
        
        // Link status
        uint8_t seen_device_id[6];
        bool device_id_valid;
        uint32_t last_rx_ms;
        uint32_t last_keepalive_tx_ms;
        
        // Stats for link quality (counters wrap, differences used)
        uint16_t tx_count;              // Frames we sent on this UART
        uint16_t tx_count_at_last_ka;   // tx_count when last keepalive received
        uint16_t last_remote_rx_good;   // rx_good from last keepalive
        uint16_t lost_since_last_ka;    // Frames lost in last measurement period
        
        // Rolling loss rate (0-255 scaled, where 255 = 100% loss)
        uint8_t loss_rate_u8;
        
        // Health state
        enum class Health : uint8_t {
            GOOD,       // Normal operation, 500ms keepalives
            DISABLED    // Not used for data TX, 50ms keepalives for recovery
        };
        Health health;
        
        uint16_t rx_good;  // Data frames we received on this UART (for keepalive stats)
    };
    UARTState uarts[MAX_UARTS];
    uint8_t num_uarts = 0;
    
    // Remote device (learned from keepalives)
    uint8_t remote_device_id[6];
    bool remote_id_known = false;
    
    // TX state (ganged mode)
    uint16_t tx_seq = 0;
    HAL_Semaphore tx_sem;
    
    // TX buffers
    static constexpr size_t MAX_FRAME = 1522;
    uint8_t tx_encode_buffer[1600];
    uint8_t tx_input_buffer[MAX_FRAME + 6];  // seq(2) + frame + CRC(4)
    
    // RX reorder buffer (ganged mode) - lazily allocated, sized to num_uarts
    // Formula: 2 * num_uarts slots (handles 2x latency variance)
    static constexpr uint8_t MIN_REORDER_SLOTS = 4;
    struct ReorderSlot {
        uint8_t frame[MAX_FRAME];
        size_t len;
        uint16_t seq;
        bool occupied;
    };
    ReorderSlot *reorder_buffer = nullptr;  // Allocated on first ganged RX
    uint8_t reorder_window = 0;             // Actual allocated size (power of 2)
    uint8_t reorder_mask = 0;               // reorder_window - 1
    uint16_t rx_expected_seq = 0;
    HAL_Semaphore rx_sem;
    
    // RX buffer
    uint8_t rx_frame_buffer[MAX_FRAME + 6];
    
    // Statistics
    uint32_t rx_count = 0;
    uint32_t tx_count = 0;
    uint32_t rx_errors = 0;
    uint32_t crc_errors = 0;
    uint32_t reorder_count = 0;
    uint32_t reorder_timeout_count = 0;
    
    // Constants
    static constexpr uint16_t KEEPALIVE_INTERVAL_MS = 500;
    static constexpr uint16_t KEEPALIVE_FLOOD_INTERVAL_MS = 50;  // When disabled
    static constexpr uint16_t KEEPALIVE_TIMEOUT_MS = 2000;
    static constexpr uint8_t LOSS_RATE_DISABLE_PCT = 15;         // Disable above this AND...
    static constexpr uint8_t LOSS_RATE_DISABLE_MARGIN = 3;       // ...if 3x worse than best
    static constexpr uint8_t LOSS_RATE_ENABLE_PCT = 5;           // Re-enable below this, OR...
    static constexpr uint8_t LOSS_RATE_ENABLE_RELATIVE_PCT = 10; // ...below this if ≤2x best
    static constexpr uint16_t MIN_SAMPLE_SIZE = 20;              // Frames before judging
    
    // TX buffer size (for completion time calculation)
    static constexpr uint32_t TX_BUFFER_SIZE = 16384;
    
    // Methods
    void thread();
    void process_rx(uint8_t uart_idx);
    bool handle_decoded_frame(uint8_t uart_idx, const uint8_t *data, size_t len);
    void handle_keepalive(uint8_t uart_idx, const uint8_t *data);
    bool handle_data_frame_single(uint8_t uart_idx, const uint8_t *data, size_t len);
    bool handle_data_frame_ganged(uint8_t uart_idx, uint16_t seq, const uint8_t *data, size_t len);
    void send_keepalive(uint8_t uart_idx);
    void check_keepalive_tx();
    bool ensure_reorder_buffer();
    void deliver_reordered_frames();
    void update_uart_health();
    void check_uart_timeouts();
    uint8_t select_tx_uart(size_t frame_size);  // Select UART with earliest completion
    uint8_t get_best_loss_rate() const;
};
```

---

## TX Implementation

```cpp
void AP_Networking_Port_COBS::deliver_frame(const uint8_t *frame, size_t len) {
    if (frame == nullptr || len == 0 || len > MAX_FRAME) {
        return;
    }
    
    WITH_SEMAPHORE(tx_sem);
    
    size_t input_len;
    
    if (state == State::SINGLE) {
        // Non-ganged: frame + CRC32(frame)
        memcpy(tx_input_buffer, frame, len);
        uint32_t crc = crc_crc32(0, frame, len);
        memcpy(&tx_input_buffer[len], &crc, 4);
        input_len = len + 4;
    } else {
        // Ganged: seq + frame + CRC32(seq + frame + "GANG")
        uint16_t seq = tx_seq++;
        memcpy(tx_input_buffer, &seq, 2);
        memcpy(&tx_input_buffer[2], frame, len);
        
        uint32_t crc = crc_crc32(0, tx_input_buffer, 2 + len);
        crc = crc_crc32(crc, GANG_CRC_SUFFIX, 4);
        memcpy(&tx_input_buffer[2 + len], &crc, 4);
        input_len = 2 + len + 4;
    }
    
    // COBS encode
    size_t enc_len = AP_Networking_COBS::encode(tx_input_buffer, input_len, 
                                                 tx_encode_buffer, sizeof(tx_encode_buffer) - 1);
    if (enc_len == 0) return;
    tx_encode_buffer[enc_len] = 0;  // Delimiter
    
    // Select UART based on earliest completion time
    uint8_t uart_idx = (state == State::SINGLE) ? 0 : select_tx_uart(enc_len + 1);
    UARTState &u = uarts[uart_idx];
    
    // Send
    if (u.uart->txspace() >= enc_len + 1) {
        u.uart->write(tx_encode_buffer, enc_len + 1);
        u.tx_count++;
        tx_count++;
    }
}
```

---

## RX Frame Handling

Data frames must increment `rx_good` for the UART they arrived on (for keepalive stats):

```cpp
bool AP_Networking_Port_COBS::handle_data_frame_single(uint8_t uart_idx, 
                                                        const uint8_t *data, size_t len) {
    uarts[uart_idx].rx_good++;
    hub->route_frame(this, data, len);
    rx_count++;
    uarts[uart_idx].last_rx_ms = AP_HAL::millis();
    return true;
}
```

For ganged frames, `rx_good` is incremented in `handle_data_frame_ganged()` before reorder buffering.

---

## Add UART Implementation

Called by hub when merging ports with same remote device:

```cpp
bool AP_Networking_Port_COBS::add_uart(AP_HAL::UARTDriver *uart, uint32_t baud) {
    if (num_uarts >= MAX_UARTS || uart == nullptr) {
        return false;
    }
    
    UARTState &u = uarts[num_uarts];
    memset(&u, 0, sizeof(u));
    u.uart = uart;
    u.baud = baud;
    u.health = UARTState::Health::GOOD;
    u.decoder.reset();
    
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
```

---

## Keepalive Implementation

### TX - Periodic on All UARTs

Keepalives are sent **every 500ms regardless of other traffic** to ensure consistent stats.
When a UART is DISABLED, keepalives are sent more frequently (every 50ms) to gather recovery statistics faster.

```cpp
void AP_Networking_Port_COBS::check_keepalive_tx() {
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

void AP_Networking_Port_COBS::send_keepalive(uint8_t uart_idx) {
    UARTState &u = uarts[uart_idx];
    if (u.uart == nullptr) return;
    
    uint8_t ka[COBS_KA_LEN];  // 10 bytes
    memcpy(ka, "KA", 2);
    memcpy(&ka[2], local_device_id, 6);
    memcpy(&ka[8], &u.rx_good, 2);  // LE
    
    // Add CRC
    uint8_t ka_with_crc[COBS_KA_LEN + 4];
    memcpy(ka_with_crc, ka, COBS_KA_LEN);
    uint32_t crc = crc_crc32(0, ka, COBS_KA_LEN);
    memcpy(&ka_with_crc[COBS_KA_LEN], &crc, 4);
    
    // COBS encode and send
    uint8_t encoded[32];
    size_t enc_len = AP_Networking_COBS::encode(ka_with_crc, sizeof(ka_with_crc), 
                                                 encoded, sizeof(encoded) - 1);
    if (enc_len == 0) return;
    encoded[enc_len] = 0;
    
    if (u.uart->txspace() >= enc_len + 1) {
        u.uart->write(encoded, enc_len + 1);
    }
}
```

### RX

```cpp
void AP_Networking_Port_COBS::handle_keepalive(uint8_t uart_idx, const uint8_t *data) {
    UARTState &u = uarts[uart_idx];
    
    // Extract device ID
    uint8_t ka_device_id[6];
    memcpy(ka_device_id, &data[2], 6);
    
    // Hotplug detection: device ID changed on this UART?
    if (u.device_id_valid && memcmp(ka_device_id, u.seen_device_id, 6) != 0) {
        hub->request_cobs_split(this, uart_idx, ka_device_id);
        return;
    }
    
    // Check against port's established remote
    if (remote_id_known && memcmp(ka_device_id, remote_device_id, 6) != 0) {
        hub->request_cobs_split(this, uart_idx, ka_device_id);
        return;
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
    
    // Extract rx_good (remote's stats for our TX)
    uint16_t their_rx_good;
    memcpy(&their_rx_good, &data[8], 2);
    
    // Calculate loss for this measurement period
    uint16_t frames_sent = u.tx_count - u.tx_count_at_last_ka;
    uint16_t frames_received = their_rx_good - u.last_remote_rx_good;
    
    if (frames_sent > 0) {
        // Calculate loss (handle wrapping)
        int16_t lost = (int16_t)(frames_sent - frames_received);
        if (lost < 0) lost = 0;  // Clock drift or counter reset
        u.lost_since_last_ka = lost;
        
        // Update rolling loss rate (exponential moving average)
        // Scale: 0 = 0% loss, 255 = 100% loss
        uint8_t new_loss_rate = (frames_sent > 0) ? 
                                MIN(255, (uint16_t)lost * 255 / frames_sent) : 0;
        // EMA: new = old * 0.75 + sample * 0.25
        u.loss_rate_u8 = (u.loss_rate_u8 * 3 + new_loss_rate) / 4;
    }
    
    u.tx_count_at_last_ka = u.tx_count;
    u.last_remote_rx_good = their_rx_good;
    
    // Note: rx_good is incremented in handle_data_frame_*, not here.
    // This keeps rx_good consistent with tx_count (data frames only).
}
```

---

## Automatic UART Health Management

### Health States

| State | Description | Keepalive Rate | Data TX |
|-------|-------------|----------------|---------|
| GOOD | Normal operation | 500ms | Yes |
| DISABLED | Too lossy relative to others | 50ms (flood) | No |

**Recovery mechanism**: DISABLED UARTs are flooded with keepalives (50ms) to maintain fresh statistics. This allows quick re-enabling when conditions improve.

### Relative Loss Detection

The key insight: **don't disable a UART just because it has some loss - only disable it if it's significantly worse than the best available link**.

Hysteresis prevents oscillation:
- **Disable at**: loss > 3× best AND loss > 15%
- **Re-enable at**: loss ≤ 5% OR (loss ≤ 2× best AND loss ≤ 10%)

```cpp
uint8_t AP_Networking_Port_COBS::get_best_loss_rate() const {
    uint8_t best = 255;
    for (uint8_t i = 0; i < num_uarts; i++) {
        if (uarts[i].device_id_valid && uarts[i].loss_rate_u8 < best) {
            best = uarts[i].loss_rate_u8;
        }
    }
    return best;
}

void AP_Networking_Port_COBS::update_uart_health() {
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
```

### Example Scenarios

| UART 0 Loss | UART 1 Loss | UART 0 Status | UART 1 Status | Reasoning |
|-------------|-------------|---------------|---------------|-----------|
| 2% | 3% | GOOD | GOOD | Both acceptable |
| 2% | 20% | GOOD | DISABLED | 20% > 2%×3=6% AND 20% > 15% |
| 5% | 50% | GOOD | DISABLED | 50% > 5%×3=15% AND 50% > 15% |
| 40% | 50% | GOOD | GOOD | 50% > 40%×3=120%? NO - not 3× worse |
| 45% | 50% | GOOD | GOOD | 50% > 45%×3=135%? NO - not 3× worse |
| 0% | 10% | GOOD | GOOD | 10% not > 15% |
| 0% | 16% | GOOD | DISABLED | 16% > 0%×3=0% AND 16% > 15% |

**Recovery**: DISABLED UARTs flood keepalives (50ms) so statistics stay fresh. When loss drops to ≤5% or becomes comparable to other links (≤2× best AND ≤10%), the UART is re-enabled.

---

## Hotplug Support

### Timeout Handling

```cpp
void AP_Networking_Port_COBS::check_uart_timeouts() {
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
```

### Hub Split Operation

When a UART sees a different device_id, the hub splits it into a new port:

```cpp
// In AP_Networking_Hub
void request_cobs_split(AP_Networking_Port_COBS *port, uint8_t uart_idx,
                        const uint8_t new_device_id[6]) {
    // Extract UART from existing port
    AP_HAL::UARTDriver *uart;
    uint32_t baud;
    if (!port->extract_uart(uart_idx, uart, baud)) {
        return;
    }
    
    // Create new single-mode port
    auto *new_port = new AP_Networking_Port_COBS(this, uart, baud, local_device_id);
    if (new_port == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: COBS split failed");
        return;
    }
    
    if (!new_port->init()) {
        delete new_port;
        return;
    }
    
    register_port(new_port);
    add_to_cobs_array(new_port);
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: COBS split - new device %02X:%02X:%02X:%02X:%02X:%02X",
                  new_device_id[0], new_device_id[1], new_device_id[2],
                  new_device_id[3], new_device_id[4], new_device_id[5]);
}
```

### Port extract_uart Method

```cpp
bool AP_Networking_Port_COBS::extract_uart(uint8_t idx,
                                            AP_HAL::UARTDriver *&uart_out,
                                            uint32_t &baud_out) {
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
    }
    
    return true;
}
```

---

## Hub Integration

### Periodic Ganging Check

```cpp
void AP_Networking_Hub::check_cobs_ganging() {
    for (uint8_t i = 0; i < num_cobs_ports; i++) {
        auto *port_i = cobs_ports[i];
        if (!port_i->has_remote_device_id()) continue;
        
        uint8_t id_i[6];
        port_i->get_remote_device_id(id_i);
        
        for (uint8_t j = i + 1; j < num_cobs_ports; j++) {
            auto *port_j = cobs_ports[j];
            if (!port_j->has_remote_device_id()) continue;
            
            uint8_t id_j[6];
            port_j->get_remote_device_id(id_j);
            
            if (memcmp(id_i, id_j, 6) == 0) {
                // Same remote device - merge port j into port i
                for (uint8_t k = 0; k < port_j->get_num_uarts(); k++) {
                    port_i->add_uart(port_j->get_uart(k), port_j->get_baud(k));
                }
                
                unregister_port(port_j);
                delete port_j;
                remove_from_cobs_array(j);
                j--;  // Re-check this index
                
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: COBS ports merged");
            }
        }
    }
}
```

---

## Reorder Buffer (Ganged RX) - Lazy Allocation

The reorder buffer is only allocated when the first ganged frame is received, and is sized based on the number of UARTs in the gang.

### Buffer Sizing

With N UARTs, frames can arrive out of order due to latency differences. Worst case with 2x latency variance: `(N-1) * 2` frames of reorder.

**Formula**: `2 * num_uarts` slots, rounded up to power of 2, minimum 4

| UARTs | Calculated | Rounded | RAM Cost |
|-------|------------|---------|----------|
| 2 | 4 | 4 | ~6 KB |
| 3 | 6 | 8 | ~12 KB |
| 4 | 8 | 8 | ~12 KB |

```cpp
bool AP_Networking_Port_COBS::ensure_reorder_buffer() {
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
    
    reorder_buffer = new ReorderSlot[reorder_window];
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

bool AP_Networking_Port_COBS::handle_data_frame_ganged(uint8_t uart_idx, uint16_t seq,
                                                        const uint8_t *data, size_t len) {
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
        // Out of window - might indicate sync loss
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

void AP_Networking_Port_COBS::deliver_reordered_frames() {
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
```

---

## Constants Summary

```cpp
// Keepalive
static constexpr uint8_t COBS_KA_MARKER[] = {'K', 'A'};
static constexpr size_t COBS_KA_LEN = 10;               // Before CRC
static constexpr uint16_t KEEPALIVE_INTERVAL_MS = 500;  // Normal rate
static constexpr uint16_t KEEPALIVE_FLOOD_INTERVAL_MS = 50;  // When disabled
static constexpr uint16_t KEEPALIVE_TIMEOUT_MS = 2000;

// Frames
static constexpr size_t MAX_FRAME = 1522;
static constexpr size_t MIN_ETH_FRAME = 60;             // Minimum ethernet (no FCS)
static constexpr uint8_t GANG_CRC_SUFFIX[] = {'G', 'A', 'N', 'G'};

// TX
static constexpr uint32_t TX_BUFFER_SIZE = 16384;       // For completion time calculation

// Reorder buffer (lazily allocated, dynamically sized)
static constexpr uint8_t MIN_REORDER_SLOTS = 4;         // Minimum buffer size
// Actual size: MAX(4, 2*num_uarts) rounded up to power of 2

// Health monitoring (hysteresis thresholds)
static constexpr uint16_t MIN_SAMPLE_SIZE = 20;
static constexpr uint8_t LOSS_RATE_DISABLE_PCT = 15;         // Disable above this AND 3x worse
static constexpr uint8_t LOSS_RATE_DISABLE_MARGIN = 3;       // Disable if 3x worse than best
static constexpr uint8_t LOSS_RATE_ENABLE_PCT = 5;           // Re-enable below this
static constexpr uint8_t LOSS_RATE_ENABLE_RELATIVE_PCT = 10; // Re-enable if ≤2x best AND below this

// Limits
static constexpr uint8_t MAX_UARTS = 4;
```

---

## Files to Modify/Delete

### Delete
- `AP_Networking_Port_COBS_Gang.h`
- `AP_Networking_Port_COBS_Gang.cpp`
- `AP_Networking_COBS_Discovery.h`
- `AP_Networking_COBS_Discovery.cpp`

### Modify
- `AP_Networking_Port_COBS.h` - Add ganging support
- `AP_Networking_Port_COBS.cpp` - Implement unified logic
- `AP_Networking_Hub.h` - Add `request_cobs_split()`, `check_cobs_ganging()`
- `AP_Networking_Hub.cpp` - Implement gang management
- `AP_Networking.cpp` - Simplify port creation (no discovery phase)

---

## Benefits Summary

| Aspect | Before | After |
|--------|--------|-------|
| Port classes | 2 (`Port_COBS`, `Port_COBS_Gang`) | 1 (`Port_COBS`) |
| Discovery | 2s blocking delay | None (immediate start) |
| Single-mode overhead | 0 bytes | 0 bytes |
| Gang negotiation | Via keepalive flags | None (self-describing frames) |
| Hotplug | Not supported | Full support |
| Bad UART handling | Manual | Automatic (relative to other links) |
| TX UART selection | Round-robin | Earliest completion time |
| Reorder buffer RAM | Always allocated (~24KB) | Lazy, scaled to num_uarts (6-12KB) |
| CRC efficiency | N/A | Single base CRC, extend for gang check |
| Keepalive timing | While idle | Fixed 500ms interval (50ms when disabled) |
