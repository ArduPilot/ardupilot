/* 
   AP_Logger Remote(via MAVLink) logging
*/

#include "AP_Logger_MAVLink.h"

#if HAL_LOGGING_MAVLINK_ENABLED

#include "LogStructure.h"

#define REMOTE_LOG_DEBUGGING 0

#if REMOTE_LOG_DEBUGGING
#include <stdio.h>
 # define Debug(fmt, args ...)  do {fprintf(stderr, "%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

#include <AP_InternalError/AP_InternalError.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;


bool LoggerThread_MAVLink::Init(uint8_t bufsize_kb)
{
    _blockcount = (1024*bufsize_kb) / sizeof(struct dm_block);

    _blocks = nullptr;
    while (_blockcount >= 8) { // 8 is a *magic* number
        _blocks = (struct dm_block *) calloc(_blockcount, sizeof(struct dm_block));
        if (_blocks != nullptr) {
            break;
        }
        _blockcount /= 2;
    }

    if (_blocks == nullptr) {
        return false;
    }

    free_all_blocks();
    stats_init();

    // let the timer() call start to do works
    _initialised = true;

    return true;
}

// initialisation
void AP_Logger_MAVLink::Init()
{
    if (!_iothread_mavlink.Init((uint8_t)_front._params.mav_bufsize)) {
        return;
    }

    _initialised = true;
}

bool AP_Logger_MAVLink::logging_failed() const
{
    return _iothread_mavlink.logging_failed();
}

bool LoggerThread_MAVLink::logging_failed() const
{
    // FIXME: protect access?!
    return !_sending_to_client;
}

uint32_t LoggerThread_MAVLink::bufferspace_available()
{
    // FIXME: why doesn't WITH_SEMAPHORE work here?!
    WITH_SEMAPHORE(semaphore);
    return (_blockcount_free * 200 + remaining_space_in_current_block());
}

uint32_t AP_Logger_MAVLink::bufferspace_available()
{
    return _iothread_mavlink.bufferspace_available();
}

uint8_t LoggerThread_MAVLink::remaining_space_in_current_block() const {
    // note that _current_block *could* be NULL ATM.
    return (MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN - _latest_block_len);
}

void LoggerThread_MAVLink::enqueue_block(dm_block_queue_t &queue, struct dm_block *block)
{
    if (queue.youngest != nullptr) {
        queue.youngest->next = block;
    } else {
        queue.oldest = block;
    }
    queue.youngest = block;
}

struct LoggerThread_MAVLink::dm_block *LoggerThread_MAVLink::dequeue_seqno(LoggerThread_MAVLink::dm_block_queue_t &queue, uint32_t seqno)
{
    struct dm_block *prev = nullptr;
    for (struct dm_block *block=queue.oldest; block != nullptr; block=block->next) {
        if (block->seqno == seqno) {
            if (prev == nullptr) {
                if (queue.youngest == queue.oldest) {
                    queue.oldest = nullptr;
                    queue.youngest = nullptr;
                } else {
                    queue.oldest = block->next;
                }
            } else {
                if (queue.youngest == block) {
                    queue.youngest = prev;
                }
                prev->next = block->next;
            }
            block->next = nullptr;
            return block;
        }
        prev = block;
    }
    return nullptr;
}

bool LoggerThread_MAVLink::free_seqno_from_queue(uint32_t seqno, LoggerThread_MAVLink::dm_block_queue_t &queue)
{
    struct dm_block *block = dequeue_seqno(queue, seqno);
    if (block != nullptr) {
        block->next = _blocks_free;
        _blocks_free = block;
        _blockcount_free++; // comment me out to expose a bug!
        return true;
    }
    return false;
}
    

bool LoggerThread_MAVLink::WritesOK() const
{
    if (!_sending_to_client) {
        return false;
    }
    return true;
}

bool AP_Logger_MAVLink::WritesOK() const
{
    return _iothread_mavlink.WritesOK();
}

/* Write a block of data at current offset */

bool AP_Logger_MAVLink::_WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical)
{
    if (!_iothread_mavlink.semaphore.take_nonblocking()) {
        _iothread_mavlink.dropped++;
        return false;
    }

    if (! WriteBlockCheckStartupMessages()) {
        _iothread_mavlink.semaphore.give();
        return false;
    }

    bool ret = _iothread_mavlink._WritePrioritisedBlock(pBuffer, size, is_critical);

    _iothread_mavlink.semaphore.give();

    return ret;
}

// DM_write: 70734 events, 0 overruns, 167806us elapsed, 2us avg, min 1us max 34us 0.620us rms
bool LoggerThread_MAVLink::_WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical)
{
    if (bufferspace_available() < size) {
        if (startup_messagewriter->finished()) {
            // do not count the startup packets as being dropped...
            dropped++;
        }
        return false;
    }

    uint16_t copied = 0;

    while (copied < size) {
        if (_current_block == nullptr) {
            _current_block = next_block();
            if (_current_block == nullptr) {
                // should not happen - there's a sanity check above
                INTERNAL_ERROR(AP_InternalError::error_t::logger_bad_current_block);
                return false;
            }
        }
        uint16_t remaining_to_copy = size - copied;
        uint16_t _curr_remaining = remaining_space_in_current_block();
        uint16_t to_copy = (remaining_to_copy > _curr_remaining) ? _curr_remaining : remaining_to_copy;
        memcpy(&(_current_block->buf[_latest_block_len]), &((const uint8_t *)pBuffer)[copied], to_copy);
        copied += to_copy;
        _latest_block_len += to_copy;
        if (_latest_block_len == MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN) {
            //block full, mark it to be sent:
            enqueue_block(_blocks_pending, _current_block);
            _current_block = next_block();
        }
    }

    return true;
}

//Get a free block
struct LoggerThread_MAVLink::dm_block *LoggerThread_MAVLink::next_block()
{
    LoggerThread_MAVLink::dm_block *ret = _blocks_free;
    if (ret != nullptr) {
        _blocks_free = ret->next;
        _blockcount_free--;
        ret->seqno = _next_seq_num++;
        ret->last_sent = 0;
        ret->next = nullptr;
        _latest_block_len = 0;
    }
    return ret;
}

void LoggerThread_MAVLink::free_all_blocks()
{
    _blocks_free = nullptr;
    _current_block = nullptr;

    _blocks_pending.sent_count = 0;
    _blocks_pending.oldest = _blocks_pending.youngest = nullptr;
    _blocks_retry.sent_count = 0;
    _blocks_retry.oldest = _blocks_retry.youngest = nullptr;
    _blocks_sent.sent_count = 0;
    _blocks_sent.oldest = _blocks_sent.youngest = nullptr;

    // add blocks to the free stack:
    for(uint8_t i=0; i < _blockcount; i++) {
        _blocks[i].next = _blocks_free;
        _blocks_free = &_blocks[i];
        // this value doesn't really matter, but it stops valgrind
        // complaining when acking blocks (we check seqno before
        // state).  Also, when we receive ACKs we check seqno, and we
        // want to ack the *real* block zero!
        _blocks[i].seqno = 9876543;
    }
    _blockcount_free = _blockcount;

    _latest_block_len = 0;
}

void LoggerThread_MAVLink::stop_logging()
{
    if (_sending_to_client) {
        _sending_to_client = false;
        _last_response_time = AP_HAL::millis();
    }
}

void AP_Logger_MAVLink::stop_logging(void)
{
    if (!complete_iothread_request(LoggerThreadRequest::Type::StopLogging)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "stop_logging failed");
    }
}

void LoggerThread_MAVLink::process_request(LoggerThreadRequest &request)
{
    switch (request.type) {
    case LoggerThreadRequest::Type::HandleRemoteLogBlockStatus:
        if(request.parameters.HandleRemoteLogBlockStatus.status == MAV_REMOTE_LOG_DATA_BLOCK_NACK) {
            handle_retry(request);
        } else {
            handle_ack(request);
        }
        break;
    default:
        LoggerBackendThread::process_request(request);
    }
}

void LoggerThread_MAVLink::handle_ack(LoggerThreadRequest &request)
{
    if (!_initialised) {
        return;
    }
    const uint32_t seqno = request.parameters.HandleRemoteLogBlockStatus.seqno;
    const GCS_MAVLINK &link = *(request.parameters.HandleRemoteLogBlockStatus.link);
    const uint8_t sysid = request.parameters.HandleRemoteLogBlockStatus.sysid;
    const uint8_t compid = request.parameters.HandleRemoteLogBlockStatus.compid;

    WITH_SEMAPHORE(semaphore);
    if(seqno == MAV_REMOTE_LOG_DATA_BLOCK_STOP) {
        Debug("Received stop-logging packet");
        stop_logging();
        return;
    }
    if(seqno == MAV_REMOTE_LOG_DATA_BLOCK_START) {
        if (!_sending_to_client) {
            Debug("Starting New Log");
            free_all_blocks();
            // _current_block = next_block();
            // if (_current_block == nullptr) {
            //     Debug("No free blocks?!!!\n");
            //     return;
            // }
            stats_init();
            _sending_to_client = true;
            _target_system_id = sysid;
            _target_component_id = compid;
            _link = &link;
            _next_seq_num = 0;
            start_new_log_reset_variables();
            _last_response_time = AP_HAL::millis();
            Debug("Target: (%u/%u)", _target_system_id, _target_component_id);
        }
        return;
    }

    // check SENT blocks (VERY likely to be first on the list):
    if (free_seqno_from_queue(seqno, _blocks_sent)) {
        // celebrate
        _last_response_time = AP_HAL::millis();
    } else if(free_seqno_from_queue(seqno, _blocks_retry)) {
        // party
        _last_response_time = AP_HAL::millis();
    } else {
        // probably acked already and put on the free list.
    }
}

void AP_Logger_MAVLink::remote_log_block_status_msg(GCS_MAVLINK &link,
                                                    const mavlink_message_t& msg)
{
    LoggerThreadRequest *request;
    LoggerThread &loggerthread{AP::logger().get_loggerthread()};
    WITH_SEMAPHORE(loggerthread.requests_semaphore);
    request = loggerthread.claim_free_request();
    if (request == nullptr) {
        return;
    }

    const bool synchronous = false; // FIXME

    request->state = LoggerThreadRequest::State::PENDING;
    loggerthread.requests_count++;
    if (!synchronous) {
        request->free_after_processing = true;
    }

    mavlink_remote_log_block_status_t packet;
    mavlink_msg_remote_log_block_status_decode(&msg, &packet);

    request->type = LoggerThreadRequest::Type::HandleRemoteLogBlockStatus;
    request->parameters.HandleRemoteLogBlockStatus = {
        &link,
        msg.sysid,
        msg.compid,
        packet.seqno,
        packet.status,
    };
}

void LoggerThread_MAVLink::handle_retry(LoggerThreadRequest &request)
{
    const uint8_t seqno = request.parameters.HandleRemoteLogBlockStatus.seqno;

    if (!_initialised || !_sending_to_client) {
        return;
    }

    WITH_SEMAPHORE(semaphore);
    struct dm_block *victim = dequeue_seqno(_blocks_sent, seqno);
    if (victim != nullptr) {
        _last_response_time = AP_HAL::millis();
        enqueue_block(_blocks_retry, victim);
    }
}

void LoggerThread_MAVLink::stats_init() {
    dropped = 0;
    stats.resends = 0;
    stats_reset();
}
void LoggerThread_MAVLink::stats_reset() {
    stats.state_free = 0;
    stats.state_free_min = -1; // unsigned wrap
    stats.state_free_max = 0;
    stats.state_pending = 0;
    stats.state_pending_min = -1; // unsigned wrap
    stats.state_pending_max = 0;
    stats.state_retry = 0;
    stats.state_retry_min = -1; // unsigned wrap
    stats.state_retry_max = 0;
    stats.state_sent = 0;
    stats.state_sent_min = -1; // unsigned wrap
    stats.state_sent_max = 0;
    stats.collection_count = 0;
}

void LoggerThread_MAVLink::Write_MAV()
{
    LoggerThread_MAVLink &logger_mav = *this;
    if (logger_mav.stats.collection_count == 0) {
        return;
    }
    const struct log_MAV_Stats pkt{
        LOG_PACKET_HEADER_INIT(LOG_MAV_STATS),
        timestamp         : AP_HAL::micros64(),
        seqno             : logger_mav._next_seq_num-1,
        dropped           : logger_mav.dropped,
        retries           : logger_mav._blocks_retry.sent_count,
        resends           : logger_mav.stats.resends,
        state_free_avg    : (uint8_t)(logger_mav.stats.state_free/logger_mav.stats.collection_count),
        state_free_min    : logger_mav.stats.state_free_min,
        state_free_max    : logger_mav.stats.state_free_max,
        state_pending_avg : (uint8_t)(logger_mav.stats.state_pending/logger_mav.stats.collection_count),
        state_pending_min : logger_mav.stats.state_pending_min,
        state_pending_max : logger_mav.stats.state_pending_max,
        state_sent_avg    : (uint8_t)(logger_mav.stats.state_sent/logger_mav.stats.collection_count),
        state_sent_min    : logger_mav.stats.state_sent_min,
        state_sent_max    : logger_mav.stats.state_sent_max,
    };
    AP::logger().WriteBlock(&pkt,sizeof(pkt));
}

void LoggerThread_MAVLink::stats_log()
{
    if (!_initialised) {
        return;
    }
    if (stats.collection_count == 0) {
        return;
    }
    Write_MAV();
#if REMOTE_LOG_DEBUGGING
    printf("D:%d Retry:%d Resent:%d SF:%d/%d/%d SP:%d/%d/%d SS:%d/%d/%d SR:%d/%d/%d\n",
           _dropped,
           _blocks_retry.sent_count,
           stats.resends,
           stats.state_free_min,
           stats.state_free_max,
           stats.state_free/stats.collection_count,
           stats.state_pending_min,
           stats.state_pending_max,
           stats.state_pending/stats.collection_count,
           stats.state_sent_min,
           stats.state_sent_max,
           stats.state_sent/stats.collection_count,
           stats.state_retry_min,
           stats.state_retry_max,
           stats.state_retry/stats.collection_count
        );
#endif
    stats_reset();
}

uint8_t LoggerThread_MAVLink::stack_size(struct dm_block *stack)
{
    uint8_t ret = 0;
    for (struct dm_block *block=stack; block != nullptr; block=block->next) {
        ret++;
    }
    return ret;
}
uint8_t LoggerThread_MAVLink::queue_size(dm_block_queue_t queue)
{
    return stack_size(queue.oldest);
}

// must be called only when initialised and semaphore held:
void LoggerThread_MAVLink::stats_collect()
{
    uint8_t pending = queue_size(_blocks_pending);
    uint8_t sent = queue_size(_blocks_sent);
    uint8_t retry = queue_size(_blocks_retry);
    uint8_t sfree = stack_size(_blocks_free);

    if (sfree != _blockcount_free) {
        INTERNAL_ERROR(AP_InternalError::error_t::logger_blockcount_mismatch);
    }

    stats.state_pending += pending;
    stats.state_sent += sent;
    stats.state_free += sfree;
    stats.state_retry += retry;

    if (pending < stats.state_pending_min) {
        stats.state_pending_min = pending;
    }
    if (pending > stats.state_pending_max) {
        stats.state_pending_max = pending;
    }
    if (retry < stats.state_retry_min) {
        stats.state_retry_min = retry;
    }
    if (retry > stats.state_retry_max) {
        stats.state_retry_max = retry;
    }
    if (sent < stats.state_sent_min) {
        stats.state_sent_min = sent;
    }
    if (sent > stats.state_sent_max) {
        stats.state_sent_max = sent;
    }
    if (sfree < stats.state_free_min) {
        stats.state_free_min = sfree;
    }
    if (sfree > stats.state_free_max) {
        stats.state_free_max = sfree;
    }
    
    stats.collection_count++;
}

/* while we "successfully" send log blocks from a queue, move them to
 * the sent list. DO NOT call this for blocks already sent!
*/
bool LoggerThread_MAVLink::send_log_blocks_from_queue(dm_block_queue_t &queue)
{
    uint8_t sent_count = 0;
    while (queue.oldest != nullptr) {
        if (sent_count++ > _max_blocks_per_send_blocks) {
            return false;
        }
        if (! send_log_block(*queue.oldest)) {
            return false;
        }
        queue.sent_count++;
        struct dm_block *tmp = dequeue_seqno(queue,queue.oldest->seqno);
        if (tmp != nullptr) { // should never be nullptr
            enqueue_block(_blocks_sent, tmp);
        } else {
            INTERNAL_ERROR(AP_InternalError::error_t::logger_dequeue_failure);
        }
    }
    return true;
}

void LoggerThread_MAVLink::timer(void)
{
    if (!_initialised || !_sending_to_client) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // if we're sending but not receiving then timeout:
    if (now_ms - _last_send_time < 1000) {
        if (now_ms - _last_response_time > 10000) {
            // other end appears to have timed out!
            gcs().send_text(MAV_SEVERITY_WARNING, "timeout");
            Debug("Client timed out");
            _sending_to_client = false;
            return;
        }
    } else {
        // if we're not sending blocks to the client it is
        // unreasonable to expect a response from the client.  In the
        // case we stop sending blocks for a while then restart we
        // don't want the client-timeout code above to trigger - so
        // we'll lie about the last response time:
        _last_response_time = AP_HAL::millis();
    }

    if (!semaphore.take_nonblocking()) {
        return;
    }

    if (! send_log_blocks_from_queue(_blocks_retry)) {
        semaphore.give();
        return;
    }

    if (! send_log_blocks_from_queue(_blocks_pending)) {
        semaphore.give();
        return;
    }

    if (now_ms - last_10Hz_ms > 100) {
        last_10Hz_ms = now_ms;
        do_resends(now_ms);
        stats_collect();
    }

    semaphore.give();
}

// must be called only when initialised and while semaphore held:
void LoggerThread_MAVLink::do_resends(uint32_t now)
{
    uint8_t count_to_send = 5;
    if (_blockcount < count_to_send) {
        count_to_send = _blockcount;
    }
    uint32_t oldest = now - 100; // 100 milliseconds before resend.  Hmm.
    while (count_to_send-- > 0) {
        for (struct dm_block *block=_blocks_sent.oldest; block != nullptr; block=block->next) {
            // only want to send blocks every now-and-then:
            if (block->last_sent < oldest) {
                if (! send_log_block(*block)) {
                    // failed to send the block; try again later....
                    return;
                }
                stats.resends++;
            }
        }
    }
}

// NOTE: any functions called from these periodic functions MUST
// handle locking of the blocks structures by taking the semaphore
// appropriately!
void AP_Logger_MAVLink::periodic_1Hz()
{
    if (rate_limiter == nullptr && _front._params.mav_ratemax > 0) {
        // setup rate limiting
        rate_limiter = new AP_Logger_RateLimiter(_front, _front._params.mav_ratemax);
    }
}

void AP_Logger_MAVLink::periodic_fullrate()
{
    AP_Logger_Backend::push_log_blocks();
}

//TODO: handle full txspace properly
bool LoggerThread_MAVLink::send_log_block(struct dm_block &block)
{
    if (!_initialised) {
       return false;
    }
    if (_link == nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return false;
    }
    // don't completely fill buffers - and also ensure there's enough
    // room to send at least one packet:
    const uint16_t min_payload_space = 500;
    static_assert(MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK_LEN <= min_payload_space,
                  "minimum allocated space is less than payload length");
    if (_link->txspace() < min_payload_space) {
        return false;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // deliberately fail 10% of the time in SITL:
    if ((rand() % 100 + 1) < 10) {
        return false;
    }
#endif
    
#if DF_MAVLINK_DISABLE_INTERRUPTS
    void *istate = hal.scheduler->disable_interrupts_save();
#endif

// DM_packing: 267039 events, 0 overruns, 8440834us elapsed, 31us avg, min 31us max 32us 0.488us rms

    mavlink_message_t msg;
    mavlink_status_t *chan_status = mavlink_get_channel_status(_link->get_chan());
    uint8_t saved_seq = chan_status->current_tx_seq;
    chan_status->current_tx_seq = mavlink_seq++;
    // Debug("Sending block (%d)", block.seqno);
    mavlink_msg_remote_log_data_block_pack(mavlink_system.sysid,
                                           MAV_COMP_ID_LOG,
                                           &msg,
                                           _target_system_id,
                                           _target_component_id,
                                           block.seqno,
                                           block.buf);

#if DF_MAVLINK_DISABLE_INTERRUPTS
    hal.scheduler->restore_interrupts(istate);
#endif

    block.last_sent = AP_HAL::millis();
    chan_status->current_tx_seq = saved_seq;

    // _last_send_time is set even if we fail to send the packet; if
    // the txspace is repeatedly chockas we should not add to the
    // problem and stop attempting to log
    _last_send_time = AP_HAL::millis();

    _mavlink_resend_uart(_link->get_chan(), &msg);

    return true;
}
#endif
