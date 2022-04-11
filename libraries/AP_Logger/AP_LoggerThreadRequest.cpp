#include "AP_LoggerThreadRequest.h"

#include <GCS_MAVLink/GCS.h>

// bool AP_LoggerThreadRequestQueue::Init()
// {
//     requests = newAP_LoggerThreadRequest()[_request_count];
//     if (requests == nullptr) {
//         return false;
//     }
//     return true;
// }

// find a freee request slot.  *MUST* be called with the
// loggerthreadrequest.requests_semaphore held!
AP_LoggerThreadRequest *AP_LoggerThreadRequestQueue::claim_free_request()
{
    WITH_SEMAPHORE(requests_semaphore);
    if (requests_count == ARRAY_SIZE(requests)) {
        return nullptr;
    }

    for (AP_LoggerThreadRequest &r : requests) {
        if (r.state == AP_LoggerThreadRequest::State::FREE) {
            r.state = AP_LoggerThreadRequest::State::CLAIMED;
            requests_count++;
            return &r;
        }
    }
    // no free slot - we held the sempahore, so internal error here
    return nullptr;
}

// reserve a request slot and fill it in a bit
AP_LoggerThreadRequest *AP_LoggerThreadRequestQueue::claim_simple_iothread_request(AP_LoggerThreadRequest::Type type)
{
   AP_LoggerThreadRequest *request = claim_free_request();
    if (request == nullptr) {
        return nullptr;
    }
    request->type = type;
    requests_count++;
    return request;
}

// template <uint8_t n>
// bool AP_LoggerThreadRequestQueue<n>::make_async_iothread_request(AP_LoggerThreadRequest::Type type, const char *name)
// {
//     WITH_SEMAPHORE(requests_semaphore);
//    AP_LoggerThreadRequest *request = claim_simple_iothread_request(type);
//     if (request == nullptr) {
//         gcs().send_text(MAV_SEVERITY_WARNING, "%s failed", name);
//         return false;
//     }
//     request->free_after_processing = true;
//     return true;
// }
