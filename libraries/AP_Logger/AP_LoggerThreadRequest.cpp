#include "AP_LoggerThreadRequest.h"

#include <GCS_MAVLink/GCS.h>

// find a free request slot.
AP_LoggerThreadRequest *AP_LoggerThreadRequestQueue::claim_thread_request()
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
AP_LoggerThreadRequest *AP_LoggerThreadRequestQueue::claim_thread_request(AP_LoggerThreadRequest::Type type)
{
   AP_LoggerThreadRequest *request = claim_thread_request();
    if (request == nullptr) {
        return nullptr;
    }
    request->type = type;
    return request;
}
