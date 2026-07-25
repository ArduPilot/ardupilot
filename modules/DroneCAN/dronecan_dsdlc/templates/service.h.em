@[if msg.kind == msg.KIND_SERVICE]@
@{from dronecan_dsdlc_helpers import *}@
#pragma once
#include <@(msg_header_name_request(msg))>
#include <@(msg_header_name_response(msg))>

#define @(underscored_name(msg).upper())_ID @(msg.default_dtid)
#define @(underscored_name(msg).upper())_SIGNATURE @('(0x%08XULL)' % (msg.get_data_type_signature(),))
@[end if]@

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
#include <canard/cxx_wrappers.h>
SERVICE_MESSAGE_CXX_IFACE(@(underscored_name(msg)), @(underscored_name(msg).upper())_ID, @(underscored_name(msg).upper())_SIGNATURE, @(underscored_name(msg).upper())_REQUEST_MAX_SIZE, @(underscored_name(msg).upper())_RESPONSE_MAX_SIZE);
#endif
