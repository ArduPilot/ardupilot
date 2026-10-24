#include "AP_Scripting_config.h"

#if AP_SCRIPTING_LUA_ENABLED

#include "AP_Scripting_helpers.h"
#include <AP_Scripting/lua_generated_bindings.h>

/// Fast param access via pointer helper class

// Custom lua constructor with optional param name
int lua_new_Parameter(lua_State *L) {

    const int args = lua_gettop(L);
    if (args > 1) {
        return luaL_argerror(L, args, "too many arguments");
    }
    const char * name = nullptr;
    if (args == 1) {
        name = luaL_checkstring(L, 1);
    }

    auto *p = new_Parameter(L);

    if (args == 0) {
        // no arguments, nothing to do
        return 1;
    }

    if (!p->init(name)) {
        return luaL_error(L, "No parameter: %s", name);
    }

    return 1;
}

#if HAL_ENABLE_DRONECAN_DRIVERS

#define IFACE_ALL uint8_t(((1U<<(HAL_NUM_CAN_IFACES))-1U))

/************************************************
  DroneCAN broadcast and request message handling
 ************************************************/

/*
  broadcast a message, takes a lua string as payload
 */
int DroneCAN_Handle::broadcast(lua_State *L)
{
    binding_argcheck(L, 2);

    DroneCAN_Handle *h = check_DroneCAN_Handle(L, 1);

    size_t data_length;
    const void *data = luaL_checklstring(L, 2, &data_length);
    
    auto &iface = h->dc->get_canard_iface();
    Canard::Transfer transfer;

    transfer.transfer_type = CanardTransferTypeBroadcast;
    transfer.data_type_signature = h->signature;
    transfer.data_type_id = h->data_type;
    transfer.inout_transfer_id = &h->transfer_id;
    transfer.priority = 0;
    transfer.payload = data;
    transfer.payload_len = data_length;
    transfer.iface_mask = IFACE_ALL;
#if CANARD_ENABLE_CANFD
    transfer.canfd = h->canfd;
#endif
    transfer.timeout_ms = 10;

    bool ok = iface.broadcast(transfer);
    lua_pushboolean(L, ok);

    return 1;
}

/*
  send a request message that expects a reply
 */
int DroneCAN_Handle::request(lua_State *L)
{
    binding_argcheck(L, 3);

    DroneCAN_Handle *h = check_DroneCAN_Handle(L, 1);
    uint8_t target_node = luaL_checknumber(L, 2);

    size_t data_length;
    const void *data = luaL_checklstring(L, 3, &data_length);

    auto &iface = h->dc->get_canard_iface();

    Canard::Transfer transfer;

    transfer.transfer_type = CanardTransferTypeRequest;
    transfer.data_type_signature = h->signature;
    transfer.data_type_id = h->data_type;
    transfer.inout_transfer_id = &h->transfer_id;
    transfer.priority = 0;
    transfer.payload = data;
    transfer.payload_len = data_length;
    transfer.iface_mask = IFACE_ALL;
#if CANARD_ENABLE_CANFD
    transfer.canfd = h->canfd;
#endif
    transfer.timeout_ms = 10;

    WITH_SEMAPHORE(iface.get_sem_rx());

    if (h->subscriber != nullptr) {
        delete h->subscriber;
    }
    
    h->subscriber = NEW_NOTHROW Subscriber(*h, CanardTransferTypeResponse);
    bool ok = h->subscriber != nullptr;

    if (ok) {
        h->subscriber->node_id = target_node;
        ok &= iface.request(target_node, transfer);
    }

    lua_pushboolean(L, ok);

    return 1;
}

/*
  subscribe to broadcast messages
 */
bool DroneCAN_Handle::subscribe(void)
{
    WITH_SEMAPHORE(dc->get_canard_iface().get_sem_rx());
    if (subscriber != nullptr) {
        delete subscriber;
    }
    subscriber = NEW_NOTHROW Subscriber(*this, CanardTransferTypeBroadcast);
    return subscriber != nullptr;
}

DroneCAN_Handle::Subscriber::Subscriber(DroneCAN_Handle &_handle, CanardTransferType _transfer_type) :
    Canard::HandlerList(_transfer_type, _handle.data_type, _handle.signature, _handle.dc->get_driver_index())
{
    handle = &_handle;
    trans_type = _transfer_type;
    link();
}

DroneCAN_Handle::Subscriber::~Subscriber(void)
{
    unlink();
    Payload payload;
    while (payloads.pop(payload)) {
        free(payload.data);
    }
}

/*
  handle an incoming subscribed message
 */
bool DroneCAN_Handle::Subscriber::handle_message(const CanardRxTransfer& transfer)
{
    if (transfer.transfer_type == CanardTransferTypeResponse &&
        (node_id != transfer.source_node_id ||
         ((transfer.transfer_id+1)&0xFF) != handle->transfer_id)) {
        // not from right node, or not right transfer ID
        return false;
    }

    if (payloads.space() == 0) {
        // already have the max number of messages pending, discard
        // this one, but return true to say it was for us
        return true;
    }
    Payload payload;
    payload.data = (uint8_t *)malloc(transfer.payload_len);
    if (payload.data == nullptr) {
        // discard messages if we can't allocate
        // but return true to say it was for us
        return true;
    }

    payload.length = transfer.payload_len;
    payload.node_id = transfer.source_node_id;
    payload.timestamp = transfer.timestamp_usec;
#if CANARD_ENABLE_CANFD
    payload.canfd = transfer.canfd;
#endif

    uint32_t bits_remaining = transfer.payload_len * 8;
    uint32_t ofs = 0;

    // decode in bytes, we don't have a canardDecodeBuffer function
    while (bits_remaining > 0) {
        uint8_t nbits = MIN(8U, bits_remaining);
        canardDecodeScalar(&transfer, ofs*8, nbits, false, (void*)&payload.data[ofs]);
        ofs++;
        bits_remaining -= nbits;
    }

    // push to queue
    payloads.push(payload);

    return true;
}

/*
  check for an incoming message
 */
int DroneCAN_Handle::check_message(lua_State *L)
{
    /*
      get the receive mutex for this driver
     */
    DroneCAN_Handle *h = check_DroneCAN_Handle(L, 1);

    auto *s = h->subscriber;

    if (s == nullptr) {
        return 0;
    }
    Subscriber::Payload payload;
    /*
      peek first to avoid a memory leak if lua_pushlstring() faults
     */
    if (!s->payloads.peek(payload)) {
        return 0;
    }

    lua_pushlstring(L, (char *)payload.data, payload.length);

    s->payloads.pop();
    free(payload.data);

    lua_pushinteger(L, payload.node_id);
    *new_uint64_t(L) = payload.timestamp;
#if CANARD_ENABLE_CANFD
    lua_pushboolean(L, payload.canfd);
#else
    lua_pushboolean(L, false);
#endif

    if (s->trans_type == CanardTransferTypeResponse) {
        // request reply removes the subscriber
        WITH_SEMAPHORE(h->dc->get_canard_iface().get_sem_rx());
        delete h->subscriber;
        h->subscriber = nullptr;
    }
    
    return 4;
}

/*
  garbage collect a handle
 */
int DroneCAN_Handle::__gc(lua_State *L)
{
    DroneCAN_Handle *h = check_DroneCAN_Handle(L, 1);
    WITH_SEMAPHORE(h->dc->get_canard_iface().get_sem_rx());

    if (h->subscriber != nullptr) {
        delete h->subscriber;
        h->subscriber = nullptr;
    }
    return 0;
}

// lua constructor for DroneCAN_Handle
int DroneCAN_Handle::new_handle(lua_State *L)
{
    lua_Number driver_index = luaL_checknumber(L, 1);
    const uint64_t sig = *check_uint64_t(L, 2);
    const uint16_t dtype = luaL_checkinteger(L, 3);
#if CANARD_ENABLE_CANFD
    const int args = lua_gettop(L);
    bool send_canfd = false;
    if (args > 3) {
        send_canfd = (bool)lua_toboolean(L, 4);
    }
#endif

    auto *dc = AP_DroneCAN::get_dronecan(driver_index);
    if (dc == nullptr) {
        return 0;
    }

    auto *h = new_DroneCAN_Handle(L);

    h->dc = dc;
    h->signature = sig;
    h->data_type = dtype;
#if CANARD_ENABLE_CANFD
    h->canfd = send_canfd;
#endif

    return 1;
}

#endif // HAL_ENABLE_DRONECAN_DRIVERS

#endif  // AP_SCRIPTING_LUA_ENABLED
