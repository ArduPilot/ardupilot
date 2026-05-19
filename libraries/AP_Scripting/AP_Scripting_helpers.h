#pragma once

#include <AP_Param/AP_Param.h>
#include "lua/src/lua.hpp"
#include <AP_DroneCAN/AP_DroneCAN.h>

int lua_new_Parameter(lua_State *L);

/// Fast param access via pointer helper
class Parameter
{
public:

    // init to param by name
    bool init(const char *name);

    // init by token, to get the value of old params
    bool init_by_info(uint16_t key, uint32_t group_element, enum ap_var_type type);

    // setters and getters
    bool set(float value);
    bool set_and_save(float value);
    bool get(float &value);
    bool configured();
    bool set_default(float value);

private:
    enum ap_var_type vtype;
    AP_Param *vp;
};


#if HAL_ENABLE_DRONECAN_DRIVERS

#ifndef DRONECAN_HANDLE_MAX_PAYLOADS
#define DRONECAN_HANDLE_MAX_PAYLOADS 8
#endif

/*
  access to sending DroneCAN broadcast messages and requests
 */
class DroneCAN_Handle {
public:
    static int new_handle(lua_State *L);
    static int broadcast(lua_State *L);
    static int __gc(lua_State *L);
    void close(void);
    static int request(lua_State *L);
    static int check_message(lua_State *L);
    bool subscribe(void);

    uint64_t signature;
    uint16_t data_type;
    uint8_t transfer_id;
    bool canfd;

private:
    AP_DroneCAN *dc;

    // class for subscribing to messages
    class Subscriber : public Canard::HandlerList {
    public:
        Subscriber(DroneCAN_Handle &_handle, CanardTransferType transfer_type);
        virtual ~Subscriber(void);

        uint8_t node_id;
        struct Payload {
            uint8_t *data;
            uint16_t length;
            uint8_t node_id;
#if CANARD_ENABLE_CANFD
            bool canfd;
#endif
            uint64_t timestamp;
        };
        ObjectBuffer<Payload> payloads{DRONECAN_HANDLE_MAX_PAYLOADS};
        CanardTransferType trans_type;

    private:
        bool handle_message(const CanardRxTransfer& transfer) override;
        DroneCAN_Handle *handle;
    };

    Subscriber *subscriber;
};

#endif // HAL_ENABLE_DRONECAN_DRIVERS
