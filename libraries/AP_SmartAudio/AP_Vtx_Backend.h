//
// Created by Lucho on 2020-07-02.
//
#include "AP_Vtx_Params.h"
#include <AP_HAL/AP_HAL.h>


class AP_Vtx_Backend
{
public:


    /**
     *
     * VTX CURRENT STATUS
     *
     * */
    enum class Status {
        not_connected = 0,
        processing,
        idle,
        desynchronized
    };



    /**
     *
     * VTX AS SENSOR INFORMATION. EXTENDED IN THE SPECIALIZED BACKENDS
     *
     * */
    struct Vtx_State {
        uint32_t last_io_time;       // system time of last successful update from vtx

    };

    // TODO: STORE IN AP_VTX_FRONTEND
    enum class Type {
        none   = 0,
        smartaudio,
        tramp
    };


    // constructor. This incorporates initialisation as well.
    AP_Vtx_Backend(Vtx_State &_state, AP_Vtx_Params &_params);

    // we declare a virtual destructor so that RangeFinder drivers can
    // override with a custom destructor if need be
    virtual ~AP_Vtx_Backend(void) {}

    // update the state structure
    // virtual void update() = 0;
    // true if vtx is returning data
    bool has_data() const;



    /////
    void set_status(AP_Vtx_Backend::Status status)
    {
        _status=status;
    };
    AP_Vtx_Backend::Status status()
    {
        return _status;
    };

    ////
    //    PROTECTED
    ////
protected:

    /*** SPECIALIZED SUBCLASSES MUST IMPLEMENT THIS BASIC INFO */
    virtual uint8_t band() =0;
    /*** SPECIALIZED SUBCLASSES MUST IMPLEMENT THIS BASIC INFO */
    virtual uint8_t channel() =0;
    /*** SPECIALIZED SUBCLASSES MUST IMPLEMENT THIS BASIC INFO */
    virtual uint8_t frecuency() =0;
    /*** SPECIALIZED SUBCLASSES MUST IMPLEMENT THIS BASIC INFO */
    virtual uint8_t power() =0;
    /*** SPECIALIZED SUBCLASSES MUST IMPLEMENT THIS BASIC INFO */
    virtual AP_Vtx_Backend::Type type() =0;//{ return (AP_Vtx_Backend::Type)params.type.get(); }



    Vtx_State get_state()
    {
        return state;
    }
    void set_state(Vtx_State newState)
    {
        state=newState;
    }

    AP_Vtx_Backend::Vtx_State &state;
    AP_Vtx_Params &params;
    Status _status;

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;

};