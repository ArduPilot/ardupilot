#include <stdio.h>
#include "AP_SerialManager/AP_SerialManager.h"
#include "AP_Vtx_Backend.h"




class AP_Vtx_SerialBackend: public AP_Vtx_Backend
{

public:
    /**
    *
    * VTX AS SENSOR INFORMATION. EXTENDED IN THE SPECIALIZED BACKENDS
    *
    * */
    struct Vtx_Serial_State : public Vtx_State {
        uint32_t last_reading_ms;       // system time of last successful update from vtx
        uint32_t last_writing_ms;       // system time of last successful update from vtx
    };

    enum class RqStatus {
        requesting = 0,
        waiting_response,
        reading_response,
        idle
    };
    // constructor
    AP_Vtx_SerialBackend(AP_Vtx_SerialBackend::Vtx_Serial_State &_state,
                         AP_Vtx_Params &_params,
                         uint8_t serial_instance,
                         AP_SerialManager::SerialProtocol protocol
                        );

    // static detection function
    static bool detect(AP_SerialManager::SerialProtocol protocol,uint8_t serial_instance);
    void set_write_time(uint32_t newTime)
    {
        state->last_io_time=newTime;
        state->last_writing_ms=newTime;
    }
    uint32_t get_write_time()
    {
        return get_state().last_writing_ms;
    }
    void set_read_time(uint32_t newTime)
    {
        state->last_io_time=newTime;
        state->last_reading_ms=newTime;
    }
    uint32_t get_read_time()
    {
        return get_state().last_reading_ms;
    }
    bool is_io_timeout()
    {
        return ((AP_HAL::millis() - state->last_io_time) > read_timeout_ms());
    }
    void set_io_status(RqStatus status);

    AP_Vtx_SerialBackend::RqStatus get_io_status()
    {
        return rq_status;
    }

protected:
    Vtx_Serial_State *state;

    enum RqStatus rq_status;

    // baudrate used during object construction:
    virtual uint32_t initial_baudrate(uint8_t serial_instance) const
    {
        return 0;
    };

    // the value 0 is special to the UARTDriver - it's "use default"
    virtual uint16_t rx_bufsize() const
    {
        return 0;
    }
    virtual uint16_t tx_bufsize() const
    {
        return 0;
    }


    AP_HAL::UARTDriver *uart = nullptr;

    // update state; not all backends call this!
    //virtual void update(void) override;

    // maximum time between readings before we change state to NoData:
    virtual uint16_t read_timeout_ms() const
    {
        return 200;
    }

    Vtx_Serial_State get_state()
    {
        return *state;
    }


    // TODO: Utility function to print hex buffer to string, maybe there is one yet in ap
    void print_bytes_to_hex_string(uint8_t buf[],uint8_t x)
    {
        int i;
        for (i = 0; i < x; i++) {
            if (i > 0) {
                printf(":");
            }
            printf("%02X", buf[i]);
        }
        printf("\n");
    }




};