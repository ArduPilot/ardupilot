#include "uxr/client/client.h"
#include "ucdr/microcdr.h"

#include "Num.h"
#include "AP_INS.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Scheduler.h>
#include <AP_HAL/Semaphores.h>
#include <AP_AHRS/AP_AHRS.h>

#include "fcntl.h"
#define STREAM_HISTORY 8
#define BUFFER_SIZE_UDP    UXR_CONFIG_UDP_TRANSPORT_MTU * STREAM_HISTORY
#define BUFFER_SIZE_SERIAL UXR_CONFIG_SERIAL_TRANSPORT_MTU * STREAM_HISTORY

extern const AP_HAL::HAL& hal;

class AP_XRCE_Client {
 
private:

    // Initialize
    uint32_t max_topics; // Maximum number of topics the client can use
    
    
    char* ip;
    char* port;
    
    uint32_t fd;
    uint8_t relativeSerialAgentAddr;
    uint8_t relativeSerialClientAddr;
    uxrUDPTransport udp_transport; // Client uxr serial transport
    uxrSerialTransport serial_transport; // client uxr serial transport
    uxrSession session; //Session

    // Input Stream

    uint8_t input_reliable_stream[BUFFER_SIZE_UDP];
    uxrStreamId reliable_in;
    
    // Output Stream

    uint8_t output_reliable_stream[BUFFER_SIZE_UDP];
    uxrStreamId reliable_out;
    
    // Create
    // Participant

    uxrObjectId participant_id ; 
    uint16_t participant_req ;
    
    // Topic
    
    uxrObjectId topic_id ; 
    uint16_t topic_req ;
    
    // Publisher
    
    uxrObjectId pub_id ; 
    uint16_t pub_req ;
    
    // DataWriter
    
    uxrObjectId dwriter_id ;
    uint16_t dwriter_req ;
    
    //Status requests
    uint8_t status[4];

    // Write
    Num topic;  // simple int topic for testing
    AP_INS ins_topic; // INS topics

    // Semaphore
    AP_HAL::Semaphore *_csem;

    // connection parametrics
    bool connected;
    bool use_serial;

public:
    // Constructor (takes maximum number of topics as argument,by default it is 1)
    AP_XRCE_Client(uint32_t maxtopics=1,bool useserial=false) {
        this->max_topics = maxtopics;
        ip=(char *)"127.0.0.1";
        port=(char *)"14551";
        this->use_serial=useserial;
        if(this->use_serial) {
            this->relativeSerialClientAddr=1;
            this->relativeSerialAgentAddr=0;
        }
        else{
            ip=(char *)"127.0.0.1";
            port=(char *)"14551";
        }
        
        connected=true;
        
        //this->topic.num=0;
        
        this->ins_topic.accel_count=0;
        this->ins_topic.gyro_count=0;
        for(uint8_t i=0;i<3;i++){
            this->ins_topic.accel_scale[i]=0.0;
            this->ins_topic.accel_offsets[i]=0.0;
            this->ins_topic.gyro_offsets[i]=0.0;
        }
    }

    bool init();
    bool create();
    void write();
    void updateINSTopic(AP_InertialSensor& ins); 
    AP_HAL::Semaphore* get_clientsemaphore();
};

