// #define DEBUG_MOTORS    
// #define DEBUG_SENSORS   
 #define DEBUG_USE_KB 
// #define DEBUG_INPUT_DATA
// #define LINEAR_THRUST   


#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof(_arr[0]))


enum data_type {
        DATA_FLOAT,
        DATA_DOUBLE,
        DATA_VECTOR4F,
    };

struct vector4f 
{
    float w,x,y,z;
};

typedef struct vector4f VECTOR4F;

struct {
        double timestamp;
        VECTOR4F motors;
        } state, last_state;



// table to aid parsing of JSON sensor data
struct keytable {
        const char *section;
        const char *key;
        void *ptr;
        enum data_type type;

} keytable[1] = {
        //{ "", "timestamp", &state.timestamp, DATA_DOUBLE },
        { "", "engines",    &state.motors, DATA_VECTOR4F }
};