#define DEBUG_INPUT_DATA
#define LINEAR_THRUST  

#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof(_arr[0]))


enum data_type {
        DATA_FLOAT,
        DATA_DOUBLE,
        DATA_VECTOR4F,
        DATA_VECTOR2F,
    };

struct vector4f 
{
    float w,x,y,z;
};

struct vector2f 
{
    float x,y;
};

typedef struct vector4f VECTOR4F;
typedef struct vector2f VECTOR2F;

struct {
        double timestamp;
        VECTOR4F motors;
        } state, last_state;



// table to aid parsing of JSON sensor data
struct keytableROV {
        const char *section;
        const char *key;
        void *ptr;
        enum data_type type;

} keytableROV[1] = {
        //{ "", "timestamp", &state.timestamp, DATA_DOUBLE },
        { "", "engines",    &state.motors, DATA_VECTOR2F }
};


