/*******************************************************************************
* roboticscape_motors.h
*
* function declarations for starting the motor lib files
* the rest of the motor functions are in roboticscape.h for the user to access
*******************************************************************************/
#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC int initialize_motors();
