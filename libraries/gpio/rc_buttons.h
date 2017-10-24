/*******************************************************************************
* roboticscape_buttons.h
*
* function declarations for starting and stopping button handlers
* the rest of the button functions are in roboticscape.h for the user to access
*******************************************************************************/


#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif	
int initialize_button_handlers();
EXTERNC int wait_for_button_handlers_to_join();
