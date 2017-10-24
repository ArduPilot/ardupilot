/*******************************************************************************
* rc_bb_model.c
*
* Because we wish to support different beaglemodel products with this same
* library, we must internally determine which model we are running on to decide
* which pins to use. We make these functions available to the user in case they
* wish to do the same. 
* See the check_model example for a demonstration.
*******************************************************************************/

#include "roboticscape/roboticscape.h"
#include "stdio.h"
#include "string.h"

#define MODEL_DIR "/proc/device-tree/model"
#define BUF_SIZE 128

// current model stored in memory as enum for fast access
rc_bb_model_t model;

// global variable always initialized at 0
// set to 1 once the model id has been pulled from /proc/
int has_checked; 


/*******************************************************************************
* return global variable 'model' from device tree
* then store it for later use
*******************************************************************************/
rc_bb_model_t rc_get_bb_model_from_device_tree(){
	char c[BUF_SIZE];
    FILE *fd;

    if ((fd = fopen(MODEL_DIR, "r")) == NULL)
    {
        printf("ERROR: can't open %s \n", MODEL_DIR);
        has_checked = 1;
		return UNKNOWN_MODEL;     
    }

    // read model
    memset(c, 0, BUF_SIZE);
    fgets(c, BUF_SIZE, fd);
    fclose(fd);

    // now do the checks
    if(		strcmp(c, "TI AM335x BeagleBone Black"				)==0) model=BB_BLACK;
    else if(strcmp(c, "TI AM335x BeagleBone Black RoboticsCape"	)==0) model=BB_BLACK_RC;
    else if(strcmp(c, "TI AM335x BeagleBone Blue"				)==0) model=BB_BLUE;
    else if(strcmp(c, "TI AM335x BeagleBone Black Wireless"		)==0) model=BB_BLACK_W;
    else if(strcmp(c, "TI AM335x BeagleBone Black Wireless RoboticsCape")==0) model=BB_BLACK_W_RC;
    else if(strcmp(c, "TI AM335x BeagleBone Green"				)==0) model=BB_GREEN;
    else if(strcmp(c, "TI AM335x BeagleBone Green Wireless"		)==0) model=BB_GREEN_W;
    else model = UNKNOWN_MODEL;

    // mark has-checked as 1 to prevent future slow checks
    has_checked = 1;
    return model;
}


/*******************************************************************************
* return global variable 'model' if already read from device tree
* otherwise get from device tree which stores it for later use
*******************************************************************************/
rc_bb_model_t rc_get_bb_model(){
	if(has_checked) return model;
	else return rc_get_bb_model_from_device_tree();
}


/*******************************************************************************
* print global variable 'model'
* if it hasn't been checked yet, do so first.
*******************************************************************************/
void rc_print_bb_model(){
	if(has_checked==0) rc_get_bb_model_from_device_tree();

	switch(model){
	case(UNKNOWN_MODEL):
		printf("UNKNOWN_MODEL");
		break;
	case(BB_BLACK):
		printf("BB_BLACK");
		break;
	case(BB_BLACK_RC):
		printf("BB_BLACK_RC");
		break;
	case(BB_BLACK_W):
		printf("BB_BLACK_W");
		break;
	case(BB_BLACK_W_RC):
		printf("BB_BLACK_W_RC");
		break;
	case(BB_GREEN):
		printf("BB_GREEN");
		break;
	case(BB_GREEN_W):
		printf("BB_GREEN_W");
		break;
	case(BB_BLUE):
		printf("BB_BLUE");
		break;
	default:
		printf("ERROR: invalid case in rc_print_bb_model()\n");
		break;
	}

	return;
}
