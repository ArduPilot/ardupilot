

/*******************************************************************************
* int initialize_pru()
* 
* Enables the PRU and gets a pointer to the PRU shared memory which is used by 
* the servo and encoder functions in this C file. 
* Return 0 on success, -1 on failure.
*******************************************************************************/
int initialize_pru();

/*******************************************************************************
* int restart_pru()
* 
* Unloads pru binaries if loaded, then load them again.
*******************************************************************************/
int restart_pru();

/*******************************************************************************
* int get_pru_encoder_pos();
* 
* returns the encoder position or -1 if there was a problem.
*******************************************************************************/
int get_pru_encoder_pos();

/*******************************************************************************
* int set_pru_encoder_pos()
* 
* Set the encoder position, return 0 on success, -1 on failure.
*******************************************************************************/
int set_pru_encoder_pos(int val);