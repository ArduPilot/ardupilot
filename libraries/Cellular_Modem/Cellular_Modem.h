// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Cellular_Modem.h
 *
 */
///@file	Cellular_Modem.h
///@brief	"Hayes" AT-command compatible cellular/GPRS/IP modem initialization & control library

#ifndef CELLULAR_MODEM_H
#define CELLULAR_MODEM_H

#include <FastSerial.h>

// This activates the initialization of the cellular modem by the APM. Set to 1 to activate, anything else to de-activate
//#define CELLULAR_MODEM_INIT 1 // de-activate
#define CELLULAR_MODEM_INIT 1 // activate

// Init scripts for specific types of modems can be found in Cellular_Modem.cpp

//  == Modem timing ==
// Modems are generally sensitive to timing and need to be "paced" through the commands with slight pauses.
// Without pauses, the modems may miss commands.
// If we add too many pauses, they add up to a long delay in APM startup as the modem is initialized very very slowly.
// Total init time =
//  ((# of characters in line * CHAR_DELAY) +
//     (# of lines in script) * (LINE_DELAY+RESP_DELAY))
//  * (# of lines in script).
//
#define CHAR_DELAY 10 // number of milliseconds to wait between sending each character - most modems needs to be "paced"
#define RESP_DELAY 400 // number of milliseconds to wait before checking for a response
#define LINE_DELAY 400  // number of milliseconds to wait after each line is sent
#define RESP_TIMEOUT 20000 // How many milliseconds to wait for a response before giving up with an error  - 20 seconds by default
#define RESP_BUFFER 512 // size in bytes of response buffer

//
//  == Escape and substitution characters  ==
// The modem initialization script will also need to contain user defined variables
// that depend on the country, region, carrier choice and billing plan that corresponds
// to the SIM card or manufacturer of the modem. Such as the carrier's APN
// the user name and password for the carrier's APN, the IP address for the
// destination of the traffic and the protocol (TCP/UDP) that will be used
// Other variables may be needed in the future. For simplicity, we use single character variable names
// and a single escape character. For example, '%' is the escape character
// and 'i' is the variable for the IP address, then '%i' in the init script will be
// substituted for the IP value - eg '10.1.2.3'
// Note: Any enclosing quotes or other necessary 'garnishments' for the init script should
// go in the init script, not in the variable, because they are specific to the modem, not the variable.
//
// Example:
// CORRECT: 	init_script="....AT+IPADDRESS=\"%i\".....", i="10.1.2.3"
// INCORRECT:	init_script="....AT+IPADDRESS=%i.....",  i="\"10.1.2.3\""
//
//
//

#define PAUSE 500  // 500 msec pause
#define PAUSE_CHAR  '~' // this character in the init script signifies a pause of PAUSE milliseconds

// PAUSE does not need to be escaped and can be repeated: ~~~~~~

#define ESC_CHAR '%'

#define VAR_APN 	'a' // substitute for the APN - %a
#define VAR_USER 	'u' // substitute for the user name %u
#define VAR_PW 		'p'	// substitute for the password - %p
#define VAR_IP 		'i'  // substitute for the IP address - %i
#define VAR_PROTO 	't' // substitute for the protocol - %t
#define VAR_PORT	'd' // substitute for the port - %d


// TODO: UI for user to change these values in runtime or by MAVLINK PARAM
// rather than having to edit source code

#define VAL_APN 	"epc.tmobile.com" // value for the APN - try "internet" also
#define VAL_USER 	"" // value for the user name
#define VAL_PW 		""	// value for the password
#define VAL_IP 		"uav.willnotwork.example.com"  // value for the IP address
#define VAL_PROTO 	"udp" // value for the protocol
#define VAL_PORT    "14550"




/// @class	Cellular_Modem
/// @brief	Class to manage a Cellular Modem
class Cellular_Modem  {

public:

	// Constructor
	Cellular_Modem(FastSerial *port);

	static const char* init_script[];
	bool init_modem();
	bool parse_status(char *response);
	void send_test();
	//char *parse_notices();

protected:
	FastSerial *_port;
	uint32_t init_time;
};


#endif /* CELLULAR_MODEM_H_ */


