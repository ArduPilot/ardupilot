#include <Cellular_Modem.h>

// script for generic GSM/GPRS AT-command capable modem (based on SIM900 dronecell) - VERBOSE
// TODO use prog_char and PROGMEM here

const char* Cellular_Modem::init_script[] = {
"+++ATH0",					// Break into channel if there is an existing connection
"AT+CFUN=1,1",			// Hard reboot of modem
"~~~~~~~~~~~~~~~~~~~~AT",			        // Wait...
"AT V1 E1 X1 S0=0",		// Set error response and do not pickup on ring
"AT+CREG=2",			// Set various notice messages and parameters
"AT+CMEE=2",
"AT+CR=1",
"AT+CRC=1",
"AT+CSNS=4",
"AT+CSMINS=1",
"AT+CSCLK=0",
"AT+CIURC=1",
"AT+CGEREP=2",
//"AT+IFC=2,2",			// Hardware flow control
"AT+CIPMUX=0",			// Single channel communication (ie only one socket can be opened)
"AT+CIPMODE=1",			// Transparent bridge mode
"AT+CIPCCFG=8,10,10,0", // GPRS params
"~~~~AT+CMUX=0,0,4,127,10,3,30,10,2", // GPRS/IP params
"~~~~AT+CSTT=\"%a\",\"%u\",\"%p\"",   // AT+CSTT="APN","username","password" - login to service provider/carrier
"~~~~AT+CIICR",				// Connect!
"~~~~~~~~~~AT+CIFSR",			// Get IP address (for info only)
"~~~~AT+CIPSTART=\"%t\",\"%i\",\"%d\"",   // AT+CIPSTART="protocol","ip address or domain","port #" - make the bridge
NULL, // MUST have a NULL termination or you get an infinite loop
};

Cellular_Modem::Cellular_Modem(FastSerial *port) {
	this->_port = port;
}

bool Cellular_Modem::init_modem() {
	unsigned int i = 0;
	int c = 0;
	uint32_t start;

	//Serial.print("parse_status test: ");
	//Serial.printf("%s\n", (parse_status((char *)"+++\n +CPIN:READY\n\t +PDP:0\n AT\nOK\n") ? "PASSED":"FAILED"));

	init_time = millis();

	BetterStream	*p = (BetterStream *)_port;

	p->flush(); // flush the write buffer, to clear it.

	do { c = p->read(); } while (c > -1); // flush read buffer

	i = 0;

	// Run through the init script, one line at a time.

	while (init_script[i] != NULL) {
		char resp_buffer[RESP_BUFFER] = ""; // buffer for responses
		unsigned int r=0, r_len=0, j=0; // index variables

		while (init_script[i][j] != '\0') { // if first character is not 0 (NULL)

			c = init_script[i][j]; // grab a character from the line

			switch (c) {

			case PAUSE_CHAR:  // this character is used to trigger a PAUSE.
				delay(PAUSE);
				break;

			case ESC_CHAR: // this character indicates an escape sequence or substitution
				j++; // move to the next character
				c = init_script[i][j];
				if (c == '\0') return false; // this should not happen - TODO handle better

				switch (c) { // check the character after ESC_CHAR

				case ESC_CHAR: // twice in a row, means the literal character, so we send it.
					p->write(c);
					break;

				case VAR_APN: // substitute for the APN
					p->print(VAL_APN);
					break;
				case VAR_USER: // substitute for the user name
					p->print(VAL_USER);
					break;
				case VAR_PW:	// substitute for the password
					p->print(VAL_PW);
					break;
				case VAR_IP:  // substitute for the IP address
					p->print(VAL_IP);
					break;
				case VAR_PROTO: // substitute for the protocol
					p->print(VAL_PROTO);
					break;
				case VAR_PORT: //substitute for the port
					p->print(VAL_PORT);
					break;

				default: // unknown substitution - this should not happen
					return false;
					break; // never reached
				}
				break;

			default:		// any other character, we send to the modem
				p->write(c);
				break;
			}

			j++; // next character in line
			delay(CHAR_DELAY); // wait between sending characters - most modems need to be "paced"
		}
		p->println(); // Send the newline to "enter" the command

		Serial.printf("\n--MODEM: [%s]\n",init_script[i]);  // DEBUG

		// loop until response -- need breaker to avoid inf loop
		start = millis(); // for timeout counting
	    while (!p->available() && (millis() - start) < RESP_TIMEOUT) { // keep trying until timeout
	    	delay(10);
	    }
	    if (!p->available())  { // no response - timeout!
	    			Serial.println("Error: timeout - no response from modem\n");
	    			break; // next line
	    }
		else { // get the response
			delay(RESP_DELAY);  // Give the modem some time to respond

			do {
				r_len = p->available(); // see how long the response is

				if (r_len > RESP_BUFFER) return false; // our buffer is too small - TODO handle better


				for (r=0; r < r_len; r++) { // read the response
					resp_buffer[r] = p->read();
				}
				resp_buffer[r_len] = '\0'; // NULL terminate the buffer

				// experimental
				if (this->parse_status(resp_buffer)) { // look for "OK" or "0"
					Serial.printf("Success: %s",resp_buffer); // DEBUG
					break; // send next line
				}
			} while (p->available() && (millis() - start) < RESP_TIMEOUT);
		}
		i++;  // next line
		delay(LINE_DELAY);
	}
    // we're done, without an error
	Serial.printf("--MODEM Init: %lu msec", millis() - init_time); // DEBUG
	return true;  // successful init
	//send_test();
}

void Cellular_Modem::send_test() {

	BetterStream	*p = (BetterStream *)_port;

	Serial.println("MAVLINKoCIP TEST");
	do { // forever
		p->printf("MAVLINKoCIP TEST\n", 0x55);
		Serial.write(".");
		delay(300);
		p->flush();
	} while (true);
}

bool Cellular_Modem::parse_status(char *response) {
	unsigned int i = 0;
	char c;

	while ((c = response[i]) != '\0') {
		//Serial.printf("-%02d",i);
		//Serial.println(c);
		switch (c) {
			case '\r': // <CR>
			case '\n': // <LF>
			case ' ': // space
			case '\t': // tab
				//Serial.printf(".%02d",i);
				//Serial.println(c);
				break; // skip it!

			case 'A': // A for AT? means we are seeing our own commands echo'd back
				if (response[i+1] == 'T')
				while ( c != '\0' && c != '\n') {
					//Serial.printf("*%02d",i);
					//Serial.println(c);
					i++;
					c = response[i];
				}
			case '+': // notice/informational message
				// skip to next line
				c = response[i];
				while ( c != '\0' && c != '\n') {
					//Serial.printf("x%02d",i);
					//Serial.println(c);
					i++;
					c = response[i];
				}
				break;

			case '0': // numeric response for OK
				return true;
				break; // never reached

			case 'O': // O for OK?
				if (response[i+1] == 'K') return true;
				break;

			default: // anything else
				//Serial.printf("!%02d",i);
				//Serial.println(c);
				return false;
				break;
		}
		i++;
	}
	// should never reach the end without finding an 'OK' or '0', therefore it failed.
	return false;
}

// Other AT test commands follow

// script for generic GSM/GPRS AT-command capable modem - NOT VERBOSE
//const char* Cellular_Modem::init_script[] = {
//"+++",
//"AT+CFUN=1,1",
//"~~~~~~~~AT",
//"AT V0 E0 X0 S0=0",
//"AT+CREG=0",
//"AT+CMEE=0",
//"AT+IFC=2,2",
//"AT+CIPMUX=0",
//"AT+CIPMODE=1",
//"~~~~~~~AT+CSTT=\"%a\"",
//"AT+CIICR",
//"~~~~AT+CIFSR",
//"AT+CIPSTART=\"%t\",\"%i\",\"%d\"",
//NULL,
//};



//"+++",
//"ATH0",
//"AT+CFUN=1,1",
//"AT",
//"AT",
//"AT",
//"AT V1 E1 X1 S0=0",
//"AT+CREG=2",
//"AT+CMEE=2",
//"AT+CR=1",
//"AT+CRC=1",
//"AT+CSNS=4",
//"AT+CSMINS=1",
//"AT+CSCLK=0",
//"AT+CIURC=1",
//"AT+CGEREP=2",
//"AT+CSPN?",
//"AT+CBC",
//"AT+GCAP",
//"AT+GSV",
//"AT+COPS?",
//"AT+CPIN?",
//"AT+CGMSCLA SS?",
//"AT+CMUX?",
//"AT+CIPXMUX?",
//"AT+CGQMIN?",
//"AT+CDNSCFG?",
//"AT+CIPCSGP?",
//"AT+CGCLASS?",
//"AT+CIPSTATUS",
//"AT+IFC=2,2",
//"AT+CIPMUX=0",
//"AT+CIPMODE=1",
//"AT+CIPCCFG=8,2,1,0",
//"AT+CMUX=0,0,4,1,10,3,30,10,2"
//"AT+CSTT=\"%a\",\"\",\"\"",
//"AT+CIPSTATUS",
//"AT+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\"",
//"AT+CIICR",
//"AT+CIPSTATUS",
//"AT+CIFSR",
//"AT+CGATT?",
//"AT+CIPSCONT?",
//"AT+CIPSCONT",
//"AT+CSQ",
//"AT+CIPSTATUS",
//"AT+CIPSEND?",
//"AT+CIPSTART=\"%t\",\"%i\",\"%d\"",

