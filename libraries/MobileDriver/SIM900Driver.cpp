#include <inttypes.h>
#include "SIM900Driver.h"
#include <HAL.h>

#define TRANSITIONS(t) sizeof(t)/sizeof(MobileDriver::Transition), t

using ::MobileDriver;

const prog_char R_OK[] PROGMEM = "OK";
const prog_char R_ERROR[] PROGMEM = "ERROR";
const prog_char R_SHUT_OK[] PROGMEM = "SHUT OK";
const prog_char R_ANYTHING[] PROGMEM = "";

const prog_char R_RDY[] PROGMEM = "RDY";
const prog_char R_ANY_CFUN[] PROGMEM = "+CFUN: ?";
const prog_char R_CGREG_1[] PROGMEM = "+CGREG: 1";
const prog_char R_CGREG_3[] PROGMEM = "+CGREG: 3";
const prog_char R_CGREG_5[] PROGMEM = "+CGREG: 5";
const prog_char R_IPD[] PROGMEM = "+IPD,";
const prog_char R_CMTI[] PROGMEM = "+CMTI: ";
const prog_char R_CONNECT_OK[] PROGMEM = "CONNECT OK";
const prog_char R_ALREADY_CONNECT[] PROGMEM = "ALREADY CONNECT";
const prog_char R_SEND_OK[] PROGMEM = "SEND OK";
const prog_char R_SEND_FAIL[] PROGMEM = "SEND FAIL";
const prog_char R_SIMNOTREADY[] PROGMEM = "+CPIN: NOT READY";
const prog_char R_PHONEBOOKENTRY[] PROGMEM = "+CPBF:";
// If AT is coming back as prefix of a response then we know it is really an echo. Run the anti echo command again.
const prog_char R_ECHO_IS_ON[] PROGMEM = "AT";
const prog_char R_RECEIVING_SMS[] PROGMEM = "+CMGR:";
const prog_char R_SENT_SMS[] = "+CMGS:";

// The repeated init is for the case when the modem was turned on after first init was sent.
// That would cause the E0 to get lost and the result is echo and confusion.
// Simpler solution wanted.
const prog_char C_INIT0[] PROGMEM = "\r\nATV1 E0 X1 S0=0 +CMEE=0";
const prog_char C_READPHONEBOOK[] PROGMEM = "AT+CPBS=\"SM\";+CPBF";
const prog_char C_CONFIGURATION1[] PROGMEM = "AT+CGEREP=2,0;+CIURC=0;+CFGRI=1;+CIPHEAD=1;+CIPSPRT=1;+CMGF=1;+CIPCSGP=1,\"%s\"";
const prog_char C_START1[] PROGMEM = "AT+CSTT";
const prog_char C_START2[] PROGMEM = "AT+CIICR";
const prog_char C_START3[] PROGMEM = "AT+CIFSR";
const prog_char C_DATA_CONNECTION[] PROGMEM = "AT+CIPSTART=\"%s\",\"%s\",\"%s\"";
const prog_char C_RESET[] PROGMEM = "AT+CFUN=1,1";
const prog_char C_RESET_IP[] PROGMEM = "AT+CIPSHUT";
const prog_char C_RECEIVE_SMS[] PROGMEM = "AT+CMGR=%s;+CMGD=%s";
const prog_char C_TRANSMIT_SMS[] PROGMEM = "AT+CMGS=\"%s\"";
const prog_char C_TRANSMIT_DATA[] PROGMEM = "AT+CIPSEND=%s";

extern const MobileDriver::ReadPhonebookState S_READPHONEBOOK;
extern const MobileDriver::CommandState S_CONFIGURATION1;
extern const MobileDriver::CommandState S_RESET;
extern const MobileDriver::CommandState S_INITIAL;
extern const MobileDriver::SMSReceptionState S_RECEIVE_SMS;

const MobileDriver::Transition T_READPHONEBOOK[] = {
		MobileDriver::Transition(R_OK, &S_CONFIGURATION1),
		MobileDriver::Transition(R_ERROR, &S_INITIAL),
		MobileDriver::Transition(R_PHONEBOOKENTRY, &S_READPHONEBOOK)
};

const MobileDriver::ReadPhonebookState S_READPHONEBOOK =
		MobileDriver::ReadPhonebookState(C_READPHONEBOOK, 3, T_READPHONEBOOK, &S_CONFIGURATION1);

const MobileDriver::Transition T_INITIAL[] = {
		MobileDriver::Transition(R_OK, &S_READPHONEBOOK),
//		MobileDriver::Transition(R_ERROR, &S_RESET),
		MobileDriver::Transition(R_ERROR, &S_INITIAL),
		MobileDriver::Transition(R_ECHO_IS_ON, &S_INITIAL),
};

const MobileDriver::CommandState S_INITIAL =
		MobileDriver::CommandState(C_INIT0, 0, TRANSITIONS(T_INITIAL));

const MobileDriver::Transition T_RESET[] = {
		MobileDriver::Transition(R_OK, &S_INITIAL),
		MobileDriver::Transition(R_ERROR, &S_INITIAL)
};

const MobileDriver::CommandState S_RESET =
		MobileDriver::CommandState(C_RESET, 0, TRANSITIONS(T_RESET));

const MobileDriver::Transition T_RESET_IP[] = { MobileDriver::Transition(
		R_SHUT_OK, &S_CONFIGURATION1), MobileDriver::Transition(R_ERROR, &S_CONFIGURATION1) };

const MobileDriver::CommandState S_RESET_IP =
		MobileDriver::CommandState(C_RESET_IP, 0, TRANSITIONS(T_RESET_IP));

const MobileDriver::SIMNotReadyState S_SIMNOTREADY =
		MobileDriver::SIMNotReadyState();

const MobileDriver::DeadState S_DATA_CONNECTION_OPENING =
		MobileDriver::DeadState();

const MobileDriver::DeadState S_DATA_CONNECTION_OPEN =
		MobileDriver::DeadState();

const MobileDriver::Transition T_DATA_CONNECTION[] = { MobileDriver::Transition(
		R_OK, &S_DATA_CONNECTION_OPENING), MobileDriver::Transition(R_ERROR, &S_INITIAL) };

const MobileDriver::CommandState S_DATA_CONNECTION =
		MobileDriver::CommandState(C_DATA_CONNECTION, 3, TRANSITIONS(T_DATA_CONNECTION),
				&MobileDriver::protocol, &MobileDriver::hostname,&MobileDriver::port);

const MobileDriver::SMSInterruptState S_SMS_INTERRUPT =
		MobileDriver::SMSInterruptState();

extern const MobileDriver::SMSTransmissionState S_TRANSMIT_SMS;

const MobileDriver::Transition T_TRANSMIT_SMS[] = { MobileDriver::Transition(
		R_SENT_SMS, &S_TRANSMIT_SMS), MobileDriver::Transition(R_OK, NULL)};

const MobileDriver::SMSTransmissionState S_TRANSMIT_SMS =
		MobileDriver::SMSTransmissionState(C_TRANSMIT_SMS, TRANSITIONS(T_TRANSMIT_SMS));

const MobileDriver::Transition T_RECEIVE_SMS[] = { MobileDriver::Transition(
		R_RECEIVING_SMS, &S_RECEIVE_SMS), MobileDriver::Transition(
				R_OK, &S_TRANSMIT_SMS)};

// This uses an argument but that is added in the ctor.
const MobileDriver::SMSReceptionState S_RECEIVE_SMS =
		MobileDriver::SMSReceptionState(C_RECEIVE_SMS, TRANSITIONS(T_RECEIVE_SMS));

const MobileDriver::Transition T_START3[] = {
		MobileDriver::Transition(R_ANYTHING, &S_DATA_CONNECTION),
		MobileDriver::Transition(R_ERROR, &S_RESET_IP) };

const MobileDriver::CommandState S_START3 =
		MobileDriver::CommandState(C_START3, 0, TRANSITIONS(T_START3));

const MobileDriver::Transition T_START2[] = {
		MobileDriver::Transition(R_OK, &S_START3),
		MobileDriver::Transition(R_ERROR, &S_RESET_IP) };

const MobileDriver::CommandState S_START2 =
		MobileDriver::CommandState(C_START2, 0, TRANSITIONS(T_START2));

const MobileDriver::Transition T_START1[] = {
		MobileDriver::Transition(R_OK, &S_START2),
		MobileDriver::Transition(R_ERROR, &S_RESET_IP) };

const MobileDriver::CommandState S_START1 =
		MobileDriver::CommandState(C_START1, 0, TRANSITIONS(T_START1));

const MobileDriver::Transition T_CONFIGURATION1[] = {
		MobileDriver::Transition(R_OK, &S_START1),
		MobileDriver::Transition(R_ERROR, &S_RESET_IP) };

const MobileDriver::CommandState S_CONFIGURATION1 =
		MobileDriver::CommandState(C_CONFIGURATION1, 1, TRANSITIONS(T_CONFIGURATION1),
				&MobileDriver::apn);

const MobileDriver::Transition T_DATA_TRANSMISSION[] = {
		MobileDriver::Transition(R_SEND_OK, &S_DATA_CONNECTION_OPEN),
		MobileDriver::Transition(R_SEND_FAIL, &S_DATA_CONNECTION_OPEN),
		MobileDriver::Transition(R_ERROR, &S_RESET_IP) };

// This uses an argument but that is added in the ctor.
const MobileDriver::DataTransmissionState S_DATA_TRANSMISSION =
		MobileDriver::DataTransmissionState(C_TRANSMIT_DATA, TRANSITIONS(T_DATA_TRANSMISSION));

const MobileDriver::DataReceptionState S_DATA_RECEPTION =
		MobileDriver::DataReceptionState();

/*
 * Responses to URCs (Unsolicited Return Codes) from mobile.
 */
const MobileDriver::Transition URCTransitions[] = {
		MobileDriver::Transition(R_RDY, &S_INITIAL), MobileDriver::Transition(
				R_ANY_CFUN, &S_INITIAL), MobileDriver::Transition(R_CGREG_1,
				&S_INITIAL), MobileDriver::Transition(R_CGREG_3, &S_INITIAL),
		MobileDriver::Transition(R_CGREG_5, &S_INITIAL),
		MobileDriver::Transition(R_CONNECT_OK, &S_DATA_CONNECTION_OPEN),
		MobileDriver::Transition(R_ALREADY_CONNECT, &S_RESET_IP),
		MobileDriver::Transition(R_SEND_OK, &S_DATA_CONNECTION_OPEN),
		MobileDriver::Transition(R_SEND_FAIL, &S_DATA_CONNECTION_OPEN),
		MobileDriver::Transition(R_SIMNOTREADY, &S_SIMNOTREADY)
};

const uint8_t numURCTransitions = sizeof(URCTransitions) / sizeof(MobileDriver::Transition);

/*
 * URC interrupts come as <CR><LF>text data....
 * These cannot be delimited by trailing <CR><LF>
 */
const MobileDriver::Transition URCInterrupts[] = {
		MobileDriver::Transition(R_IPD, &S_DATA_RECEPTION),
		MobileDriver::Transition(R_CMTI, &S_SMS_INTERRUPT)
};

const uint8_t numURCInterrupts = sizeof(URCInterrupts) / sizeof(MobileDriver::Transition);

const MobileDriver::State* SIM900Driver::matchURCs(void) const {
	uint8_t i;
	for (i = 0; i < numURCTransitions; i++) {
		if (match(URCTransitions[i]._token)) {
			return URCTransitions[i]._state;
		}
	}
	return NULL;
}

const MobileDriver::State* SIM900Driver::matchInterrupts (void) const {
	uint8_t i;
	for (i = 0; i < numURCInterrupts; i++) {
		if (match(URCInterrupts[i]._token)) {
			return URCInterrupts[i]._state;
		}
	}
	return NULL;
}

const MobileDriver::State* SIM900Driver::initialState(void) const { return &S_INITIAL; }
const MobileDriver::State* SIM900Driver::dataTransmissionState(void) const { return &S_DATA_TRANSMISSION; }
const MobileDriver::State* SIM900Driver::resetState(void) const { return &S_RESET; }
const MobileDriver::State* SIM900Driver::receiveSMSState(void) const { return &S_RECEIVE_SMS; }
bool SIM900Driver::isConnectionOpen(const MobileDriver::State* state) const { return state == &S_DATA_CONNECTION_OPEN; }
