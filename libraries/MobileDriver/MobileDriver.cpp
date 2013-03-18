#include <stdarg.h>
#include <stdio.h>
#include "MobileDriver.h"

int  MobileDriver::numPhonebookEntriesRead;
char MobileDriver::hostname[21] = "www.sky-cam.dk";
char MobileDriver::protocol[4] = "UDP";
char MobileDriver::port[6] = "10000";
char MobileDriver::apn[21] = "internet";
char MobileDriver::pilotsNumber[16] = "";
MobileDriver::IncomingSMS MobileDriver::incomingSMS;
char MobileDriver::dataTransmissionSize[4];

MobileDriver::MobileDriver() :
		_mobile(NULL),
		_txBuffer(),
		_rxBuffer()
		{
}

void MobileDriver::State::checkTimeout(MobileDriver & outer) const {
	if (outer._timeout++ >= TIMEOUT) {
		// Send some harmless dummy spaces. This finishes messages that were never sent (for whatever crazy reason).
		int16_t numSpaces = outer._mobile->txspace();
		if(numSpaces>62) numSpaces = 62;
		for(int16_t i=0; i<numSpaces; i++) {
			outer._mobile->write(' ');
		}
		outer._mobile->write('\r');
		outer._mobile->write('\n');
		outer.restartState();
	}
}

MobileDriver::DecisionState::DecisionState() :
		_numTransitions(0), _transitions(NULL) {
}

MobileDriver::DecisionState::DecisionState(
		uint8_t numTransitions,
		Transition const * transitions) :
		_numTransitions(numTransitions),
		_transitions(transitions) {
}

/*
 * Search for an applicable transition. 
 */
const MobileDriver::State* MobileDriver::DecisionState::match(MobileDriver & outer) const {
	uint8_t i;
	for (i = 0; i < _numTransitions; i++) {
		if (outer.match(_transitions[i]._token)) {
			return _transitions[i]._state;
		}
	}
	return NULL;
}

/*
 * Receive response message.
 */
void MobileDriver::DecisionState::receiveResponseLine(MobileDriver & outer) const {
	int16_t data;

	// If there is no (more) data, quit.
	if ((data = outer._mobile->read()) < 0) {
		return;
	}

	/*
	 *  This will accept one or more CR/LF chars as a response header,
	 *  and consume them all.
	 *  State is changed after a CR/LF character is received. The second CR/LF character is
	 *  simply ignored. This means that some multi line responses with single CRs or LFs in
	 *  them will be interpreted as multiple one liners; maybe this is not ideal.
	 *  2 solution ideas: 1) Check for both CRLF always, or 2) peek when expecting
	 *  multiline responses and see if there is the 2nd CR or LF.
	 */
	/*
	if (data == '\r' || data == '\n') {
		if (outer._progress == P_WAITING_INITIAL_WS) {
			outer._progress = P_BUFFERING_RESPONSE_LINE;
		} else if (outer._progress == P_BUFFERING_RESPONSE_LINE && outer._bufptr) {
			outer._progress = P_RESPONSE_LINE_COMPLETE;

		}
		// throw away whitespace.
	} else {
		outer.add((uint8_t) data);
	}
	*/

	if (data == '\r' || data == '\n') {
		if (outer._progress == P_WAITING_INITIAL_WS) {
			if (outer._numCRLF) {
				outer._progress = P_BUFFERING_RESPONSE_LINE;
				outer._numCRLF = 0; // reset for next state.
			}
			else outer._numCRLF = 1;
		} else if (outer._progress == P_BUFFERING_RESPONSE_LINE) {
			if (outer._numCRLF) {
				outer._progress = P_RESPONSE_LINE_COMPLETE;
				outer._numCRLF = 0;
			}
			else outer._numCRLF = 1;
		}
		// throw away whitespace.
	} else {
		outer.add((uint8_t) data);
		outer._numCRLF = 0;
	}
}

const MobileDriver::State* MobileDriver::DecisionState::findSuccessor(MobileDriver & outer) const {
	const State* result = match(outer);
	// States switched to by normal response matching are never immediate but just deferred.
	if (result == NULL) {
		// Try if the response is really no response at all, but a known URC.
		// Unfortunately, the SIM900 does not hold back URCs between the start of an AT command
		// and the end of its response so we can expect them almost any time!!
		result = outer.matchURCs();
	}
	return result;
}

void MobileDriver::DecisionState::responseLineCompleted(MobileDriver & outer) const {
	const State* successor = findSuccessor(outer);
	if (successor == NULL) {
		// We have a complete line but no known code or even URC matched. Panic. Restart state.
		outer.restartState();
	} else {
		// Found successor, apply that.
		outer.beginState(successor);
	}
}

void MobileDriver::DecisionState::task(MobileDriver & outer) const {
	// When the command has been sent, we switch to waiting for <CR><LF>response<CR><LF>...
	// When returning from URC "interrupts" aka receiving data, we wait for the original RC still outstanding.
	checkTimeout(outer);
	if (outer._progress == P_RETURNED_FROM_INTERRUPT) {
		outer._progress = P_WAITING_INITIAL_WS;
	}

	if (outer._progress >= P_WAITING_INITIAL_WS) {

		receiveResponseLine(outer);

		if(outer._progress == P_BUFFERING_RESPONSE_LINE) {
			const State* interrupt = outer.matchInterrupts();
			if (interrupt != NULL) {
				outer.saveState();
				outer.beginState(interrupt);
			}
		} else  if (outer._progress == P_RESPONSE_LINE_COMPLETE) {
			responseLineCompleted(outer);
		}
	}
}

MobileDriver::CommandState::CommandState(
		const prog_char* command,
		uint8_t numTransitions,
		Transition const * transitions
		): DecisionState(numTransitions, transitions), _command(command) {
}

MobileDriver::CommandState::CommandState(
		const prog_char* command,
		uint8_t numArguments,
		uint8_t numTransitions,
		Transition const * transitions,
		...) : DecisionState(numTransitions, transitions), _command(command) {
    va_list ap;
    va_start(ap, transitions);
    setArgs(numArguments, ap);
    va_end(ap);
}

void MobileDriver::CommandState::setArgs(uint8_t numArguments, va_list arguments) {
    if(numArguments>4) numArguments=4;
    uint8_t i;
    for(i=0; i<numArguments; i++) {
        _arguments[i] = va_arg(arguments, const char*);
    }
}

void MobileDriver::CommandState::task(MobileDriver & outer) const {
	if (outer._progress == P_RETURNED_FROM_INTERRUPT) {
		outer.restartState();
	}

	if (outer._progress == P_START) {
		// Make sure there is space for the command and a trailing CR.
		if ((size_t) (outer._mobile->txspace()) > strlen_P(_command)+20) {
			sendCommand(outer);
			outer._progress = P_WAITING_INITIAL_WS;
		}
	}
	DecisionState::task(outer);
}

void MobileDriver::CommandState::sendCommand(MobileDriver & outer) const {
	outer._mobile->printf_P(_command, _arguments[0], _arguments[1], _arguments[2], _arguments[3]);
	outer._mobile->print_P(PSTR("\r\n"));
}

MobileDriver::QueryState::QueryState (
		const prog_char* query,
		uint8_t numArguments,
		uint8_t numTransitions,
		Transition const * transitions,
		...) :
		CommandState(query, numTransitions, transitions) {
    va_list ap;
    va_start(ap, transitions);
    setArgs(numArguments, ap);
    va_end(ap);
}

MobileDriver::QueryState::QueryState (
		const prog_char* query,
		uint8_t numTransitions,
		Transition const * transitions
		) :
		CommandState(query, numTransitions, transitions) {
}

MobileDriver::ReadPhonebookState::ReadPhonebookState (
		const prog_char* query,
		uint8_t numTransitions,
		Transition const * transitions,
		const State* continueState) :
		QueryState(query, numTransitions, transitions), _continueState(continueState) {
}

// This impl. of task is for repeating the read of the phonebook until is is eventually loaded from the slow SIM card.
void MobileDriver::ReadPhonebookState::task(MobileDriver & outer) const {
	// Only read if not done already.
	if (outer._progress == P_START && MobileDriver::numPhonebookEntriesRead != 0) {
		outer.beginState(_continueState);
		return;
	}
	QueryState::task(outer);
}

void MobileDriver::ReadPhonebookState::responseLineCompleted(MobileDriver & outer) const {
		const State * successor = findSuccessor(outer);
		if(successor==this) { // there is a result of the query (the response begins with +CPBF,
			// for which a transition to "this" was configured).
			queryResult(outer);
			outer._progress = P_WAITING_INITIAL_WS;
			outer._bufptr = 0;
		} else if(successor == _continueState && MobileDriver::numPhonebookEntriesRead == 0) {
				// WTF, there were no phonebook entries? Maybe not read from SIM card, try again.
			outer.restartState();
		} else {
			QueryState::responseLineCompleted(outer);
		}
}

#define NAMELENGTH 20

void MobileDriver::ReadPhonebookState::queryResult(MobileDriver & outer) const {
	// Set a NUL at end of string.
	// parseState._responseBuffer[parseState._bufptr] = 0;
	char number[PHONENUMBERLENGTH];	  // We only bother with the first few characters of the number.
	char name[NAMELENGTH+1]; 		  // oh thats not much. 20.
	uint8_t i=0, j=0;
	while(i<outer._bufptr && outer._responseBuffer[i] != '"') i++;
	i++; // skip 1st quote also.
	while(j<PHONENUMBERLENGTH-1 && i<outer._bufptr && outer._responseBuffer[i] != '"') {
		number[j++] = outer._responseBuffer[i++];
	}
	number[j] = 0;
	// skip 2 commas.
	i++;
	while(i<outer._bufptr && outer._responseBuffer[i] != '"') i++;
	i++; j=0;
	while(j<NAMELENGTH && i<outer._bufptr && outer._responseBuffer[i] != '"') {
		name[j++] = outer._responseBuffer[i++];
	}
	name[j] = 0;

	if(!strcasecmp_P(name, PSTR("my pilot"))) {
		strcpy(MobileDriver::pilotsNumber, number);
	}

	else if(strlen(number)==3) {
		if(number[0]=='#' && number[2]=='#') {
			switch(number[1]) {
			case '1':
				strcpy(MobileDriver::apn, name); break;
			case '2':
				strcpy(MobileDriver::protocol, name); break;
			case '3':
				strcpy(MobileDriver::hostname, name); break;
			case '4':
				strcpy(MobileDriver::port, name); break;
			default: break;
			}
		}
	}

	outer._progress = P_WAITING_INITIAL_WS;
	outer._bufptr = 0;
	MobileDriver::numPhonebookEntriesRead++;
}

MobileDriver::SMSReceptionState::SMSReceptionState (
		const prog_char* query,
		uint8_t numTransitions,
		Transition const * transitions) :
		QueryState(query, 2, numTransitions, transitions, &MobileDriver::incomingSMS.index, &MobileDriver::incomingSMS.index) {
}

void MobileDriver::SMSReceptionState::task(MobileDriver & outer) const {
	if (outer._progress == P_RECEIVING_SMS) {
		int16_t data = outer._mobile->peek();
		if (data<0) return; // Achtung! Timeouts
		if(data=='\n'||data=='\r') {
			if(outer._numCRLF) {
				// We now know the SMS text is finished.
				outer._progress = P_WAITING_INITIAL_WS;
				outer._numCRLF = outer._bufptr = 0;
				// Overwrite the final LF with a NUL
				MobileDriver::incomingSMS.textLength--;
				MobileDriver::incomingSMS.text[MobileDriver::incomingSMS.textLength] = 0;
				return;
			}
			outer._numCRLF++;
		} else {
			outer._numCRLF = 0;
		}
		// Consume it.
		outer._mobile->read();
		// Store data if there is enough space.
		if (MobileDriver::incomingSMS.textLength < INCOMING_SMS_MSGLENGTH) {
			MobileDriver::incomingSMS.text[MobileDriver::incomingSMS.textLength++] = data;
		}
	}
	else {
		QueryState::task(outer);
	}
}

void MobileDriver::SMSReceptionState::responseLineCompleted(MobileDriver & outer) const {
	const State* successor = findSuccessor(outer);
	if(successor==this) { // there is a result of the query (the response begins with +C).
		// Process the SMS header here.
		// Now enter the special SMS mode.
		outer.incomingSMS.textLength = 0;
		uint8_t i=0, j=0;
		do {
			if(i>=outer._bufptr) break;
			if(outer._responseBuffer[i++] == '"') j++;
		} while (j<3);
		if (j==3) {
			j=0;
			while(j<PHONENUMBERLENGTH-1 && i<outer._bufptr && outer._responseBuffer[i] != '"') {
				outer.incomingSMS.sender[j++] = outer._responseBuffer[i++];
			}
			outer.incomingSMS.sender[j] = 0;
		}
		outer._progress = P_RECEIVING_SMS;
	}
	else {
		outer.beginState(successor);
	}
}

MobileDriver::SMSTransmissionState::SMSTransmissionState (
		const prog_char* command,
		uint8_t numTransitions,
		Transition const * transitions) :
		CommandState(command, 1, numTransitions, transitions, MobileDriver::incomingSMS.sender) {
		//CommandState(command, numTransitions, transitions) {
}

void MobileDriver::SMSTransmissionState::task(MobileDriver & outer) const {
	// Are we done transmitting already?
	if (outer._progress >= P_WAITING_INITIAL_WS) {
		DecisionState::task(outer);
		return;
	}

	// First send the command..
	if (outer._progress == P_START) {
		if(outer._mobile->txspace() >= 20) {
			sendCommand(outer);
			outer._progress = P_WAITING_TRANSMIT_WS;
		}
	}

	else if (outer._progress == P_BUFFERING_RESPONSE_LINE) {
		DecisionState::task(outer);
	}

	else if (outer._progress >= P_WAITING_TRANSMIT_WS) {
		int16_t prompt = outer._mobile->peek();
		if (prompt == '\n' || prompt == '\r') {
			// Consume CRs and LFs.
			outer._mobile->read();
		} else if (prompt == '>' || prompt == ' ') {
			outer._mobile->read(); // consume it.
			if (outer._progress < P_TRANSMITTING)
				outer._progress = P_TRANSMITTING;
		}
		// After seeing input which is neither CR/LF nor > nor space nor nothing, we have a response.
		else if (prompt >= 0) {
			// Receiving anything else at this stage means that an interrupt-like URC was received.
			outer._progress = P_BUFFERING_RESPONSE_LINE;
		}

		// Then send the data in the largest possible chunks.
		if (outer._progress == P_TRANSMITTING) {
			int16_t numBytes, i;

			numBytes = MobileDriver::incomingSMS.textLength;
			i = outer._mobile->txspace()-1;

			if (i < numBytes)
				numBytes = i;

			for (i=0; i<numBytes; i++) {
				outer._mobile->write(MobileDriver::incomingSMS.text[i]);
			}

			MobileDriver::incomingSMS.textLength -= numBytes;
			if (MobileDriver::incomingSMS.textLength==0) {
				outer._mobile->write(26);
				outer._progress = P_DONE_TRANSMITTING;
			}
		}
	}
}

void MobileDriver::SMSTransmissionState::responseLineCompleted(MobileDriver & outer) const {
	// After SMS send got the first +CMGS response, stay here.
	const State* successor = findSuccessor(outer);
	if (successor == this) {
		outer._progress = P_WAITING_INITIAL_WS;
		outer._bufptr = 0;
	} else {
		outer.restoreState();
	}
}

void MobileDriver::DeadState::task(MobileDriver & outer) const {
	int numBytes;

	// Dead states never send any commands, so skip to waiting for whitespace from mobile.
	if (outer._progress == P_START) {
		outer._progress = P_WAITING_INITIAL_WS;
	}

	// See if user application wanted to transmit data, and in that case do it.
	// A transmission is started only when an URC reception is not in progress. 
	// Because the GCS (currently) buffers data as whole MAVLink messages at a time, it is simple
	// to get whole messages i UDP package: Any data waiting to be sent is one or more complete
	// MAVLink messages.
	// TODO: Not all dead states should support transmit.

	// Balance between prioritizing input and prioritizing output. Mission Planner sends so few messages that we barely hear them
	// - QGroundStation sends so many that we have to ignore it sometimes...

	if ((outer._progress == P_WAITING_INITIAL_WS || outer._progress == P_RETURNED_FROM_INTERRUPT) &&
			outer.isConnectionOpen(this)) {
		if ((numBytes = outer.backend_available()) > 0) {
			outer._numTxBytes = (size_t) numBytes;
			outer.beginState(outer.dataTransmissionState());
			// Switch immediately to transmit state;
			return;
		}
	}

	DecisionState::task(outer);
}

void MobileDriver::SIMNotReadyState::task(MobileDriver & outer) const {
	// Do something to indicate the problem, if required. LED on, ....
	// fprintf(stderr, "SIM NOT READY DETECTED!\n");
	// Bloody heck, We can do nothing but to try restart the modem.
	outer.beginState(outer.resetState());
}

MobileDriver::DataTransmissionState::DataTransmissionState(
		const prog_char* command,
		uint8_t numTransitions,
		Transition const * transitions) :
		CommandState(command, 1, numTransitions, transitions, &MobileDriver::dataTransmissionSize) {
}

void MobileDriver::DataTransmissionState::task(MobileDriver & outer) const {
	int16_t prompt;
	int16_t numBytes;
	int16_t tmpNumBytes;
	int16_t i;

	checkTimeout(outer);

	// An interrupt can only have happened after waiting for the prompt. Return to there.
	if (outer._progress == P_RETURNED_FROM_INTERRUPT) {
		outer._progress = P_WAITING_TRANSMIT_WS;
	}

	// Are we done transmitting already?
	if (outer._progress >= P_WAITING_INITIAL_WS) {
		DecisionState::task(outer);
		return;
	}

	// First send the command..
	if (outer._progress == P_START) {
		// Make sure there is space for the command and a trailing CR.
		if ((size_t) outer._mobile->txspace() >= 20) {
			sprintf(MobileDriver::dataTransmissionSize,"%d",outer._numTxBytes);
			sendCommand(outer);
			outer._progress = P_WAITING_TRANSMIT_WS;
		}
	}

	// Then wait for "> " from mobile (skipping this step caused the mobile to lose data!)
	// UNFORTUNATELY (cheap SIM900 crap!) URCs may come mixed with the > and we are chanceless.
	// We need an extra timeout implementation for
	if (outer._progress == P_WAITING_TRANSMIT_WS) {
		// We surely would prefer to have the peek operation!
		// This allows us to check for the > without consuming other data.
		prompt = outer._mobile->peek();
		//prompt = outer.mobile->read();
		if (prompt == '\n' || prompt == '\r') {
			outer._mobile->read();
		} else {
			if (prompt == '>' /* || prompt == ' '*/) {
				outer._mobile->read(); // consume it.
				outer._progress = P_TRANSMITTING;
			} else if (prompt >= 0) {
				// Receiving anything else at this stage means that an interrupt-like URC was received.
				outer._progress = P_BUFFERING_RESPONSE_LINE;
				return; // now the decision state dung should handle it. The most recent input char was not consumed.
			}
		}
	}

	// Then send the data in the largest possible chunks.
	if (outer._progress == P_TRANSMITTING) {
		numBytes = outer._numTxBytes;

		tmpNumBytes = outer.backend_available();

		if (tmpNumBytes < numBytes)
			numBytes = tmpNumBytes;

		tmpNumBytes = outer._mobile->txspace();

		if (tmpNumBytes < numBytes)
			numBytes = tmpNumBytes;

		for (i = 0; i < numBytes; i++) {
			outer._mobile->write(outer.backend_read());
		}

		outer._numTxBytes -= numBytes;
		if (outer._numTxBytes == 0) {
			outer._progress = P_WAITING_INITIAL_WS;
			outer._bufptr = 0;
			if(outer._mobile->peek()==' ') outer._mobile->read(); // consume space trailing prompt.
		}
	}
}

void MobileDriver::DataReceptionState::task(MobileDriver & outer) const {
	int16_t numBytes;
	int16_t tmpNumBytes;

	if (outer._progress == P_START) {
		outer._numRxBytes = 0;
		outer._progress = P_RECEIVING_DATALENGTH;
		outer._timeout = 0;
	}

	// First, get the byte count terminated by ':' 
	if (outer._progress == P_RECEIVING_DATALENGTH) {
		int data = outer._mobile->read();
		if (data < 0)
			return;
		if (data == ':') {
			outer._progress = P_RECEIVING_DATA;
		} else {
			outer._numRxBytes *= 10;
			outer._numRxBytes += (data - '0');
		}
	}

	// Then get the data in the largest possible chunks.
	if (outer._progress == P_RECEIVING_DATA) {
		numBytes = outer._numRxBytes;

		tmpNumBytes = outer._mobile->available();

		if (tmpNumBytes < numBytes)
			numBytes = tmpNumBytes;

		tmpNumBytes = outer.backend_txspace();

		if (tmpNumBytes < numBytes)
			numBytes = tmpNumBytes;

		if (numBytes < 0)
			numBytes = 0;

		int16_t i;
		for (i = 0; i < numBytes; i++) {
			outer.backend_write(outer._mobile->read());
		}
		outer._numRxBytes -= numBytes;
		if (outer._numRxBytes == 0) {
			outer.restoreState();
		}
	}
}

void MobileDriver::SMSInterruptState::task(MobileDriver & outer) const {
	if (outer._progress == P_START) {
		outer._numRxBytes = 0;
		outer._progress = P_RECEIVING_SMSINDEX;
	}
	if (outer._progress == P_RECEIVING_SMSINDEX) {
		int16_t data = outer._mobile->read();
		if (data < 0)
			return;
		if (data == '\n') {
			MobileDriver::incomingSMS.index[outer._numRxBytes] = 0;
			outer.beginState(outer.receiveSMSState());
		} else if (data >= '0' && data <= '9') {
			MobileDriver::incomingSMS.index[outer._numRxBytes++] = data;
		}
	}
}

void MobileDriver::begin(BetterStream* mobile, int16_t rxSpace, int16_t txSpace) {
	_mobile = mobile;
	_rxBuffer.begin(rxSpace);
	_txBuffer.begin(txSpace);
	// The application is expected to init the serial port! Not we. However it might
	// make sense to do it anyway, reducing buffer sizes if possible.
	// With 57600 baud and 50 invocations/sec, there should still be enough space for
	// receiving 116 bytes though.
	// mobile->begin(rxSpace, txSpace);
	beginState(_savedState = initialState());
}

/*
 * Begin message and end message events may be invoked from GCS_MAVLink!
 * Right now, they seem not necessary.
 */
void MobileDriver::beginTransmitMessage(int16_t length) {
}
void MobileDriver::endTransmitMessage(int16_t length) {
}


/*
 * Main driver of the state machine.
 */
void MobileDriver::task(void) {
	do {
		_state->task(*this);
	} while (_mobile->available() > 0);
}
