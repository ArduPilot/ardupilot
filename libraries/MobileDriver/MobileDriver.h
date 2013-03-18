#ifndef MobileDriver_h
#define MobileDriver_h

#include <inttypes.h>
#include <stdlib.h>
#include <utility/BetterStream.h>

class MobileDriver : public AP_HAL::BetterStream  {
public:
	static const uint8_t RESPONSE_BUFLEN = 80;
	// After 15 seconds of no reception of any kind of response from mobile, we restart.
	static const uint16_t TIMEOUT = 15 * 50;

	static const uint8_t PHONENUMBERLENGTH = 16;

	enum Progress_t {
		P_START = 0,

		P_WAITING_TRANSMIT_WS = 23,
		P_PRETRANSMIT = 25,
		P_TRANSMITTING = 28,
		P_DONE_TRANSMITTING = 29,

		P_WAITING_INITIAL_WS = 30,
		//P_SAW_INITIAL_WS = 31,

		P_BUFFERING_RESPONSE_LINE = 32,

		//P_SAW_TRAILING_WS = 40,
		P_RESPONSE_LINE_COMPLETE = 41,

		P_RECEIVING_DATALENGTH = 50,
		P_RECEIVING_SMSINDEX = 51,
		P_RECEIVING_DATA = 60,
		P_RECEIVING_SMS = 61,
		// For queries, one can add state for parsing the 2nd line.
		P_TRAILING_WS = 70,
		P_DONE = 80,
		P_RETURNED_FROM_INTERRUPT = 100
	};

	static const uint8_t INCOMING_SMS_MSGLENGTH = 10; // The real limit is 160 but we don't have that much space to waste.
	struct IncomingSMS {
		char index[4];
		char sender[PHONENUMBERLENGTH]; 		// null terminated.
		char text[INCOMING_SMS_MSGLENGTH];
		uint8_t textLength;
	};

	class State;
	struct Transition {
		prog_char const * _token;
		const State * _state;

		Transition(void) :
			_token(NULL),
			_state(NULL) {}

		Transition(const prog_char * token, const State * state) :
			_token(token),
			_state(state) {}
	};

	/*
	 * Base class for States. Pretty much just an implementation of the State pattern.
	 */
	class State {
	public:
		virtual void task(MobileDriver & outer) const = 0;
		void checkTimeout(MobileDriver & outer) const;
	};

	class DecisionState: public State {
	private:
		uint8_t _numTransitions;
		Transition const * _transitions;
	protected:
		void receiveResponseLine(MobileDriver & outer) const;
		const State* match(MobileDriver & outer) const;
	public:
		DecisionState(void);
		DecisionState(uint8_t numTransitions, Transition const * transitions);
		virtual void task(MobileDriver & outer) const;
		virtual void responseLineCompleted(MobileDriver & outer) const;
		const State* findSuccessor(MobileDriver & outer) const;
	};

	/*
	 * A State that sends no commands or queries to the mobile. It just waits for URCs or
	 * data to transmit.
	 */
	class DeadState: public DecisionState {
	public:
		virtual void task(MobileDriver & outer) const;
	};

	class SIMNotReadyState: public DeadState {
	public:
		virtual void task(MobileDriver & outer) const;
	};

	/*
	 * A DeadState which sends a keep-alive pseudo message every now and then to keep NATs
	 * going. When using MAVLink heartbeat messages, there is already enough link traffic
	 * to suppress keep-alives.
	 */
	class KeepAliveState: public DeadState {
	private:
		long _lastTime;
	public:
		virtual void task(MobileDriver & outer) const;
	};

	class CommandState: public DecisionState {
	public:
		CommandState(const prog_char* command, uint8_t numTransitions, Transition const * transitions);
		CommandState(const prog_char* command, uint8_t numArguments, uint8_t numTransitions, Transition const * transitions, ...);
		virtual void sendCommand(MobileDriver & outer) const;
		virtual void task(MobileDriver & outer) const;
	protected:
		void setArgs(uint8_t numArguments, va_list arguments);
		const char* _command;
		const char* _arguments[4]; // 4 ought to be enough for anybody.
	};

	class QueryState : public CommandState {
	public:
		QueryState(const prog_char* query, uint8_t numTransitions, Transition const * transitions);
		QueryState(const prog_char* query, uint8_t numArguments, uint8_t numTransitions, Transition const * transitions, ...);
	};

	class ReadPhonebookState : public QueryState {
	public:
		ReadPhonebookState(const prog_char* query, uint8_t numTransitions, Transition const * transitions,
				const State * continueState);
	protected:
		virtual void task(MobileDriver & outer) const;
		virtual void responseLineCompleted(MobileDriver & outer) const;
		void queryResult(MobileDriver & outer) const;
		const State* _continueState;
	};

	class SMSReceptionState : public QueryState {
	public:
		SMSReceptionState(const prog_char* command, uint8_t numTransitions, Transition const * transitions);
	protected:
		virtual void task(MobileDriver & outer) const;
		virtual void responseLineCompleted(MobileDriver & outer) const;
	};

	class SMSTransmissionState : public CommandState {
	public:
		SMSTransmissionState(const prog_char* command, uint8_t numTransitions, Transition const * transitions);
	protected:
		virtual void task(MobileDriver & outer) const;
		void responseLineCompleted(MobileDriver & outer) const;
	};

	class DataTransmissionState: public CommandState {
	public:
		DataTransmissionState(const prog_char* command, uint8_t numTransitions, Transition const * transitions);
		virtual void task(MobileDriver & outer) const;
	};

	class DataReceptionState: public State {
	public:
		virtual void task(MobileDriver & outer) const;
	};

	class SMSInterruptState: public State {
	public:
		virtual void task(MobileDriver & outer) const;
	};

	MobileDriver(void);
	void begin(BetterStream* mobile, int16_t rxSpace, int16_t txSpace);

	// Regular task that does all the work. Should be called frequently, like in fast_loop.
	void task(void);

	// At this message, we can mark the beginning of a new MAVLink message.
	void beginTransmitMessage(int16_t length);
	// At this message, we know that there is a complete MAVLink message in the buffer ready to be sent.
	void endTransmitMessage(int16_t length);

	// Public implementation of BetterStream methods. These are DUPLICATED from UARTDriver.
	// This could have been avoided by either:
	// 1) Making an implementation class of BetterStream
	// 2) Move all printing stuff out of Stream and into a separate PrintStream decorator over any Stream. Then we could make
	// MobileDriver a subclass of that decorator (and UARTDriver too).
    void print_P(const prog_char_t* s) {
        char    c;
        while ('\0' != (c = pgm_read_byte((const prog_char *)s++)))
                write(c);
    }

    void println_P(const prog_char_t* s) {
        print_P(s);
        println();
    }

    void printf(const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        vprintf(fmt, ap);
        va_end(ap);
    }

    /* No format checking on printf_P: can't currently support that on AVR */
    void _printf_P(const prog_char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        vprintf_P(fmt, ap);
        va_end(ap);
    }

    void vprintf(const char* fmt, va_list ap) {
        //print_vprintf((AP_HAL::Print*)this, 0, fmt, ap);
    }

    void vprintf_P(const prog_char* fmt, va_list ap) {
        //print_vprintf((AP_HAL::Print*)this, 1, fmt, ap);
    }

	// Implementation of BetterStream:
    int16_t available(void) {
    	return _rxBuffer.available();
    }

    int16_t txspace(void) {
    	return _txBuffer.txspace();
    }

    int16_t read(void) {
    	return _rxBuffer.read();
    }

    int16_t peek(void) {
    	return _rxBuffer.peek();
    }

    size_t write(uint8_t data) {
    	return _txBuffer.write(data);
    }

	private:
		/// Transmit/receive buffer descriptor.
		class Buffer {
		public:
			Buffer(void) :
				_bytes(NULL)
			{}

			bool begin(int16_t size) {
				if(_bytes) {
					free(_bytes);
				}
				if (!(_bytes = (uint8_t*)malloc(size))) return false;
				_head = _tail = size;
				_mask = size - 1;
				return true;
			}

			int16_t available(void) {
				return ((_head - _tail) & _mask);
			}

			int16_t read(void) {
				if (_head == _tail) return -1;
				int c = _bytes[_tail];
				_tail = (_tail + 1) & _mask;
				return c;
			}

			int16_t peek(void) {
				if (_head == _tail) return -1;
				int c = _bytes[_tail];
				return c;
			}

			void flush(void) {
				_head = _tail;
			}

			int16_t txspace(void) {
				return _mask - available();
			}

			int8_t write(uint8_t c) {
				int16_t i = (_head + 1) & _mask;
				if (i != _tail) {
					_bytes[_head] = c;
					_head = i;
					return 1;
				}
				return 0;
			}

		private:
			int16_t _head, _tail;				///< head and tail pointers
			int16_t _mask;					///< buffer size mask for pointer wrap
			uint8_t* _bytes;					///< pointer to allocated buffer
		};

		// Private implementation of back side of BetterStream.
	int16_t backend_available(void) {
		return _txBuffer.available();
	}

	int backend_read(void) {
		return _txBuffer.read();
	}

	int16_t backend_txspace(void) {
		return _rxBuffer.txspace();
	}

	void backend_write(uint8_t c) {
		_rxBuffer.write(c);
	}

protected:
	void add(uint8_t data) {
		_responseBuffer[_bufptr++] = data;
		if (_bufptr == RESPONSE_BUFLEN)
			_bufptr = 0;
	}

	void saveState(void) {
		_savedState = _state;
	}

	void beginState(const State* const state) {
		_state = state;
		_progress = P_START;
		_bufptr = 0;
		_timeout = 0;
		_numCRLF = 0;
	}

	void restartState(void) {
		beginState(_state);
	}

	void restoreState(void) {
		_state = _savedState;
		_savedState = NULL;
		_progress = P_RETURNED_FROM_INTERRUPT;
		_bufptr = 0;
		_numCRLF = 0;
	}

	// Pattern match of response string to tokens. There is a wildcard '?'.
	bool match(prog_char const * token) const {
		uint8_t tokenLength = strlen_P(token);
		if(_bufptr < tokenLength)
			return false;
		uint8_t idx;
		for (idx=0; idx<tokenLength; idx++) {
			char t = pgm_read_byte(token+idx);
			char i = _responseBuffer[idx];
			if (t != i && t != '?')
				return false;
		}
		return true;
	}

	// Implement in subclass. TODO: Not virtual.
	virtual const State * initialState(void) const = 0;
	virtual const State * dataTransmissionState(void) const = 0;
	virtual const State * resetState(void) const = 0;
	virtual const State * receiveSMSState(void) const = 0;
	virtual const State * matchInterrupts(void) const = 0;
	virtual const State * matchURCs(void) const = 0;
	virtual bool isConnectionOpen(const State* state) const = 0;

	BetterStream* _mobile;
	Buffer _txBuffer;
	Buffer _rxBuffer;

	// The State that we are in now.
	const State * _state;

	// How far we are through parsing input for the current state.
	Progress_t _progress;

	// Small buffer for mobile response, and index/length of same.
	// The extra 1 byte is for a trailing NUL.
	uint8_t _responseBuffer[RESPONSE_BUFLEN];
	uint16_t _bufptr;
	uint8_t _numCRLF;

	// Some states act like interrupts. Here the previous state is stored to be returned to.
	const State * _savedState;

	// for Receive and Transmit Data states: Data length.
	int16_t _numRxBytes;
	int16_t _numTxBytes;

	// The amount of time we have waited for some sort of response. If a limit is exceeded, a state or progress
	// reset is done to avoid getting stuck waiting.
	// The time is just the number of invocations of MobileDriver::task().
	// uint16_t _responseTimeout;
	uint16_t _timeout;
	uint8_t _txrxScheduling;

	// We really need public so that "inner classes" can access?
public:
	static int  numPhonebookEntriesRead;
	static char hostname[21];
	static char protocol[4];
	static char port[6];
	static char apn[21];
	static char pilotsNumber[PHONENUMBERLENGTH];
	static IncomingSMS  incomingSMS;

	static char dataTransmissionSize[4];
};

#endif

#include "SIM900Driver.h"
