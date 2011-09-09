// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	HIL.h
/// @brief	Interface definition for the various Ground Control System protocols.

#ifndef __HIL_H
#define __HIL_H

# if HIL_MODE != HIL_MODE_DISABLED

#include <BetterStream.h>
#include <AP_Common.h>
#include <APM_BinComm.h>
#include <GPS.h>
#include <Stream.h>
#include <stdint.h>

///
/// @class	HIL
/// @brief	Class describing the interface between the APM code 
///			proper and the HIL implementation.
///
/// HIL' are currently implemented inside the sketch and as such have
/// access to all global state.  The sketch should not, however, call HIL
/// internal functions - all calls to the HIL should be routed through
/// this interface (or functions explicitly exposed by a subclass).
///
class HIL_Class
{
public:
	
	/// Startup initialisation.
	///
	/// This routine performs any one-off initialisation required before
	/// HIL messages are exchanged.
	///
	/// @note The stream is expected to be set up and configured for the
	///       correct bitrate before ::init is called.
	///
	/// @note The stream is currently BetterStream so that we can use the _P
	///	      methods; this may change if Arduino adds them to Print.
	///
	/// @param	port		The stream over which messages are exchanged.
	///
	void		init(BetterStream *port) { _port = port; }

	/// Update HIL state.
	///
	/// This may involve checking for received bytes on the stream,
	/// or sending additional periodic messages.
	void		update(void) {}

	/// Send a message with a single numeric parameter.
	///
	/// This may be a standalone message, or the HIL driver may
	/// have its own way of locating additional parameters to send.
	///
	/// @param	id			ID of the message to send.
	/// @param	param		Explicit message parameter.
	///
	void		send_message(uint8_t id, int32_t param = 0) {}

	/// Send a text message.
	///
	/// @param	severity	A value describing the importance of the message.
	/// @param	str			The text to be sent.
	///
	void		send_text(uint8_t severity, const char *str) {}

	/// Send acknowledgement for a message.
	///
	/// @param	id			The message ID being acknowledged.
	/// @param	sum1		Checksum byte 1 from the message being acked.
	/// @param	sum2		Checksum byte 2 from the message being acked.
	///
	void		acknowledge(uint8_t id, uint8_t sum1, uint8_t sum2) {}

protected:
	/// The stream we are communicating over
	BetterStream			*_port;
};

//
// HIL class definitions.
//
// These are here so that we can declare the HIL object early in the sketch
// and then reference it statically rather than via a pointer.
//

///
/// @class	HIL_MAVLINK
/// @brief	The mavlink protocol for hil
///
#if HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK
class HIL_MAVLINK : public HIL_Class
{
public:
	HIL_MAVLINK();
	void    update(void);
	void	init(BetterStream *port);
	void	send_message(uint8_t id, uint32_t param = 0);
	void	send_text(uint8_t severity, const char *str);
	void	acknowledge(uint8_t id, uint8_t sum1, uint8_t sum2);
private:
	void 	handleMessage(mavlink_message_t * msg);
	mavlink_channel_t chan;
    uint16_t packet_drops;
};
#endif // HIL_PROTOCOL_MAVLINK

///
/// @class	HIL_XPLANE
/// @brief	The xplane protocol for hil
///
#if HIL_PROTOCOL == HIL_PROTOCOL_XPLANE
class HIL_XPLANE : public HIL_Class
{
public:
	HIL_XPLANE();
	void    update(void);
	void	init(BetterStream *port);
	void	send_message(uint8_t id, uint32_t param = 0);
	void	send_text(uint8_t severity, const char *str);
	void	acknowledge(uint8_t id, uint8_t sum1, uint8_t sum2);
private:
	void output_HIL();
	void output_HIL_();
	void output_int(int value);
	void output_byte(byte value);
	void print_buffer();
	
	AP_GPS_IMU * hilPacketDecoder;
	byte buf_len;
	byte out_buffer[32];
};
#endif // HIL_PROTOCOL_XPLANE

#endif // HIL not disabled

#endif // __HIL_H

