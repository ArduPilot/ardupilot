// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	GCS.h
/// @brief	Interface definition for the various Ground Control System protocols.

#ifndef __GCS_H
#define __GCS_H

#include <FastSerial.h>
#include <AP_Common.h>
#include <GCS_MAVLink.h>
#include <GPS.h>
#include <Stream.h>
#include <stdint.h>

///
/// @class	GCS
/// @brief	Class describing the interface between the APM code
///			proper and the GCS implementation.
///
/// GCS' are currently implemented inside the sketch and as such have
/// access to all global state.  The sketch should not, however, call GCS
/// internal functions - all calls to the GCS should be routed through
/// this interface (or functions explicitly exposed by a subclass).
///
class GCS_Class
{
public:

	/// Startup initialisation.
	///
	/// This routine performs any one-off initialisation required before
	/// GCS messages are exchanged.
	///
	/// @note The stream is expected to be set up and configured for the
	///       correct bitrate before ::init is called.
	///
	/// @note The stream is currently BetterStream so that we can use the _P
	///	      methods; this may change if Arduino adds them to Print.
	///
	/// @param	port		The stream over which messages are exchanged.
	///
	void		init(FastSerial *port) {
        _port = port;
        initialised = true;
    }

	/// Update GCS state.
	///
	/// This may involve checking for received bytes on the stream,
	/// or sending additional periodic messages.
	void		update(void) {}

	/// Send a message with a single numeric parameter.
	///
	/// This may be a standalone message, or the GCS driver may
	/// have its own way of locating additional parameters to send.
	///
	/// @param	id			ID of the message to send.
	/// @param	param		Explicit message parameter.
	///
	void		send_message(enum ap_message id) {}

	/// Send a text message.
	///
	/// @param	severity	A value describing the importance of the message.
	/// @param	str			The text to be sent.
	///
	void		send_text(gcs_severity severity, const char *str) {}

	/// Send a text message with a PSTR()
	///
	/// @param	severity	A value describing the importance of the message.
	/// @param	str			The text to be sent.
	///
	void		send_text(gcs_severity severity, const prog_char_t *str) {}

    // test if frequency within range requested for loop
    // used by data_stream_send
    static bool freqLoopMatch(uint16_t freq, uint16_t freqMin, uint16_t freqMax)
    {
        if (freq != 0 && freq >= freqMin && freq < freqMax) return true;
        else return false;
    }

    // send streams which match frequency range
    void data_stream_send(uint16_t freqMin, uint16_t freqMax);

    // set to true if this GCS link is active
    bool initialised;

protected:
	/// The stream we are communicating over
	FastSerial			*_port;
};

//
// GCS class definitions.
//
// These are here so that we can declare the GCS object early in the sketch
// and then reference it statically rather than via a pointer.
//

///
/// @class	GCS_MAVLINK
/// @brief	The mavlink protocol for qgroundcontrol
///
class GCS_MAVLINK : public GCS_Class
{
public:
	GCS_MAVLINK();
	void    update(void);
	void	init(FastSerial *port);
	void	send_message(enum ap_message id);
	void	send_text(gcs_severity severity, const char *str);
	void	send_text(gcs_severity severity, const prog_char_t *str);
    void    data_stream_send(uint16_t freqMin, uint16_t freqMax);
	void    queued_param_send();
	void    queued_waypoint_send();

    static const struct AP_Param::GroupInfo var_info[];

private:
	void 	handleMessage(mavlink_message_t * msg);

	/// Perform queued sending operations
	///
	AP_Param   *_queued_parameter;                  ///< next parameter to be sent in queue
    enum ap_var_type _queued_parameter_type;        ///< type of the next parameter
    uint16_t	_queued_parameter_token;            ///AP_Param token for next() call
	uint16_t    _queued_parameter_index;            ///< next queued parameter's index
    uint16_t    _queued_parameter_count;            ///< saved count of parameters for queued send

	/// Count the number of reportable parameters.
	///
	/// Not all parameters can be reported via MAVlink.  We count the number that are
	/// so that we can report to a GCS the number of parameters it should expect when it
	/// requests the full set.
	///
	/// @return         The number of reportable parameters.
	///
    uint16_t    _count_parameters();                ///< count reportable parameters

    uint16_t    _parameter_count;                   ///< cache of reportable parameters

	mavlink_channel_t chan;
    uint16_t packet_drops;

#if CLI_ENABLED == ENABLED
    // this allows us to detect the user wanting the CLI to start
    uint8_t crlf_count;
#endif

	// waypoints
	uint16_t waypoint_request_i; // request index
	uint16_t waypoint_dest_sysid; // where to send requests
	uint16_t waypoint_dest_compid; // "
	bool waypoint_sending; // currently in send process
	bool waypoint_receiving; // currently receiving
	uint16_t waypoint_count;
	uint32_t waypoint_timelast_send; // milliseconds
	uint32_t waypoint_timelast_receive; // milliseconds
	uint32_t waypoint_timelast_request; // milliseconds
	uint16_t waypoint_send_timeout; // milliseconds
	uint16_t waypoint_receive_timeout; // milliseconds

	// data stream rates
	AP_Int16 streamRateRawSensors;
	AP_Int16 streamRateExtendedStatus;
	AP_Int16 streamRateRCChannels;
	AP_Int16 streamRateRawController;
	AP_Int16 streamRatePosition;
	AP_Int16 streamRateExtra1;
	AP_Int16 streamRateExtra2;
	AP_Int16 streamRateExtra3;
};

#endif // __GCS_H
