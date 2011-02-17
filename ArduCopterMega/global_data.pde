// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

global_struct::global_struct() :
	// parameters
	// note, all values not explicitly initialised here are zeroed
	waypoint_send_timeout(1000), // 1 second
	waypoint_receive_timeout(1000), // 1 second

	// stream rates
	streamRateRawSensors(0),
	streamRateExtendedStatus(0),
	streamRateRCChannels(0),
	streamRateRawController(0),
	//streamRateRawSensorFusion(0),
	streamRatePosition(0),
	streamRateExtra1(0),
	streamRateExtra2(0),
	streamRateExtra3(0)
{
}
