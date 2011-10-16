/** @file
 *	@brief MAVLink comm protocol testsuite generated from ardupilotmega.xml
 *	@see http://qgroundcontrol.org/mavlink/
 *	Generated on Wed Aug 24 17:14:16 2011
 */
#ifndef ARDUPILOTMEGA_TESTSUITE_H
#define ARDUPILOTMEGA_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

static void mavtest_generate_outputs(mavlink_channel_t chan)
{
	mavlink_msg_sensor_offsets_send(chan , 19107 , 19211 , 19315 , 17.0 , 963497672 , 963497880 , 101.0 , 129.0 , 157.0 , 185.0 , 213.0 , 241.0 );
	mavlink_msg_set_mag_offsets_send(chan , 151 , 218 , 17235 , 17339 , 17443 );
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ARDUPILOTMEGA_TESTSUITE_H
