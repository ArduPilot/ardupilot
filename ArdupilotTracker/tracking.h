/*
 * We may add other formats such as polulu binary format or even a raw GPS
 * format to use for tracking a (FPV?) UAV that has a GPS connected to a
 * telemetry xmitter and no Ardupilot at all.
 */
enum TrackingModes {
	POSITION_FROM_MAVLINK,
	//POSITION_FROM_ARDUTRACKER,
	AZI_ELE_FROM_ARDUTRACKER
};

struct AzimuthElevation {
	uint32_t timestamp;
	int32_t azimuth_cd; 	// on a -180..180 degrees interval.
	int32_t elevation_cd;	// on a -90..90 degrees interval.
};

struct TargetPosition {
	uint32_t timestamp;
	struct Location position;
};
