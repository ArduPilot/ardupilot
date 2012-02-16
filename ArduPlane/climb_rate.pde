// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if 0 // currently unused

struct DataPoint {
	unsigned long	x;
	long			y;
};

DataPoint		history[ALTITUDE_HISTORY_LENGTH]; // Collection of (x,y) points to regress a rate of change from
unsigned char	hindex; // Index in history for the current data point

unsigned long	xoffset;
unsigned char	n;

// Intermediate variables for regression calculation
long			xi;
long			yi;
long			xiyi;
unsigned long	xi2;

void add_altitude_data(unsigned long xl, long y)
{
	//Reset the regression if our X variable overflowed
	if (xl < xoffset)
		n = 0;

	//To allow calculation of sum(xi*yi), make sure X hasn't exceeded 2^32/2^15/length
	if (xl - xoffset > 131072/ALTITUDE_HISTORY_LENGTH)
		n = 0;

	if (n == ALTITUDE_HISTORY_LENGTH) {
		xi -= history[hindex].x;
		yi -= history[hindex].y;
		xiyi -= (long)history[hindex].x * history[hindex].y;
		xi2 -= history[hindex].x * history[hindex].x;
	} else {
		if (n == 0) {
			xoffset = xl;
			xi = 0;
			yi = 0;
			xiyi = 0;
			xi2 = 0;
		}
		n++;
	}

	history[hindex].x = xl - xoffset;
	history[hindex].y = y;

	xi += history[hindex].x;
	yi += history[hindex].y;
	xiyi += (long)history[hindex].x * history[hindex].y;
	xi2 += history[hindex].x * history[hindex].x;

	if (++hindex >= ALTITUDE_HISTORY_LENGTH)
		hindex = 0;
}
#endif

