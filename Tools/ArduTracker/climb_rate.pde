struct DataPoint {
	unsigned long	x;
	long			y;
};

DataPoint		history[ALTITUDE_HISTORY_LENGTH]; // Collection of (x,y) points to regress a rate of change from
unsigned char	index; // Index in history for the current data point

unsigned long	xoffset;
unsigned char	n;

// Intermediate variables for regression calculation
long			xi;
long			yi;
long			xiyi;
unsigned long	xi2;


void add_altitude_data(unsigned long xl, long y)
{
	unsigned char i;
	int dx;

	//Reset the regression if our X variable overflowed
	if (xl < xoffset)
		n = 0;

	//To allow calculation of sum(xi*yi), make sure X hasn't exceeded 2^32/2^15/length
	if (xl - xoffset > 131072/ALTITUDE_HISTORY_LENGTH)
		n = 0;

	if (n == ALTITUDE_HISTORY_LENGTH) {
		xi -= history[index].x;
		yi -= history[index].y;
		xiyi -= (long)history[index].x * history[index].y;
		xi2 -= history[index].x * history[index].x;
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

	history[index].x = xl - xoffset;
	history[index].y = y;

	xi += history[index].x;
	yi += history[index].y;
	xiyi += (long)history[index].x * history[index].y;
	xi2 += history[index].x * history[index].x;

	if (++index >= ALTITUDE_HISTORY_LENGTH)
		index = 0;
}

void recalc_climb_rate()
{
	float slope = ((float)xi*(float)yi - ALTITUDE_HISTORY_LENGTH*(float)xiyi) / ((float)xi*(float)xi - ALTITUDE_HISTORY_LENGTH*(float)xi2);
	climb_rate = (int)(slope*100);
}

void print_climb_debug_info()
{
	unsigned char i, j;
	recalc_climb_rate();
	SendDebugln_P("Climb rate:");
	for (i=0; i<ALTITUDE_HISTORY_LENGTH; i++) {
		j = i + index;
		if (j >= ALTITUDE_HISTORY_LENGTH) j -= ALTITUDE_HISTORY_LENGTH;
		SendDebug_P("  ");
		SendDebug(j,DEC);
		SendDebug_P(": ");
		SendDebug(history[j].x,DEC);
		SendDebug_P(", ");
		SendDebugln(history[j].y,DEC);
	}
	SendDebug_P("  sum(xi) = ");
	SendDebugln(xi,DEC);
	SendDebug_P("  sum(yi) = ");
	SendDebugln(yi,DEC);
	SendDebug_P("  sum(xi*yi) = ");
	SendDebugln(xiyi,DEC);
	SendDebug_P("  sum(xi^2) = ");
	SendDebugln(xi2,DEC);
	SendDebug_P("  Climb rate = ");
	SendDebug((float)climb_rate/100,2);
	SendDebugln_P(" m/s");
}
