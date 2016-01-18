
#include <AP_Curve.h>


// Constructor
template <class T, uint8_t SIZE>
AP_Curve<T,SIZE>::AP_Curve() :
	_num_points(0)
{
	// clear the curve
	clear();
};

// clear the curve
template <class T, uint8_t SIZE>
void AP_Curve<T,SIZE>::clear() {
	// clear the curve
	for( uint8_t i=0; i<SIZE; i++ ) {
		_x[i] = 0;
		_y[i] = 0;
		_slope[i] = 0.0;
	}
	_num_points = 0;
}

// add_point - adds a point to the curve
template <class T, uint8_t SIZE>
bool AP_Curve<T,SIZE>::add_point( T x, T y )
{
	if( _num_points < SIZE ) {
		_x[_num_points] = x;
		_y[_num_points] = y;

		// increment the number of points
		_num_points++;

		// if we have at least two points calculate the slope
		if( _num_points > 1 ) {
			_slope[_num_points-2] = (float)(_y[_num_points-1] - _y[_num_points-2]) / (float)(_x[_num_points-1] - _x[_num_points-2]);
			_slope[_num_points-1] = _slope[_num_points-2];	// the final slope is for interpolation beyond the end of the curve
		}
		return true;
	}else{
		// we do not have room for the new point
		return false;
	}
}

// get_y - returns the y value on the curve for a given x value
template <class T, uint8_t SIZE>
T AP_Curve<T,SIZE>::get_y( T x )
{
	uint8_t i;
	T result;

	// deal with case where ther is no curve
	if( _num_points == 0 ) {
		return x;
	}

	// when x value is lower than the first point's x value, return minimum y value
	if( x <= _x[0] ) {
		return _y[0];
	}

    // when x value is higher than the last point's x value, return maximum y value
	if( x >= _x[_num_points-1] ) {
		return _y[_num_points-1];
	}

	// deal with the normal case
	for( i=0; i<_num_points-1; i++ ) {
		if( x >= _x[i] && x <= _x[i+1] ) {
			result = _y[i] + (x - _x[i]) * _slope[i];
			return result;
		}
	}

	// we should never get here
	return x;
}
// displays the contents of the curve (for debugging)
template <class T, uint8_t SIZE>
void AP_Curve<T,SIZE>::dump_curve(AP_HAL::BetterStream* s)
{
	s->println_P(PSTR("Curve:"));
	for( uint8_t i = 0; i<_num_points; i++ ){
		s->print_P(PSTR("x:"));
		s->print(_x[i]);
		s->print_P(PSTR("\ty:"));
		s->print(_y[i]);
		s->print_P(PSTR("\tslope:"));
		s->print(_slope[i],4);
		s->println();
	}
}

template class AP_Curve<int16_t,3>;
template class AP_Curve<int16_t,4>;
template class AP_Curve<int16_t,5>;
template class AP_Curve<uint16_t,3>;
template class AP_Curve<uint16_t,4>;
template class AP_Curve<uint16_t,5>;

