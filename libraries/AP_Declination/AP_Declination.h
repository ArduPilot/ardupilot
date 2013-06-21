// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_Declination_h
#define AP_Declination_h

/*
 *	Adam M Rivera
 *	With direction from: Andrew Tridgell, Jason Short, Justin Beech
 *
 *	Adapted from: http://www.societyofrobots.com/robotforum/index.php?topic=11855.0
 *	Scott Ferguson
 *	scottfromscott@gmail.com
 *
 */
class AP_Declination
{
public:
    static float            get_declination(float lat, float lon);
private:
    static int16_t          get_lookup_value(uint8_t x, uint8_t y);
};

#endif // AP_Declination_h
