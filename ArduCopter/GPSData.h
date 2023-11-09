#pragma once 

#ifndef GPSDATA_H
#define GPSDATA_H

class GPSData
{
private:
    double latitude;
    double longitude;
    float altitude;
    
public:
    GPSData(/* args */);
    ~GPSData() = default;
    double get_latitude();
    double get_longitude();
    float get_altitude();
};


#endif // GPSDATA_H
