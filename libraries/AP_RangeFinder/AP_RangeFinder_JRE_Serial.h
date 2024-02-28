#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_JRE_SERIAL_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/*
The data received from the radio altimeter is as follows.
The format of the received data frame varies depending on the mode, and is stored in "read_buff" with a fixed value of 16 bytes, 32 bytes, or 48 bytes.

[1]
Measurement mode: 1 data mode
Packet length: 16 bytes
Altitude data used: 4,5 bytes
|----------------------------------------------------------------------------------------------------------------------------------------------------|
| BYTE |   0    |   1    |    2     |    3     |    4     |    5     | 6 | 7 | 8 | 9 |     10    |     11    |    12     |    13     |  14   |  15   |
|------|---------------------------------------------------------------------------------------------------------------------------------------------|
| NAME | header | header | version  |  frame   | altitude | altitude |    reserved   | FFT value | FFT value |  status   |  status   |  CRC  |  CRC  |
|      |  (H)   |  (L)   |          |  count   |    (H)   |    (L)   |               |    (H)    |    (L)    |   (H)     |   (L)     |  (L)  |  (H)  |
|------|---------------------------------------------------------------------------------------------------------------------------------------------|
|      |        |        |          |          |                     |               |                       | BIT 15to8: reserved   |               |
|      |        |        |          | unsigned |       unsigned      |               |       unsigned        | BIT 7to4: GAIN detail |   CRC result  |
| DATA |  'R'   |  'A'   | 0 to 255 | 0 to 255 |      LSB;0.01 m     |      N/A      |        LSB:1          | BIT 3-2: reserved     |   from BYTE   |
|      |        |        |          |          |      0 to 65535     |               |      0 to 65535       | BIT 1: TRK / NTRK     |    0 to 13    |
|      |        |        |          |          |                     |               |                       | BIT 0: GAIN LOW/HIGH  |               |
|----------------------------------------------------------------------------------------------------------------------------------------------------|

[2]
Measurement mode: 3 data mode
Packet length: 32 bytes
Altitude data used: 4,5 bytes
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| BYTE |   0    |   1    |    2     |    3     |    4     |    5     | 6 | 7 | 8 | 9 |     10    |     11    |    12    |    13    | 14| 15| 16| 17|     18    |     19    |    20    |    21    | 22| 23| 24| 25|     26    |     27    |    28     |    29     |  30   |  31   |
|------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| NAME | header | header | version  |  frame   | altitude | altitude |    reserved   | FFT value | FFT value | altitude | altitude |    reserved   | FFT value | FFT value | altitude | altitude |    reserved   | FFT value | FFT value |  status   |  status   |  CRC  |  CRC  |
|      |  (H)   |  (L)   |          |  count   |    (H)   |    (L)   |               |    (H)    |    (L)    |    (H)   |    (L)   |               |    (H)    |    (L)    |    (H)   |    (L)   |               |    (H)    |    (L)    |    (H)    |    (L)    |  (L)  |  (H)  |
|------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|      |        |        |          |          |                     |               |                       |                     |               |                       |                     |               |                       | BIT 15to8: reserved   |               |
|      |        |        |          | unsigned |       unsigned      |               |       unsigned        |       unsigned      |               |       unsigned        |       unsigned      |               |       unsigned        | BIT 7to4: GAIN detail |   CRC result  |
| DATA |  'R'   |  'B'   | 0 to 255 | 0 to 255 |      LSB;0.01 m     |      N/A      |        LSB:1          |      LSB;0.01 m     |      N/A      |        LSB:1          |      LSB;0.01 m     |      N/A      |        LSB:1          | BIT 3-2: reserved     |   from BYTE   |
|      |        |        |          |          |      0 to 65535     |               |      0 to 65535       |      0 to 65535     |               |      0 to 65535       |      0 to 65535     |               |      0 to 65535       | BIT 1: TRK / NTRK     |    0 to 13    |
|      |        |        |          |          |                     |               |                       |                     |               |                       |                     |               |                       | BIT 0: GAIN LOW/HIGH  |               |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

[3]
Measurement mode: 5 data mode
Packet length: 48 bytes
Altitude data used: 4,5 bytes
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| BYTE |   0    |   1    |    2     |    3     |    4     |    5     | 6 | 7 | 8 | 9 |     10    |     11    |    12    |    13    | 14| 15| 16| 17|     18    |     19    |    20    |    21    | 22| 23| 24| 25|     26    |     27    |    28    |    29    | 30| 31| 32| 33|     34    |     35    |    36    |    37    | 38| 39| 40| 41|     42    |     43    |    44     |    45     |  46   |  47   |
|------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| NAME | header | header | version  |  frame   | altitude | altitude |    reserved   | FFT value | FFT value | altitude | altitude |    reserved   | FFT value | FFT value | altitude | altitude |    reserved   | FFT value | FFT value | altitude | altitude |    reserved   | FFT value | FFT value | altitude | altitude |    reserved   | FFT value | FFT value |  status   |  status   |  CRC  |  CRC  |
|      |  (H)   |  (L)   |          |  count   |    (H)   |    (L)   |               |    (H)    |    (L)    |    (H)   |    (L)   |               |    (H)    |    (L)    |    (H)   |    (L)   |               |    (H)    |    (L)    |    (H)   |    (L)   |               |    (H)    |    (L)    |    (H)   |    (L)   |               |    (H)    |    (L)    |    (H)    |    (L)    |  (L)  |  (H)  |
|------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|      |        |        |          |          |                     |               |                       |                     |               |                       |                     |               |                       |                     |               |                       |                     |               |                       | BIT 15to8: reserved   |               |
|      |        |        |          | unsigned |       unsigned      |               |       unsigned        |       unsigned      |               |       unsigned        |       unsigned      |               |       unsigned        |       unsigned      |               |       unsigned        |       unsigned      |               |       unsigned        | BIT 7to4: GAIN detail |   CRC result  |
| DATA |  'R'   |  'C'   | 0 to 255 | 0 to 255 |      LSB;0.01 m     |      N/A      |        LSB:1          |      LSB;0.01 m     |      N/A      |        LSB:1          |      LSB;0.01 m     |      N/A      |        LSB:1          |      LSB;0.01 m     |      N/A      |        LSB:1          |      LSB;0.01 m     |      N/A      |        LSB:1          | BIT 3-2: reserved     |   from BYTE   |
|      |        |        |          |          |      0 to 65535     |               |      0 to 65535       |      0 to 65535     |               |      0 to 65535       |      0 to 65535     |               |      0 to 65535       |      0 to 65535     |               |      0 to 65535       |      0 to 65535     |               |      0 to 65535       | BIT 1: TRK / NTRK     |    0 to 13    |
|      |        |        |          |          |                     |               |                       |                     |               |                       |                     |               |                       |                     |               |                       |                     |               |                       | BIT 0: GAIN LOW/HIGH  |               |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*/
class AP_RangeFinder_JRE_Serial : public AP_RangeFinder_Backend_Serial
{

public:
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return new AP_RangeFinder_JRE_Serial(_state, _params);
    }

protected:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override
    {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    int8_t get_signal_quality_pct() const override
    {
        return no_signal ? RangeFinder::SIGNAL_QUALITY_MIN : RangeFinder::SIGNAL_QUALITY_MAX;
    }

private:

    // get a reading
    bool get_reading(float &reading_m) override;

    void move_preamble_in_buffer(uint8_t search_start_pos);

    uint8_t data_buff[48 * 3];  // 48 is longest possible packet
    uint8_t data_buff_ofs;      // index where next item will be added in data_buff

    bool no_signal;     // true if the latest read attempt found no valid distances
};
#endif  // AP_RANGEFINDER_JRE_SERIAL_ENABLED
