#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_JRE_SERIAL_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/*
The data received from the radio altimeter is as follows.
The format of the received data frame varies depending on the mode, and is stored in "read_buff" with a fixed value of 16 bytes, 32 bytes, or 48 bytes in Ver.1,
and 34 bytes, 54 bytes, or 74 bytes in Ver.2.

Ver.1
[1]
Measurement mode: 1 data mode
Packet length: 16 bytes
Altitude data used: 4,5 bytes
|------------------------------------------------------------------------------------------------------------|
| BYTE | 0  | 1  |    2     |    3     |  4  |  5  | 6| 7| 8| 9| 10 | 11 |    12     |    13     | 14  | 15  |
|------|-----------------------------------------------------------------------------------------------------|
| NAME | Header  | Version  |  Frame   | Altitude1 | Reserved  | Radio   |        Status         |    CRC    |
|      |         |          |   Count  |           |           |  Power1 |                       |           |
|      |         |          |          | (H) | (L) |           |(H) |(L) |    (H)    |    (L)    | (L) | (H) |
|------|-----------------------------------------------------------------------------------------------------|
|      |         |          |          |           |           |         | BIT 15to8: reserved   |           |
|      |         |          | unsigned | unsigned  |           |unsigned | BIT 7to4: GAIN detail |CRC result |
| DATA | 'R' 'A' | 0 to 255 | 0 to 255 | LSB:0.01m |   N/A     |LSB:1    | BIT 3-2: reserved     |from BYTE  |
|      |         |          |          |           |           |         | BIT 1: TRK / NTRK     | 0 to 13   |
|      |         |          |          |           |           |         | BIT 0: GAIN LOW/HIGH  |           |
|------------------------------------------------------------------------------------------------------------|

[2]
Measurement mode: 3 data mode
Packet length: 32 bytes
Altitude data used: 4,5 bytes
|------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| BYTE | 0  | 1  |    2     |    3     |  4  |  5  | 6| 7| 8| 9| 10 | 11 | 12  | 13  |14|15|16|17| 18 | 19 | 20  | 21  |22|23|24|25| 26 | 27 |    28     |    29     | 30  | 31  |
|------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| NAME | Header  | Version  |  Frame   | Altitude1 | Reserved  | Radio   | Altitude2 | Reserved  | Radio   | Altitude3 | Reserved  | Radio   |        Status         |    CRC    |
|      |         |          |   Count  |           |           |  Power1 |           |           |  Power2 |           |           |  Power3 |                       |           |
|      |         |          |          | (H) | (L) |           |(H) |(L) | (H) | (L) |           |(H) |(L) | (H) | (L) |           |(H) |(L) |    (H)    |    (L)    | (L) | (H) |
|------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|      |         |          |          |           |           |         |           |           |         |           |           |         | BIT 15to8: reserved   |           |
|      |         |          | unsigned | unsigned  |           |unsigned | unsigned  |           |unsigned | unsigned  |           |unsigned | BIT 7to4: GAIN detail |CRC result |
| DATA | 'R' 'B' | 0 to 255 | 0 to 255 | LSB:0.01m |   N/A     |LSB:1    | LSB:0.01m |    N/A    |LSB:1    | LSB:0.01m |    N/A    |LSB:1    | BIT 3-2: reserved     |from BYTE  |
|      |         |          |          |           |           |         |           |           |         |           |           |         | BIT 1: TRK / NTRK     | 0 to 29   |
|      |         |          |          |           |           |         |           |           |         |           |           |         | BIT 0: GAIN LOW/HIGH  |           |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

[3]
Measurement mode: 5 data mode
Packet length: 48 bytes
Altitude data used: 4,5 bytes
|------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| BYTE | 0  | 1  |    2     |    3     |  4  |  5  | 6| 7| 8| 9| 10 | 11 | 12  | 13  |14|15|16|17| 18 | 19 | 20  | 21  |22|23|24|25| 26 | 27 | 28  | 29  |30|31|32|33| 34 | 35 | 36  | 37  |38|39|40|41| 42 | 43 |    44     |    45     | 46  | 47  |
|------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| NAME | Header  | Version  |  Frame   | Altitude1 | Reserved  | Radio   | Altitude2 | Reserved  | Radio   | Altitude3 | Reserved  | Radio   | Altitude4 | Reserved  | Radio   | Altitude5 | reserved  | Radio   |         Status        |    CRC    |
|      |         |          |   Count  |           |           |  Power1 |           |           |  Power2 |           |           |  Power3 |           |           |  Power4 |           |           |  Power5 |                       |           |
|      |         |          |          | (H) | (L) |           |(H) |(L) | (H) | (L) |           |(H) |(L) | (H) | (L) |           |(H) |(L) | (H) | (L) |           |(H) |(L) | (H) | (L) |           |(H) |(L) |    (H)    |    (L)    | (L) | (H) |
|------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|      |         |          |          |           |           |         |           |           |         |           |           |         |           |           |         |           |           |         | BIT 15to8: reserved   |           |
|      |         |          | unsigned | unsigned  |           |unsigned | unsigned  |           |unsigned | unsigned  |           |unsigned | unsigned  |           |unsigned | unsigned  |           |unsigned | BIT 7to4: GAIN detail |CRC result |
| DATA | 'R' 'C' | 0 to 255 | 0 to 255 | LSB:0.01m |   N/A     |LSB:1    | LSB:0.01m |    N/A    |LSB:1    | LSB:0.01m |    N/A    |LSB:1    | LSB:0.01m |    N/A    |LSB:1    | LSB:0.01m |    N/A    |LSB:1    | BIT 3-2: reserved     |from BYTE  |
|      |         |          |          |           |           |         |           |           |         |           |           |         |           |           |         |           |           |         | BIT 1: TRK / NTRK     | 0 to 45   |
|      |         |          |          |           |           |         |           |           |         |           |           |         |           |           |         |           |           |         | BIT 0: GAIN LOW/HIGH  |           |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

Ver.2
[1]
Measurement mode: 1 data mode
Packet length: 34 bytes
Altitude data used: 11,12 bytes
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| BYTE | 0 | 1 | 2 | 3 |   4   | 5| 6| 7| 8 | 9 |   10   | 11  | 12  | 13  | 14  | 15  | 16  | 17  | 18  | 19 | 20 | 21 | 22 |  23  |  24  |     25      |26|27|28|29| 30 | 31 | 32  | 33  |
|------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| NAME |    Header     |Device |Serial  |Version| Frame  | Altitude1 | Velocity1 | Angle X1  | Angle Y1  | Radio   |Antenna  | PWM Pulse   |   Status    | Fail      |  Temp.  |    CRC    |
|      |               |   Type|    Num |       |  Count |           |           |           |           |  Power1 |    Gain |       Width |             |    Status |         |           |
|      |               |       |        |       |        | (H) | (L) | (H) | (L) | (H) | (L) | (H) | (L) |(H) |(L) |(H) |(L) | (H)  | (L)  |             |           |(H) |(L) | (L) | (H) |
|------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|      |               |       |        |       |        |           |           |           |           |         |         |             |bit7-4:RSVD  |           |         |           |
|      |               |       |        |       |unsigned| unsigned  |signed     |signed     |signed     |unsigned |         |             |bit3:Fail    |           |signed   |CRC result |
| DATA | 'R''X''A''A'  |       |        |       |0 to 255| LSB:0.01m |LSB:0.01m/s|LSB:       |LSB:       |LSB:1    |         |             |bit2:RSVD    |           |LSB:     |from BYTE  |
|      |               |       |        |       |        |           |           |  0.0001rad|  0.0001rad|         |         |             |bit1:TRK/NTRK|           | 0.01DegC| 0 to 31   |
|      |               |       |        |       |        |           |           |           |           |         |         |             |bit0:Gain    |           |         |           |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

[2]
Measurement mode: 3 data mode
Packet length: 54 bytes
Altitude data used: 11,12 bytes
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| BYTE | 0 | 1 | 2 | 3 |   4   | 5| 6| 7| 8 | 9 |   10   | 11  | 12  | 13  | 14  | 15  | 16  | 17  | 18  | 19 | 20 | 21  | 22  | 23  | 24  | 25  | 26  | 27  | 28  | 29 | 30 | 31  | 32  | 33  | 34  | 35  | 36  | 37  | 38  | 39 | 40 | 41 | 42 |  43  |  44  |     45      |46|47|48|49| 50 | 51 | 52  | 53  |
|------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| NAME |    Header     |Device |Serial  |Version| Frame  | Altitude1 | Velocity1 | Angle X1  | Angle Y1  | Radio   | Altitude2 | Velocity2 | Angle X2  | Angle Y2  | Radio   | Altitude3 | Velocity3 | Angle X3  | Angle Y3  | Radio   |Antenna  | PWM Pulse   |   Status    | Fail      |  Temp.  |    CRC    |
|      |               |   Type|    Num |       |  Count |           |           |           |           |  Power1 |           |           |           |           |  Power2 |           |           |           |           |  Power3 |    Gain |       Width |             |    Status |         |           |
|      |               |       |        |       |        | (H) | (L) | (H) | (L) | (H) | (L) | (H) | (L) |(H) |(L) | (H) | (L) | (H) | (L) | (H) | (L) | (H) | (L) |(H) |(L) | (H) | (L) | (H) | (L) | (H) | (L) | (H) | (L) |(H) |(L) |(H) |(L) | (H)  | (L)  |             |           |(H) |(L) | (L) | (H) |
|------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|      |               |       |        |       |        |           |           |           |           |         |           |           |           |           |         |           |           |           |           |         |         |             |bit7-4:RSVD  |           |         |           |
|      |               |       |        |       |unsigned| unsigned  |signed     |signed     |signed     |unsigned | unsigned  |signed     |signed     |signed     |unsigned | unsigned  |signed     |signed     |signed     |unsigned |         |             |bit3:Fail    |           |signed   |CRC result |
| DATA | 'R''X''A''B'  |       |        |       |0 to 255| LSB:0.01m |LSB:0.01m/s|LSB:       |LSB:       |LSB:1    | LSB:0.01m |LSB:0.01m/s|LSB:       |LSB:       |LSB:1    | LSB:0.01m |LSB:0.01m/s|LSB:       |LSB:       |LSB:1    |         |             |bit2:RSVD    |           |LSB:     |from BYTE  |
|      |               |       |        |       |        |           |           |  0.0001rad|  0.0001rad|         |           |           |  0.0001rad|  0.0001rad|         |           |           |  0.0001rad|  0.0001rad|         |         |             |bit1:TRK/NTRK|           | 0.01DegC| 0 to 51   |
|      |               |       |        |       |        |           |           |           |           |         |           |           |           |           |         |           |           |           |           |         |         |             |bit0:Gain    |           |         |           |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

[3]
Measurement mode: 5 data mode
Packet length: 74 bytes
Altitude data used: 11,12 bytes
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| BYTE | 0 | 1 | 2 | 3 |   4   | 5| 6| 7| 8 | 9 |   10   | 11  | 12  | 13  | 14  | 15  | 16  | 17  | 18  | 19 | 20 | 21  | 22  | 23  | 24  | 25  | 26  | 27  | 28  | 29 | 30 | 31  | 32  | 33  | 34  | 35  | 36  | 37  | 38  | 39 | 40 | 41  | 42  | 43  | 44  | 45  | 46  | 47  | 48  | 49 | 50 | 51  | 52  | 53  | 54  | 55  | 56  | 57  | 58  | 59 | 60 | 61 | 62 |  63  |  64  |     65      |66|67|68|69| 70 | 71 | 72  | 73  |
|------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| NAME |    Header     |Device |Serial  |Version| Frame  | Altitude1 | Velocity1 | Angle X1  | Angle Y1  | Radio   | Altitude2 | Velocity2 | Angle X2  | Angle Y2  | Radio   | Altitude3 | Velocity3 | Angle X3  | Angle Y3  | Radio   | Altitude4 | Velocity4 | Angle X4  | Angle Y4  | Radio   | Altitude5 | Velocity5 | Angle X5  | Angle Y5  | Radio   |Antenna  | PWM Pulse   |   Status    | Fail      |  Temp.  |    CRC    |
|      |               |   Type|    Num |       |  Count |           |           |           |           |  Power1 |           |           |           |           |  Power2 |           |           |           |           |  Power3 |           |           |           |           |  Power4 |           |           |           |           |  Power5 |    Gain |       Width |             |    Status |         |           |
|      |               |       |        |       |        | (H) | (L) | (H) | (L) | (H) | (L) | (H) | (L) |(H) |(L) | (H) | (L) | (H) | (L) | (H) | (L) | (H) | (L) |(H) |(L) | (H) | (L) | (H) | (L) | (H) | (L) | (H) | (L) |(H) |(L) | (H) | (L) | (H) | (L) | (H) | (L) | (H) | (L) |(H) |(L) | (H) | (L) | (H) | (L) | (H) | (L) | (H) | (L) |(H) |(L) |(H) |(L) | (H)  | (L)  |             |           |(H) |(L) | (L) | (H) |
|------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|      |               |       |        |       |        |           |           |           |           |         |           |           |           |           |         |           |           |           |           |         |           |           |           |           |         |           |           |           |           |         |         |             |bit7-4:RSVD  |           |         |           |
|      |               |       |        |       |unsigned| unsigned  |signed     |signed     |signed     |unsigned | unsigned  |signed     |signed     |signed     |unsigned | unsigned  |signed     |signed     |signed     |unsigned | unsigned  |signed     |signed     |signed     |unsigned | unsigned  |signed     |signed     |signed     |unsigned |         |             |bit3:Fail    |           |signed   |CRC result |
| DATA | 'R''X''A''C'  |       |        |       |0 to 255| LSB:0.01m |LSB:0.01m/s|LSB:       |LSB:       |LSB:1    | LSB:0.01m |LSB:0.01m/s|LSB:       |LSB:       |LSB:1    | LSB:0.01m |LSB:0.01m/s|LSB:       |LSB:       |LSB:1    | LSB:0.01m |LSB:0.01m/s|LSB:       |LSB:       |LSB:1    | LSB:0.01m |LSB:0.01m/s|LSB:       |LSB:       |LSB:1    |         |             |bit2:RSVD    |           |LSB:     |from BYTE  |
|      |               |       |        |       |        |           |           |  0.0001rad|  0.0001rad|         |           |           |  0.0001rad|  0.0001rad|         |           |           |  0.0001rad|  0.0001rad|         |           |           |  0.0001rad|  0.0001rad|         |           |           |  0.0001rad|  0.0001rad|         |         |             |bit1:TRK/NTRK|           | 0.01DegC| 0 to 71   |
|      |               |       |        |       |        |           |           |           |           |         |           |           |           |           |         |           |           |           |           |         |           |           |           |           |         |           |           |           |           |         |         |             |bit0:Gain    |           |         |           |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

*/
class AP_RangeFinder_JRE_Serial : public AP_RangeFinder_Backend_Serial
{

public:
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return NEW_NOTHROW AP_RangeFinder_JRE_Serial(_state, _params);
    }

protected:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override
    {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    int8_t get_signal_quality_pct() const override
    {
        return no_signal ? RangeFinder::SIGNAL_QUALITY_MIN : signal_quality_pct;
    }

private:

    // get a reading
    bool get_reading(float &reading_m) override;

    void move_preamble_in_buffer(uint8_t search_start_pos);

    uint8_t data_buff[74 * 3];  // 74 is longest possible packet
    uint8_t data_buff_ofs;      // index where next item will be added in data_buff

    bool no_signal;     // true if the latest read attempt found no valid distances

    int8_t signal_quality_pct;
};
#endif  // AP_RANGEFINDER_JRE_SERIAL_ENABLED
