#include "SIM_config.h"

#if AP_SIM_GPS_NOVA_ENABLED

#include "SIM_GPS_NOVA.h"

#include <SITL/SITL.h>

using namespace SITL;

void GPS_NOVA::publish(const GPS_Data *d)
{
    static struct PACKED nova_header
    {
        // 0
        uint8_t preamble[3];
        // 3
        uint8_t headerlength;
        // 4
        uint16_t messageid;
        // 6
        uint8_t messagetype;
        //7
        uint8_t portaddr;
        //8
        uint16_t messagelength;
        //10
        uint16_t sequence;
        //12
        uint8_t idletime;
        //13
        uint8_t timestatus;
        //14
        uint16_t week;
        //16
        uint32_t tow;
        //20
        uint32_t recvstatus;
        // 24
        uint16_t resv;
        //26
        uint16_t recvswver;
    } header;

    struct PACKED psrdop
    {
        float gdop;
        float pdop;
        float hdop;
        float htdop;
        float tdop;
        float cutoff;
        uint32_t svcount;
        // extra data for individual prns
    } psrdop {};

    struct PACKED bestpos
    {
        uint32_t solstat;
        uint32_t postype;
        double lat;
        double lng;
        double hgt;
        float undulation;
        uint32_t datumid;
        float latsdev;
        float lngsdev;
        float hgtsdev;
        // 4 bytes
        uint8_t stnid[4];
        float diffage;
        float sol_age;
        uint8_t svstracked;
        uint8_t svsused;
        uint8_t svsl1;
        uint8_t svsmultfreq;
        uint8_t resv;
        uint8_t extsolstat;
        uint8_t galbeisigmask;
        uint8_t gpsglosigmask;
    } bestpos {};

    struct PACKED bestvel
    {
        uint32_t solstat;
        uint32_t veltype;
        float latency;
        float age;
        double horspd;
        double trkgnd;
        // + up
        double vertspd;
        float resv;
    } bestvel {};

    const auto gps_tow = gps_time();

    header.preamble[0] = 0xaa;
    header.preamble[1] = 0x44;
    header.preamble[2] = 0x12;
    header.headerlength = sizeof(header);
    header.week = gps_tow.week;
    header.tow = gps_tow.ms;

    header.messageid = 174;
    header.messagelength = sizeof(psrdop);
    header.sequence += 1;

    psrdop.hdop = 1.20;
    psrdop.htdop = 1.20;
    nova_send_message((uint8_t*)&header,sizeof(header),(uint8_t*)&psrdop, sizeof(psrdop));


    header.messageid = 99;
    header.messagelength = sizeof(bestvel);
    header.sequence += 1;

    bestvel.horspd = norm(d->speedN, d->speedE);
    bestvel.trkgnd = ToDeg(atan2f(d->speedE, d->speedN));
    bestvel.vertspd = -d->speedD;

    nova_send_message((uint8_t*)&header,sizeof(header),(uint8_t*)&bestvel, sizeof(bestvel));


    header.messageid = 42;
    header.messagelength = sizeof(bestpos);
    header.sequence += 1;

    bestpos.lat = d->latitude;
    bestpos.lng = d->longitude;
    bestpos.hgt = d->altitude;
    bestpos.svsused = d->have_lock ? _sitl->gps_numsats[instance] : 3;
    bestpos.latsdev=0.2;
    bestpos.lngsdev=0.2;
    bestpos.hgtsdev=0.2;
    bestpos.solstat=0;
    bestpos.postype=32;

    nova_send_message((uint8_t*)&header,sizeof(header),(uint8_t*)&bestpos, sizeof(bestpos));
}

void GPS_NOVA::nova_send_message(uint8_t *header, uint8_t headerlength, uint8_t *payload, uint8_t payloadlen)
{
    write_to_autopilot((char*)header, headerlength);
write_to_autopilot((char*)payload, payloadlen);

    uint32_t crc = CalculateBlockCRC32(headerlength, header, (uint32_t)0);
    crc = CalculateBlockCRC32(payloadlen, payload, crc);

    write_to_autopilot((char*)&crc, 4);
}

#define CRC32_POLYNOMIAL 0xEDB88320L
uint32_t GPS_NOVA::CRC32Value(uint32_t icrc)
{
    int i;
    uint32_t crc = icrc;
    for ( i = 8 ; i > 0; i-- )
    {
        if ( crc & 1 )
            crc = ( crc >> 1 ) ^ CRC32_POLYNOMIAL;
        else
            crc >>= 1;
    }
    return crc;
}

uint32_t GPS_NOVA::CalculateBlockCRC32(uint32_t length, uint8_t *buffer, uint32_t crc)
{
    while ( length-- != 0 )
    {
        crc = ((crc >> 8) & 0x00FFFFFFL) ^ (CRC32Value(((uint32_t) crc ^ *buffer++) & 0xff));
    }
    return( crc );
}

#endif  // AP_SIM_GPS_NOVA_ENABLED
