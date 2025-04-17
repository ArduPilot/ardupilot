#include "RCOutput_Multi.h"
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>

#define MIN(a,b) ((a)<(b)?(a):(b))

#ifdef DEBUG
#define debug(fmt, args ...) do { fprintf(stderr,"[RCOutput_Multi]: %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while (0)
#else
#define debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

using namespace Linux;

RCOutput_Multi::~RCOutput_Multi()
{
}

void RCOutput_Multi::init()
{
    for( unsigned g = 0; g < n_out_groups; ++g )
        {
        std::get<0>(out_groups[g]).init();
        }
}

bool RCOutput_Multi::resolve_channel(uint8_t ch, unsigned & gid, unsigned &cid)
{
for( unsigned g = 0; g < n_out_groups; ++g)
    {
        int group_ch_count = std::get<1>(out_groups[g]);
        if(ch < group_ch_count)
        {
            gid = g;
            cid = ch;
            return true;
        }
        else
        {
            ch -= group_ch_count;
        }
    }
return false;
}


void RCOutput_Multi::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    debug("chmask=%u freq_hz=%u", chmask, freq_hz);
    for( unsigned g = 0; g < n_out_groups; ++g )
        {
        int group_chcount = std::get<1>(out_groups[g]);
        uint32_t group_chmask = (1<<group_chcount)-1;
        if(chmask & group_chmask) {
            std::get<0>(out_groups[g]).set_freq(chmask & group_chmask, freq_hz);
        }
        chmask >>= group_chcount;
        }
}

uint16_t RCOutput_Multi::get_freq(uint8_t ch)
{
    debug("ch=%u", ch);
    for( unsigned g = 0; g < n_out_groups; ++g )
        {
        int group_chcount = std::get<1>(out_groups[g]);
        if( ch < group_chcount )
            return std::get<0>(out_groups[g]).get_freq(ch);
        ch -= group_chcount;
        }
    return 0;
}

void RCOutput_Multi::enable_ch(uint8_t ch)
{
   debug("ch=%u", ch);
    for( unsigned g = 0; g < n_out_groups; ++g )
        {
        int group_chcount = std::get<1>(out_groups[g]);
        if( ch < group_chcount )
            {
            std::get<0>(out_groups[g]).enable_ch(ch);
            break;
            }
        ch -= group_chcount;
        }
}

void RCOutput_Multi::disable_ch(uint8_t ch)
{
    debug("ch=%u", ch);
    for( unsigned g = 0; g < n_out_groups; ++g )
        {
        int group_chcount = std::get<1>(out_groups[g]);
        if( ch < group_chcount )
            {
            std::get<0>(out_groups[g]).disable_ch(ch);
            break;
            }
        ch -= group_chcount;
        }
}

bool RCOutput_Multi::force_safety_on() {
    debug("");
    bool ret = true;
    /* Shutdown before sleeping. */
    for( unsigned g = 0; g < n_out_groups; ++g )
        {
        ret &= std::get<0>(out_groups[g]).force_safety_on();
        }
    return ret;
}

void RCOutput_Multi::force_safety_off() {
    debug("");
    for( unsigned g = 0; g < n_out_groups; ++g )
        {
        std::get<0>(out_groups[g]).force_safety_off();
        }
}

void RCOutput_Multi::cork()
{
    for( unsigned g = 0; g < n_out_groups; ++g )
        {
        std::get<0>(out_groups[g]).cork();
        }
}

void RCOutput_Multi::push()
{
    for( unsigned g = 0; g < n_out_groups; ++g )
        {
        std::get<0>(out_groups[g]).push();
        }
}

void RCOutput_Multi::write(uint8_t ch, uint16_t period_us)
{
    debug("write(ch=%u, period_us=%u", ch, period_us);
    for( unsigned g = 0; g < n_out_groups; ++g )
        {
        int group_chcount = std::get<1>(out_groups[g]);
        if( ch < group_chcount ) {
            std::get<0>(out_groups[g]).write(ch, period_us);
            return;
            }
        ch -= group_chcount;
        }
}

uint16_t RCOutput_Multi::read(uint8_t ch)
{
    debug("read(ch=%u)", ch);
    for( unsigned g = 0; g < n_out_groups; ++g )
        {
        int group_chcount = std::get<1>(out_groups[g]);
        if( ch < group_chcount )
            return std::get<0>(out_groups[g]).read(ch);
        ch -= group_chcount;
        }

    return 0;
}

void RCOutput_Multi::read(uint16_t* period_us, uint8_t len)
{
    debug("read(%u)", len);
    for( unsigned g = 0; g < n_out_groups; ++g )
        {
        uint8_t group_chcount = std::get<1>(out_groups[g]);
        for( unsigned c = 0; c < MIN(group_chcount,len); ++c)
            {
            period_us[c] = std::get<0>(out_groups[g]).read(c);
            }

        if( len < group_chcount )
            return;

        period_us += group_chcount;
        len -= group_chcount;
        }
}
