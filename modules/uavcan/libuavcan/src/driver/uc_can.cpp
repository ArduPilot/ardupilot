/*
 * CAN bus driver interface.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/driver/can.hpp>
#include <cassert>

namespace uavcan
{

const uint32_t CanFrame::MaskStdID;
const uint32_t CanFrame::MaskExtID;
const uint32_t CanFrame::FlagEFF;
const uint32_t CanFrame::FlagRTR;
const uint32_t CanFrame::FlagERR;
const uint8_t CanFrame::MaxDataLen;

bool CanFrame::priorityHigherThan(const CanFrame& rhs) const
{
    const uint32_t clean_id     = id     & MaskExtID;
    const uint32_t rhs_clean_id = rhs.id & MaskExtID;

    /*
     * STD vs EXT - if 11 most significant bits are the same, EXT loses.
     */
    const bool ext     = id     & FlagEFF;
    const bool rhs_ext = rhs.id & FlagEFF;
    if (ext != rhs_ext)
    {
        const uint32_t arb11     = ext     ? (clean_id >> 18)     : clean_id;
        const uint32_t rhs_arb11 = rhs_ext ? (rhs_clean_id >> 18) : rhs_clean_id;
        if (arb11 != rhs_arb11)
        {
            return arb11 < rhs_arb11;
        }
        else
        {
            return rhs_ext;
        }
    }

    /*
     * RTR vs Data frame - if frame identifiers and frame types are the same, RTR loses.
     */
    const bool rtr     = id     & FlagRTR;
    const bool rhs_rtr = rhs.id & FlagRTR;
    if (clean_id == rhs_clean_id && rtr != rhs_rtr)
    {
        return rhs_rtr;
    }

    /*
     * Plain ID arbitration - greater value loses.
     */
    return clean_id < rhs_clean_id;
}

#if UAVCAN_TOSTRING
std::string CanFrame::toString(StringRepresentation mode) const
{
    UAVCAN_ASSERT(mode == StrTight || mode == StrAligned);

    static const unsigned AsciiColumnOffset = 36U;

    char buf[50];
    char* wpos = buf;
    char* const epos = buf + sizeof(buf);
    fill(buf, buf + sizeof(buf), '\0');

    if (id & FlagEFF)
    {
        wpos += snprintf(wpos, unsigned(epos - wpos), "0x%08x  ", unsigned(id & MaskExtID));
    }
    else
    {
        const char* const fmt = (mode == StrAligned) ? "     0x%03x  " : "0x%03x  ";
        wpos += snprintf(wpos, unsigned(epos - wpos), fmt, unsigned(id & MaskStdID));
    }

    if (id & FlagRTR)
    {
        wpos += snprintf(wpos, unsigned(epos - wpos), " RTR");
    }
    else if (id & FlagERR)
    {
        wpos += snprintf(wpos, unsigned(epos - wpos), " ERR");
    }
    else
    {
        for (int dlen = 0; dlen < dlc; dlen++)                                 // hex bytes
        {
            wpos += snprintf(wpos, unsigned(epos - wpos), " %02x", unsigned(data[dlen]));
        }

        while ((mode == StrAligned) && (wpos < buf + AsciiColumnOffset))       // alignment
        {
            *wpos++ = ' ';
        }

        wpos += snprintf(wpos, unsigned(epos - wpos), "  \'");                 // ascii
        for (int dlen = 0; dlen < dlc; dlen++)
        {
            uint8_t ch = data[dlen];
            if (ch < 0x20 || ch > 0x7E)
            {
                ch = '.';
            }
            wpos += snprintf(wpos, unsigned(epos - wpos), "%c", ch);
        }
        wpos += snprintf(wpos, unsigned(epos - wpos), "\'");
    }
    (void)wpos;
    return std::string(buf);
}
#endif

}
