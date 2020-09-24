#include "AP_Vtx_Params.h"
#include <AP_Param/AP_Param.h>

// table of user settable parameters
const AP_Param::GroupInfo AP_Vtx_Params::var_info[] = {
    /*    // @Param: TYPE
       // @DisplayName: PROTOCOL TYPE
       // @Description: What protocol type of vtx device that is connected
       // @Values: 0:None,1:SmartAudio,2:Tramp
       // @User: Standard
       AP_GROUPINFO("TYPE", 11, AP_Vtx_Params, type, 0),

       // @Param: MODE
       // @DisplayName: VTX MODE
       // @Description: Vtx mode to operate
       // @Values: 0:PIT MODE,1:RACE, 2:FREESTYLE
       // @User: Standard
       AP_GROUPINFO("MODE",     2, AP_Vtx_Params, mode, 0),

       // @Param: BAND
       // @DisplayName: BAND USED
       // @Description: Band used to transmit video
       // @Values: 0:VTX DEFAULT,1:BAND A ,2:BAND B, 3:BAND E,4: AIRWAVE, 5:RACEBAND, 6: LOW RACE
       // @User: Standard
       AP_GROUPINFO("BAND",     3, AP_Vtx_Params, band, 0),

       // @Param: CHANNEL
       // @DisplayName: Channel number into the band seleted
       // @Description: The channel in which is the vtx transmitting.
       // @Values: 0:VTX DEFAULT,1:Channel 1,2:Channel 2,3:Channel 3,4:Channel 4,5:Channel 5,6:Channel 6,7:Channel 7,8:Channel 8,
       // @User: Standard
       AP_GROUPINFO("CHAN",     4, AP_Vtx_Params, channel, 0),

       // @Param: FRECUENCY
       // @DisplayName: Channel number into the band seleted
       // @Description: The channel in which is the vtx transmitting.
       // @Units: mhz
       // @Increment:1
       // @Values: 0:VTX DEFAULT
       // @User: Standard
       AP_GROUPINFO("FREC",     5, AP_Vtx_Params, frecuency, 0),


       // @Param: POWER
       // @DisplayName: Channel number into the band seleted
       // @Description: The channel in which is the vtx transmitting.
       // @Units: mW
       // @Increment:1
       // @Values: 0:VTX DEFAULT
       // @User: Advanced
       AP_GROUPINFO("POWER",     6, AP_Vtx_Params, power, 0),

    */
    AP_GROUPEND
};

AP_Vtx_Params::AP_Vtx_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}
