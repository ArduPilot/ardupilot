/*
  wrapper for OSD code (https://github.com/night-ghost/minimosd-extra) to run in the HAL as independent process

    (c) night_ghost@ykoctpa.ru 2017

*/

#include <AP_HAL/AP_HAL.h>

#ifdef BOARD_OSD_CS_PIN

#include <utility>

#include "osd_core/compat.h"



using namespace F4Light;

#include <AP_Common/AP_Common.h>
#include <stdio.h>

#include <hal.h>
#include "ring_buffer.h"

#include <inttypes.h>
#include <AP_HAL/utility/print_vprintf.h>
#include <AP_HAL_F4Light/AP_HAL_F4Light.h>
#include <AP_HAL_F4Light/SPIDevice.h>

#define SLAVE_BUILD

// remove some things defined in AP_HAL 
#undef round
#define round(x) roundf(x)

#include "osd_core/Defs.h"

#include "osd.h"


#include "osd_eeprom.h"
#include "osd_core/eeprom.h"
#include "osd_core/version.h"
#include <sd/SD.h>


namespace OSDns {

#include "osd_core/GCS_MAVLink.h"

#include "osd_core/OSD_Max7456.h"
OSD osd; //OSD object

#include "osd_core/prototypes.h"
#include "osd_core/Vars.h"



#include "osd_core/Config_Func.h"
#include "osd_core/Config.h"
#include "osd_core/Func.h"
#include "osd_core/protocols.h"
#include "osd_core/misc.h"

#include "osd_core/Params.h"
#include "osd_core/Panels.h"


// TODO: чтение конфига и еепром с карты памяти, чтобы закинуть .mcm и .osd и все       
static const char * const words[] = {
    "Air Speed",        // 1
    "Altitude",         // 2
    "Auto Mode",        // 3
    "Auto Screen Switch", // 4 
    "batt_a_k",         // 5
    "BattB",            // 6
    "batt_b_k",         // 7
    "Battery",          // 8
    "Battery A",        // 9
    "Battery B",        // 10
    "Battery Percent",  // 11
    "Battery Warning Level", // 12
    "Call Sign",        // 13
    "Chanel Rotation Switching", // 14
    "Channel Raw",      // 15
    "Channel Scale",    // 16
    "Channel state",    // 17
    "Channel Value",    // 18

    "Configuration",    // 19
    "Current",          // 20
    "curr_k",           // 21
    "Efficiency",       // 22
    "fBattA",           // 23
    "fBattB",           // 24      
    "fCurr",            // 25
    "fILS",             // 26
    "flgHUD",           // 27
    "flgOnce",          // 28
    "flgTrack",         // 29
    "Flight Data",      // 30
    "Flight Mode",      // 31
    "fRussianHUD",      // 32
    "GPS Coord",        // 33
    "GPS HDOP",         // 34
    "Heading",          // 35
    "Heading Rose",     // 36
    "Home Altitude",    // 37
    "Home Direction",   // 38
    "Home Distance",    // 39
    "Horizon",          // 40
    "HOS",              // 41
    "Message",          // 42
    "Model Type",       // 43
    "NSCREENS",         // 44
    "OSD Brightness",   // 45
    "Overspeed",        // 46

   "Panel",             // 47
    "Pitch",            // 48
    "pitch_k",          // 49
    "pitch_kn",         // 50
    "PWMDST",           // 51
    "PWMSRC",           // 52
    "Radar Scale",      // 53
    "Real heading",     // 54
    "Roll",             // 55
    "roll_k",           // 56
    "roll_kn",          // 57
    "RSSI",             // 58
    "RSSI Enable Raw",  // 59
    "RSSI High",        // 60
    "rssi_k",           // 61
    "RSSI Low",         // 62
    "RSSI Warning Level", // 63
    "SAdd1",            // 64
    "SAdd2",            // 65
    "SAdd3",            // 66
    "SAdd4",            // 67
    "Sensor 1",         // 68
    "Sensor 2",         // 69
    "Sensor 3",         // 70
    "Sensor 4",         // 71
    "SFactor1",         // 72
    "SFactor2",         // 73
    "SFactor3",         // 74
    "SFactor4",         // 75
    "SFormat1",         // 76
    "SFormat2",         // 77
    "SFormat3",         // 78
    "SFormat4",         // 79
    "Stall",            // 80
    "Temperature",      // 81
    "Throttle",         // 82
    "Time",             // 83
    "Toggle Channel",   // 84
    "Trip Distance",    // 85
    "Tune",             // 86
    "txtTime0",         // 87
    "txtTime1",         // 88
    "txtTime2",         // 89
    "txtTime3",         // 90
    "Units",            // 91
    "Velocity",         // 92
    "Vertical Speed",   // 93
    "Video Mode",       // 94
    "Visible Sats",     // 95
    "VOS",              // 96
    "Warnings",         // 97
    "Wind Speed",       // 98
    "WP Direction",     // 99
    "WP Distance",      // 100
    "#",                // 101
    "Power",            // 102
    "Date",             // 103
    "Time of day",      // 104
    "Motors",           // 105
    "Vibrations",       // 106
    "Variometer",       // 107
    "GPS Coord Lat",    // 108
    "GPS Coord Lon",    // 109
};
    
typedef struct OSD_PAN {
    uint8_t dst;
    uint8_t size;
    char fmt;
    uint8_t pan_n;
} OSD_pan;

#define m1 ((uint8_t)-1)

static const OSD_pan pan_tbl[]={
    { 0, 0, 0, 0, },
    { 0, 0, 0, ID_of(airSpeed), },     //    "Air Speed",        // 1
    { 0, 0, 0, ID_of(alt), },          //    "Altitude",         // 2
    { 9,    m1, 0, 0, },               //        "Auto Mode",        // 3 - bit in flags
    { 7,    m1, 0, 0, },               //        "Auto Screen Switch", // 4 
    { offsetof(Settings,evBattA_koef),    sizeof(sets.evBattA_koef),   'f', 0, }, //        "batt_a_k",         
    { offsetof(Settings,battBv),          sizeof(sets.battBv),         'b', 0, },  //        "BattB",            
    { offsetof(Settings,evBattB_koef),    sizeof(sets.evBattB_koef),   'f', 0, }, //        "batt_b_k",         
    { offsetof(Settings,battv),           sizeof(sets.battv),          'b', 0, },  //        "Battery",          
    { 0, 0, 0, ID_of(batt_A), },       //        "Battery A",        // 9
    { 0, 0, 0, ID_of(batt_B), },       //        "Battery B",        // 10
    { 0, 0, 0, ID_of(batteryPercent), },//       "Battery Percent",  // 11
    { offsetof(Settings,batt_warn_level), sizeof(sets.batt_warn_level),'b', 0, },  //        "Battery Warning Level", // 12
    { offsetof(Settings,OSD_CALL_SIGN),   sizeof(sets.OSD_CALL_SIGN),  'c', 0, },  //        "Call Sign",        // 13
    { offsetof(Settings,switch_mode),     sizeof(sets.switch_mode),    'b', 0, },  //        "Chanel Rotation Switching", // 14
    { 0, 0, 0, ID_of(ch), },          //        "Channel Raw",      // 15
    { 0, 0, 0, ID_of(Scale), },       //        "Channel Scale",    // 16
    { 0, 0, 0, ID_of(State), },       //        "Channel state",    // 17
    { 0, 0, 0, ID_of(CValue), },      //        "Channel Value",    // 18

    { 0, 0, 0, 0, },                  //         "Configuration",    // 19
    { 0, 0, 0, ID_of(curr_A), },      //        "Current",          // 20
    { offsetof(Settings,eCurrent_koef),   sizeof(sets.eCurrent_koef),  'f', 0, },      //        "curr_k",           // 21
    { 0, 0, 0, ID_of(eff), },         //        "Efficiency",       // 22
    { 4, m1,   0, 0, },               //        "fBattA",           // 23
    { 5, m1,   0, 0, },               //        "fBattB",           // 24      
    { 6, m1,   0, 0, },               //        "fCurr",            // 25
    { 0, 0,    0, 0, },               // 26
    { 0, 0,    0, 0, },               // 27
    { 0, m1,   0, 0, },               //        "flgOnce",          // 28
    { 0, 0,    0, 0, },         //29
    { 0, 0, 0, ID_of(Fdata), },       //        "Flight Data",      // 30
    { 0, 0, 0, ID_of(FMod), },        //        "Flight Mode",      // 31
    { 0, 0, 0, 0, },                  //        "fRussianHUD",      // 32
    { 0, 0, 0, ID_of(GPS), },         //        "GPS Coord",        // 33
    { 0, 0, 0, ID_of(Hdop), },        //        "GPS HDOP",         // 34
    { 0, 0, 0, ID_of(heading), },     //        "Heading",          // 35
    { 0, 0, 0, ID_of(rose), },        //        "Heading Rose",     // 36
    { 0, 0, 0, ID_of(homeAlt), },     //        "Home Altitude",    // 37
    { 0, 0, 0, ID_of(homeDir), },     //        "Home Direction",   // 38
    { 0, 0, 0, ID_of(homeDist), },    //        "Home Distance",    // 39
    { 0, 0, 0, ID_of(horizon), },     //        "Horizon",          // 40
    { offsetof(Settings,horiz_offs),      sizeof(sets.horiz_offs),    'b', 0, },  //        "HOS",              // 41
    { 0, 0, 0, ID_of(message), },         //        "Message",          // 42
    { offsetof(Settings,model_type),      sizeof(sets.model_type),    'b', 0, },  //        "Model Type",       // 43
    { offsetof(Settings,n_screens),       sizeof(sets.n_screens),     'b', 0, },  //        "NSCREENS",         // 44
    { offsetof(Settings,OSD_BRIGHTNESS),  sizeof(sets.OSD_BRIGHTNESS),'b', 0, },  //        "OSD Brightness",   // 45
    { offsetof(Settings,overspeed),       sizeof(sets.overspeed),     'b', 0, },  //        "Overspeed",        // 46

    { 0, 0, 0, 0, },                      //       "Panel",             // 47
    { 0, 0, 0, ID_of(pitch), },           //        "Pitch",            // 48
    { offsetof(Settings,horiz_kPitch),    sizeof(sets.horiz_kPitch),  'f', 0, }, //        "pitch_k",          // 49
    { offsetof(Settings,horiz_kPitch_a),  sizeof(sets.horiz_kPitch_a),'f', 0, }, //        "pitch_kn",         // 50
    { offsetof(Settings,pwm_dst),         sizeof(sets.pwm_dst),       'b', 0, }, //        "PWMDST",           // 51
    { offsetof(Settings,pwm_src),         sizeof(sets.pwm_src),       'b', 0, }, //        "PWMSRC",           // 52
    { 0, 0, 0, ID_of(RadarScale), },         //        "Radar Scale",      // 53
    { 0, 0, 0, ID_of(COG), },                //        "Real heading",     // 54
    { 0, 0, 0, ID_of(roll), },               //        "Roll",             // 55
    { offsetof(Settings,horiz_kRoll),    sizeof(sets.horiz_kRoll),    'f', 0, }, //        "roll_k",           // 56
    { offsetof(Settings,horiz_kRoll_a),  sizeof(sets.horiz_kRoll_a),  'f', 0, }, //        "roll_kn",          // 57
    { 0, 0, 0, ID_of(RSSI), },               //        "RSSI",             // 58
    { offsetof(Settings,RSSI_raw),       sizeof(sets.RSSI_raw),       'b', 0, }, //        "RSSI Enable Raw",  // 59
    { offsetof(Settings,RSSI_16_high),   sizeof(sets.RSSI_16_high),   'w', 0, }, //        "RSSI High",        // 60
    { offsetof(Settings,eRSSI_koef),     sizeof(sets.eRSSI_koef),     'f', 0, }, //        "rssi_k",           // 61
    { offsetof(Settings,RSSI_16_low),    sizeof(sets.RSSI_16_low),    'w', 0, }, //        "RSSI Low",         // 62
    { offsetof(Settings,rssi_warn_level),sizeof(sets.rssi_warn_level),'b', 0, }, //        "RSSI Warning Level", // 63
    { 0, 0, 0, 0, },         //        "SAdd1",            // 64     // sensors not supported
    { 0, 0, 0, 0, },         //        "SAdd2",            // 65
    { 0, 0, 0, 0, },         //        "SAdd3",            // 66
    { 0, 0, 0, 0, },         //        "SAdd4",            // 67
    { 0, 0, 0, ID_of(sensor1), },         //        "Sensor 1",         // 68
    { 0, 0, 0, ID_of(sensor2), },         //        "Sensor 2",         // 69
    { 0, 0, 0, ID_of(sensor3), },         //        "Sensor 3",         // 70
    { 0, 0, 0, ID_of(sensor4), },         //        "Sensor 4",         // 71
    { 0, 0, 0, 0, },         //        "SFactor1",         // 72
    { 0, 0, 0, 0, },         //        "SFactor2",         // 73
    { 0, 0, 0, 0, },         //        "SFactor3",         // 74
    { 0, 0, 0, 0, },         //        "SFactor4",         // 75
    { 0, 0, 0, 0, },         //        "SFormat1",         // 76
    { 0, 0, 0, 0, },         //        "SFormat2",         // 77
    { 0, 0, 0, 0, },         //        "SFormat3",         // 78
    { 0, 0, 0, 0, },         //        "SFormat4",         // 79
    { offsetof(Settings,stall),         sizeof(sets.stall),           'b', 0, },         //        "Stall",            // 80
    { 0, 0, 0, ID_of(temp), },         //        "Temperature",      // 81
    { 0, 0, 0, ID_of(throttle), },     //        "Throttle",         // 82
    { 0, 0, 0, ID_of(time), },         //        "Time",             // 83
    { offsetof(Settings,ch_toggle),     sizeof(sets.ch_toggle),       'b', 0, },         //        "Toggle Channel",   // 84
    { 0, 0, 0, ID_of(distance), },     //        "Trip Distance",    // 85
    { 0, 0, 0, ID_of(tune), },         //        "Tune",             // 86
    { 0, 0, 0, 0, },         //        "txtTime0",         // 87
    { 0, 0, 0, 0, },         //        "txtTime1",         // 88
    { 0, 0, 0, 0, },                        //        "txtTime2",         // 89
    { 0, 0, 0, 0, },                        //        "txtTime3",         // 90
    { 1, m1, 0, 0, },                       //        "Units",            // 91
    { 0, 0, 0, ID_of(vel), },               //        "Velocity",         // 92
    { 0, 0, 0, ID_of(climb), },             //        "Vertical Speed",   // 93
    { 3, m1, 0, 0, },                       //        "Video Mode",       // 94
    { 0, 0, 0, ID_of(GPS_sats), },          //        "Visible Sats",     // 95
    { offsetof(Settings,vert_offs),     sizeof(sets.vert_offs),       'b', 0, },         //        "VOS",              // 96
    { 0, 0, 0, ID_of(warn), },              //        "Warnings",         // 97
    { 0, 0, 0, ID_of(windSpeed), },         //        "Wind Speed",       // 98
    { 0, 0, 0, ID_of(WP_dir), },            //        "WP Direction",     // 99
    { 0, 0, 0, ID_of(WP_dist), },           //        "WP Distance",      // 100
    { 0, 0, 0, 0, },                        // #
    { 0, 0, 0, ID_of(Power), },             //        "Power",            // 102
    { 0, 0, 0, ID_of(fDate), },             //        "Date",             // 103
    { 0, 0, 0, ID_of(dayTime), },           //        "Time of day",      // 104
    { 0, 0, 0, ID_of(pMotor), },            //        "Motors",           // 105
    { 0, 0, 0, ID_of(fVibe), },             //        "Vibrations",       // 106
    { 0, 0, 0, ID_of(fVario), },            //        "Variometer",       // 107
    { 0, 0, 0, ID_of(coordLat), },          //        "GPS Coord Lat",    // 108
    { 0, 0, 0, ID_of(coordLon), },          //        "GPS Coord Lon",    // 109

};

static ring_buffer osd_rxrb IN_CCM;
static uint8_t osd_rx_buf[OSD_RX_BUF_SIZE] IN_CCM;

static ring_buffer osd_txrb IN_CCM;
static uint8_t osd_tx_buf[OSD_TX_BUF_SIZE] IN_CCM;

AP_HAL::OwnPtr<F4Light::SPIDevice> osd_spi;
AP_HAL::Semaphore                *osd_spi_sem;

//static volatile byte vas_vsync=false;

mavlink_system_t mavlink_system = {12,1};  // sysid, compid


#ifdef OSD_DMA_TRANSFER
 #define DMA_BUFFER_SIZE 510
    static uint8_t  dma_buffer[DMA_BUFFER_SIZE+1]; // in RAM for DMA
    static uint16_t dma_transfer_length IN_CCM;

static uint8_t shadowbuf[sizeof(OSD::osdbuf)] IN_CCM;

#endif

static bool diff_done;


extern void heartBeat();
extern void writePanels();

void On100ms();
void On20ms() {}
void osd_loop();
void vsync_ISR();
void max_do_transfer(const char *buffer, uint16_t len);

static void max7456_cs_off(){
    osd_spi->wait_busy(); // wait for transfer complete
    
    const stm32_pin_info &pp = PIN_MAP[BOARD_OSD_CS_PIN];
    gpio_write_bit(pp.gpio_device, pp.gpio_bit, HIGH);

    delay_ns100(3);
}

static void max7456_cs_on(){
    const stm32_pin_info &pp = PIN_MAP[BOARD_OSD_CS_PIN];
    gpio_write_bit(pp.gpio_device, pp.gpio_bit, LOW);
    delay_ns100(1);
}


static uint32_t sem_count=0;

void max7456_on(){
    
    max7456_cs_on();

    osd_spi->set_speed(AP_HAL::Device::SPEED_HIGH);
}

static void max7456_sem_on(){
    
    if(osd_spi_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        max7456_on();
    }
}

void max7456_off(){
    max7456_cs_off();
}

static void max7456_sem_off(){
    max7456_off();
    osd_spi_sem->give(); // give sem on last count
}

void MAX_write(byte addr, byte data){
    max7456_cs_on();
    osd_spi->transfer(addr); // this transfer don't controls CS
    osd_spi->transfer(data);
    max7456_cs_off();
}

byte MAX_read(byte addr){
    max7456_cs_on();
    osd_spi->transfer(addr);      // this transfer don't controls CS
    uint8_t ret = osd_spi->transfer(0xff);
    max7456_cs_off();
    return ret;
}

byte MAX_rw(byte b){
    max7456_cs_on();
    uint8_t ret=osd_spi->transfer(b);
    max7456_cs_off();
    return ret;
}

static uint16_t rdb_ptr IN_CCM;


#ifdef OSD_DMA_TRANSFER
static void prepare_dma_buffer(){
    uint16_t rp;
    uint16_t wp=0;

    uint8_t last_h=0xff;

//    MAX_write(MAX7456_DMM_reg, 0); 
//    MAX_write(MAX7456_VM1_reg, B01000111); 

    memset(dma_buffer,0xff,sizeof(dma_buffer));

    dma_buffer[wp++] = MAX7456_DMM_reg;  dma_buffer[wp++] = 0; 
    dma_buffer[wp++] = MAX7456_VM1_reg;  dma_buffer[wp++] = B01000111; 

    
    // сначала все изменения
    for(rp=0; rp<MAX7456_screen_size ; rp++){
        uint8_t c = OSD::osdbuf[rp];
        if(c != shadowbuf[rp] ){
            if(wp>=DMA_BUFFER_SIZE-6) break;
            uint8_t h = rp>>8;
            if(last_h != h){                
                last_h = h;
                dma_buffer[wp++] = MAX7456_DMAH_reg;  dma_buffer[wp++] = h; 
                if(wp>=DMA_BUFFER_SIZE-6) break;
            }
        
            dma_buffer[wp++] = MAX7456_DMAL_reg;  dma_buffer[wp++] = rp&0xFF; 
            dma_buffer[wp++] = MAX7456_DMDI_reg;  dma_buffer[wp++] = c; 
            shadowbuf[rp] = c;
        }
    }
    
    // а в оставшееся место все остальное по кольцу. таким образом пересылка у нас всегда 500 байт, и на частоте 4.5МГц занимает ~1ms. 
    // длинные пересылки имеют низкий приоритет, и никому не мешают
    while(wp<DMA_BUFFER_SIZE-6){
            uint8_t c = OSD::osdbuf[rdb_ptr];
            uint8_t h = rdb_ptr>>8;
            if(last_h != h){                
                last_h = h;
                dma_buffer[wp++] = MAX7456_DMAH_reg;  dma_buffer[wp++] = h; 
                if(wp>=DMA_BUFFER_SIZE-6) break;
            }
        
            dma_buffer[wp++] = MAX7456_DMAL_reg;  dma_buffer[wp++] = rdb_ptr&0xFF; 
            dma_buffer[wp++] = MAX7456_DMDI_reg;  dma_buffer[wp++] = c; 
            shadowbuf[rdb_ptr] = c;
            rdb_ptr++;
            if(rdb_ptr >= MAX7456_screen_size) rdb_ptr=0; // loop
    }
    
//    dma_buffer[wp++] = MAX7456_VM0_reg;  dma_buffer[wp++] = MAX7456_ENABLE_display | MAX7456_SYNC_autosync | OSD::video_mode; 
        
    dma_transfer_length = wp;
    diff_done = true;
}
#endif


uint32_t get_word(char *buf, char * &ptr){
    uint32_t sel_len=0;
    uint8_t  sel_id=0;
    
    for(uint32_t i=0; i<ARRAY_SIZE(words); i++){
        uint32_t len=strlen(words[i]);
        if(strncmp(buf, words[i],len)==0){
            if(len > sel_len) { // longest match
                sel_len = len;
                sel_id = i+1;
            }
        }
    }
    ptr=buf+sel_len;
    return sel_id;
}

char * get_lex(char * &ptro){
    char *ptr;
    char *buf = ptro;
    while(*buf && (*buf=='\t' || *buf == ' ')) buf++;
    ptr=buf;
    while(*ptr && *ptr!='\t') ptr++;
    if(*ptr==0) {
        ptro=NULL;
    } else {
        *ptr=0;
        ptro=ptr+1;
    }
    return buf;
    
}

static bool get_flag(char *p) {
    if(!p) return false;
    
    if(*p=='T' || *p=='t' || *p=='1') return true;
    return false;
}

//                          x,         y,           vis,          sign,    Altf,        Alt2,        Alt3,          Alt4,     strings
//                         30         15          False            0       1            1             1              1          A||||
static point create_point(char *px,   char *py, char *pVis,  char *pSign, char *pAlt,  char *pAlt2, char *pAlt3, char *pAlt4,  char *ps ){
    point p;
    p.x = strtoul(px, nullptr, 10);
    p.y = strtoul(py, nullptr, 10);
    p = do_on(p,   get_flag(pVis));
    p = do_sign(p, get_flag(pSign));
    if(get_flag(pAlt))  p=do_alt(p);
    if(get_flag(pAlt2)) p=do_alt2(p);
    if(get_flag(pAlt3)) p=do_alt3(p);
    if(get_flag(pAlt4)) p=do_alt4(p);

//    if(ps) collect_strings(ps); not supported
    return p;
}

#define write_point(n,p) eeprom_write_len((byte *)&p,  OffsetBITpanel * (int)panel_num + n * sizeof(Point),  sizeof(Point) );

static void load_config(){
    File fd = SD.open("eeprom.osd", FILE_READ);
    if (fd) {
        printf("\nLoading OSD config\n");
        char buf[80];
//        memset(buf, 0, sizeof(buf));
        uint32_t panel_num=-1;
        bool is_config=false;
        
        while(fd.gets(buf, sizeof(buf)-1) > 0) {
            // we readed one line
            char *ptr;
            char *p[10];

            uint8_t word=get_word(buf,ptr);
            switch(word) {
            case 0: // not found
            case 101: // #
                continue;
                
            case 47: { // panel
                    char *p2 = get_lex(ptr);
                    panel_num=strtoul(p2, nullptr, 10);
                    uint16_t flags=strtoul(ptr, nullptr, 10);
                    write_point(0,flags);
                }break;
                
            case 19: // config
                is_config=true;
                break;
                
            default:
                char **pp = p;
                memset(p,0,sizeof(p));
                do {
                    *pp++ = get_lex(ptr);
                }while(ptr);
                
                                
                if(is_config){
                    if(pan_tbl[word].size == (uint8_t)-1){ // bit flags
                        uint32_t flags = sets.flags.dw;
                        if(get_flag(p[0])) flags |=  (1<<pan_tbl[word].dst);
                        else               flags &= ~(1<<pan_tbl[word].dst);
                        sets.flags.dw=flags;
                    } else {
                        union {
                            float f;
                            uint8_t b;
                            char buf[8];
                            uint16_t w;
                        } val;
                        
                        switch(pan_tbl[word].fmt){
                        case 'f': // float
                            val.f = atof(p[0]);
                            break;
                        case 'b': // byte
                            val.b=(uint8_t)strtoul(p[0], nullptr, 10);
                            break;
                        case 'c': // char
                            strncpy(val.buf, p[0], 8);
                            break;
                        case 'w':
                            val.w=(uint16_t)strtoul(p[0], nullptr, 10);
                            break;                    
                        default:
                            continue; // ignore this line
                        }                    
                        memmove(         ((uint8_t*)(&sets)) + pan_tbl[word].dst, &val, pan_tbl[word].size);
                        eeprom_write_len(((uint8_t*)(&sets)) + pan_tbl[word].dst, EEPROM_offs(sets) + pan_tbl[word].dst,  pan_tbl[word].size);
                    }
                }else{ // panel
                    uint8_t id = pan_tbl[word].pan_n;
                    if(id) {
                //                           30    15    False   0     1     1     1     1   A||||
                        point po=create_point(p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8] );
                    
                        write_point(id, po); // save to eeprom
                    }
                }
                
                break;
            }
            
        }
        fd.close();

        // we don't save flags when parsing so lets do it now
        eeprom_write_len( &sets.flags.pad[0],  EEPROM_offs(sets) + ((byte *)&sets.flags.pad - (byte *)&sets),  sizeof(sets.flags.pad));

        readSettings(); // re-read new values back, for settings that not in config.osd get all-1 state
        
        sets.CHK1_VERSION = VER;        // set version - EEPROM OK
        sets.CHK2_VERSION = (VER ^ 0x55);
        eeprom_write_len( &sets.CHK1_VERSION,  EEPROM_offs(sets) + ((byte *)&sets.CHK1_VERSION - (byte *)&sets),  2 );
    }
}

static void load_font(){
    const char font[]="font.mcm";
    File fd = SD.open(font, FILE_READ);
    if (fd) {
        char buf[80];
        
        if(fd.gets(buf, sizeof(buf)-1)){
            printf("\nLoading OSD font\n");

            OSD::setPanel(5, 5);
            osd_print_S(PSTR("font uploading "));
            OSD::update();// Show it
            
            char patt[]="MAX7456";
            
            if(strncmp(buf,patt,strlen(patt))==0){
                uint8_t character_bitmap[0x40];
                
                uint16_t font_count = 0;
                byte byte_count = 0;
                byte bit_count=0;

                uint8_t chk=0;
                uint8_t c=0;
                uint8_t last_c;

                uint8_t b=0;

                uint8_t cnt=0;
                while(font_count < 256) {
                    last_c = c;
                    if(fd.read(&c,1)<=0) break; // get a char
                                            
                    switch(c){ // parse and decode mcm file
                    case 0x0A: // line feed - skip
                        if(last_c == 0x0d) continue;
                        // lf without cr cause line end

                    case 0x0d: // carridge return, end of line
                        if (bit_count == 8)  {
                            chk ^= b;
                            character_bitmap[byte_count] = b;
                            b = 0;
                            byte_count++;
                        }
                        bit_count = 0;
                        break;

                    case 0x30: // ascii '0'
                    case 0x31: // ascii '1' 
                        b <<= 1;
                        if(c == 0x31)
                            b += 1;
                        bit_count++;

                        break;

                    default:
                        break;
                    }
            
                    // we have one completed character
                    // write the character to NVM 
                    if(byte_count == 64) {
                        osd.write_NVM(font_count, character_bitmap);
                        byte_count = 0;
                        font_count++;
                        printf(".");
                        chk=0;
                    }                    
                }
                printf("done\n");
            }                    
        }

        fd.close();
//* not in debug mode
        SD.remove(font); // once!
//*/
    }

}

static bool osd_need_redraw = false;
static void * task_handle;

// slowly write all buffer
static void write_buff_to_MAX(bool all){

    max7456_on();
    MAX_write(MAX7456_DMM_reg, 0); 

    // clear internal memory
    uint8_t old_h=0xff;
    for(uint16_t len = MAX7456_screen_size, cnt=0;len--; cnt++){
        uint8_t c= OSD::osdbuf[cnt];
        if(all || c!=0x20) {
            max7456_cs_on();
            uint8_t h = cnt>>8;
            if(old_h!=h){
                MAX_write(MAX7456_DMAH_reg, h);
                old_h = h;
            }
            MAX_write(MAX7456_DMAL_reg, cnt&0xFF);
            MAX_write(MAX7456_DMDI_reg, c);
            max7456_cs_off();
        }
#ifdef OSD_DMA_TRANSFER
        shadowbuf[cnt] = c;
#endif
    }
    max7456_off();
}



void osd_begin(AP_HAL::OwnPtr<F4Light::SPIDevice> spi){

    osd_spi = std::move(spi);
    
    osd_spi_sem = osd_spi->get_semaphore(); // bus semaphore
    {
        const stm32_pin_info &pp = PIN_MAP[BOARD_OSD_CS_PIN];
        gpio_set_mode(pp.gpio_device, pp.gpio_bit, GPIO_OUTPUT_PP);
        gpio_set_speed(pp.gpio_device, pp.gpio_bit, GPIO_speed_100MHz); 
        gpio_write_bit(pp.gpio_device, pp.gpio_bit, HIGH);
    }

#ifdef BOARD_OSD_RESET_PIN
    {
        const stm32_pin_info &pp = PIN_MAP[BOARD_OSD_RESET_PIN];
        gpio_set_mode(pp.gpio_device, pp.gpio_bit, GPIO_OUTPUT_PP);
        gpio_set_speed(pp.gpio_device, pp.gpio_bit, GPIO_speed_25MHz); 
        gpio_write_bit(pp.gpio_device, pp.gpio_bit, LOW);
        delayMicroseconds(50);
        gpio_write_bit(pp.gpio_device, pp.gpio_bit, HIGH);
        delayMicroseconds(120);
    }
#endif


    rb_init(&osd_rxrb, OSD_RX_BUF_SIZE, osd_rx_buf);
    rb_init(&osd_txrb, OSD_TX_BUF_SIZE, osd_tx_buf);

    OSD_EEPROM::init();

    // clear memory
    memset(OSD::osdbuf,0x20, sizeof(OSD::osdbuf));
#ifdef OSD_DMA_TRANSFER
    memset(shadowbuf,  0x20, sizeof(shadowbuf));
#endif

/*
    lets try to load settings from SD card
*/
    load_config();

    readSettings();

    doScreenSwitch(); // set vars for startup screen

    if( sets.CHK1_VERSION != VER || sets.CHK2_VERSION != (VER ^ 0x55)) { // wrong version
        lflags.bad_config=1;
        
        // some useful defaults
        sets.OSD_BRIGHTNESS = 2;
        sets.horiz_offs = 0x20;
        sets.vert_offs  = 0x10;
        
    }

    while(millis()<1000) { // delay initialization until video stabilizes
        hal_yield(1000);
    }

    max7456_sem_on();

    write_buff_to_MAX(true);


    for(uint8_t i=0; i<100; i++) {
        osd.init();    // Start display
        
        max7456_on();
        uint8_t vm0 = MAX_read(MAX7456_VM0_reg | MAX7456_reg_read); // check register
        max7456_off();

        uint8_t patt = MAX7456_ENABLE_display | MAX7456_SYNC_autosync | OSD::video_mode;

        if(vm0==patt) break;
    }

    max7456_off();

    load_font();

    max7456_sem_off();

#define REL_1 int(RELEASE_NUM/100)
#define REL_2 int((RELEASE_NUM - REL_1*100 )/10) 
#define REL_3 int((RELEASE_NUM - REL_1*100 - REL_2*10  )) 

    if(sets.FW_VERSION[0]!=(REL_1 + '0') || sets.FW_VERSION[1]!=(REL_2 + '0') || sets.FW_VERSION[2]!=(REL_3 + '0') ){
        sets.FW_VERSION[0]=REL_1 + '0';
        sets.FW_VERSION[1]=REL_2 + '0';
        sets.FW_VERSION[2]=REL_3 + '0';

        eeprom_write_len( sets.FW_VERSION,  EEPROM_offs(sets) + ((uint8_t *)sets.FW_VERSION - (uint8_t *)&sets),  sizeof(sets.FW_VERSION) );
    }
    
    logo();

    
#ifdef BOARD_OSD_VSYNC_PIN
    Revo_hal_handler h = { .vp = vsync_ISR };
    
    GPIO::_attach_interrupt(BOARD_OSD_VSYNC_PIN, h.h, RISING, VSI_INT_PRIORITY);
#endif

    task_handle = Scheduler::start_task(OSDns::osd_loop, SMALL_TASK_STACK); // 
    Scheduler::set_task_priority(task_handle, OSD_LOW_PRIORITY); // less than main task
    Scheduler::set_task_period(task_handle, 10000);              // 100Hz 
}

// all task is in one thread so no sync required

void osd_loop() {
    if(osd_need_redraw){ // если была отложенная передача
        osd_need_redraw=false;
        
        OSD::update();           
        Scheduler::set_task_priority(task_handle, OSD_LOW_PRIORITY); // restore priority to low
    }

    uint32_t pt=millis();

    seconds = pt / 1000;

    osd_dequeue(); // we MUST parse input even in case  of bad config because it is the only way to communicate

    if(pt < BOOTTIME || lflags.bad_config){ // startup delay for fonts or EEPROM error
            logo();
            return;
    }

#if defined(MAV_REQUEST) && (defined(USE_MAVLINK) || defined(USE_MAVLINKPX4))
    if(apm_mav_system && !lflags.mav_request_done){ // we got HEARTBEAT packet and still don't send requests
        for(uint8_t n = 3; n >0; n--){
            request_mavlink_rates();        //Three times to certify it will be readed
            delay_150();
        }
        lflags.mav_request_done=1;
    }
#endif

    if(lflags.got_data){ // if new data comes

        pan_toggle(); // check for screen toggle

        if(!lflags.need_redraw) {
            lflags.need_redraw=1;
            vsync_wait=1; // will wait for interrupt
        }

        lflags.got_data=0; // data parsed
    }
    
    if( lflags.need_redraw) {                 
        lflags.need_redraw=0; // screen drawn

        setHomeVars();   // calculate and set Distance from home and Direction to home

        setFdataVars();  // statistics and min/max 

        writePanels();   // writing enabled panels (check OSD_Panels Tab)

#ifdef OSD_DMA_TRANSFER
        prepare_dma_buffer(); // prepare diff with addresses
#endif

        update_screen = 1; // data comes, wee need to redraw screen
    }

    if(pt > timer_20ms){
        timer_20ms+=20;
        On20ms();
        
        if(update_screen && vsync_wait && (millis() - vsync_time)>50){ // interrupts stopped - more than 50 ms passed from the last one
            vsync_wait=0; // хватит ждать
            Scheduler::set_task_priority(task_handle, OSD_HIGH_PRIORITY); // equal to main 
            OSD::update(); // update compulsorily (and then update every 20ms)
            update_screen = 0;
            Scheduler::set_task_priority(task_handle, OSD_LOW_PRIORITY);
        }
    }

    if(pt > timer_100ms){
        timer_100ms+= 100;
        On100ms(); 

        lflags.flag_01s = !lflags.flag_01s;

        if(lflags.flag_01s) {

            if(skip_inc) {
                skip_inc++;

                if(skip_inc >=3){
                    count02s++;
                    skip_inc=0; // we go again
                }

            } else
                count02s++;
        }
//      count01s++;
    }

    if(pt > timer_500ms){
        timer_500ms+= 500;
        lflags.got_data=1; // every half second forcibly
        update_screen = 1; 

        lflags.flag_05s = 1;

        count05s++;

        lflags.blinker = !lflags.blinker;
        if(lflags.blinker) {
    //        seconds++;
            lflags.one_sec_timer_switch = 1; // for warnings

            if(lflags.got_date) day_seconds++; // if we has GPS time - let it ticks

            if( vas_vsync && vsync_count < 5) { // at a frame rate they should be 25 or 50
                                                    // but there are boards where this pin is not connected. China...
                max7456_err_count++;
                if(max7456_err_count>3) { // 3 seconds bad sync
#ifdef DEBUG   
                    printf(PSTR("restart MAX! vsync_count=%d\n"),vsync_count);
#endif
                    osd.reset();    // restart MAX7456
                }
            } else  max7456_err_count=0;

            vsync_count=0;

            heartBeat();
        }
    }
}

float avgRSSI(uint16_t d){
    static uint8_t ind = -1;
    static uint16_t RSSI_rawArray[8];

    RSSI_rawArray[(++ind)%8] = d;
    d=0;
    for (uint8_t i=8;i!=0;)
        d += RSSI_rawArray[--i];

    return d/8.0;
}

void On100ms() {

        byte ch = sets.RSSI_raw / 2;

        uint16_t d;


//DBG_PRINTF("\n RSSI ch=%d ", ch);

        switch(ch) {

        case 1: // analog RSSI from pin
        case 2: // digital RSSI from pin

        case 0:
            d=osd_rssi; // mavlink
            goto case_4;

        case 3: // 3dr modem rssi
            d=telem_rssi;
            goto case_4;

        case 4:
        default:
            d = chan_raw[7]; // ch 8

//DBG_PRINTF("ch8_rssi=%d\n", d );

case_4:
            rssi_in = avgRSSI(d);

// RSSI source is not pin so we can read it for sensor
#if defined(USE_SENSORS)
            if(SENSOR4_ON) {
                if(fPulseSensor[3])
                    d=pulseIn(RssiPin,HIGH,10000);
                else
                    d=analogRead(RssiPin);

                sensorData[3] = (sensorData[3]*7 + d) /8;
            }
#endif
            break;
        }
}


void vsync_ISR(){
    vas_vsync=true;
    vsync_wait=0;       // note its presence

    vsync_count++; // count VSYNC interrupts
    vsync_time=millis(); // and note a time

    if(update_screen) { // there is data for screen
        osd_need_redraw=true;
        Scheduler::set_task_priority(task_handle, OSD_HIGH_PRIORITY); // higher than all drivers so it will be scheduled just after semaphore release
        Scheduler::task_resume(task_handle); // task should be finished at this time so resume it
        update_screen = 0;
    }
}



// is there any chars in ring buffer?
int16_t osd_available(){
    return rb_full_count(&osd_rxrb);
}

void osd_queue(uint8_t c) {    // push bytes from OSD to FC around in the ring buffer
    uint8_t cnt=10;
    while(rb_is_full(&osd_rxrb)) {
        hal_yield(0);
        if(--cnt == 0) return; // destination don't listen
    }
    rb_push_insert(&osd_rxrb, c);
}


int16_t osd_getc(){ // get char from ring buffer
    return rb_remove(&osd_rxrb);
}

uint32_t osd_txspace() {
    return osd_txrb.size - rb_full_count(&osd_txrb);
}


void osd_putc(uint8_t c){ 
    uint8_t cnt=10;
    while(rb_is_full(&osd_txrb)) {
        Scheduler::set_task_priority(task_handle, OSD_HIGH_PRIORITY); // to run in time of yield()
        hal_yield(0);
        if(--cnt == 0) break; // destination don't listen
    }
    rb_push_insert(&osd_txrb, c);
    Scheduler::set_task_priority(task_handle, OSD_LOW_PRIORITY); // restore priority to low
}

void osd_dequeue() {
    Scheduler::set_task_priority(task_handle, 100); // equal to main to not overflow buffers on packet decode

    while(!rb_is_empty(&osd_txrb)) {
        extern bool mavlink_one_byte(char c);
        char c = rb_remove(&osd_txrb);
    
        if(mavlink_one_byte(c)) lflags.got_data=true;
    }
    Scheduler::set_task_priority(task_handle, OSD_LOW_PRIORITY); // restore priority to low

}


static uint8_t max_err_cnt=0;

void update_max_buffer(const uint8_t *buffer, uint16_t len){
    max7456_sem_on();

    uint16_t cnt=0;
    
    
#if 1
    uint8_t patt = MAX7456_ENABLE_display | MAX7456_SYNC_autosync | OSD::video_mode;
    max7456_cs_on();
    uint8_t vm0 = MAX_read(MAX7456_VM0_reg | MAX7456_reg_read);
        
    if(vm0 != patt) {
        max_err_cnt++;
        if(max_err_cnt<3) {
            OSD::hw_init(); // first try without reset
        } else {
                // 3 errors together - nothing helps :(
#ifdef BOARD_OSD_RESET_PIN
            {
                const stm32_pin_info &pp = PIN_MAP[BOARD_OSD_RESET_PIN];
                gpio_write_bit(pp.gpio_device, pp.gpio_bit, LOW);
                delayMicroseconds(50);
                gpio_write_bit(pp.gpio_device, pp.gpio_bit, HIGH);
                delayMicroseconds(120);
            }
#endif
            OSD::init();
            max_err_cnt=0; 
        }
    } else {
        max_err_cnt=0;
    }

    MAX_write(MAX7456_DMAH_reg, 0);
    MAX_write(MAX7456_DMAL_reg, 0);
    MAX_write(MAX7456_DMM_reg, 1); // set address auto-increment

    max7456_cs_off();
    
    if(osd_spi->send_strobe(buffer, len)!=len) {
    /*/// for debug - mark last written char
        MAX_rw(0x86); // finish transfer
    //*///
        MAX_rw(0xff); // finish transfer
    }
    
#elif 0
    MAX_write(MAX7456_DMAH_reg, 0);
    MAX_write(MAX7456_DMAL_reg, 0);
    MAX_write(MAX7456_DMM_reg, 0); 

// try to send just diffenence - don't clears last chars
    uint8_t last_h=0;
    while(len--){
        if(*buffer != shadowbuf[cnt]){
            uint8_t h = cnt>>8 ;
            if(last_h != h){
                MAX_write(MAX7456_DMAH_reg, h);
                last_h = h;
            }
            MAX_write(MAX7456_DMAL_reg, cnt&0xFF);
            MAX_write(MAX7456_DMDI_reg, *buffer);
            shadowbuf[cnt] = *buffer;
        }
        buffer++;
        cnt++;
    }

#elif 0

// a try to do writes in software strobe mode
    MAX_write(MAX7456_DMAH_reg, 0);
    MAX_write(MAX7456_DMAL_reg, 0);
    MAX_write(MAX7456_DMM_reg, 1); // set address auto-increment
    max7456_cs_off();
    while(len--){
        max7456_cs_on();
        // osd_spi->transfer(*buffer++); MAX7456
        MAX_rw(*buffer++);
        buffer++;
        cnt++;
        osd_spi->wait_busy();
        max7456_cs_off();
    }
    max7456_cs_on();
    MAX_write(MAX7456_DMM_reg, 0); // clear address auto-increment
#else
// just write all to MAX
    MAX_write(MAX7456_DMAH_reg, 0);
    MAX_write(MAX7456_DMAL_reg, 0);
    MAX_write(MAX7456_DMM_reg, 0); 

    uint8_t last_h=0;
    while(len--){
        uint8_t h = cnt>>8 ;
        if(last_h != h){
            MAX_write(MAX7456_DMAH_reg, h);
            last_h = h;
        }
        
        MAX_write(MAX7456_DMAL_reg, cnt&0xFF);
        MAX_write(MAX7456_DMDI_reg, *buffer);
        buffer++;
        cnt++;
    }

#endif    

    max7456_sem_off();
}



} // namespace


#endif