/*
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org/>
*/

/*
This version is for pigpio version 70+
*/

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>
#include <inttypes.h>

#include "pigpio.h"
#include "command.h"

cmdInfo_t cmdInfo[]=
{
   /* num          str    vfyt retv script*/

   {PI_CMD_BC1,   "BC1",   111, 1, 1}, // gpioWrite_Bits_0_31_Clear
   {PI_CMD_BC2,   "BC2",   111, 1, 1}, // gpioWrite_Bits_32_53_Clear

   {PI_CMD_BI2CC, "BI2CC", 112, 0, 1}, // bbI2CClose
   {PI_CMD_BI2CO, "BI2CO", 131, 0, 1}, // bbI2COpen
   {PI_CMD_BI2CZ, "BI2CZ", 193, 6, 0}, // bbI2CZip

   {PI_CMD_BR1,   "BR1",   101, 3, 1}, // gpioRead_Bits_0_31
   {PI_CMD_BR2,   "BR2",   101, 3, 1}, // gpioRead_Bits_32_53

   {PI_CMD_BS1,   "BS1",   111, 1, 1}, // gpioWrite_Bits_0_31_Set
   {PI_CMD_BS2,   "BS2",   111, 1, 1}, // gpioWrite_Bits_32_53_Set

   {PI_CMD_BSCX,  "BSCX",  193, 8, 0}, // bscXfer

   {PI_CMD_BSPIC, "BSPIC", 112, 0, 1}, // bbSPIClose
   {PI_CMD_BSPIO, "BSPIO", 134, 0, 0}, // bbSPIOpen
   {PI_CMD_BSPIX, "BSPIX", 193, 6, 0}, // bbSPIXfer

   {PI_CMD_CF1,   "CF1",   195, 2, 0}, // gpioCustom1
   {PI_CMD_CF2,   "CF2",   195, 6, 0}, // gpioCustom2

   {PI_CMD_CGI,   "CGI",   101, 4, 1}, // gpioCfgGetInternals
   {PI_CMD_CSI,   "CSI",   111, 1, 1}, // gpioCfgSetInternals

   {PI_CMD_EVM,   "EVM",   122, 1, 1}, // eventMonitor
   {PI_CMD_EVT,   "EVT",   112, 0, 1}, // eventTrigger

   {PI_CMD_FC,    "FC",    112, 0, 1}, // fileClose

   {PI_CMD_FG,    "FG",    121, 0, 1}, // gpioGlitchFilter

   {PI_CMD_FL,    "FL",    127, 6, 0}, // fileList

   {PI_CMD_FN,    "FN",    131, 0, 1}, // gpioNoiseFilter

   {PI_CMD_FO,    "FO",    127, 2, 0}, // fileOpen
   {PI_CMD_FR,    "FR",    121, 6, 0}, // fileRead
   {PI_CMD_FS,    "FS",    133, 2, 1}, // fileSeek
   {PI_CMD_FW,    "FW",    193, 0, 0}, // fileWrite

   {PI_CMD_GDC,   "GDC",   112, 2, 1}, // gpioGetPWMdutycycle
   {PI_CMD_GPW,   "GPW",   112, 2, 1}, // gpioGetServoPulsewidth

   {PI_CMD_HELP,  "H",     101, 5, 0}, // cmdUsage
   {PI_CMD_HELP,  "HELP",  101, 5, 0}, // cmdUsage

   {PI_CMD_HC,    "HC",    121, 0, 1}, // gpioHardwareClock
   {PI_CMD_HP,    "HP",    131, 0, 1}, // gpioHardwarePWM

   {PI_CMD_HWVER, "HWVER", 101, 4, 1}, // gpioHardwareRevision

   {PI_CMD_I2CC,  "I2CC",  112, 0, 1}, // i2cClose
   {PI_CMD_I2CO,  "I2CO",  131, 2, 1}, // i2cOpen

   {PI_CMD_I2CPC, "I2CPC", 131, 2, 1}, // i2cProcessCall
   {PI_CMD_I2CPK, "I2CPK", 194, 6, 0}, // i2cBlockProcessCall

   {PI_CMD_I2CRB, "I2CRB", 121, 2, 1}, // i2cReadByteData
   {PI_CMD_I2CRD, "I2CRD", 121, 6, 0}, // i2cReadDevice
   {PI_CMD_I2CRI, "I2CRI", 131, 6, 0}, // i2cReadI2CBlockData
   {PI_CMD_I2CRK, "I2CRK", 121, 6, 0}, // i2cReadBlockData
   {PI_CMD_I2CRS, "I2CRS", 112, 2, 1}, // i2cReadByte
   {PI_CMD_I2CRW, "I2CRW", 121, 2, 1}, // i2cReadWordData

   {PI_CMD_I2CWB, "I2CWB", 131, 0, 1}, // i2cWriteByteData
   {PI_CMD_I2CWD, "I2CWD", 193, 0, 0}, // i2cWriteDevice
   {PI_CMD_I2CWI, "I2CWI", 194, 0, 0}, // i2cWriteI2CBlockData
   {PI_CMD_I2CWK, "I2CWK", 194, 0, 0}, // i2cWriteBlockData
   {PI_CMD_I2CWQ, "I2CWQ", 121, 0, 1}, // i2cWriteQuick
   {PI_CMD_I2CWS, "I2CWS", 121, 0, 1}, // i2cWriteByte
   {PI_CMD_I2CWW, "I2CWW", 131, 0, 1}, // i2cWriteWordData

   {PI_CMD_I2CZ,  "I2CZ",  193, 6, 0}, // i2cZip

   {PI_CMD_MICS,  "MICS",  112, 0, 1}, // gpioDelay
   {PI_CMD_MILS,  "MILS",  112, 0, 1}, // gpioDelay

   {PI_CMD_MODEG, "MG"   , 112, 2, 1}, // gpioGetMode
   {PI_CMD_MODEG, "MODEG", 112, 2, 1}, // gpioGetMode

   {PI_CMD_MODES, "M",     125, 0, 1}, // gpioSetMode
   {PI_CMD_MODES, "MODES", 125, 0, 1}, // gpioSetMode

   {PI_CMD_NB,    "NB",    122, 0, 1}, // gpioNotifyBegin
   {PI_CMD_NC,    "NC",    112, 0, 1}, // gpioNotifyClose
   {PI_CMD_NO,    "NO",    101, 2, 1}, // gpioNotifyOpen
   {PI_CMD_NP,    "NP",    112, 0, 1}, // gpioNotifyPause

   {PI_CMD_PADG,  "PADG",  112, 2, 1}, // gpioGetPad
   {PI_CMD_PADS,  "PADS",  121, 0, 1}, // gpioSetPad

   {PI_CMD_PARSE, "PARSE", 115, 0, 0}, // cmdParseScript

   {PI_CMD_PFG,   "PFG",   112, 2, 1}, // gpioGetPWMfrequency
   {PI_CMD_PFS,   "PFS",   121, 2, 1}, // gpioSetPWMfrequency

   {PI_CMD_PIGPV, "PIGPV", 101, 4, 1}, // gpioVersion

   {PI_CMD_PRG,   "PRG",   112, 2, 1}, // gpioGetPWMrange

   {PI_CMD_PROC,  "PROC",  115, 2, 0}, // gpioStoreScript
   {PI_CMD_PROCD, "PROCD", 112, 0, 0}, // gpioDeleteScript
   {PI_CMD_PROCP, "PROCP", 112, 7, 0}, // gpioScriptStatus
   {PI_CMD_PROCR, "PROCR", 191, 0, 0}, // gpioRunScript
   {PI_CMD_PROCS, "PROCS", 112, 0, 0}, // gpioStopScript
   {PI_CMD_PROCU, "PROCU", 191, 0, 0}, // gpioUpdateScript

   {PI_CMD_PRRG,  "PRRG",  112, 2, 1}, // gpioGetPWMrealRange
   {PI_CMD_PRS,   "PRS",   121, 2, 1}, // gpioSetPWMrange

   {PI_CMD_PUD,   "PUD",   126, 0, 1}, // gpioSetPullUpDown

   {PI_CMD_PWM,   "P",     121, 0, 1}, // gpioPWM
   {PI_CMD_PWM,   "PWM",   121, 0, 1}, // gpioPWM

   {PI_CMD_READ,  "R",     112, 2, 1}, // gpioRead
   {PI_CMD_READ,  "READ",  112, 2, 1}, // gpioRead

   {PI_CMD_SERC,  "SERC",  112, 0, 1}, // serClose
   {PI_CMD_SERDA, "SERDA", 112, 2, 1}, // serDataAvailable
   {PI_CMD_SERO,  "SERO",  132, 2, 0}, // serOpen
   {PI_CMD_SERR,  "SERR",  121, 6, 0}, // serRead
   {PI_CMD_SERRB, "SERRB", 112, 2, 1}, // serReadByte
   {PI_CMD_SERW,  "SERW",  193, 0, 0}, // serWrite
   {PI_CMD_SERWB, "SERWB", 121, 0, 1}, // serWriteByte

   {PI_CMD_SERVO, "S",     121, 0, 1}, // gpioServo
   {PI_CMD_SERVO, "SERVO", 121, 0, 1}, // gpioServo

   {PI_CMD_SHELL, "SHELL", 128, 2, 0}, // shell

   {PI_CMD_SLR,   "SLR",   121, 6, 0}, // gpioSerialRead
   {PI_CMD_SLRC,  "SLRC",  112, 0, 1}, // gpioSerialReadClose
   {PI_CMD_SLRO,  "SLRO",  131, 0, 1}, // gpioSerialReadOpen
   {PI_CMD_SLRI,  "SLRI",  121, 0, 1}, // gpioSerialReadInvert

   {PI_CMD_SPIC,  "SPIC",  112, 0, 1}, // spiClose
   {PI_CMD_SPIO,  "SPIO",  131, 2, 1}, // spiOpen
   {PI_CMD_SPIR,  "SPIR",  121, 6, 0}, // spiRead
   {PI_CMD_SPIW,  "SPIW",  193, 0, 0}, // spiWrite
   {PI_CMD_SPIX,  "SPIX",  193, 6, 0}, // spiXfer

   {PI_CMD_TICK,  "T",     101, 4, 1}, // gpioTick
   {PI_CMD_TICK,  "TICK",  101, 4, 1}, // gpioTick

   {PI_CMD_TRIG,  "TRIG",  131, 0, 1}, // gpioTrigger

   {PI_CMD_WDOG,  "WDOG",  121, 0, 1}, // gpioSetWatchdog

   {PI_CMD_WRITE, "W",     121, 0, 1}, // gpioWrite
   {PI_CMD_WRITE, "WRITE", 121, 0, 1}, // gpioWrite

   {PI_CMD_WVAG,  "WVAG",  192, 2, 0}, // gpioWaveAddGeneric
   {PI_CMD_WVAS,  "WVAS",  196, 2, 0}, // gpioWaveAddSerial
   {PI_CMD_WVBSY, "WVBSY", 101, 2, 1}, // gpioWaveTxBusy
   {PI_CMD_WVCHA, "WVCHA", 197, 0, 0}, // gpioWaveChain
   {PI_CMD_WVCLR, "WVCLR", 101, 0, 1}, // gpioWaveClear
   {PI_CMD_WVCRE, "WVCRE", 101, 2, 1}, // gpioWaveCreate 
   {PI_CMD_WVCAP, "WVCAP", 112, 2, 1}, // gpioWaveCreatePad
   {PI_CMD_WVDEL, "WVDEL", 112, 0, 1}, // gpioWaveDelete
   {PI_CMD_WVGO,  "WVGO" , 101, 2, 0}, // gpioWaveTxStart
   {PI_CMD_WVGOR, "WVGOR", 101, 2, 0}, // gpioWaveTxStart
   {PI_CMD_WVHLT, "WVHLT", 101, 0, 1}, // gpioWaveTxStop
   {PI_CMD_WVNEW, "WVNEW", 101, 0, 1}, // gpioWaveAddNew
   {PI_CMD_WVSC,  "WVSC",  112, 2, 1}, // gpioWaveGet*Cbs
   {PI_CMD_WVSM,  "WVSM",  112, 2, 1}, // gpioWaveGet*Micros
   {PI_CMD_WVSP,  "WVSP",  112, 2, 1}, // gpioWaveGet*Pulses
   {PI_CMD_WVTAT, "WVTAT", 101, 2, 1}, // gpioWaveTxAt
   {PI_CMD_WVTX,  "WVTX",  112, 2, 1}, // gpioWaveTxSend
   {PI_CMD_WVTXM, "WVTXM", 121, 2, 1}, // gpioWaveTxSend
   {PI_CMD_WVTXR, "WVTXR", 112, 2, 1}, // gpioWaveTxSend

   {PI_CMD_ADD  , "ADD"  , 111, 0, 1},
   {PI_CMD_AND  , "AND"  , 111, 0, 1},
   {PI_CMD_CALL , "CALL" , 114, 0, 1},
   {PI_CMD_CMDR  ,"CMDR" , 111, 0, 1},
   {PI_CMD_CMDW , "CMDW" , 111, 0, 1},
   {PI_CMD_CMP  , "CMP"  , 111, 0, 1},
   {PI_CMD_DCR  , "DCR"  , 113, 0, 1},
   {PI_CMD_DCRA , "DCRA" , 101, 0, 1},
   {PI_CMD_DIV  , "DIV"  , 111, 0, 1},
   {PI_CMD_EVTWT, "EVTWT", 111, 0, 1},
   {PI_CMD_HALT , "HALT" , 101, 0, 1},
   {PI_CMD_INR  , "INR"  , 113, 0, 1},
   {PI_CMD_INRA , "INRA" , 101, 0, 1},
   {PI_CMD_JM   , "JM"   , 114, 0, 1},
   {PI_CMD_JMP  , "JMP"  , 114, 0, 1},
   {PI_CMD_JNZ  , "JNZ"  , 114, 0, 1},
   {PI_CMD_JP   , "JP"   , 114, 0, 1},
   {PI_CMD_JZ   , "JZ"   , 114, 0, 1},
   {PI_CMD_LD   , "LD"   , 123, 0, 1},
   {PI_CMD_LDA  , "LDA"  , 111, 0, 1},
   {PI_CMD_LDAB , "LDAB" , 111, 0, 1},
   {PI_CMD_MLT  , "MLT"  , 111, 0, 1},
   {PI_CMD_MOD  , "MOD"  , 111, 0, 1},
   {PI_CMD_NOP  , "NOP"  , 101, 0, 1},
   {PI_CMD_OR   , "OR"   , 111, 0, 1},
   {PI_CMD_POP  , "POP"  , 113, 0, 1},
   {PI_CMD_POPA , "POPA" , 101, 0, 1},
   {PI_CMD_PUSH , "PUSH" , 113, 0, 1},
   {PI_CMD_PUSHA, "PUSHA", 101, 0, 1},
   {PI_CMD_RET  , "RET"  , 101, 0, 1},
   {PI_CMD_RL   , "RL"   , 123, 0, 1},
   {PI_CMD_RLA  , "RLA"  , 111, 0, 1},
   {PI_CMD_RR   , "RR"   , 123, 0, 1},
   {PI_CMD_RRA  , "RRA"  , 111, 0, 1},
   {PI_CMD_STA  , "STA"  , 113, 0, 1},
   {PI_CMD_STAB , "STAB" , 111, 0, 1},
   {PI_CMD_SUB  , "SUB"  , 111, 0, 1},
   {PI_CMD_SYS  , "SYS"  , 116, 0, 1},
   {PI_CMD_TAG  , "TAG"  , 114, 0, 1},
   {PI_CMD_WAIT , "WAIT" , 111, 0, 1},
   {PI_CMD_X    , "X"    , 124, 0, 1},
   {PI_CMD_XA   , "XA"   , 113, 0, 1},
   {PI_CMD_XOR  , "XOR"  , 111, 0, 1},

};


char * cmdUsage = "\n\
BC1 bits         Clear GPIO in bank 1\n\
BC2 bits         Clear GPIO in bank 2\n\
BI2CC sda        Close bit bang I2C\n\
BI2CO sda scl baud | Open bit bang I2C\n\
BI2CZ sda ...    I2C bit bang multiple transactions\n\
\n\
BSPIC cs        Close bit bang SPI\n\
BSPIO cs miso mosi sclk baud flag | Open bit bang SPI\n\
BSPIX cs ...    SPI bit bang transfer\n\
\n\
BR1              Read bank 1 GPIO\n\
BR2              Read bank 2 GPIO\n\
\n\
BS1 bits         Set GPIO in bank 1\n\
BS2 bits         Set GPIO in bank 2\n\
\n\
BSCX bctl bvs    BSC I2C/SPI transfer\n\
\n\
CF1 ...          Custom function 1\n\
CF2 ...          Custom function 2\n\
\n\
CGI              Configuration get internals\n\
CSI v            Configuration set internals\n\
\n\
EVM h bits       Set events to monitor\n\
EVT n            Trigger event\n\
\n\
FC h             Close file handle\n\
FG g steady      Set glitch filter on GPIO\n\
FL pat n         List files which match pattern\n\
FN g steady active | Set noise filter on GPIO\n\
FO file mode     Open a file in mode\n\
FR h n           Read bytes from file handle\n\
FS h n from      Seek to file handle position\n\
FW h ...         Write bytes to file handle\n\
\n\
GDC g            Get PWM dutycycle for GPIO\n\
GPW g            Get servo pulsewidth for GPIO\n\
\n\
H/HELP           Display command help\n\
HC g f           Set hardware clock frequency\n\
HP g f dc        Set hardware PWM frequency and dutycycle\n\
HWVER            Get hardware version\n\
\n\
I2CC h           Close I2C handle\n\
I2CO bus device flags | Open I2C bus and device with flags\n\
I2CPC h r word   SMBus Process Call: exchange register with word\n\
I2CPK h r ...    SMBus Block Process Call: exchange data bytes with register\n\
I2CRB h r        SMBus Read Byte Data: read byte from register\n\
I2CRD h n        I2C Read bytes\n\
I2CRI h r n      SMBus Read I2C Block Data: read bytes from register\n\
I2CRK h r        SMBus Read Block Data: read data from register\n\
I2CRS h          SMBus Read Byte: read byte\n\
I2CRW h r        SMBus Read Word Data: read word from register\n\
I2CWB h r byte   SMBus Write Byte Data: write byte to register\n\
I2CWD h ...      I2C Write data\n\
I2CWI h r ...    SMBus Write I2C Block Data\n\
I2CWK h r ...    SMBus Write Block Data: write data to register\n\
I2CWQ h b        SMBus Write Quick: write bit\n\
I2CWS h b        SMBus Write Byte: write byte\n\
I2CWW h r word   SMBus Write Word Data: write word to register\n\
I2CZ  h ...      I2C multiple transactions\n\
\n\
M/MODES g mode   Set GPIO mode\n\
MG/MODEG g       Get GPIO mode\n\
MICS n           Delay for microseconds\n\
MILS n           Delay for milliseconds\n\
\n\
NB h bits        Start notification\n\
NC h             Close notification\n\
NO               Request a notification\n\
NP h             Pause notification\n\
\n\
P/PWM g v        Set GPIO PWM value\n\
PADG pad         Get pad drive strength\n\
PADS pad v       Set pad drive strength\n\
PARSE text       Validate script\n\
PFG g            Get GPIO PWM frequency\n\
PFS g v          Set GPIO PWM frequency\n\
PIGPV            Get pigpio library version\n\
PRG g            Get GPIO PWM range\n\
PROC text        Store script\n\
PROCD sid        Delete script\n\
PROCP sid        Get script status and parameters\n\
PROCR sid ...    Run script\n\
PROCS sid        Stop script\n\
PROCU sid ...    Set script parameters\n\
PRRG g           Get GPIO PWM real range\n\
PRS g v          Set GPIO PWM range\n\
PUD g pud        Set GPIO pull up/down\n\
\n\
R/READ g         Read GPIO level\n\
\n\
S/SERVO g v      Set GPIO servo pulsewidth\n\
SERC h           Close serial handle\n\
SERDA h          Check for serial data ready to read\n\
SERO text baud flags | Open serial device at baud with flags\n\
SERR h n         Read bytes from serial handle\n\
SERRB h          Read byte from serial handle\n\
SERW h ...       Write bytes to serial handle\n\
SERWB h byte     Write byte to serial handle\n\
SHELL name str   Execute a shell command\n\
SLR g v          Read bit bang serial data from GPIO\n\
SLRC g           Close GPIO for bit bang serial data\n\
SLRO g baud bitlen | Open GPIO for bit bang serial data\n\
SLRI g invert    Invert serial logic (1 invert, 0 normal)\n\
SPIC h           SPI close handle\n\
SPIO channel baud flags | SPI open channel at baud with flags\n\
SPIR h v         SPI read bytes from handle\n\
SPIW h ...       SPI write bytes to handle\n\
SPIX h ...       SPI transfer bytes to handle\n\
\n\
T/TICK           Get current tick\n\
TRIG g micros l  Trigger level for micros on GPIO\n\
\n\
W/WRITE g l      Write level to GPIO\n\
WDOG g millis    Set millisecond watchdog on GPIO\n\
WVAG triplets    Wave add generic pulses\n\
WVAS g baud bitlen stopbits offset ... | Wave add serial data\n\
WVBSY            Check if wave busy\n\
WVCHA            Transmit a chain of waves\n\
WVCLR            Wave clear\n\
WVCRE            Create wave from added pulses\n\
WVDEL wid        Delete waves w and higher\n\
WVGO             Wave transmit (DEPRECATED)\n\
WVGOR            Wave transmit repeatedly (DEPRECATED)\n\
WVHLT            Wave stop\n\
WVNEW            Start a new empty wave\n\
WVSC 0,1,2       Wave get DMA control block stats\n\
WVSM 0,1,2       Wave get micros stats\n\
WVSP 0,1,2       Wave get pulses stats\n\
WVTAT            Returns the current transmitting wave\n\
WVTX wid         Transmit wave as one-shot\n\
WVTXM wid wmde   Transmit wave using mode\n\
WVTXR wid        Transmit wave repeatedly\n\
\n\
Numbers may be entered as hex (prefix 0x), octal (prefix 0),\n\
otherwise they are assumed to be decimal.\n\
\n\
Examples\n\
\n\
pigs w 4 1         # set GPIO 4 high\n\
pigs r 5           # read GPIO 5\n\
pigs t             # get current tick\n\
pigs i2co 1 0x20 0 # get handle to device 0x20 on I2C bus 1\n\
\n\
man pigs for full details.\n\
\n";

typedef struct
{
   int error;
   char * str;
} errInfo_t;

static errInfo_t errInfo[]=
{
   {PI_INIT_FAILED      , "pigpio initialisation failed"},
   {PI_BAD_USER_GPIO    , "GPIO not 0-31"},
   {PI_BAD_GPIO         , "GPIO not 0-53"},
   {PI_BAD_MODE         , "mode not 0-7"},
   {PI_BAD_LEVEL        , "level not 0-1"},
   {PI_BAD_PUD          , "pud not 0-2"},
   {PI_BAD_PULSEWIDTH   , "pulsewidth not 0 or 500-2500"},
   {PI_BAD_DUTYCYCLE    , "dutycycle not 0-range (default 255)"},
   {PI_BAD_TIMER        , "timer not 0-9"},
   {PI_BAD_MS           , "ms not 10-60000"},
   {PI_BAD_TIMETYPE     , "timetype not 0-1"},
   {PI_BAD_SECONDS      , "seconds < 0"},
   {PI_BAD_MICROS       , "micros not 0-999999"},
   {PI_TIMER_FAILED     , "gpioSetTimerFunc failed"},
   {PI_BAD_WDOG_TIMEOUT , "timeout not 0-60000"},
   {PI_NO_ALERT_FUNC    , "DEPRECATED"},
   {PI_BAD_CLK_PERIPH   , "clock peripheral not 0-1"},
   {PI_BAD_CLK_SOURCE   , "DEPRECATED"},
   {PI_BAD_CLK_MICROS   , "clock micros not 1, 2, 4, 5, 8, or 10"},
   {PI_BAD_BUF_MILLIS   , "buf millis not 100-10000"},
   {PI_BAD_DUTYRANGE    , "dutycycle range not 25-40000"},
   {PI_BAD_SIGNUM       , "signum not 0-63"},
   {PI_BAD_PATHNAME     , "can't open pathname"},
   {PI_NO_HANDLE        , "no handle available"},
   {PI_BAD_HANDLE       , "unknown handle"},
   {PI_BAD_IF_FLAGS     , "ifFlags > 4"},
   {PI_BAD_CHANNEL      , "DMA channel not 0-14"},
   {PI_BAD_SOCKET_PORT  , "socket port not 1024-30000"},
   {PI_BAD_FIFO_COMMAND , "unknown fifo command"},
   {PI_BAD_SECO_CHANNEL , "DMA secondary channel not 0-14"},
   {PI_NOT_INITIALISED  , "function called before gpioInitialise"},
   {PI_INITIALISED      , "function called after gpioInitialise"},
   {PI_BAD_WAVE_MODE    , "waveform mode not 0-1"},
   {PI_BAD_CFG_INTERNAL , "bad parameter in gpioCfgInternals call"},
   {PI_BAD_WAVE_BAUD    , "baud rate not 50-250K(RX)/50-1M(TX)"},
   {PI_TOO_MANY_PULSES  , "waveform has too many pulses"},
   {PI_TOO_MANY_CHARS   , "waveform has too many chars"},
   {PI_NOT_SERIAL_GPIO  , "no bit bang serial read in progress on GPIO"},
   {PI_BAD_SERIAL_STRUC , "bad (null) serial structure parameter"},
   {PI_BAD_SERIAL_BUF   , "bad (null) serial buf parameter"}, 
   {PI_NOT_PERMITTED    , "no permission to update GPIO"},
   {PI_SOME_PERMITTED   , "no permission to update one or more GPIO"},
   {PI_BAD_WVSC_COMMND  , "bad WVSC subcommand"},
   {PI_BAD_WVSM_COMMND  , "bad WVSM subcommand"},
   {PI_BAD_WVSP_COMMND  , "bad WVSP subcommand"},
   {PI_BAD_PULSELEN     , "trigger pulse length not 1-100"},
   {PI_BAD_SCRIPT       , "invalid script"},
   {PI_BAD_SCRIPT_ID    , "unknown script id"},
   {PI_BAD_SER_OFFSET   , "add serial data offset > 30 minute"},
   {PI_GPIO_IN_USE      , "GPIO already in use"},
   {PI_BAD_SERIAL_COUNT , "must read at least a byte at a time"},
   {PI_BAD_PARAM_NUM    , "script parameter id not 0-9"},
   {PI_DUP_TAG          , "script has duplicate tag"},
   {PI_TOO_MANY_TAGS    , "script has too many tags"},
   {PI_BAD_SCRIPT_CMD   , "illegal script command"},
   {PI_BAD_VAR_NUM      , "script variable id not 0-149"},
   {PI_NO_SCRIPT_ROOM   , "no more room for scripts"},
   {PI_NO_MEMORY        , "can't allocate temporary memory"},
   {PI_SOCK_READ_FAILED , "socket read failed"},
   {PI_SOCK_WRIT_FAILED , "socket write failed"},
   {PI_TOO_MANY_PARAM   , "too many script parameters (> 10)"},
   {PI_SCRIPT_NOT_READY , "script initialising"},
   {PI_BAD_TAG          , "script has unresolved tag"},
   {PI_BAD_MICS_DELAY   , "bad MICS delay (too large)"},
   {PI_BAD_MILS_DELAY   , "bad MILS delay (too large)"},
   {PI_BAD_WAVE_ID      , "non existent wave id"},
   {PI_TOO_MANY_CBS     , "No more CBs for waveform"},
   {PI_TOO_MANY_OOL     , "No more OOL for waveform"},
   {PI_EMPTY_WAVEFORM   , "attempt to create an empty waveform"},
   {PI_NO_WAVEFORM_ID   , "no more waveform ids"},
   {PI_I2C_OPEN_FAILED  , "can't open I2C device"},
   {PI_SER_OPEN_FAILED  , "can't open serial device"},
   {PI_SPI_OPEN_FAILED  , "can't open SPI device"},
   {PI_BAD_I2C_BUS      , "bad I2C bus"},
   {PI_BAD_I2C_ADDR     , "bad I2C address"},
   {PI_BAD_SPI_CHANNEL  , "bad SPI channel"},
   {PI_BAD_FLAGS        , "bad i2c/spi/ser open flags"},
   {PI_BAD_SPI_SPEED    , "bad SPI speed"},
   {PI_BAD_SER_DEVICE   , "bad serial device name"},
   {PI_BAD_SER_SPEED    , "bad serial baud rate"},
   {PI_BAD_PARAM        , "bad i2c/spi/ser parameter"},
   {PI_I2C_WRITE_FAILED , "I2C write failed"},
   {PI_I2C_READ_FAILED  , "I2C read failed"},
   {PI_BAD_SPI_COUNT    , "bad SPI count"},
   {PI_SER_WRITE_FAILED , "ser write failed"},
   {PI_SER_READ_FAILED  , "ser read failed"},
   {PI_SER_READ_NO_DATA , "ser read no data available"},
   {PI_UNKNOWN_COMMAND  , "unknown command"},
   {PI_SPI_XFER_FAILED  , "spi xfer/read/write failed"},
   {PI_BAD_POINTER      , "bad (NULL) pointer"},
   {PI_NO_AUX_SPI       , "no auxiliary SPI on Pi A or B"},
   {PI_NOT_PWM_GPIO     , "GPIO is not in use for PWM"},
   {PI_NOT_SERVO_GPIO   , "GPIO is not in use for servo pulses"},
   {PI_NOT_HCLK_GPIO    , "GPIO has no hardware clock"},
   {PI_NOT_HPWM_GPIO    , "GPIO has no hardware PWM"},
   {PI_BAD_HPWM_FREQ    , "invalid hardware PWM frequency"},
   {PI_BAD_HPWM_DUTY    , "hardware PWM dutycycle not 0-1M"},
   {PI_BAD_HCLK_FREQ    , "invalid hardware clock frequency"},
   {PI_BAD_HCLK_PASS    , "need password to use hardware clock 1"},
   {PI_HPWM_ILLEGAL     , "illegal, PWM in use for main clock"},
   {PI_BAD_DATABITS     , "serial data bits not 1-32"},
   {PI_BAD_STOPBITS     , "serial (half) stop bits not 2-8"},
   {PI_MSG_TOOBIG       , "socket/pipe message too big"},
   {PI_BAD_MALLOC_MODE  , "bad memory allocation mode"},
   {PI_TOO_MANY_SEGS    , "too many I2C transaction segments"},
   {PI_BAD_I2C_SEG      , "an I2C transaction segment failed"},
   {PI_BAD_SMBUS_CMD    , "SMBus command not supported by driver"},
   {PI_NOT_I2C_GPIO     , "no bit bang I2C in progress on GPIO"},
   {PI_BAD_I2C_WLEN     , "bad I2C write length"},
   {PI_BAD_I2C_RLEN     , "bad I2C read length"},
   {PI_BAD_I2C_CMD      , "bad I2C command"},
   {PI_BAD_I2C_BAUD     , "bad I2C baud rate, not 50-500k"},
   {PI_CHAIN_LOOP_CNT   , "bad chain loop count"},
   {PI_BAD_CHAIN_LOOP   , "empty chain loop"},
   {PI_CHAIN_COUNTER    , "too many chain counters"},
   {PI_BAD_CHAIN_CMD    , "bad chain command"},
   {PI_BAD_CHAIN_DELAY  , "bad chain delay micros"},
   {PI_CHAIN_NESTING    , "chain counters nested too deeply"},
   {PI_CHAIN_TOO_BIG    , "chain is too long"},
   {PI_DEPRECATED       , "deprecated function removed"},
   {PI_BAD_SER_INVERT   , "bit bang serial invert not 0 or 1"},
   {PI_BAD_EDGE         , "bad ISR edge, not 1, 1, or 2"},
   {PI_BAD_ISR_INIT     , "bad ISR initialisation"},
   {PI_BAD_FOREVER      , "loop forever must be last chain command"},
   {PI_BAD_FILTER       , "bad filter parameter"},
   {PI_BAD_PAD          , "bad pad number"},
   {PI_BAD_STRENGTH     , "bad pad drive strength"},
   {PI_FIL_OPEN_FAILED  , "file open failed"},
   {PI_BAD_FILE_MODE    , "bad file mode"},
   {PI_BAD_FILE_FLAG    , "bad file flag"},
   {PI_BAD_FILE_READ    , "bad file read"},
   {PI_BAD_FILE_WRITE   , "bad file write"},
   {PI_FILE_NOT_ROPEN   , "file not open for read"},
   {PI_FILE_NOT_WOPEN   , "file not open for write"},
   {PI_BAD_FILE_SEEK    , "bad file seek"},
   {PI_NO_FILE_MATCH    , "no files match pattern"},
   {PI_NO_FILE_ACCESS   , "no permission to access file"},
   {PI_FILE_IS_A_DIR    , "file is a directory"},
   {PI_BAD_SHELL_STATUS , "bad shell return status"},
   {PI_BAD_SCRIPT_NAME  , "bad script name"},
   {PI_BAD_SPI_BAUD     , "bad SPI baud rate, not 50-500k"},
   {PI_NOT_SPI_GPIO     , "no bit bang SPI in progress on GPIO"},
   {PI_BAD_EVENT_ID     , "bad event id"},
   {PI_CMD_INTERRUPTED  , "command interrupted, Python"},
   {PI_NOT_ON_BCM2711   , "not available on BCM2711"},
   {PI_ONLY_ON_BCM2711  , "only available on BCM2711"},

};

static char * fmtMdeStr="RW540123";
static char * fmtPudStr="ODU";

static int cmdMatch(char *str)
{
   int i;

   for (i=0; i<(sizeof(cmdInfo)/sizeof(cmdInfo_t)); i++)
   {
      if (strcasecmp(str, cmdInfo[i].name) == 0) return i;
   }
   return CMD_UNKNOWN_CMD;
}

static int getNum(char *str, uintptr_t *val, int8_t *opt)
{
   int f, n;
   intmax_t v;

   *opt = 0;

   f = sscanf(str, " %ji %n", &v, &n);

   if (f == 1)
   {
      *val = v;
      *opt = CMD_NUMERIC;
      return n;
   }

   f = sscanf(str, " v%ji %n", &v, &n);

   if (f == 1)
   {
      *val = v;
      if (v < PI_MAX_SCRIPT_VARS) *opt = CMD_VAR;
      else *opt = -CMD_VAR;
      return n;
   }

   f = sscanf(str, " p%ji %n", &v, &n);

   if (f == 1)
   {
      *val = v;
      if (v < PI_MAX_SCRIPT_PARAMS) *opt = CMD_PAR;
      else *opt = -CMD_PAR;
      return n;
   }

   return 0;
}

static char intCmdStr[32];
static int intCmdIdx;

char *cmdStr(void)
{
   return intCmdStr;
}

int cmdParse(
   char *buf, uintptr_t *p, unsigned ext_len, char *ext, cmdCtlParse_t *ctl)
{
   int f, valid, idx, val, pp, pars, n, n2;
   char *p8;
   int32_t *p32;
   char c;
   uintptr_t tp1=0, tp2=0, tp3=0, tp4=0, tp5=0;
   int8_t to1, to2, to3, to4, to5;
   int eaten;

   /* Check that ext is big enough for the largest message. */
   if (ext_len < (4 * CMD_MAX_PARAM)) return CMD_EXT_TOO_SMALL;

   bzero(&ctl->opt, sizeof(ctl->opt));

   sscanf(buf+ctl->eaten, " %31s %n", intCmdStr, &pp);

   ctl->eaten += pp;

   p[0] = -1;

   idx = cmdMatch(intCmdStr);

   intCmdIdx = idx;

   if (idx < 0) return idx;

   valid = 0;

   p[0] = cmdInfo[idx].cmd;
   p[1] = 0;
   p[2] = 0;
   p[3] = 0;

   switch (cmdInfo[idx].vt)
   {
      case 101: /* BR1  BR2  CGI  H  HELP  HWVER
                   DCRA  HALT  INRA  NO
                   PIGPV  POPA  PUSHA  RET  T  TICK  WVBSY  WVCLR
                   WVCRE  WVGO  WVGOR  WVHLT  WVNEW

                   No parameters, always valid.
                */
         valid = 1;

         break;

      case 111: /* ADD  AND  BC1  BC2  BS1  BS2  
                   CMP  CSI  DIV  LDA  LDAB  MLT
                   MOD  OR  RLA  RRA  STAB  SUB  WAIT  XOR

                   One parameter, any value.
                */
         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);

         if (ctl->opt[1] > 0) valid = 1;

         break;

      case 112: /* BI2CC FC  GDC  GPW  I2CC  I2CRB
                   MG  MICS  MILS  MODEG  NC  NP  PADG PFG  PRG
                   PROCD  PROCP  PROCS  PRRG  R  READ  SLRC  SPIC
                   WVCAP WVDEL  WVSC  WVSM  WVSP  WVTX  WVTXR  BSPIC

                   One positive parameter.
                */
         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);

         if ((ctl->opt[1] > 0) && ((int)p[1] >= 0)) valid = 1;

         break;

      case 113: /* DCR  INR  POP  PUSH  STA  XA

                   One register parameter.
                */
         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);

         if ((ctl->opt[1] > 0) && (p[1] < PI_MAX_SCRIPT_VARS)) valid = 1;

         break;

      case 114: /* CALL  JM  JMP  JNZ  JP  JZ  TAG

                   One numeric parameter, any value.
                */
         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);
         if (ctl->opt[1] == CMD_NUMERIC) valid = 1;

         break;

      case 115: /* PARSE  PROC

                   One parameter, string (rest of input).
                */
         p[3] = strlen(buf+ctl->eaten);
         memcpy(ext, buf+ctl->eaten, p[3]);
         ctl->eaten += p[3];
         valid = 1;

         break;

      case 116: /* SYS

                   One parameter, a string.
                */
         f = sscanf(buf+ctl->eaten, " %*s%n %n", &n, &n2);
         if ((f >= 0) && n)
         {
            p[3] = n;
            ctl->opt[3] = CMD_NUMERIC;
            memcpy(ext, buf+ctl->eaten, n);
            ctl->eaten += n2;
            valid = 1;
         }

         break;

      case 121: /* HC  FR  I2CRD  I2CRR  I2CRW  I2CWB I2CWQ  P
                   PADS  PFS  PRS  PWM  S  SERVO  SLR  SLRI  W
                   WDOG  WRITE  WVTXM

                   Two positive parameters.
                */
         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);
         ctl->eaten += getNum(buf+ctl->eaten, &p[2], &ctl->opt[2]);

         if ((ctl->opt[1] > 0) && ((int)p[1] >= 0) &&
             (ctl->opt[2] > 0) && ((int)p[2] >= 0)) valid = 1;

         break;

      case 122: /* NB

                   Two parameters, first positive, second any value.
                */
         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);
         ctl->eaten += getNum(buf+ctl->eaten, &p[2], &ctl->opt[2]);

         if ((ctl->opt[1] > 0) && ((int)p[1] >= 0) &&
             (ctl->opt[2] > 0)) valid = 1;

         break;

      case 123: /* LD  RL  RR

                   Two parameters, first register, second any value.
                */
         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);
         ctl->eaten += getNum(buf+ctl->eaten, &p[2], &ctl->opt[2]);

         if ((ctl->opt[1] > 0) &&
             (p[1] < PI_MAX_SCRIPT_VARS) &&
             (ctl->opt[2] > 0)) valid = 1;

         break;

      case 124: /* X

                   Two register parameters.
                */
         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);
         ctl->eaten += getNum(buf+ctl->eaten, &p[2], &ctl->opt[2]);

         if ((ctl->opt[1] > 0) && (p[1] < PI_MAX_SCRIPT_VARS) &&
             (ctl->opt[2] > 0) && (p[2] < PI_MAX_SCRIPT_VARS)) valid = 1;

         break;

      case 125: /* M MODES

                   Two parameters, first positive, second in 'RW540123'.
                */
         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);

         f = sscanf(buf+ctl->eaten, " %c %n", &c, &n);

         if ((ctl->opt[1] > 0) && ((int)p[1] >= 0) && (f >= 1))
         {
            ctl->eaten += n;
            val = toupper(c);
            p8 = strchr(fmtMdeStr, val);

            if (p8 != NULL)
            {
               val = p8 - fmtMdeStr;
               p[2] = val;
               valid = 1;
            }
         }

         break;

      case 126: /* PUD

                   Two parameters, first positive, second in 'ODU'.
                */
         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);

         f = sscanf(buf+ctl->eaten, " %c %n", &c, &n);

         if ((ctl->opt[1] > 0) && ((int)p[1] >= 0)  && (f >= 1))
         {
            ctl->eaten += n;
            val = toupper(c);
            p8 = strchr(fmtPudStr, val);
            if (p8 != NULL)
            {
               val = p8 - fmtPudStr;
               p[2] = val;
               valid = 1;
            }
         }

         break;

      case 127: /* FL  FO

                   Two parameters, first a string, other positive.
                */
         f = sscanf(buf+ctl->eaten, " %*s%n %n", &n, &n2);
         if ((f >= 0) && n)
         {
            p[3] = n;
            ctl->opt[2] = CMD_NUMERIC;
            memcpy(ext, buf+ctl->eaten, n);
            ctl->eaten += n2;

            ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);

            if ((ctl->opt[1] > 0) && ((int)p[1] >= 0))
               valid = 1;
         }

         break;

      case 128: /* SHELL

                   Two string parameters, the first space teminated.
                   The second arbitrary.
                */
         f = sscanf(buf+ctl->eaten, " %*s%n %n", &n, &n2);

         if ((f >= 0) && n)
         {
            valid = 1;

            p[1] = n;
            memcpy(ext, buf+ctl->eaten, n);
            ctl->eaten += n;
            ext[n] = 0; /* terminate first string */

            n2 = strlen(buf+ctl->eaten+1);
            memcpy(ext+n+1, buf+ctl->eaten+1, n2);
            ctl->eaten += n2;
            ctl->eaten ++;
            p[3] = p[1] + n2 + 1;
         }

         break;

      case 131: /* BI2CO  HP  I2CO  I2CPC  I2CRI  I2CWB  I2CWW
                   SLRO  SPIO  TRIG

                   Three positive parameters.
                */
         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);
         ctl->eaten += getNum(buf+ctl->eaten, &p[2], &ctl->opt[2]);
         ctl->eaten += getNum(buf+ctl->eaten, &tp1, &ctl->opt[3]);

         if ((ctl->opt[1] > 0) && ((int)p[1] >= 0) &&
             (ctl->opt[2] > 0) && ((int)p[2] >= 0) &&
             (ctl->opt[3] > 0) && ((int)tp1 >= 0))
         {
            p[3] = 4;
            memcpy(ext, &tp1, 4);
            valid = 1;
         }

         break;

      case 132: /* SERO

                   Three parameters, first a string, rest >=0
                */
         f = sscanf(buf+ctl->eaten, " %*s%n %n", &n, &n2);
         if ((f >= 0) && n)
         {
            p[3] = n;
            ctl->opt[2] = CMD_NUMERIC;
            memcpy(ext, buf+ctl->eaten, n);
            ctl->eaten += n2;

            ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);
            ctl->eaten += getNum(buf+ctl->eaten, &p[2], &ctl->opt[2]);

            if ((ctl->opt[1] > 0) && ((int)p[1] >= 0) &&
                (ctl->opt[2] > 0) && ((int)p[2] >= 0))
               valid = 1;
         }

         break;

      case 133: /* FS

                   Three parameters.  First and third positive.
                   Second may be negative when interpreted as an int.
                */
         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);
         ctl->eaten += getNum(buf+ctl->eaten, &p[2], &ctl->opt[2]);
         ctl->eaten += getNum(buf+ctl->eaten, &tp1, &to1);

         if ((ctl->opt[1] > 0) && ((int)p[1] >= 0) &&
             (ctl->opt[2] > 0) &&
             (to1 == CMD_NUMERIC) && ((int)tp1 >= 0))
         {
            p[3] = 4;
            memcpy(ext, &tp1, 4);
            valid = 1;
         }

         break;

      case 134: /* BSPIO

                   Six parameters.  First to Fifth positive.
                   Sixth may be negative when interpreted as an int.
                */
         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);
         ctl->eaten += getNum(buf+ctl->eaten, &tp1, &to1);
         ctl->eaten += getNum(buf+ctl->eaten, &tp2, &to2);
         ctl->eaten += getNum(buf+ctl->eaten, &tp3, &to3);
         ctl->eaten += getNum(buf+ctl->eaten, &tp4, &to4);
         ctl->eaten += getNum(buf+ctl->eaten, &tp5, &to5);
                                    
         if ((ctl->opt[1] > 0) && ((int)p[1] >= 0) &&
             (to1 == CMD_NUMERIC) && ((int)tp1 >= 0) &&
             (to2 == CMD_NUMERIC) && ((int)tp2 >= 0) &&
             (to3 == CMD_NUMERIC) && ((int)tp3 >= 0) &&
             (to4 == CMD_NUMERIC) && ((int)tp4 >= 0) &&
             (to5 == CMD_NUMERIC))
         {
            p[3] = 5 * 4;
            memcpy(ext+ 0, &tp1, 4);
            memcpy(ext+ 4, &tp2, 4);
            memcpy(ext+ 8, &tp3, 4);
            memcpy(ext+12, &tp4, 4);
            memcpy(ext+16, &tp5, 4);
            valid = 1;
         }

         break;

      case 191: /* PROCR PROCU

                   One to 11 parameters, first positive,
                   optional remainder, any value.
                */
         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);

         if ((ctl->opt[1] == CMD_NUMERIC) && ((int)p[1] >= 0))
         {
            pars = 0;
            p32 = (int32_t *)ext;

            while (pars < PI_MAX_SCRIPT_PARAMS)
            {
               ctl->eaten += getNum(buf+ctl->eaten, &tp1, &to1);
               if (to1 == CMD_NUMERIC)
               {
                  pars++;
                  *p32++ = tp1;
               }
               else break;
            }

            p[3] = pars * 4;

            valid = 1;
         }

         break;

      case 192: /* WVAG

                   One or more triplets (gpios on, gpios off, delay),
                   any value.
                */

         pars = 0;
         p32 = (int32_t *)ext;

         while (pars < CMD_MAX_PARAM)
         {
            ctl->eaten += getNum(buf+ctl->eaten, &tp1, &to1);
            if (to1 == CMD_NUMERIC)
            {
               pars++;
               *p32++ = tp1;
            }
            else break;
         }

         p[3] = pars * 4;

         if (pars && ((pars % 3) == 0)) valid = 1;

         break;

      case 193: /* BI2CZ  BSCX  BSPIX  FW  I2CWD  I2CZ  SERW
		   SPIW  SPIX

                   Two or more parameters, first >=0, rest 0-255.

		   BSCX is special case one or more.
                */
         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);

         if ((ctl->opt[1] == CMD_NUMERIC) && ((int)p[1] >= 0))
         {
            pars = 0;
            p8 = ext;

            while (pars < CMD_MAX_PARAM)
            {
               eaten = getNum(buf+ctl->eaten, &tp1, &to1);
               if (to1 == CMD_NUMERIC)
               {
                  if (((int)tp1>=0) && ((int)tp1<=255))
                  {
                     pars++;
                     *p8++ = tp1;
                     ctl->eaten += eaten;
                  }
                  else break; /* invalid number, end of command */
               }
               else break;
            }

            p[3] = pars;

            if (pars || (p[0]==PI_CMD_BSCX)) valid = 1;
         }

         break;

      case 194: /* I2CPK  I2CWI  I2CWK

                   Three to 34 parameters, all 0-255.
                */

         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);
         ctl->eaten += getNum(buf+ctl->eaten, &p[2], &ctl->opt[2]);

         if ((ctl->opt[1] == CMD_NUMERIC) &&
             (ctl->opt[2] == CMD_NUMERIC) &&
             ((int)p[1]>=0) && ((int)p[2]>=0) && ((int)p[2]<=255))
         {
            pars = 0;
            p8 = ext;

            while (pars < 32)
            {
               eaten = getNum(buf+ctl->eaten, &tp1, &to1);
               if (to1 == CMD_NUMERIC)
               {
                  if (((int)tp1>=0) && ((int)tp1<=255))
                  {
                     pars++;
                     *p8++ = tp1;
                     ctl->eaten += eaten;
                  }
                  else break; /* invalid number, end of command */
               }
               else break;
            }

            p[3] = pars;

            if (pars > 0) valid = 1;
         }

         break;

      case 195: /* CF1  CF2

                   Zero or more parameters, first two >=0, rest 0-255.
                */
         valid = 1;

         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);

         if (ctl->opt[1] == CMD_NUMERIC)
         {
            if ((int)p[1] >= 0)
            {
               ctl->eaten += getNum(buf+ctl->eaten, &p[2], &ctl->opt[2]);

               if (ctl->opt[2] == CMD_NUMERIC)
               {
                  if ((int)p[2] >= 0)
                  {
                     pars = 0;
                     p8 = ext;

                     while (pars < CMD_MAX_PARAM)
                     {
                        eaten = getNum(buf+ctl->eaten, &tp1, &to1);
                        if (to1 == CMD_NUMERIC)
                        {
                           if (((int)tp1>=0) && ((int)tp1<=255))
                           {
                              pars++;
                              *p8++ = tp1;
                              ctl->eaten += eaten;
                           }
                           else break;
                        }
                        else break;
                     }

                     p[3] = pars;
                  }
                  else valid = 0;
               }
            }
            else valid = 0;
         }

         break;

      case 196: /* WVAS

                   gpio baud offset char...

                   p1 gpio
                   p2 baud
                   p3 len + 4
                   ---------
                   uint32_t databits
                   uint32_t stophalfbits
                   uint32_t offset
                   uint8_t[len]
                */
         ctl->eaten += getNum(buf+ctl->eaten, &p[1], &ctl->opt[1]);
         ctl->eaten += getNum(buf+ctl->eaten, &p[2], &ctl->opt[2]);
         ctl->eaten += getNum(buf+ctl->eaten, &tp1, &to1);
         ctl->eaten += getNum(buf+ctl->eaten, &tp2, &to2);
         ctl->eaten += getNum(buf+ctl->eaten, &tp3, &to3);

         if ((ctl->opt[1] == CMD_NUMERIC) && ((int)p[1] >= 0) &&
             (ctl->opt[2] == CMD_NUMERIC) && ((int)p[2] > 0) &&
             (to1 == CMD_NUMERIC) &&
             (to2 == CMD_NUMERIC) &&
             (to3 == CMD_NUMERIC))
         {
            pars = 0;

            memcpy(ext, &tp1, 4);
            memcpy(ext+4, &tp2, 4);
            memcpy(ext+8, &tp3, 4);
            p8 = ext + 12;
            while (pars < CMD_MAX_PARAM)
            {
               eaten = getNum(buf+ctl->eaten, &tp1, &to1);
               if (to1 == CMD_NUMERIC)
               {
                  if (((int)tp1>=0) && ((int)tp1<=255))
                  {
                     *p8++ = tp1;
                     pars++;
                     ctl->eaten += eaten;
                  }
                  else break; /* invalid number, end of command */
               }
               else break;
            }

            p[3] = pars + 12;

            if (pars > 0) valid = 1;
         }

         break;

      case 197: /* WVCHA

                   One or more parameters, all 0-255.
                */
         pars = 0;
         p8 = ext;

         while (pars < CMD_MAX_PARAM)
         {
            eaten = getNum(buf+ctl->eaten, &tp1, &to1);
            if (to1 == CMD_NUMERIC)
            {
               if (((int)tp1>=0) && ((int)tp1<=255))
               {
                  pars++;
                  *p8++ = tp1;
                  ctl->eaten += eaten;
               }
               else break; /* invalid number, end of command */
            }
            else break;
         }

         p[3] = pars;

         if (pars) valid = 1;

         break;


   }

   if (valid) return idx; else return CMD_BAD_PARAMETER;
}

char * cmdErrStr(int error)
{
   int i;

   for (i=0; i<(sizeof(errInfo)/sizeof(errInfo_t)); i++)
   {
      if (errInfo[i].error == error) return errInfo[i].str;
   }
   return "unknown error";
}

int cmdParseScript(char *script, cmdScript_t *s, int diags)
{
   int idx, len, b, i, j, tags, resolved;
   int status;
   uintptr_t p[10];
   cmdInstr_t instr;
   cmdCtlParse_t ctl;
   char v[CMD_MAX_EXTENSION];

   ctl.eaten = 0;

   status = 0;

   cmdTagStep_t tag_step[PI_MAX_SCRIPT_TAGS];

   len = strlen(script);

   /* calloc space for PARAMS, VARS, CMDS, and STRINGS */

   b = (sizeof(int) * (PI_MAX_SCRIPT_PARAMS + PI_MAX_SCRIPT_VARS)) +
       (sizeof(cmdInstr_t) * (len + 2) / 2) + len;

   s->par = calloc(1, b);

   if (s->par == NULL) return -1;

   s->var = s->par + PI_MAX_SCRIPT_PARAMS;

   s->instr = (cmdInstr_t *)(s->var + PI_MAX_SCRIPT_VARS);

   s->str_area = (char *)(s->instr + ((len + 2) / 2));

   s->str_area_len = len;
   s->str_area_pos = 0;

   s->instrs = 0;

   tags = 0;

   idx = 0;

   while (ctl.eaten<len)
   {
      idx = cmdParse(script, p, CMD_MAX_EXTENSION, v, &ctl);

      /* abort if command is illegal in a script */

      if ((idx >= 0) || (idx != CMD_UNKNOWN_CMD))
      {
         if (!cmdInfo[intCmdIdx].cvis) idx = CMD_NOT_IN_SCRIPT;
      }

      if (idx >= 0)
      {
         if (p[3])
         {
            memcpy(s->str_area + s->str_area_pos, v, p[3]);
            s->str_area[s->str_area_pos + p[3]] = 0;
            p[4] = (intptr_t) s->str_area + s->str_area_pos;
            s->str_area_pos += (p[3] + 1);
         }

         memcpy(&instr.p, p, sizeof(instr.p));

         if (instr.p[0] == PI_CMD_TAG)
         {
            if (tags < PI_MAX_SCRIPT_TAGS)
            {
               /* check tag not already used */
               for (j=0; j<tags; j++)
               {
                  if (tag_step[j].tag == instr.p[1])
                  {
                     if (diags)
                     {
                        fprintf(stderr, "Duplicate tag: %"PRIdPTR"\n", instr.p[1]);
                     }

                     if (!status) status = PI_DUP_TAG;
                     idx = -1;
                  }
               }

               tag_step[tags].tag = instr.p[1];
               tag_step[tags].step = s->instrs;
               tags++;
            }
            else
            {
               if (diags)
               {
                  fprintf(stderr, "Too many tags: %"PRIdPTR"\n", instr.p[1]);
               }
               if (!status) status = PI_TOO_MANY_TAGS;
               idx = -1;
            }
         }
      }
      else
      {
         if (diags)
         {
            if (idx == CMD_UNKNOWN_CMD)
               fprintf(stderr, "Unknown command: %s\n", cmdStr());
            else if (idx == CMD_NOT_IN_SCRIPT)
               fprintf(stderr, "Command illegal in script: %s\n", cmdStr());
            else
               fprintf(stderr, "Bad parameter to %s\n", cmdStr());
         }
         if (!status) status = PI_BAD_SCRIPT_CMD;
      }

      if (idx >= 0)
      {
         if (instr.p[0] != PI_CMD_TAG)
         {
            memcpy(instr.opt, &ctl.opt, sizeof(instr.opt));
            s->instr[s->instrs++] = instr;
         }
      }
   }

   for (i=0; i<s->instrs; i++)
   {
      instr = s->instr[i];

      /* resolve jumps */

      if ((instr.p[0] == PI_CMD_JMP) || (instr.p[0] == PI_CMD_CALL) ||
          (instr.p[0] == PI_CMD_JZ)  || (instr.p[0] == PI_CMD_JNZ)  ||
          (instr.p[0] == PI_CMD_JM)  || (instr.p[0] == PI_CMD_JP))
      {
         resolved = 0;

         for (j=0; j<tags; j++)
         {
            if (instr.p[1] == tag_step[j].tag)
            {
               s->instr[i].p[1] = tag_step[j].step;
               resolved = 1;
               break;
            }
         }

         if (!resolved)
         {
            if (diags)
            {
               fprintf(stderr, "Can't resolve tag %"PRIdPTR"\n", instr.p[1]);
            }
            if (!status) status = PI_BAD_TAG;
         }
      }
   }
   return status;
}

