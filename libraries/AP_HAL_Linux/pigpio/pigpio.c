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

/* pigpio version 79 */

/* include ------------------------------------------------------- */

#define _GNU_SOURCE

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdarg.h>
#include <ctype.h>
#include <syslog.h>
#include <poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <sys/ioctl.h>
#include <limits.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/sysmacros.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <fnmatch.h>
#include <glob.h>
#include <arpa/inet.h>

#include "pigpio.h"

#include "command.h"


/* --------------------------------------------------------------- */

/*
 0 GPFSEL0   GPIO Function Select 0
 1 GPFSEL1   GPIO Function Select 1
 2 GPFSEL2   GPIO Function Select 2
 3 GPFSEL3   GPIO Function Select 3
 4 GPFSEL4   GPIO Function Select 4
 5 GPFSEL5   GPIO Function Select 5
 6 -         Reserved
 7 GPSET0    GPIO Pin Output Set 0
 8 GPSET1    GPIO Pin Output Set 1
 9 -         Reserved
10 GPCLR0    GPIO Pin Output Clear 0
11 GPCLR1    GPIO Pin Output Clear 1
12 -         Reserved
13 GPLEV0    GPIO Pin Level 0
14 GPLEV1    GPIO Pin Level 1
15 -         Reserved
16 GPEDS0    GPIO Pin Event Detect Status 0
17 GPEDS1    GPIO Pin Event Detect Status 1
18 -         Reserved
19 GPREN0    GPIO Pin Rising Edge Detect Enable 0
20 GPREN1    GPIO Pin Rising Edge Detect Enable 1
21 -         Reserved
22 GPFEN0    GPIO Pin Falling Edge Detect Enable 0
23 GPFEN1    GPIO Pin Falling Edge Detect Enable 1
24 -         Reserved
25 GPHEN0    GPIO Pin High Detect Enable 0
26 GPHEN1    GPIO Pin High Detect Enable 1
27 -         Reserved
28 GPLEN0    GPIO Pin Low Detect Enable 0
29 GPLEN1    GPIO Pin Low Detect Enable 1
30 -         Reserved
31 GPAREN0   GPIO Pin Async. Rising Edge Detect 0
32 GPAREN1   GPIO Pin Async. Rising Edge Detect 1
33 -         Reserved
34 GPAFEN0   GPIO Pin Async. Falling Edge Detect 0
35 GPAFEN1   GPIO Pin Async. Falling Edge Detect 1
36 -         Reserved
37 GPPUD     GPIO Pin Pull-up/down Enable
38 GPPUDCLK0 GPIO Pin Pull-up/down Enable Clock 0
39 GPPUDCLK1 GPIO Pin Pull-up/down Enable Clock 1
40 -         Reserved
41 -         Test
42-56        Reserved
57 GPPUPPDN1 Pin pull-up/down for pins 15:0
58 GPPUPPDN1 Pin pull-up/down for pins 31:16
59 GPPUPPDN2 Pin pull-up/down for pins 47:32
60 GPPUPPDN3 Pin pull-up/down for pins 57:48
*/

/*
0 CS           DMA Channel 0 Control and Status
1 CPI_ONBLK_AD DMA Channel 0 Control Block Address
2 TI           DMA Channel 0 CB Word 0 (Transfer Information)
3 SOURCE_AD    DMA Channel 0 CB Word 1 (Source Address)
4 DEST_AD      DMA Channel 0 CB Word 2 (Destination Address)
5 TXFR_LEN     DMA Channel 0 CB Word 3 (Transfer Length)
6 STRIDE       DMA Channel 0 CB Word 4 (2D Stride)
7 NEXTCPI_ONBK DMA Channel 0 CB Word 5 (Next CB Address)
8 DEBUG        DMA Channel 0 Debug
*/

/*
DEBUG register bits

bit 2 READ_ERROR

   Slave Read Response Error RW 0x0

   Set if the read operation returned an error value on
   the read response bus. It can be cleared by writing
   a 1.

bit 1 FIFO_ERROR

   Fifo Error RW 0x0

   Set if the optional read Fifo records an error
   condition. It can be cleared by writing a 1.

bit 0 READ_LAST_NOT_SET_ERROR

   Read Last Not Set Error RW 0x0

   If the AXI read last signal was not set when
   expected, then this error bit will be set. It can be
   cleared by writing a 1.
*/

/*
0 CTL        PWM Control
1 STA        PWM Status
2 DMAC       PWM DMA Configuration
4 RNG1       PWM Channel 1 Range
5 DAT1       PWM Channel 1 Data
6 FIF1       PWM FIFO Input
8 RNG2       PWM Channel 2 Range
9 DAT2       PWM Channel 2 Data
*/

/*
0 PCM_CS     PCM Control and Status
1 PCM_FIFO   PCM FIFO Data
2 PCM_MODE   PCM Mode
3 PCM_RXC    PCM Receive Configuration
4 PCM_TXC    PCM Transmit Configuration
5 PCM_DREQ   PCM DMA Request Level
6 PCM_INTEN  PCM Interrupt Enables
7 PCM_INTSTC PCM Interrupt Status & Clear
8 PCM_GRAY   PCM Gray Mode Control
*/

/*
0 CS  System Timer Control/Status
1 CLO System Timer Counter Lower 32 bits
2 CHI System Timer Counter Higher 32 bits
3 C0  System Timer Compare 0
4 C1  System Timer Compare 1
5 C2  System Timer Compare 2
6 C3  System Timer Compare 3
*/

/* define -------------------------------------------------------- */

#define THOUSAND 1000
#define MILLION  1000000
#define BILLION  1000000000

#define BANK (gpio>>5)

#define BIT  (1<<(gpio&0x1F))

#ifndef EMBEDDED_IN_VM
#define DBG(level, format, arg...) DO_DBG(level, format, ## arg)
#else
#define DBG(level, format, arg...)
#endif

#define DO_DBG(level, format, arg...)                              \
   {                                                               \
      if ((gpioCfg.dbgLevel >= level) &&                           \
         (!(gpioCfg.internals & PI_CFG_NOSIGHANDLER)))             \
         fprintf(stderr, "%s %s: " format "\n" ,                   \
            myTimeStamp(), __FUNCTION__ , ## arg);                 \
   }

#ifndef DISABLE_SER_CHECK_INITED
#define SER_CHECK_INITED CHECK_INITED
#else
#define SER_CHECK_INITED
#endif

#define CHECK_INITED                                               \
   do                                                              \
   {                                                               \
      if (!libInitialised)                                         \
      {                                                            \
         DBG(DBG_ALWAYS,                                           \
           "pigpio uninitialised, call gpioInitialise()");         \
         return PI_NOT_INITIALISED;                                \
      }                                                            \
   }                                                               \
   while (0)

#define CHECK_INITED_RET_NULL_PTR                                  \
   do                                                              \
   {                                                               \
      if (!libInitialised)                                         \
      {                                                            \
         DBG(DBG_ALWAYS,                                           \
           "pigpio uninitialised, call gpioInitialise()");         \
         return (NULL);                                            \
      }                                                            \
   }                                                               \
   while (0)

#define CHECK_INITED_RET_NIL                                       \
   do                                                              \
   {                                                               \
      if (!libInitialised)                                         \
      {                                                            \
         DBG(DBG_ALWAYS,                                           \
           "pigpio uninitialised, call gpioInitialise()");         \
      }                                                            \
   }                                                               \
   while (0)

#define CHECK_NOT_INITED                                           \
   do                                                              \
   {                                                               \
      if (libInitialised)                                          \
      {                                                            \
         DBG(DBG_ALWAYS,                                           \
            "pigpio initialised, call gpioTerminate()");           \
         return PI_INITIALISED;                                    \
      }                                                            \
   }                                                               \
   while (0)

#define SOFT_ERROR(x, format, arg...)                              \
   do                                                              \
   {                                                               \
      DBG(DBG_ALWAYS, format, ## arg);                             \
      return x;                                                    \
   }                                                               \
   while (0)

#define TIMER_ADD(a, b, result)                                    \
   do                                                              \
   {                                                               \
      (result)->tv_sec =  (a)->tv_sec  + (b)->tv_sec;              \
      (result)->tv_nsec = (a)->tv_nsec + (b)->tv_nsec;             \
      if ((result)->tv_nsec >= BILLION)                            \
      {                                                            \
        ++(result)->tv_sec;                                        \
        (result)->tv_nsec -= BILLION;                              \
      }                                                            \
   }                                                               \
   while (0)

#define TIMER_SUB(a, b, result)                                    \
   do                                                              \
   {                                                               \
      (result)->tv_sec =  (a)->tv_sec  - (b)->tv_sec;              \
      (result)->tv_nsec = (a)->tv_nsec - (b)->tv_nsec;             \
      if ((result)->tv_nsec < 0)                                   \
      {                                                            \
         --(result)->tv_sec;                                       \
         (result)->tv_nsec += BILLION;                             \
      }                                                            \
   }                                                               \
   while (0)

#define PI_PERI_BUS 0x7E000000

#define AUX_BASE   (pi_peri_phys + 0x00215000)
#define BSCS_BASE  (pi_peri_phys + 0x00214000)
#define CLK_BASE   (pi_peri_phys + 0x00101000)
#define DMA_BASE   (pi_peri_phys + 0x00007000)
#define DMA15_BASE (pi_peri_phys + 0x00E05000)
#define GPIO_BASE  (pi_peri_phys + 0x00200000)
#define PADS_BASE  (pi_peri_phys + 0x00100000)
#define PCM_BASE   (pi_peri_phys + 0x00203000)
#define PWM_BASE   (pi_peri_phys + 0x0020C000)
#define SPI_BASE   (pi_peri_phys + 0x00204000)
#define SYST_BASE  (pi_peri_phys + 0x00003000)

#define AUX_LEN   0xD8
#define BSCS_LEN  0x40
#define CLK_LEN   0xA8
#define DMA_LEN   0x1000 /* allow access to all channels */
#define GPIO_LEN  0xF4   /* 2711 has more registers */
#define PADS_LEN  0x38
#define PCM_LEN   0x24
#define PWM_LEN   0x28
#define SPI_LEN   0x18
#define SYST_LEN  0x1C

#define DMA_ENABLE (0xFF0/4)

#define GPFSEL0    0

#define GPSET0     7
#define GPSET1     8

#define GPCLR0    10
#define GPCLR1    11

#define GPLEV0    13
#define GPLEV1    14

#define GPEDS0    16
#define GPEDS1    17

#define GPREN0    19
#define GPREN1    20
#define GPFEN0    22
#define GPFEN1    23
#define GPHEN0    25
#define GPHEN1    26
#define GPLEN0    28
#define GPLEN1    29
#define GPAREN0   31
#define GPAREN1   32
#define GPAFEN0   34
#define GPAFEN1   35

#define GPPUD     37
#define GPPUDCLK0 38
#define GPPUDCLK1 39

/* BCM2711 has different pulls */

#define GPPUPPDN0 57
#define GPPUPPDN1 58
#define GPPUPPDN2 59
#define GPPUPPDN3 60

#define DMA_CS        0
#define DMA_CONBLK_AD 1
#define DMA_DEBUG     8

/* DMA CS Control and Status bits */
#define DMA_CHANNEL_RESET       (1<<31)
#define DMA_CHANNEL_ABORT       (1<<30)
#define DMA_WAIT_ON_WRITES      (1<<28)
#define DMA_PANIC_PRIORITY(x) ((x)<<20)
#define DMA_PRIORITY(x)       ((x)<<16)
#define DMA_INTERRUPT_STATUS    (1<< 2)
#define DMA_END_FLAG            (1<< 1)
#define DMA_ACTIVE              (1<< 0)

/* DMA control block "info" field bits */
#define DMA_NO_WIDE_BURSTS          (1<<26)
#define DMA_PERIPHERAL_MAPPING(x) ((x)<<16)
#define DMA_BURST_LENGTH(x)       ((x)<<12)
#define DMA_SRC_IGNORE              (1<<11)
#define DMA_SRC_DREQ                (1<<10)
#define DMA_SRC_WIDTH               (1<< 9)
#define DMA_SRC_INC                 (1<< 8)
#define DMA_DEST_IGNORE             (1<< 7)
#define DMA_DEST_DREQ               (1<< 6)
#define DMA_DEST_WIDTH              (1<< 5)
#define DMA_DEST_INC                (1<< 4)
#define DMA_WAIT_RESP               (1<< 3)
#define DMA_TDMODE                  (1<< 1)

#define DMA_DEBUG_READ_ERR           (1<<2)
#define DMA_DEBUG_FIFO_ERR           (1<<1)
#define DMA_DEBUG_RD_LST_NOT_SET_ERR (1<<0)

#define DMA_LITE_FIRST 7
#define DMA_LITE_MAX 0xfffc

#define PWM_CTL      0
#define PWM_STA      1
#define PWM_DMAC     2
#define PWM_RNG1     4
#define PWM_DAT1     5
#define PWM_FIFO     6
#define PWM_RNG2     8
#define PWM_DAT2     9

#define PWM_CTL_MSEN2 (1<<15)
#define PWM_CTL_PWEN2 (1<<8)
#define PWM_CTL_MSEN1 (1<<7)
#define PWM_CTL_CLRF1 (1<<6)
#define PWM_CTL_USEF1 (1<<5)
#define PWM_CTL_MODE1 (1<<1)
#define PWM_CTL_PWEN1 (1<<0)

#define PWM_DMAC_ENAB      (1 <<31)
#define PWM_DMAC_PANIC(x) ((x)<< 8)
#define PWM_DMAC_DREQ(x)   (x)

#define PCM_CS     0
#define PCM_FIFO   1
#define PCM_MODE   2
#define PCM_RXC    3
#define PCM_TXC    4
#define PCM_DREQ   5
#define PCM_INTEN  6
#define PCM_INTSTC 7
#define PCM_GRAY   8

#define PCM_CS_STBY     (1 <<25)
#define PCM_CS_SYNC     (1 <<24)
#define PCM_CS_RXSEX    (1 <<23)
#define PCM_CS_RXERR    (1 <<16)
#define PCM_CS_TXERR    (1 <<15)
#define PCM_CS_DMAEN    (1  <<9)
#define PCM_CS_RXTHR(x) ((x)<<7)
#define PCM_CS_TXTHR(x) ((x)<<5)
#define PCM_CS_RXCLR    (1  <<4)
#define PCM_CS_TXCLR    (1  <<3)
#define PCM_CS_TXON     (1  <<2)
#define PCM_CS_RXON     (1  <<1)
#define PCM_CS_EN       (1  <<0)

#define PCM_MODE_CLK_DIS  (1  <<28)
#define PCM_MODE_PDMN     (1  <<27)
#define PCM_MODE_PDME     (1  <<26)
#define PCM_MODE_FRXP     (1  <<25)
#define PCM_MODE_FTXP     (1  <<24)
#define PCM_MODE_CLKM     (1  <<23)
#define PCM_MODE_CLKI     (1  <<22)
#define PCM_MODE_FSM      (1  <<21)
#define PCM_MODE_FSI      (1  <<20)
#define PCM_MODE_FLEN(x)  ((x)<<10)
#define PCM_MODE_FSLEN(x) ((x)<< 0)

#define PCM_RXC_CH1WEX    (1  <<31)
#define PCM_RXC_CH1EN     (1  <<30)
#define PCM_RXC_CH1POS(x) ((x)<<20)
#define PCM_RXC_CH1WID(x) ((x)<<16)
#define PCM_RXC_CH2WEX    (1  <<15)
#define PCM_RXC_CH2EN     (1  <<14)
#define PCM_RXC_CH2POS(x) ((x)<< 4)
#define PCM_RXC_CH2WID(x) ((x)<< 0)

#define PCM_TXC_CH1WEX    (1  <<31)
#define PCM_TXC_CH1EN     (1  <<30)
#define PCM_TXC_CH1POS(x) ((x)<<20)
#define PCM_TXC_CH1WID(x) ((x)<<16)
#define PCM_TXC_CH2WEX    (1  <<15)
#define PCM_TXC_CH2EN     (1  <<14)
#define PCM_TXC_CH2POS(x) ((x)<< 4)
#define PCM_TXC_CH2WID(x) ((x)<< 0)

#define PCM_DREQ_TX_PANIC(x) ((x)<<24)
#define PCM_DREQ_RX_PANIC(x) ((x)<<16)
#define PCM_DREQ_TX_REQ_L(x) ((x)<< 8)
#define PCM_DREQ_RX_REQ_L(x) ((x)<< 0)

#define PCM_INTEN_RXERR (1<<3)
#define PCM_INTEN_TXERR (1<<2)
#define PCM_INTEN_RXR   (1<<1)
#define PCM_INTEN_TXW   (1<<0)

#define PCM_INTSTC_RXERR (1<<3)
#define PCM_INTSTC_TXERR (1<<2)
#define PCM_INTSTC_RXR   (1<<1)
#define PCM_INTSTC_TXW   (1<<0)

#define PCM_GRAY_FLUSH (1<<2)
#define PCM_GRAY_CLR   (1<<1)
#define PCM_GRAY_EN    (1<<0)

#define BCM_PASSWD  (0x5A<<24)

#define CLK_CTL_MASH(x)((x)<<9)
#define CLK_CTL_BUSY    (1 <<7)
#define CLK_CTL_KILL    (1 <<5)
#define CLK_CTL_ENAB    (1 <<4)
#define CLK_CTL_SRC(x) ((x)<<0)

#define CLK_SRCS 2

#define CLK_CTL_SRC_OSC  1
#define CLK_CTL_SRC_PLLD 6

#define CLK_OSC_FREQ        19200000
#define CLK_OSC_FREQ_2711   54000000
#define CLK_PLLD_FREQ      500000000
#define CLK_PLLD_FREQ_2711 750000000

#define CLK_DIV_DIVI(x) ((x)<<12)
#define CLK_DIV_DIVF(x) ((x)<< 0)

#define CLK_GP0_CTL 28
#define CLK_GP0_DIV 29
#define CLK_GP1_CTL 30
#define CLK_GP1_DIV 31
#define CLK_GP2_CTL 32
#define CLK_GP2_DIV 33

#define CLK_PCMCTL 38
#define CLK_PCMDIV 39

#define CLK_PWMCTL 40
#define CLK_PWMDIV 41

#define SYST_CS      0
#define SYST_CLO     1
#define SYST_CHI     2

/* SPI */

#define SPI_CS   0
#define SPI_FIFO 1
#define SPI_CLK  2
#define SPI_DLEN 3
#define SPI_LTOH 4
#define SPI_DC   5

#define SPI_CS_LEN_LONG    (1<<25)
#define SPI_CS_DMA_LEN     (1<<24)
#define SPI_CS_CSPOLS(x) ((x)<<21)
#define SPI_CS_RXF         (1<<20)
#define SPI_CS_RXR         (1<<19)
#define SPI_CS_TXD         (1<<18)
#define SPI_CS_RXD         (1<<17)
#define SPI_CS_DONE        (1<<16)
#define SPI_CS_LEN         (1<<13)
#define SPI_CS_REN         (1<<12)
#define SPI_CS_ADCS        (1<<11)
#define SPI_CS_INTR        (1<<10)
#define SPI_CS_INTD        (1<<9)
#define SPI_CS_DMAEN       (1<<8)
#define SPI_CS_TA          (1<<7)
#define SPI_CS_CSPOL(x)  ((x)<<6)
#define SPI_CS_CLEAR(x)  ((x)<<4)
#define SPI_CS_MODE(x)   ((x)<<2)
#define SPI_CS_CS(x)     ((x)<<0)

#define SPI_DC_RPANIC(x) ((x)<<24)
#define SPI_DC_RDREQ(x)  ((x)<<16)
#define SPI_DC_TPANIC(x) ((x)<<8)
#define SPI_DC_TDREQ(x)  ((x)<<0)

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

#define SPI_CS0     0
#define SPI_CS1     1
#define SPI_CS2     2

/* standard SPI gpios (ALT0) */

#define PI_SPI_CE0   8
#define PI_SPI_CE1   7
#define PI_SPI_SCLK 11
#define PI_SPI_MISO  9
#define PI_SPI_MOSI 10

/* auxiliary SPI gpios (ALT4) */

#define PI_ASPI_CE0  18
#define PI_ASPI_CE1  17
#define PI_ASPI_CE2  16
#define PI_ASPI_MISO 19
#define PI_ASPI_MOSI 20
#define PI_ASPI_SCLK 21

/* AUX */

#define AUX_IRQ     0
#define AUX_ENABLES 1

#define AUX_MU_IO_REG   16
#define AUX_MU_IER_REG  17
#define AUX_MU_IIR_REG  18
#define AUX_MU_LCR_REG  19
#define AUX_MU_MCR_REG  20
#define AUX_MU_LSR_REG  21
#define AUX_MU_MSR_REG  22
#define AUX_MU_SCRATCH  23
#define AUX_MU_CNTL_REG 24
#define AUX_MU_STAT_REG 25
#define AUX_MU_BAUD_REG 26

#define AUX_SPI0_CNTL0_REG 32
#define AUX_SPI0_CNTL1_REG 33
#define AUX_SPI0_STAT_REG  34
#define AUX_SPI0_PEEK_REG  35

#define AUX_SPI0_IO_REG    40
#define AUX_SPI0_TX_HOLD   44

#define AUX_SPI1_CNTL0_REG 48
#define AUX_SPI1_CNTL1_REG 49
#define AUX_SPI1_STAT_REG  50
#define AUX_SPI1_PEEK_REG  51

#define AUX_SPI1_IO_REG    56
#define AUX_SPI1_TX_HOLD   60

#define AUXENB_SPI2 (1<<2)
#define AUXENB_SPI1 (1<<1)
#define AUXENB_UART (1<<0)

#define AUXSPI_CNTL0_SPEED(x)      ((x)<<20)
#define AUXSPI_CNTL0_CS(x)         ((x)<<17)
#define AUXSPI_CNTL0_POSTINP         (1<<16)
#define AUXSPI_CNTL0_VAR_CS          (1<<15)
#define AUXSPI_CNTL0_VAR_WIDTH       (1<<14)
#define AUXSPI_CNTL0_DOUT_HOLD(x)  ((x)<<12)
#define AUXSPI_CNTL0_ENABLE          (1<<11)
#define AUXSPI_CNTL0_IN_RISING(x)  ((x)<<10)
#define AUXSPI_CNTL0_CLR_FIFOS       (1<<9)
#define AUXSPI_CNTL0_OUT_RISING(x) ((x)<<8)
#define AUXSPI_CNTL0_INVERT_CLK(x) ((x)<<7)
#define AUXSPI_CNTL0_MSB_FIRST(x)  ((x)<<6)
#define AUXSPI_CNTL0_SHIFT_LEN(x)  ((x)<<0)

#define AUXSPI_CNTL1_CS_HIGH(x)  ((x)<<8)
#define AUXSPI_CNTL1_TX_IRQ        (1<<7)
#define AUXSPI_CNTL1_DONE_IRQ      (1<<6)
#define AUXSPI_CNTL1_MSB_FIRST(x)((x)<<1)
#define AUXSPI_CNTL1_KEEP_INPUT    (1<<0)

#define AUXSPI_STAT_TX_FIFO(x) ((x)<<28)
#define AUXSPI_STAT_RX_FIFO(x) ((x)<<20)
#define AUXSPI_STAT_TX_FULL      (1<<10)
#define AUXSPI_STAT_TX_EMPTY     (1<<9)
#define AUXSPI_STAT_RX_EMPTY     (1<<7)
#define AUXSPI_STAT_BUSY         (1<<6)
#define AUXSPI_STAT_BITS(x)    ((x)<<0)

/* --------------------------------------------------------------- */

#define NORMAL_DMA (DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP)
#define TWO_BEAT_DMA (DMA_TDMODE | DMA_BURST_LENGTH(1))

#define TIMED_DMA(x) (DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(x))

#define PCM_TIMER (((PCM_BASE + PCM_FIFO*4) & 0x00ffffff) | PI_PERI_BUS)
#define PWM_TIMER (((PWM_BASE + PWM_FIFO*4) & 0x00ffffff) | PI_PERI_BUS)

#define DBG_MIN_LEVEL 0
#define DBG_ALWAYS    0
#define DBG_STARTUP   1
#define DBG_DMACBS    2
#define DBG_SCRIPT    3
#define DBG_USER      4
#define DBG_INTERNAL  5
#define DBG_SLOW_TICK 6
#define DBG_FAST_TICK 7
#define DBG_MAX_LEVEL 8

#define GPIO_UNDEFINED 0
#define GPIO_WRITE     1
#define GPIO_PWM       2
#define GPIO_SERVO     3
#define GPIO_HW_CLK    4
#define GPIO_HW_PWM    5
#define GPIO_SPI       6
#define GPIO_I2C       7

#define STACK_SIZE (256*1024)

#define PAGE_SIZE 4096

#define PWM_FREQS 18

#define CYCLES_PER_BLOCK 80
#define PULSE_PER_CYCLE  25

#define PAGES_PER_BLOCK 53

#define CBS_PER_IPAGE 117
#define LVS_PER_IPAGE  38
#define OFF_PER_IPAGE  38
#define TCK_PER_IPAGE   2
#define ON_PER_IPAGE    2
#define PAD_PER_IPAGE   7

#define CBS_PER_OPAGE 118
#define OOL_PER_OPAGE  79

/*
Wave Count Block

Assumes two counters per block.  Each counter 4 * 16 (16^4=65536)
   0  CB [13]  13*8  104 CBs for counter 0
 104  CB [13]  13*8  104 CBs for counter 1
 208  CB [60]  60*8  480 CBs reserved to construct wave
 688 OOL [60]  60*1   60 OOL reserved to construct wave
 748 OOL[136] 136*1  136 OOL for counter 0
 884 OOL[136] 136*1  136 OOL for counter 1
1020 pad  [4]   4*1    4 spare
*/

#define WCB_CNT_PER_PAGE 2
#define WCB_COUNTERS (WCB_CNT_PER_PAGE * PI_WAVE_COUNT_PAGES)
#define WCB_CNT_CBS 13
#define WCB_CNT_OOL 68
#define WCB_COUNTER_OOL (WCB_CNT_PER_PAGE * WCB_CNT_OOL)
#define WCB_COUNTER_CBS (WCB_CNT_PER_PAGE * WCB_CNT_CBS)
#define WCB_CHAIN_CBS   60
#define WCB_CHAIN_OOL   60

#define CBS_PER_CYCLE ((PULSE_PER_CYCLE*3)+2)

#define NUM_CBS (CBS_PER_CYCLE * bufferCycles)

#define SUPERCYCLE 800
#define SUPERLEVEL 20000

#define BLOCK_SIZE (PAGES_PER_BLOCK*PAGE_SIZE)

#define DMAI_PAGES (PAGES_PER_BLOCK * bufferBlocks)

#define DMAO_PAGES (PAGES_PER_BLOCK * PI_WAVE_BLOCKS)

#define NUM_WAVE_OOL (DMAO_PAGES * OOL_PER_OPAGE)
#define NUM_WAVE_CBS (DMAO_PAGES * CBS_PER_OPAGE)

#define TICKSLOTS 50

#define PI_I2C_CLOSED   0
#define PI_I2C_RESERVED 1
#define PI_I2C_OPENED   2

#define PI_SPI_CLOSED   0
#define PI_SPI_RESERVED 1
#define PI_SPI_OPENED   2

#define PI_SER_CLOSED   0
#define PI_SER_RESERVED 1
#define PI_SER_OPENED   2

#define PI_FILE_CLOSED   0
#define PI_FILE_RESERVED 1
#define PI_FILE_OPENED   2

#define PI_NOTIFY_CLOSED   0
#define PI_NOTIFY_RESERVED 1
#define PI_NOTIFY_CLOSING  2
#define PI_NOTIFY_OPENED   3
#define PI_NOTIFY_RUNNING  4
#define PI_NOTIFY_PAUSED   5

#define PI_WFRX_NONE     0
#define PI_WFRX_SERIAL   1
#define PI_WFRX_I2C_SDA  2
#define PI_WFRX_I2C_SCL  3
#define PI_WFRX_SPI_SCLK 4
#define PI_WFRX_SPI_MISO 5
#define PI_WFRX_SPI_MOSI 6
#define PI_WFRX_SPI_CS   7

#define PI_WF_MICROS   1

#define BPD 4

#define MAX_REPORT 250
#define MAX_SAMPLE 4000

#define DEFAULT_PWM_IDX 5

#define MAX_EMITS (PIPE_BUF / sizeof(gpioReport_t))

#define SRX_BUF_SIZE 8192

#define PI_I2C_RETRIES 0x0701
#define PI_I2C_TIMEOUT 0x0702
#define PI_I2C_SLAVE   0x0703
#define PI_I2C_FUNCS   0x0705
#define PI_I2C_RDWR    0x0707
#define PI_I2C_SMBUS   0x0720

#define PI_I2C_SMBUS_READ  1
#define PI_I2C_SMBUS_WRITE 0

#define PI_I2C_SMBUS_QUICK            0
#define PI_I2C_SMBUS_BYTE             1
#define PI_I2C_SMBUS_BYTE_DATA        2
#define PI_I2C_SMBUS_WORD_DATA        3
#define PI_I2C_SMBUS_PROC_CALL        4
#define PI_I2C_SMBUS_BLOCK_DATA       5
#define PI_I2C_SMBUS_I2C_BLOCK_BROKEN 6
#define PI_I2C_SMBUS_BLOCK_PROC_CALL  7
#define PI_I2C_SMBUS_I2C_BLOCK_DATA   8

#define PI_I2C_SMBUS_BLOCK_MAX     32
#define PI_I2C_SMBUS_I2C_BLOCK_MAX 32

#define PI_I2C_FUNC_SMBUS_QUICK            0x00010000
#define PI_I2C_FUNC_SMBUS_READ_BYTE        0x00020000
#define PI_I2C_FUNC_SMBUS_WRITE_BYTE       0x00040000
#define PI_I2C_FUNC_SMBUS_READ_BYTE_DATA   0x00080000
#define PI_I2C_FUNC_SMBUS_WRITE_BYTE_DATA  0x00100000
#define PI_I2C_FUNC_SMBUS_READ_WORD_DATA   0x00200000
#define PI_I2C_FUNC_SMBUS_WRITE_WORD_DATA  0x00400000
#define PI_I2C_FUNC_SMBUS_PROC_CALL        0x00800000
#define PI_I2C_FUNC_SMBUS_READ_BLOCK_DATA  0x01000000
#define PI_I2C_FUNC_SMBUS_WRITE_BLOCK_DATA 0x02000000
#define PI_I2C_FUNC_SMBUS_READ_I2C_BLOCK   0x04000000
#define PI_I2C_FUNC_SMBUS_WRITE_I2C_BLOCK  0x08000000

#define PI_MASH_MAX_FREQ 23800000

#define FLUSH_PAGES 1024

#define MB_DEV_MAJOR 100

#define MB_IOCTL _IOWR(MB_DEV_MAJOR, 0, char *)

#define MB_DEV1 "/dev/vcio"
#define MB_DEV2 "/dev/pigpio-mb"

#define BUS_TO_PHYS(x) ((x)&~0xC0000000)

#define MB_END_TAG 0
#define MB_PROCESS_REQUEST 0

#define MB_ALLOCATE_MEMORY_TAG 0x3000C
#define MB_LOCK_MEMORY_TAG     0x3000D
#define MB_UNLOCK_MEMORY_TAG   0x3000E
#define MB_RELEASE_MEMORY_TAG  0x3000F

#define PI_SCRIPT_FREE     0
#define PI_SCRIPT_RESERVED 1
#define PI_SCRIPT_IN_USE   2
#define PI_SCRIPT_DYING    3

#define PI_SCRIPT_HALT   0
#define PI_SCRIPT_RUN    1
#define PI_SCRIPT_DELETE 2

#define PI_SCRIPT_STACK_SIZE 256

#define PI_SPI_FLAGS_CHANNEL(x)    ((x&7)<<29)

#define PI_SPI_FLAGS_GET_CHANNEL(x) (((x)>>29)&7)
#define PI_SPI_FLAGS_GET_BITLEN(x)  (((x)>>16)&63)
#define PI_SPI_FLAGS_GET_RX_LSB(x)  (((x)>>15)&1)
#define PI_SPI_FLAGS_GET_TX_LSB(x)  (((x)>>14)&1)
#define PI_SPI_FLAGS_GET_3WREN(x)   (((x)>>10)&15)
#define PI_SPI_FLAGS_GET_3WIRE(x)   (((x)>>9)&1)
#define PI_SPI_FLAGS_GET_AUX_SPI(x) (((x)>>8)&1)
#define PI_SPI_FLAGS_GET_RESVD(x)   (((x)>>5)&7)
#define PI_SPI_FLAGS_GET_CSPOLS(x)  (((x)>>2)&7)
#define PI_SPI_FLAGS_GET_MODE(x)     ((x)&3)

#define PI_SPI_FLAGS_GET_CPHA(x)  ((x)&1)
#define PI_SPI_FLAGS_GET_CPOL(x)  ((x)&2)
#define PI_SPI_FLAGS_GET_CSPOL(x) ((x)&4)

#define PI_STARTING 0
#define PI_RUNNING  1
#define PI_ENDING   2

#define PI_THREAD_NONE    0
#define PI_THREAD_STARTED 1
#define PI_THREAD_RUNNING 2

#define PI_MAX_PATH 512

/* typedef ------------------------------------------------------- */

typedef void (*callbk_t) ();

typedef struct
{
   rawCbs_t cb           [128];
} dmaPage_t;

typedef struct
{
   rawCbs_t cb           [CBS_PER_IPAGE];
   uint32_t level        [LVS_PER_IPAGE];
   uint32_t gpioOff      [OFF_PER_IPAGE];
   uint32_t tick         [TCK_PER_IPAGE];
   uint32_t gpioOn       [ON_PER_IPAGE];
   uint32_t periphData;
   uint32_t pad          [PAD_PER_IPAGE];
} dmaIPage_t;

typedef struct
{
   rawCbs_t cb     [CBS_PER_OPAGE];
   uint32_t OOL    [OOL_PER_OPAGE];
   uint32_t periphData;
} dmaOPage_t;

typedef struct
{
   uint8_t  is;
   uint8_t  pad;
   uint16_t width;
   uint16_t range; /* dutycycles specified by 0 .. range */
   uint16_t freqIdx;
   uint16_t deferOff;
   uint16_t deferRng;
} gpioInfo_t;

typedef struct
{
   callbk_t func;
   unsigned ex;
   void *userdata;

   int      wdSteadyUs;
   uint32_t wdTick;
   uint32_t wdLBitV;

   int      nfSteadyUs;
   int      nfActiveUs;
   int      nfActive;
   uint32_t nfTick1;
   uint32_t nfTick2;
   uint32_t nfLBitV;
   uint32_t nfRBitV;

   uint32_t gfSteadyUs;
   uint8_t  gfInitialised;
   uint32_t gfTick;
   uint32_t gfLBitV;
   uint32_t gfRBitV;

} gpioAlert_t;

typedef struct
{
   callbk_t func;
   unsigned ex;
   void *userdata;
   int ignore;
   int fired;
} eventAlert_t;

typedef struct
{
   unsigned gpio;
   pthread_t *pth;
   callbk_t func;
   unsigned edge;
   int timeout;
   unsigned ex;
   void *userdata;
   int fd;
   int inited;
} gpioISR_t;

typedef struct
{
   callbk_t func;
   unsigned ex;
   void *userdata;
} gpioSignal_t;

typedef struct
{
   callbk_t func;
   unsigned ex;
   void *userdata;
   uint32_t bits;
} gpioGetSamples_t;

typedef struct
{
   callbk_t func;
   unsigned ex;
   void *userdata;
   unsigned id;
   unsigned running;
   unsigned millis;
   pthread_t pthId;
} gpioTimer_t;

typedef struct
{
   unsigned id;
   unsigned state;
   unsigned request;
   unsigned run_state;
   uint32_t waitBits;
   uint32_t eventBits;
   uint32_t changedBits;
   pthread_t *pthIdp;
   pthread_mutex_t pthMutex;
   pthread_cond_t pthCond;
   cmdScript_t script;
} gpioScript_t;


typedef struct
{
   uint16_t valid;
   uint16_t servoIdx;
} clkCfg_t;

typedef struct
{
   uint16_t seqno;
   uint16_t state;
   uint32_t bits;
   uint32_t eventBits;
   uint32_t lastReportTick;
   int      fd;
   int      pipe;
   int      max_emits;
} gpioNotify_t;

typedef struct
{
   uint16_t state;
   int16_t  fd;
   uint32_t mode;
} fileInfo_t;

typedef struct
{
   uint16_t state;
   int16_t  fd;
   uint32_t addr;
   uint32_t flags;
   uint32_t funcs;
} i2cInfo_t;

typedef struct
{
   uint16_t state;
   int16_t  fd;
   uint32_t flags;
} serInfo_t;

typedef struct
{
   uint16_t state;
   unsigned speed;
   uint32_t flags;
} spiInfo_t;

typedef struct
{
   uint32_t alertTicks;
   uint32_t lateTicks;
   uint32_t moreToDo;
   uint32_t diffTick[TICKSLOTS];
   uint32_t cbTicks;
   uint32_t cbCalls;
   uint32_t maxEmit;
   uint32_t emitFrags;
   uint32_t maxSamples;
   uint32_t numSamples;
   uint32_t DMARestarts;
   uint32_t dmaInitCbsCount;
   uint32_t goodPipeWrite;
   uint32_t shortPipeWrite;
   uint32_t wouldBlockPipeWrite;
} gpioStats_t;

typedef struct
{
   unsigned bufferMilliseconds;
   unsigned clockMicros;
   unsigned clockPeriph;
   unsigned DMAprimaryChannel;
   unsigned DMAsecondaryChannel;
   unsigned socketPort;
   unsigned ifFlags;
   unsigned memAllocMode;
   unsigned dbgLevel;
   unsigned alertFreq;
   uint32_t internals;
      /*
      0-3: dbgLevel
      4-7: alertFreq
      */
} gpioCfg_t;

typedef struct
{
   uint32_t micros;
   uint32_t highMicros;
   uint32_t maxMicros;
   uint32_t pulses;
   uint32_t highPulses;
   uint32_t maxPulses;
   uint32_t cbs;
   uint32_t highCbs;
   uint32_t maxCbs;
} wfStats_t;

typedef struct
{
   char    *buf;
   uint32_t bufSize;
   int      readPos;
   int      writePos;
   uint32_t fullBit; /* nanoseconds */
   uint32_t halfBit; /* nanoseconds */
   int      timeout; /* millisconds */
   uint32_t startBitTick; /* microseconds */
   uint32_t nextBitDiff; /* nanoseconds */
   int      bit;
   uint32_t data;
   int      bytes; /* 1, 2, 4 */
   int      level;
   int      dataBits; /* 1-32 */
   int      invert; /* 0, 1 */
} wfRxSerial_t;

typedef struct
{
   int SDA;
   int SCL;
   int delay;
   int SDAMode;
   int SCLMode;
   int started;
} wfRxI2C_t;

typedef struct
{
   int CS;
   int MISO;
   int MOSI;
   int SCLK;
   int usage;
   int delay;
   int spiFlags;
   int MISOMode;
   int MOSIMode;
   int CSMode;
   int SCLKMode;
} wfRxSPI_t;

typedef struct
{
   int      mode;
   int      gpio;
   uint32_t baud;
   pthread_mutex_t mutex;
   union
   {
      wfRxSerial_t s;
      wfRxI2C_t    I;
      wfRxSPI_t    S;
   };
} wfRx_t;

union my_smbus_data
{
   uint8_t  byte;
   uint16_t word;
   uint8_t  block[PI_I2C_SMBUS_BLOCK_MAX + 2];
};

struct my_smbus_ioctl_data
{
   uint8_t read_write;
   uint8_t command;
   uint32_t size;
   union my_smbus_data *data;
};

typedef struct
{
   pi_i2c_msg_t *msgs; /* pointers to pi_i2c_msgs */
   uint32_t     nmsgs; /* number of pi_i2c_msgs */
} my_i2c_rdwr_ioctl_data_t;

typedef struct
{
   unsigned div;
   unsigned frac;
   unsigned clock;
} clkInf_t;

typedef struct
{
   unsigned  handle;        /* mbAllocateMemory() */
   uintptr_t bus_addr;      /* mbLockMemory() */
   uintptr_t *virtual_addr; /* mbMapMem() */
   unsigned  size;          /* in bytes */
} DMAMem_t;

/* global -------------------------------------------------------- */

/* initialise once then preserve */

static volatile uint32_t piCores       = 0;
static volatile uint32_t pi_peri_phys  = 0x20000000;
static volatile uint32_t pi_dram_bus   = 0x40000000;
static volatile uint32_t pi_mem_flag   = 0x0C;
static volatile uint32_t pi_ispi       = 0;
static volatile uint32_t pi_is_2711    = 0;
static volatile uint32_t clk_osc_freq  = CLK_OSC_FREQ;
static volatile uint32_t clk_plld_freq = CLK_PLLD_FREQ;
static volatile uint32_t hw_pwm_max_freq = PI_HW_PWM_MAX_FREQ;
static volatile uint32_t hw_clk_min_freq = PI_HW_CLK_MIN_FREQ;
static volatile uint32_t hw_clk_max_freq = PI_HW_CLK_MAX_FREQ;

static int libInitialised = 0;

/* initialise every gpioInitialise */

static struct timespec libStarted;

static uint32_t sockNetAddr[MAX_CONNECT_ADDRESSES];

static int numSockNetAddr = 0;

static uint32_t reportedLevel = 0;

static int waveClockInited = 0;
static int PWMClockInited = 0;

static volatile gpioStats_t gpioStats;

static int gpioMaskSet = 0;

/* initialise if not libInitialised */

static uint64_t gpioMask;

static rawWave_t wf[3][PI_WAVE_MAX_PULSES];

static int wfc[3]={0, 0, 0};

static int wfcur=0;

static wfStats_t wfStats=
{
   0, 0, PI_WAVE_MAX_MICROS,
   0, 0, PI_WAVE_MAX_PULSES,
   0, 0, (DMAO_PAGES * CBS_PER_OPAGE)
};

static rawWaveInfo_t waveInfo[PI_MAX_WAVES];

static wfRx_t wfRx[PI_MAX_USER_GPIO+1];

static int waveOutBotCB  = PI_WAVE_COUNT_PAGES*CBS_PER_OPAGE;
static int waveOutBotOOL = PI_WAVE_COUNT_PAGES*OOL_PER_OPAGE;
static int waveOutTopOOL = NUM_WAVE_OOL;
static int waveOutCount = 0;

static uint32_t *waveEndPtr = NULL;

static volatile uint32_t alertBits   = 0;
static volatile uint32_t monitorBits = 0;
static volatile uint32_t notifyBits  = 0;
static volatile uint32_t scriptBits  = 0;
static volatile uint32_t gFilterBits = 0;
static volatile uint32_t nFilterBits = 0;
static volatile uint32_t wdogBits    = 0;

static volatile uint32_t scriptEventBits  = 0;

static volatile int runState = PI_STARTING;

static int pthAlertRunning  = PI_THREAD_NONE;
static int pthFifoRunning   = PI_THREAD_NONE;
static int pthSocketRunning = PI_THREAD_NONE;

static gpioAlert_t      gpioAlert  [PI_MAX_USER_GPIO+1];

static eventAlert_t     eventAlert [PI_MAX_EVENT+1];

static gpioISR_t        gpioISR    [PI_MAX_GPIO+1];

static gpioGetSamples_t gpioGetSamples;

static gpioInfo_t       gpioInfo   [PI_MAX_GPIO+1];

static gpioNotify_t     gpioNotify [PI_NOTIFY_SLOTS];

static fileInfo_t       fileInfo   [PI_FILE_SLOTS];
static i2cInfo_t        i2cInfo    [PI_I2C_SLOTS];
static serInfo_t        serInfo    [PI_SER_SLOTS];
static spiInfo_t        spiInfo    [PI_SPI_SLOTS];

static gpioScript_t     gpioScript [PI_MAX_SCRIPTS];

static gpioSignal_t     gpioSignal [PI_MAX_SIGNUM+1];

static gpioTimer_t      gpioTimer  [PI_MAX_TIMER+1];

static int pwmFreq[PWM_FREQS];

/* reset after gpioTerminated */

/* resources which must be released on gpioTerminate */

static FILE * inpFifo = NULL;
static FILE * outFifo = NULL;

static int fdLock       = -1;
static int fdMem        = -1;
static int fdSock       = -1;
static int fdPmap       = -1;
static int fdMbox       = -1;

static DMAMem_t *dmaMboxBlk = MAP_FAILED;
static uintptr_t * * dmaPMapBlk = MAP_FAILED;
static dmaPage_t * * dmaVirt = MAP_FAILED;
static dmaPage_t * * dmaBus = MAP_FAILED;

static dmaIPage_t * * dmaIVirt = MAP_FAILED;
static dmaIPage_t * * dmaIBus = MAP_FAILED;

static dmaOPage_t * * dmaOVirt = MAP_FAILED;
static dmaOPage_t * * dmaOBus = MAP_FAILED;

static volatile uint32_t * auxReg  = MAP_FAILED;
static volatile uint32_t * bscsReg = MAP_FAILED;
static volatile uint32_t * clkReg  = MAP_FAILED;
static volatile uint32_t * dmaReg  = MAP_FAILED;
static volatile uint32_t * gpioReg = MAP_FAILED;
static volatile uint32_t * padsReg = MAP_FAILED;
static volatile uint32_t * pcmReg  = MAP_FAILED;
static volatile uint32_t * pwmReg  = MAP_FAILED;
static volatile uint32_t * spiReg  = MAP_FAILED;
static volatile uint32_t * systReg = MAP_FAILED;

static volatile uint32_t * dmaIn   = MAP_FAILED;
static volatile uint32_t * dmaOut  = MAP_FAILED;

static uint32_t hw_clk_freq[3];
static uint32_t hw_pwm_freq[2];
static uint32_t hw_pwm_duty[2];
static uint32_t hw_pwm_real_range[2];

static volatile gpioCfg_t gpioCfg =
{
   PI_DEFAULT_BUFFER_MILLIS,
   PI_DEFAULT_CLK_MICROS,
   PI_DEFAULT_CLK_PERIPHERAL,
   PI_DEFAULT_DMA_NOT_SET, /* primary DMA */
   PI_DEFAULT_DMA_NOT_SET, /* secondary DMA */
   PI_DEFAULT_SOCKET_PORT,
   PI_DEFAULT_IF_FLAGS,
   PI_DEFAULT_MEM_ALLOC_MODE,
   0, /* dbgLevel */
   0, /* alertFreq */
   0, /* internals */
};

/* no initialisation required */

static unsigned bufferBlocks; /* number of blocks in buffer */
static unsigned bufferCycles; /* number of cycles */

static pthread_t pthAlert;
static pthread_t pthFifo;
static pthread_t pthSocket;

static uint32_t spi_dummy;

static unsigned old_mode_ce0;
static unsigned old_mode_ce1;
static unsigned old_mode_sclk;
static unsigned old_mode_miso;
static unsigned old_mode_mosi;

static uint32_t old_spi_cs;
static uint32_t old_spi_clk;

static unsigned old_mode_ace0;
static unsigned old_mode_ace1;
static unsigned old_mode_ace2;
static unsigned old_mode_asclk;
static unsigned old_mode_amiso;
static unsigned old_mode_amosi;

static uint32_t old_spi_cntl0;
static uint32_t old_spi_cntl1;

static uint32_t bscFR;

/* const --------------------------------------------------------- */

static const uint8_t clkDef[PI_MAX_GPIO + 1] =
{
 /*             0     1     2     3     4     5     6     7     8     9 */
   /* 0 */   0x00, 0x00, 0x00, 0x00, 0x84, 0x94, 0xA4, 0x00, 0x00, 0x00,
   /* 1 */   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   /* 2 */   0x82, 0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   /* 3 */   0x00, 0x00, 0x84, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00,
   /* 4 */   0x00, 0x00, 0x94, 0xA4, 0x94, 0x00, 0x00, 0x00, 0x00, 0x00,
   /* 5 */   0x00, 0x00, 0x00, 0x00,
};

/*
 7 6 5 4 3 2 1 0
 V . C C . M M M

 V: 0 no clock, 1 has a clock
CC: 00 CLK0, 01 CLK1, 10 CLK2
 M: 100 ALT0, 010 ALT5

 gpio4  GPCLK0 ALT0
 gpio5  GPCLK1 ALT0 B+ and compute module only (reserved for system use)
 gpio6  GPCLK2 ALT0 B+ and compute module only
 gpio20 GPCLK0 ALT5 B+ and compute module only
 gpio21 GPCLK1 ALT5 Not available on Rev.2 B (reserved for system use)

 gpio32 GPCLK0 ALT0 Compute module only
 gpio34 GPCLK0 ALT0 Compute module only
 gpio42 GPCLK1 ALT0 Compute module only (reserved for system use)
 gpio43 GPCLK2 ALT0 Compute module only
 gpio44 GPCLK1 ALT0 Compute module only (reserved for system use)
*/

static const uint8_t PWMDef[PI_MAX_GPIO + 1] =
{
   /*          0     1     2     3     4     5     6     7     8     9 */
   /* 0 */   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   /* 1 */   0x00, 0x00, 0x84, 0x94, 0x00, 0x00, 0x00, 0x00, 0x82, 0x92,
   /* 2 */   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   /* 3 */   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   /* 4 */   0x84, 0x94, 0x00, 0x00, 0x00, 0x94, 0x00, 0x00, 0x00, 0x00,
   /* 5 */   0x00, 0x00, 0x85, 0x95,
};

/*
 7 6 5 4 3 2 1 0
 V . . P . M M M

 V: 0 no PWM, 1 has a PWM
 P: 0 PWM0, 1 PWM1
 M: 010 ALT5, 100 ALT0, 101 ALT1

 gpio12 pwm0 ALT0
 gpio13 pwm1 ALT0
 gpio18 pwm0 ALT5
 gpio19 pwm1 ALT5
 gpio40 pwm0 ALT0
 gpio41 pwm1 ALT0
 gpio45 pwm1 ALT0
 gpio52 pwm0 ALT1
 gpio53 pwm1 ALT1
*/

static const clkCfg_t clkCfg[]=
{
   /* valid servo */
      {   0,    0}, /*  0 */
      {   1,   17}, /*  1 */
      {   1,   16}, /*  2 */
      {   0,    0}, /*  3 */
      {   1,   15}, /*  4 */
      {   1,   14}, /*  5 */
      {   0,    0}, /*  6 */
      {   0,    0}, /*  7 */
      {   1,   13}, /*  8 */
      {   0,    0}, /*  9 */
      {   1,   12}, /* 10 */
};

static const uint16_t pwmCycles[PWM_FREQS]=
   {  1,    2,    4,    5,    8,   10,   16,    20,    25,
     32,   40,   50,   80,  100,  160,  200,   400,   800};

static const uint16_t pwmRealRange[PWM_FREQS]=
   { 25,   50,  100,  125,  200,  250,  400,   500,   625,
    800, 1000, 1250, 2000, 2500, 4000, 5000, 10000, 20000};

/* prototype ----------------------------------------------------- */

static void intNotifyBits(void);

static void intScriptBits(void);

static void intScriptEventBits(void);

static int  gpioNotifyOpenInBand(int fd);

static void initHWClk
   (int clkCtl, int clkDiv, int clkSrc, int divI, int divF, int MASH);

static void initDMAgo(volatile uint32_t  *dmaAddr, uint32_t cbAddr);

int gpioWaveTxStart(unsigned wave_mode); /* deprecated */

static void closeOrphanedNotifications(int slot, int fd);


/* ======================================================================= */

int myScriptNameValid(char *name)
{
   int i, c, len, valid;

   len = strlen(name);

   valid = 1;

   for (i=0; i<len; i++)
   {
      c = name[i];

      if ((!isalnum(c)) && (c != '_') && (c != '-'))
      {
         valid = 0;
         break;
      }
   }
   return valid;
}

/* ----------------------------------------------------------------------- */

static char * myTimeStamp()
{
   static struct timeval last;
   static char buf[32];
   struct timeval now;

   struct tm tmp;

   gettimeofday(&now, NULL);

   if (now.tv_sec != last.tv_sec)
   {
      localtime_r(&now.tv_sec, &tmp);
      strftime(buf, sizeof(buf), "%F %T", &tmp);
      last.tv_sec = now.tv_sec;
   }

   return buf;
}

/* ----------------------------------------------------------------------- */

int myPathBad(char *name)
{
   int i, c, len, in_part, parts, last_char_dot;
   char *bad="/*?.";

   parts = 0;
   in_part = 0;
   last_char_dot = 0;

   if (strstr(name, "..")) return 1;
   if (strstr(name, "\\.")) return 1;

   len = strlen(name);

   for (i=0; i<len; i++)
   {
      c = name[i];

      if (memchr(bad, c, 4)) /* wildcard or directory character */
      {
         if (c == '.')
         {
            if (last_char_dot) return 1;
            last_char_dot = 1;
         }
         else last_char_dot = 0;

         in_part = 0;
      }
      else /* normal character */
      {
         last_char_dot = 0;

         if (!in_part) parts++;

         in_part = 1;
      }
   }

   if (parts < 2) return 1; else return 0;
}

/* ----------------------------------------------------------------------- */

static char *myBuf2Str(unsigned count, char *buf)
{
   static char str[128];
   int i, c;

   if (count && buf)
   {
      if (count > 40) c = 40; else c = count;

      for (i=0; i<c; i++) sprintf(str+(3*i), "%02X ", buf[i]);
      str[(3*c)-1] = 0;
   }
   else str[0] = 0;

   return str;
}

/* ----------------------------------------------------------------------- */

static int my_smbus_access(
   int fd, char rw, uint8_t cmd, int size, union my_smbus_data *data)
{
   struct my_smbus_ioctl_data args;

   DBG(DBG_INTERNAL, "rw=%d reg=%d cmd=%d data=%s",
      rw, cmd, size, myBuf2Str(data->byte+1, (char*)data));

   args.read_write = rw;
   args.command    = cmd;
   args.size       = size;
   args.data       = data;

   return ioctl(fd, PI_I2C_SMBUS, &args);
}

/* ----------------------------------------------------------------------- */

static void myGpioSetMode(unsigned gpio, unsigned mode)
{
   int reg, shift;

   reg   =  gpio/10;
   shift = (gpio%10) * 3;

   gpioReg[reg] = (gpioReg[reg] & ~(7<<shift)) | (mode<<shift);
}


/* ----------------------------------------------------------------------- */

static int myGpioRead(unsigned gpio)
{
   if ((*(gpioReg + GPLEV0 + BANK) & BIT) != 0) return PI_ON;
   else                                         return PI_OFF;
}


/* ----------------------------------------------------------------------- */

static void myGpioWrite(unsigned gpio, unsigned level)
{
   if (level == PI_OFF) *(gpioReg + GPCLR0 + BANK) = BIT;
   else                 *(gpioReg + GPSET0 + BANK) = BIT;
}

/* ----------------------------------------------------------------------- */

static void myGpioSleep(int seconds, int micros)
{
   struct timespec ts, rem;

   ts.tv_sec  = seconds;
   ts.tv_nsec = micros * 1000;

   while (clock_nanosleep(CLOCK_REALTIME, 0, &ts, &rem))
   {
      /* copy remaining time to ts */
      ts = rem;
   }
}

/* ----------------------------------------------------------------------- */

static uint32_t myGpioDelay(uint32_t micros)
{
   uint32_t start;

   start = systReg[SYST_CLO];

   if (micros <= PI_MAX_BUSY_DELAY)
   {
      while ((systReg[SYST_CLO] - start) <= micros);
   }
   else
   {
      myGpioSleep(micros/MILLION, micros%MILLION);
   }

   return (systReg[SYST_CLO] - start);
}

/* ----------------------------------------------------------------------- */

static void myCreatePipe(char * name, int perm)
{
   unlink(name);

   mkfifo(name, perm);

   if (chmod(name, perm) < 0)
   {
      DBG(DBG_ALWAYS, "Can't set permissions (%d) for %s, %m", perm, name);
      return;
   }
}

/* ----------------------------------------------------------------------- */

static void myOffPageSlot(int pos, int * page, int * slot)
{
   *page = pos/OFF_PER_IPAGE;
   *slot = pos%OFF_PER_IPAGE;
}

/* ----------------------------------------------------------------------- */

static void myLvsPageSlot(int pos, int * page, int * slot)
{
   *page = pos/LVS_PER_IPAGE;
   *slot = pos%LVS_PER_IPAGE;
}

/* ----------------------------------------------------------------------- */

static void myTckPageSlot(int pos, int * page, int * slot)
{
   *page = pos/TCK_PER_IPAGE;
   *slot = pos%TCK_PER_IPAGE;
}

/* ----------------------------------------------------------------------- */

static uint32_t myGetLevel(int pos)
{
   uint32_t level;
   int page, slot;

   myLvsPageSlot(pos, &page, &slot);

   level = dmaIVirt[page]->level[slot];

   return level;
}

/* ----------------------------------------------------------------------- */

static int myI2CGetPar(char *inBuf, int *inPos, int inLen, int *esc)
{
   int bytes;

   if (*esc) bytes = 2; else bytes = 1;

   *esc = 0;

   if (*inPos <= (inLen - bytes))
   {
      if (bytes == 1)
      {
         return inBuf[(*inPos)++];
      }
      else
      {
         (*inPos) += 2;
         return inBuf[*inPos-2] + (inBuf[*inPos-1]<<8);
      }
   }
   return -1;
}

/* ----------------------------------------------------------------------- */

static uint32_t myGetTick(int pos)
{
   uint32_t tick;
   int page, slot;

   myTckPageSlot(pos, &page, &slot);

   tick = dmaIVirt[page]->tick[slot];

   return tick;
}

static int myPermit(unsigned gpio)
{
   if (gpio <= PI_MAX_GPIO)
   {
      if (gpioMask & ((uint64_t)(1)<<gpio)) return 1;
      else return 0;
   }
   return 1; /* will fail for bad gpio number */
}

static void flushMemory(void)
{
   static int val = 0;

   void *dummy;

   dummy = mmap(
       0, (FLUSH_PAGES*PAGE_SIZE),
       PROT_READ|PROT_WRITE|PROT_EXEC,
       MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
       -1, 0);

   if (dummy == MAP_FAILED)
   {
      DBG(DBG_STARTUP, "mmap dummy failed (%m)");
   }
   else
   {
      memset(dummy, val++, (FLUSH_PAGES*PAGE_SIZE));
      memset(dummy, val++, (FLUSH_PAGES*PAGE_SIZE));
      munmap(dummy, FLUSH_PAGES*PAGE_SIZE);
   }
}

/* ----------------------------------------------------------------------- */

static void wfRx_lock(int i)
{
   pthread_mutex_lock(&wfRx[i].mutex);
}

/* ----------------------------------------------------------------------- */

static void wfRx_unlock(int i)
{
   pthread_mutex_unlock(&wfRx[i].mutex);
}

/* ----------------------------------------------------------------------- */

static void spinWhileStarting(void)
{
   while (runState == PI_STARTING)
   {
      if (piCores == 1) myGpioDelay(1000);
      else flushMemory();
   }
}

/* ----------------------------------------------------------------------- */

static int myDoCommand(uintptr_t *p, unsigned bufSize, char *buf)
{
   int res, i, j;
   uint32_t mask;
   uint32_t tmp1, tmp2, tmp3, tmp4, tmp5;
   gpioPulse_t *pulse;
   bsc_xfer_t xfer;
   int masked;
   res = 0;

   switch (p[0])
   {
      case PI_CMD_BC1:
         mask = gpioMask;

         res = gpioWrite_Bits_0_31_Clear(p[1]&mask);

         if ((mask | p[1]) != mask)
         {
            DBG(DBG_USER,
               "gpioWrite_Bits_0_31_Clear: bad bits %08"PRIXPTR" (permissions %08X)",
               p[1], mask);
            res = PI_SOME_PERMITTED;
         }
         break;

      case PI_CMD_BC2:
         mask = gpioMask>>32;

         res = gpioWrite_Bits_32_53_Clear(p[1]&mask);

         if ((mask | p[1]) != mask)
         {
            DBG(DBG_USER,
               "gpioWrite_Bits_32_53_Clear: bad bits %08"PRIXPTR" (permissions %08X)",
               p[1], mask);
            res = PI_SOME_PERMITTED;
         }
         break;

      case PI_CMD_BI2CC:
         res = bbI2CClose(p[1]);
         break;

      case PI_CMD_BI2CO:
         memcpy(&p[4], buf, 4);
         res = bbI2COpen(p[1], p[2], p[4]);
         break;

      case PI_CMD_BI2CZ:
         /* use half buffer for write, half buffer for read */
         if (p[3] > (bufSize/2)) p[3] = bufSize/2;
         res = bbI2CZip(p[1], buf, p[3], buf+(bufSize/2), bufSize/2);
         if (res > 0)
         {
            memcpy(buf, buf+(bufSize/2), res);
         }
         break;

      case PI_CMD_BSCX:
         xfer.control = p[1];
         if (p[3] > BSC_FIFO_SIZE) p[3] = BSC_FIFO_SIZE;
         xfer.txCnt = p[3];
         if (p[3]) memcpy(&xfer.txBuf, buf, p[3]);
         res = bscXfer(&xfer);
         if (res >= 0)
         {
            memcpy(buf, &res, 4);
            res = 4 + xfer.rxCnt;
            if (res > 4) memcpy(buf+4, &xfer.rxBuf, res-4);
         }
         break;

      case PI_CMD_BSPIO:

         memcpy(&tmp1, buf+ 0, 4); // MISO
         memcpy(&tmp2, buf+ 4, 4); // MOSI
         memcpy(&tmp3, buf+ 8, 4); // SCLK
         memcpy(&tmp4, buf+12, 4); // baud
         memcpy(&tmp5, buf+16, 4); // flags

         if (!myPermit(p[1]))
         {
            DBG(DBG_USER,
               "bbSPIOpen: gpio %"PRIdPTR", no permission to update CS", p[1]);
            res = PI_NOT_PERMITTED;
         }

         if (!myPermit(tmp1))
         {
            DBG(DBG_USER,
               "bbSPIOpen: gpio %d, no permission to update MISO", tmp1);
            res = PI_NOT_PERMITTED;
         }

         if (!myPermit(tmp2))
         {
            DBG(DBG_USER,
               "bbSPIOpen: gpio %d, no permission to update MOSI", tmp2);
            res = PI_NOT_PERMITTED;
         }

         if (!myPermit(tmp3))
         {
            DBG(DBG_USER,
               "bbSPIOpen: gpio %d, no permission to update SCLK", tmp3);
            res = PI_NOT_PERMITTED;
         }

         if (!res) res = bbSPIOpen(p[1], tmp1, tmp2, tmp3, tmp4, tmp5);
         break;

      case PI_CMD_BSPIC:
         res = bbSPIClose(p[1]);
         break;

      case PI_CMD_BSPIX:
         if (p[3] > bufSize) p[3] = bufSize;
            res = bbSPIXfer(p[1], buf, buf, p[3]);
         break;

      case PI_CMD_BR1: res = gpioRead_Bits_0_31(); break;

      case PI_CMD_BR2: res = gpioRead_Bits_32_53(); break;

      case PI_CMD_BS1:
         mask = gpioMask;

         res = gpioWrite_Bits_0_31_Set(p[1]&mask);

         if ((mask | p[1]) != mask)
         {
            DBG(DBG_USER,
               "gpioWrite_Bits_0_31_Set: bad bits %08"PRIXPTR" (permissions %08X)",
               p[1], mask);
            res = PI_SOME_PERMITTED;
         }
         break;

      case PI_CMD_BS2:
         mask = gpioMask>>32;

         res = gpioWrite_Bits_32_53_Set(p[1]&mask);

         if ((mask | p[1]) != mask)
         {
            DBG(DBG_USER,
               "gpioWrite_Bits_32_53_Set: bad bits %08"PRIXPTR" (permissions %08X)",
               p[1], mask);
            res = PI_SOME_PERMITTED;
         }
         break;

      case PI_CMD_CF1:
         res = gpioCustom1(p[1], p[2], buf, p[3]);
         break;

      case PI_CMD_CF2:
         /* a couple of extra precautions for untrusted code */
         if (p[2] > bufSize) p[2] = bufSize;
         res = gpioCustom2(p[1], buf, p[3], buf, p[2]);
         if (res > p[2]) res = p[2];
         break;

      case PI_CMD_CGI: res = gpioCfgGetInternals(); break;

      case PI_CMD_CSI: res = gpioCfgSetInternals(p[1]); break;

      case PI_CMD_EVM: res = eventMonitor(p[1], p[2]); break;

      case PI_CMD_EVT: res = eventTrigger(p[1]); break;

      case PI_CMD_FC: res = fileClose(p[1]); break;

      case PI_CMD_FG:
         res = gpioGlitchFilter(p[1], p[2]);
         break;

      case PI_CMD_FL:
         if (p[1] > bufSize) p[1] = bufSize;
         res = fileList(buf, buf, p[1]);
         break;

      case PI_CMD_FN:
         memcpy(&p[4], buf, 4);
         res = gpioNoiseFilter(p[1], p[2], p[4]);
         break;

      case PI_CMD_FO: res = fileOpen(buf, p[1]); break;

      case PI_CMD_FR:
         if (p[2] > bufSize) p[2] = bufSize;
         res = fileRead(p[1], buf, p[2]);
         break;

      case PI_CMD_FS:
         memcpy(&p[4], buf, 4);
         res = fileSeek(p[1], p[2], p[4]);
         break;

      case PI_CMD_FW: res = fileWrite(p[1], buf, p[3]); break;

      case PI_CMD_GDC: res = gpioGetPWMdutycycle(p[1]); break;

      case PI_CMD_GPW: res = gpioGetServoPulsewidth(p[1]); break;

      case PI_CMD_HC:
         /* special case to allow password in upper byte */
         if (myPermit(p[1]&0xFFFFFF)) res = gpioHardwareClock(p[1], p[2]);
         else
         {
            DBG(DBG_USER,
               "gpioHardwareClock: gpio %"PRIdPTR", no permission to update",
                p[1] & 0xFFFFFF);
            res = PI_NOT_PERMITTED;
         }
         break;

      case PI_CMD_HELP: break;

      case PI_CMD_HP:
         if (myPermit(p[1]))
         {
            memcpy(&p[4], buf, 4);
            res = gpioHardwarePWM(p[1], p[2], p[4]);
         }
         else
         {
            DBG(DBG_USER,
               "gpioHardwarePWM: gpio %"PRIdPTR", no permission to update", p[1]);
            res = PI_NOT_PERMITTED;
         }
         break;

      case PI_CMD_HWVER: res = gpioHardwareRevision(); break;



      case PI_CMD_I2CC: res = i2cClose(p[1]); break;

      case PI_CMD_I2CO:
         memcpy(&p[4], buf, 4);
         res = i2cOpen(p[1], p[2], p[4]);
         break;

      case PI_CMD_I2CPC:
         memcpy(&p[4], buf, 4);
         res = i2cProcessCall(p[1], p[2], p[4]);
         break;

      case PI_CMD_I2CPK:
         res = i2cBlockProcessCall(p[1], p[2], buf, p[3]);
         break;

      case PI_CMD_I2CRB: res = i2cReadByteData(p[1], p[2]); break;

      case PI_CMD_I2CRD:
         if (p[2] > bufSize) p[2] = bufSize;
         res = i2cReadDevice(p[1], buf, p[2]);
         break;

      case PI_CMD_I2CRI:
         memcpy(&p[4], buf, 4);
         res = i2cReadI2CBlockData(p[1], p[2], buf, p[4]);
         break;

      case PI_CMD_I2CRK:
         res = i2cReadBlockData(p[1], p[2], buf);
         break;

      case PI_CMD_I2CRS: res = i2cReadByte(p[1]); break;

      case PI_CMD_I2CRW: res = i2cReadWordData(p[1], p[2]); break;

      case PI_CMD_I2CWB:
         memcpy(&p[4], buf, 4);
         res = i2cWriteByteData(p[1], p[2], p[4]);
         break;

      case PI_CMD_I2CWD:
         res = i2cWriteDevice(p[1], buf, p[3]);
         break;

      case PI_CMD_I2CWI:
         res = i2cWriteI2CBlockData(p[1], p[2], buf, p[3]);
         break;

      case PI_CMD_I2CWK:
         res = i2cWriteBlockData(p[1], p[2], buf, p[3]);
         break;

      case PI_CMD_I2CWQ: res = i2cWriteQuick(p[1], p[2]); break;

      case PI_CMD_I2CWS: res = i2cWriteByte(p[1], p[2]); break;

      case PI_CMD_I2CWW:
         memcpy(&p[4], buf, 4);
         res = i2cWriteWordData(p[1], p[2], p[4]);
         break;

      case PI_CMD_I2CZ:
         /* use half buffer for write, half buffer for read */
         if (p[3] > (bufSize/2)) p[3] = bufSize/2;
         res = i2cZip(p[1], buf, p[3], buf+(bufSize/2), bufSize/2);
         if (res > 0)
         {
            memcpy(buf, buf+(bufSize/2), res);
         }
         break;

      case PI_CMD_MICS:
         if (p[1] <= PI_MAX_MICS_DELAY) myGpioDelay(p[1]);
         else res = PI_BAD_MICS_DELAY;
         break;

      case PI_CMD_MILS:
         if (p[1] <= PI_MAX_MILS_DELAY) myGpioDelay(p[1] * 1000);
         else res = PI_BAD_MILS_DELAY;
         break;

      case PI_CMD_MODEG: res = gpioGetMode(p[1]); break;

      case PI_CMD_MODES:
         if (myPermit(p[1])) res = gpioSetMode(p[1], p[2]);
         else
         {
            DBG(DBG_USER,
               "gpioSetMode: gpio %"PRIdPTR", no permission to update", p[1]);
            res = PI_NOT_PERMITTED;
         }
         break;

      case PI_CMD_NB: res = gpioNotifyBegin(p[1], p[2]); break;

      case PI_CMD_NC: res = gpioNotifyClose(p[1]); break;

      case PI_CMD_NO: res = gpioNotifyOpen();  break;

      case PI_CMD_NP: res = gpioNotifyPause(p[1]); break;

      case PI_CMD_PADG: res = gpioGetPad(p[1]); break;

      case PI_CMD_PADS: res = gpioSetPad(p[1], p[2]); break;

      case PI_CMD_PFG: res = gpioGetPWMfrequency(p[1]); break;

      case PI_CMD_PFS:
         if (myPermit(p[1])) res = gpioSetPWMfrequency(p[1], p[2]);
         else
         {
            DBG(DBG_USER,
               "gpioSetPWMfrequency: gpio %"PRIdPTR", no permission to update", p[1]);
            res = PI_NOT_PERMITTED;
         }
         break;

      case PI_CMD_PIGPV: res = gpioVersion(); break;

      case PI_CMD_PRG: res = gpioGetPWMrange(p[1]); break;

      case PI_CMD_PROC:
         res = gpioStoreScript(buf);
         break;

      case PI_CMD_PROCD: res = gpioDeleteScript(p[1]); break;

      case PI_CMD_PROCP:
         res = gpioScriptStatus(p[1], (uint32_t *)buf);
         break;

      case PI_CMD_PROCR:
         res = gpioRunScript(p[1], p[3]/4, (uint32_t *)buf);
         break;

      case PI_CMD_PROCS: res = gpioStopScript(p[1]); break;

      case PI_CMD_PROCU:
         res = gpioUpdateScript(p[1], p[3]/4, (uint32_t *)buf);
         break;

      case PI_CMD_PRRG: res = gpioGetPWMrealRange(p[1]); break;

      case PI_CMD_PRS:
         if (myPermit(p[1])) res = gpioSetPWMrange(p[1], p[2]);
         else
         {
            DBG(DBG_USER,
               "gpioSetPWMrange: gpio %"PRIdPTR", no permission to update", p[1]);
            res = PI_NOT_PERMITTED;
         }
         break;

      case PI_CMD_PUD:
         if (myPermit(p[1])) res = gpioSetPullUpDown(p[1], p[2]);
         else
         {
            DBG(DBG_USER,
               "gpioSetPullUpDown: gpio %"PRIdPTR", no permission to update", p[1]);
            res = PI_NOT_PERMITTED;
         }
         break;

      case PI_CMD_PWM:
         if (myPermit(p[1])) res = gpioPWM(p[1], p[2]);
         else
         {
            DBG(DBG_USER, "gpioPWM: gpio %"PRIdPTR", no permission to update", p[1]);
            res = PI_NOT_PERMITTED;
         }
         break;

      case PI_CMD_READ: res = gpioRead(p[1]); break;

      case PI_CMD_SERVO:
         if (myPermit(p[1])) res = gpioServo(p[1], p[2]);
         else
         {
            DBG(DBG_USER,
               "gpioServo: gpio %"PRIdPTR", no permission to update", p[1]);
            res = PI_NOT_PERMITTED;
         }
         break;



      case PI_CMD_SERRB: res = serReadByte(p[1]); break;

      case PI_CMD_SERWB: res = serWriteByte(p[1], p[2]); break;

      case PI_CMD_SERC: res = serClose(p[1]); break;

      case PI_CMD_SERDA: res = serDataAvailable(p[1]); break;

      case PI_CMD_SERO: res = serOpen(buf, p[1], p[2]); break;

      case PI_CMD_SERR:
         if (p[2] > bufSize) p[2] = bufSize;
         res = serRead(p[1], buf, p[2]);
         break;

      case PI_CMD_SERW: res = serWrite(p[1], buf, p[3]); break;


      case PI_CMD_SHELL:
          res = shell(buf, buf+p[1]+1);
          break;


      case PI_CMD_SLR:
         if (p[2] > bufSize) p[2] = bufSize;
         res = gpioSerialRead(p[1], buf, p[2]);
         break;

      case PI_CMD_SLRC: res = gpioSerialReadClose(p[1]); break;

      case PI_CMD_SLRO:
         memcpy(&p[4], buf, 4);
         res = gpioSerialReadOpen(p[1], p[2], p[4]); break;

      case PI_CMD_SLRI: res = gpioSerialReadInvert(p[1], p[2]); break;

      case PI_CMD_SPIC:
         res = spiClose(p[1]);
         break;

      case PI_CMD_SPIO:
         memcpy(&p[4], buf, 4);
         res = spiOpen(p[1], p[2], p[4]);
         break;

      case PI_CMD_SPIR:
         if (p[2] > bufSize) p[2] = bufSize;
         res = spiRead(p[1], buf, p[2]);
         break;

      case PI_CMD_SPIW:
         if (p[3] > bufSize) p[3] = bufSize;
         res = spiWrite(p[1], buf, p[3]);
         break;

      case PI_CMD_SPIX:
         if (p[3] > bufSize) p[3] = bufSize;
         res = spiXfer(p[1], buf, buf, p[3]);
         break;

      case PI_CMD_TICK: res = gpioTick(); break;

      case PI_CMD_TRIG:
         if (myPermit(p[1]))
         {
            memcpy(&p[4], buf, 4);
            res = gpioTrigger(p[1], p[2], p[4]);
         }
         else
         {
            DBG(DBG_USER,
               "gpioTrigger: gpio %"PRIdPTR", no permission to update", p[1]);
            res = PI_NOT_PERMITTED;
         }
         break;

      case PI_CMD_WDOG: res = gpioSetWatchdog(p[1], p[2]); break;

      case PI_CMD_WRITE:
         if (myPermit(p[1])) res = gpioWrite(p[1], p[2]);
         else
         {
            DBG(DBG_USER, "gpioWrite: gpio %"PRIdPTR", no permission to update", p[1]);
            res = PI_NOT_PERMITTED;
         }
         break;



      case PI_CMD_WVAG:

         /* need to mask off any non permitted gpios */

         mask = gpioMask;
         pulse = (gpioPulse_t *)buf;
         j = p[3]/sizeof(gpioPulse_t);
         masked = 0;

         for (i=0; i<j; i++)
         {
            tmp1 = pulse[i].gpioOn & mask;
            if (tmp1 != pulse[i].gpioOn)
            {
               pulse[i].gpioOn = tmp1;
               masked = 1;
            }

            tmp1 = pulse[i].gpioOff & mask;
            if (tmp1 != pulse[i].gpioOff)
            {
               pulse[i].gpioOff = tmp1;
               masked = 1;
            }
            DBG(DBG_SCRIPT, "on=%X off=%X delay=%d",
               pulse[i].gpioOn, pulse[i].gpioOff, pulse[i].usDelay);
         }

         res = gpioWaveAddGeneric(j, pulse);

         /* report permission error unless another error occurred */
         if (masked && (res >= 0)) res = PI_SOME_PERMITTED;

         break;

      case PI_CMD_WVAS:
         if (myPermit(p[1]))
         {
            memcpy(&tmp1, buf, 4);   /* databits */
            memcpy(&tmp2, buf+4, 4); /* stophalfbits */
            memcpy(&tmp3, buf+8, 4); /* offset */
            res = gpioWaveAddSerial
               (p[1], p[2], tmp1, tmp2, tmp3, p[3]-12, buf+12);
         }
         else
         {
            DBG(
               DBG_USER,
               "gpioWaveAddSerial: gpio %"PRIdPTR", no permission to update", p[1]);
            res = PI_NOT_PERMITTED;
         }
         break;

      case PI_CMD_WVBSY: res = gpioWaveTxBusy(); break;

      case PI_CMD_WVCHA:
         if (p[3] > bufSize) p[3] = bufSize;
         res = gpioWaveChain(buf, p[3]);
         break;


      case PI_CMD_WVCLR: res = gpioWaveClear(); break;

      case PI_CMD_WVCRE: res = gpioWaveCreate(); break;

      case PI_CMD_WVCAP:
         /* Make WVCAP variadic */
         if (p[3] == 4)
         {
            memcpy(&tmp3, buf, 4); /* percent TOOL */
            res = gpioWaveCreatePad(p[1], p[2], tmp3); /* rawWaveAdd* usage */
            break;
         }
         if (p[2] && p[3]==0)
         {
            res = gpioWaveCreatePad(p[1], p[2], 0);
            break;
         }
         if (p[2]==0 && p[3]==0)
         {
            res = gpioWaveCreatePad(p[1], p[1], 0); /* typical usage */
            break;
         }
         res = PI_BAD_WAVE_ID; // FIX?
         break;

      case PI_CMD_WVDEL: res = gpioWaveDelete(p[1]); break;

      case PI_CMD_WVGO:  res = gpioWaveTxStart(PI_WAVE_MODE_ONE_SHOT); break;

      case PI_CMD_WVGOR: res = gpioWaveTxStart(PI_WAVE_MODE_REPEAT); break;

      case PI_CMD_WVHLT: res = gpioWaveTxStop(); break;

      case PI_CMD_WVNEW: res = gpioWaveAddNew(); break;

      case PI_CMD_WVSC:
         switch(p[1])
         {
            case 0: res = gpioWaveGetCbs();     break;
            case 1: res = gpioWaveGetHighCbs(); break;
            case 2: res = gpioWaveGetMaxCbs();  break;
            default: res = PI_BAD_WVSC_COMMND;
         }
         break;

      case PI_CMD_WVSM:
         switch(p[1])
         {
            case 0: res = gpioWaveGetMicros();     break;
            case 1: res = gpioWaveGetHighMicros(); break;
            case 2: res = gpioWaveGetMaxMicros();  break;
            default: res = PI_BAD_WVSM_COMMND;
         }
         break;

      case PI_CMD_WVSP:
         switch(p[1])
         {
            case 0: res = gpioWaveGetPulses();     break;
            case 1: res = gpioWaveGetHighPulses(); break;
            case 2: res = gpioWaveGetMaxPulses();  break;
            default: res = PI_BAD_WVSP_COMMND;
         }
         break;

      case PI_CMD_WVTAT: res = gpioWaveTxAt(); break;

      case PI_CMD_WVTX:
         res = gpioWaveTxSend(p[1], PI_WAVE_MODE_ONE_SHOT); break;

      case PI_CMD_WVTXM:
         res = gpioWaveTxSend(p[1], p[2]); break;

      case PI_CMD_WVTXR:
         res = gpioWaveTxSend(p[1], PI_WAVE_MODE_REPEAT); break;

      default:
         res = PI_UNKNOWN_COMMAND;
         break;
   }

   return res;
}

/* ----------------------------------------------------------------------- */

static void mySetGpioOff(unsigned gpio, int pos)
{
   int page, slot;

   myOffPageSlot(pos, &page, &slot);

   dmaIVirt[page]->gpioOff[slot] |= (1<<gpio);
}

/* ----------------------------------------------------------------------- */

static void myClearGpioOff(unsigned gpio, int pos)
{
   int page, slot;

   myOffPageSlot(pos, &page, &slot);

   dmaIVirt[page]->gpioOff[slot] &= ~(1<<gpio);
}

/* ----------------------------------------------------------------------- */

static void mySetGpioOn(unsigned gpio, int pos)
{
   int page, slot;

   page = pos/ON_PER_IPAGE;
   slot = pos%ON_PER_IPAGE;

   dmaIVirt[page]->gpioOn[slot] |= (1<<gpio);
}

/* ----------------------------------------------------------------------- */

static void myClearGpioOn(unsigned gpio, int pos)
{
   int page, slot;

   page = pos/ON_PER_IPAGE;
   slot = pos%ON_PER_IPAGE;

   dmaIVirt[page]->gpioOn[slot] &= ~(1<<gpio);
}

/* ----------------------------------------------------------------------- */

static void myGpioSetPwm(unsigned gpio, int oldVal, int newVal)
{
   int switchGpioOff;
   int newOff, oldOff, realRange, cycles, i;
   int deferOff, deferRng;

   DBG(DBG_INTERNAL,
      "myGpioSetPwm %d from %d to %d", gpio, oldVal, newVal);

   switchGpioOff = 0;

   realRange = pwmRealRange[gpioInfo[gpio].freqIdx];

   cycles    = pwmCycles   [gpioInfo[gpio].freqIdx];

   newOff = (newVal * realRange)/gpioInfo[gpio].range;
   oldOff = (oldVal * realRange)/gpioInfo[gpio].range;

   deferOff = gpioInfo[gpio].deferOff;
   deferRng = gpioInfo[gpio].deferRng;

   if (gpioInfo[gpio].deferOff)
   {
      for (i=0; i<SUPERLEVEL; i+=deferRng)
      {
         myClearGpioOff(gpio, i+deferOff);
      }
      gpioInfo[gpio].deferOff = 0;
   }

   if (newOff != oldOff)
   {
      if (newOff && oldOff)                      /* PWM CHANGE */
      {
         if (newOff != realRange)
         {
            for (i=0; i<SUPERLEVEL; i+=realRange) mySetGpioOff(gpio, i+newOff);
         }

         if (newOff > oldOff)
         {
            for (i=0; i<SUPERLEVEL; i+=realRange)
               myClearGpioOff(gpio, i+oldOff);
         }
         else
         {
            gpioInfo[gpio].deferOff = oldOff;
            gpioInfo[gpio].deferRng = realRange;
         }
      }
      else if (newOff)                           /* PWM START */
      {
         if (newOff != realRange)
         {
            for (i=0; i<SUPERLEVEL; i+=realRange) mySetGpioOff(gpio, i+newOff);
         }

         /* schedule new gpio on */

         for (i=0; i<SUPERCYCLE; i+=cycles) mySetGpioOn(gpio, i);
      }
      else                                       /* PWM STOP */
      {
         /* deschedule gpio on */

         for (i=0; i<SUPERCYCLE; i+=cycles)
            myClearGpioOn(gpio, i);

         for (i=0; i<SUPERLEVEL; i+=realRange)
            myClearGpioOff(gpio, i+oldOff);

         switchGpioOff = 1;
      }

      if (switchGpioOff)
      {
         *(gpioReg + GPCLR0) = (1<<gpio);
         *(gpioReg + GPCLR0) = (1<<gpio);
      }
   }
}

/* ----------------------------------------------------------------------- */

static void myGpioSetServo(unsigned gpio, int oldVal, int newVal)
{
   int newOff, oldOff, realRange, cycles, i;
   int deferOff, deferRng;

   DBG(DBG_INTERNAL,
      "myGpioSetServo %d from %d to %d", gpio, oldVal, newVal);

   realRange = pwmRealRange[clkCfg[gpioCfg.clockMicros].servoIdx];
   cycles    = pwmCycles   [clkCfg[gpioCfg.clockMicros].servoIdx];

   newOff = (newVal * realRange)/20000;
   oldOff = (oldVal * realRange)/20000;

   deferOff = gpioInfo[gpio].deferOff;
   deferRng = gpioInfo[gpio].deferRng;

   if (gpioInfo[gpio].deferOff)
   {
      for (i=0; i<SUPERLEVEL; i+=deferRng)
      {
         myClearGpioOff(gpio, i+deferOff);
      }
      gpioInfo[gpio].deferOff = 0;
   }

   if (newOff != oldOff)
   {
      if (newOff && oldOff)                       /* SERVO CHANGE */
      {
         for (i=0; i<SUPERLEVEL; i+=realRange)
            mySetGpioOff(gpio, i+newOff);

         if (newOff > oldOff)
         {
            for (i=0; i<SUPERLEVEL; i+=realRange)
               myClearGpioOff(gpio, i+oldOff);
         }
         else
         {
            gpioInfo[gpio].deferOff = oldOff;
            gpioInfo[gpio].deferRng = realRange;
         }
      }
      else if (newOff)                            /* SERVO START */
      {
         for (i=0; i<SUPERLEVEL; i+=realRange)
            mySetGpioOff(gpio, i+newOff);

         /* schedule new gpio on */

         for (i=0; i<SUPERCYCLE; i+=cycles) mySetGpioOn(gpio, i);
      }
      else                                        /* SERVO STOP */
      {
         /* deschedule gpio on */

         for (i=0; i<SUPERCYCLE; i+=cycles)
            myClearGpioOn(gpio, i);

         /* if in pulse then delay for the last cycle to complete */

         if (myGpioRead(gpio)) myGpioDelay(PI_MAX_SERVO_PULSEWIDTH);

         /* deschedule gpio off */

         for (i=0; i<SUPERLEVEL; i+=realRange)
            myClearGpioOff(gpio, i+oldOff);
      }
   }
}

/* ======================================================================= */

/*
https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
*/

static int mbCreate(char *dev)
{
   /* <0 error */

   unlink(dev);

   return mknod(dev, S_IFCHR|0600, makedev(MB_DEV_MAJOR, 0));
}

static int mbOpen(void)
{
   /* <0 error */

   int fd;

   fd = open(MB_DEV1, 0);

   if (fd < 0)
   {
      mbCreate(MB_DEV2);
      fd = open(MB_DEV2, 0);
   }
   return fd;
}

static void mbClose(int fd)
{
   close(fd);
}

static int mbProperty(int fd, void *buf)
{
   return ioctl(fd, MB_IOCTL, buf);
}

static unsigned mbAllocateMemory(
   int fd, unsigned size, unsigned align, unsigned flags)
{
   int i=1;
   unsigned p[32];

   p[i++] = MB_PROCESS_REQUEST;
   p[i++] = MB_ALLOCATE_MEMORY_TAG;
   p[i++] = 12;
   p[i++] = 12;
   p[i++] = size;
   p[i++] = align;
   p[i++] = flags;
   p[i++] = MB_END_TAG;
   p[0] = i*sizeof(*p);

   mbProperty(fd, p);

   return p[5];
}

static unsigned mbLockMemory(int fd, unsigned handle)
{
   int i=1;
   unsigned p[32];

   p[i++] = MB_PROCESS_REQUEST;
   p[i++] = MB_LOCK_MEMORY_TAG;
   p[i++] = 4;
   p[i++] = 4;
   p[i++] = handle;
   p[i++] = MB_END_TAG;
   p[0] = i*sizeof(*p);

   mbProperty(fd, p);

   return p[5];
}

static unsigned mbUnlockMemory(int fd, unsigned handle)
{
   int i=1;
   unsigned p[32];

   p[i++] = MB_PROCESS_REQUEST;
   p[i++] = MB_UNLOCK_MEMORY_TAG;
   p[i++] = 4;
   p[i++] = 4;
   p[i++] = handle;
   p[i++] = MB_END_TAG;
   p[0] = i*sizeof(*p);

   mbProperty(fd, p);

   return p[5];
}

static unsigned mbReleaseMemory(int fd, unsigned handle)
{
   int i=1;
   unsigned p[32];

   p[i++] = MB_PROCESS_REQUEST;
   p[i++] = MB_RELEASE_MEMORY_TAG;
   p[i++] = 4;
   p[i++] = 4;
   p[i++] = handle;
   p[i++] = MB_END_TAG;
   p[0] = i*sizeof(*p);

   mbProperty(fd, p);

   return p[5];
}

static void *mbMapMem(unsigned base, unsigned size)
{
   void *mem = MAP_FAILED;

   mem = mmap(0, size, PROT_READ|PROT_WRITE, MAP_SHARED, fdMem, base);

   return mem;
}

static int mbUnmapMem(void *addr, unsigned size)
{
   /* 0 okay, -1 fail */
   return munmap(addr, size);
}

static void mbDMAFree(DMAMem_t *DMAMemP)
{
   if (DMAMemP->handle)
   {
      mbUnmapMem(DMAMemP->virtual_addr, DMAMemP->size);
      mbUnlockMemory(fdMbox, DMAMemP->handle);
      mbReleaseMemory(fdMbox, DMAMemP->handle);
      DMAMemP->handle = 0;
   }
}

static int mbDMAAlloc(DMAMem_t *DMAMemP, unsigned size, uint32_t pi_mem_flag)
{
   DMAMemP->size = size;

   DMAMemP->handle =
      mbAllocateMemory(fdMbox, size, PAGE_SIZE, pi_mem_flag);

   if (DMAMemP->handle)
   {
      DMAMemP->bus_addr = mbLockMemory(fdMbox, DMAMemP->handle);

      DMAMemP->virtual_addr =
         mbMapMem(BUS_TO_PHYS(DMAMemP->bus_addr), size);

      return 1;
   }
   return 0;
}


/* ======================================================================= */

rawCbs_t * rawWaveCBAdr(int cbNum)
{
   int page, slot;

   page = cbNum/CBS_PER_OPAGE;
   slot = cbNum%CBS_PER_OPAGE;

   return &dmaOVirt[page]->cb[slot];
}


/* ----------------------------------------------------------------------- */

static uint32_t waveCbPOadr(int pos)
{
   int page, slot;

   page = pos/CBS_PER_OPAGE;
   slot = pos%CBS_PER_OPAGE;

   //cast twice to suppress compiler warning, I belive this cast is ok
   //because dmaOBus contains bus addresses, not virtual addresses.
   return (uint32_t)(uintptr_t) &dmaOBus[page]->cb[slot];
}

/* ----------------------------------------------------------------------- */

static void waveOOLPageSlot(int pos, int *page, int *slot)
{
   *page = pos/OOL_PER_OPAGE;
   *slot = pos%OOL_PER_OPAGE;
}


/* ----------------------------------------------------------------------- */

static void waveSetOOL(int pos, uint32_t OOL)
{
   int page, slot;

   waveOOLPageSlot(pos, &page, &slot);

   dmaOVirt[page]->OOL[slot] = OOL;
}

/* ----------------------------------------------------------------------- */

static uint32_t waveOOLPOadr(int pos)
{
   int page, slot;

   waveOOLPageSlot(pos, &page, &slot);

   //cast twice to suppress compiler warning, I belive this cast is ok
   //because dmaOBus contains bus addresses, not virtual addresses.
   return (uint32_t)(uintptr_t) &dmaOBus[page]->OOL[slot];
}


/* ----------------------------------------------------------------------- */

static void waveBitDelay
   (unsigned baud, unsigned bits, unsigned stops, unsigned *bitDelay)
{
   unsigned fullBit, last, diff, t, i;

   /* scaled 1000X */

   fullBit = 1000000000 / baud;
   last = 0;

   for (i=0; i<=bits; i++)
   {
      t = (((i+1)*fullBit)+500)/1000;
      diff = t - last;
      last = t;
      bitDelay[i] = diff;
   }

   t = (((bits+1)*fullBit) + ((stops*fullBit)/2) + 500)/1000;
   diff = t - last;
   bitDelay[i] = diff;
}

static int waveDelayCBs(uint32_t delay)
{
   uint32_t cbs;

   if (!delay) return 0;
   if (gpioCfg.DMAsecondaryChannel < DMA_LITE_FIRST) return 1;
   cbs = BPD * delay / DMA_LITE_MAX;
   if  ((BPD * delay) % DMA_LITE_MAX) cbs++;
   return cbs;
}

/* ----------------------------------------------------------------------- */

static void waveCBsOOLs(int *numCBs, int *numBOOLs, int *numTOOLs)
{
   int numCB=0, numBOOL=0, numTOOL=0;

   unsigned i;

   unsigned numWaves;

   rawWave_t *waves;

   numWaves = wfc[wfcur];
   waves    = wf [wfcur];

   /* delay cb at start of DMA */

   numCB++;

   for (i=0; i<numWaves; i++)
   {
      if (waves[i].gpioOn)                 {numBOOL++;}
      if (waves[i].gpioOff)                {numBOOL++;}
      if (waves[i].gpioOn || waves[i].gpioOff) {numCB++;}
      if (waves[i].flags & WAVE_FLAG_READ) {numCB++; numTOOL++;}
      if (waves[i].flags & WAVE_FLAG_TICK) {numCB++; numTOOL++;}

      numCB += waveDelayCBs(waves[i].usDelay);
   }

   *numCBs   = numCB;
   *numBOOLs = numBOOL;
   *numTOOLs = numTOOL;
}

/* ----------------------------------------------------------------------- */

static int wave2Cbs(unsigned wave_mode, int *CB, int *BOOL, int *TOOL,
                    int numCB, int numBOOL, int numTOOL)
{
   int botCB=*CB, botOOL=*BOOL, topOOL=*TOOL;

   int status, s_stride;

   rawCbs_t *p=NULL;

   unsigned i, repeatCB;

   unsigned numWaves;

   unsigned delayCBs, dcb;

   uint32_t delayLeft;

   rawWave_t * waves;

   numWaves = wfc[wfcur];
   waves    = wf [wfcur];

   /* add delay cb at start of DMA */

   p = rawWaveCBAdr(botCB++);

   /* use the secondary clock */

   if (gpioCfg.clockPeriph != PI_CLOCK_PCM)
   {
      p->info = NORMAL_DMA | TIMED_DMA(2);
      p->dst  = PCM_TIMER;
   }
   else
   {
      p->info = NORMAL_DMA | TIMED_DMA(5);
      p->dst  = PWM_TIMER;
   }

   //cast twice to suppress compiler warning, I belive this cast is ok
   //because dmaOBus contains bus addresses, not virtual addresses.
   p->src    = (uint32_t)(uintptr_t) (&dmaOBus[0]->periphData);
   p->length = BPD * 20 / PI_WF_MICROS; /* 20 micros delay */
   p->next   = waveCbPOadr(botCB);

   repeatCB = botCB;

   for (i=0; i<numWaves; i++)
   {
      if (waves[i].gpioOn && waves[i].gpioOff)
      /* Use 2-beat burst */
      {
         p = rawWaveCBAdr(botCB++);

         p->info   = TWO_BEAT_DMA;
         p->src    = waveOOLPOadr(botOOL);
         waveSetOOL(botOOL++, waves[i].gpioOn);
         s_stride = waveOOLPOadr(botOOL) - p->src;
         waveSetOOL(botOOL++, waves[i].gpioOff);
         p->dst    = ((GPIO_BASE + (GPSET0*4)) & 0x00ffffff) | PI_PERI_BUS;
         p->length = (2<<16) + 4;         // 2 transfers of 4 bytes each
         p->stride = (12<<16) + s_stride; // d_stride = (GPCLR0-GPSET0)*4 = 12
         p->next   = waveCbPOadr(botCB);
      }
      if (waves[i].gpioOn && !waves[i].gpioOff)
      {
         waveSetOOL(botOOL, waves[i].gpioOn);

         p = rawWaveCBAdr(botCB++);

         p->info   = NORMAL_DMA;
         p->src    = waveOOLPOadr(botOOL++);
         p->dst    = ((GPIO_BASE + (GPSET0*4)) & 0x00ffffff) | PI_PERI_BUS;
         p->length = 4;
         p->next   = waveCbPOadr(botCB);
      }
      if (waves[i].gpioOff && !waves[i].gpioOn)
      {
         waveSetOOL(botOOL, waves[i].gpioOff);

         p = rawWaveCBAdr(botCB++);

         p->info   = NORMAL_DMA;
         p->src    = waveOOLPOadr(botOOL++);
         p->dst    = ((GPIO_BASE + (GPCLR0*4)) & 0x00ffffff) | PI_PERI_BUS;
         p->length = 4;
         p->next   = waveCbPOadr(botCB);
      }
      if (waves[i].flags & WAVE_FLAG_READ)
      {
         p = rawWaveCBAdr(botCB++);

         p->info   = NORMAL_DMA;
         p->src    = ((GPIO_BASE + (GPLEV0*4)) & 0x00ffffff) | PI_PERI_BUS;
         p->dst    = waveOOLPOadr(--topOOL);
         p->length = 4;
         p->next   = waveCbPOadr(botCB);
      }

      if (waves[i].flags & WAVE_FLAG_TICK)
      {
         p = rawWaveCBAdr(botCB++);

         p->info   = NORMAL_DMA;
         p->src    = ((SYST_BASE + (SYST_CLO*4)) & 0x00ffffff) | PI_PERI_BUS;
         p->dst    = waveOOLPOadr(--topOOL);
         p->length = 4;
         p->next   = waveCbPOadr(botCB);
      }

      if (waves[i].usDelay)
      {
         delayLeft = waves[i].usDelay;

         delayCBs = waveDelayCBs(delayLeft);

         for (dcb=0; dcb<delayCBs; dcb++)
         {
            p = rawWaveCBAdr(botCB++);

            /* use the secondary clock */

            if (gpioCfg.clockPeriph != PI_CLOCK_PCM)
            {
               p->info = NORMAL_DMA | TIMED_DMA(2);
               p->dst  = PCM_TIMER;
            }
            else
            {
               p->info = NORMAL_DMA | TIMED_DMA(5);
               p->dst  = PWM_TIMER;
            }

            //cast twice to suppress compiler warning, I belive this cast is ok
            //because dmaOBus contains bus addresses, not virtual addresses.
            p->src = (uint32_t)(uintptr_t) (&dmaOBus[0]->periphData);

            p->length = BPD * delayLeft / PI_WF_MICROS;

            if ((gpioCfg.DMAsecondaryChannel >= DMA_LITE_FIRST) &&
                (p->length > DMA_LITE_MAX))
            {
               p->length = DMA_LITE_MAX;
            }

            delayLeft -= (p->length / BPD);

            p->next = waveCbPOadr(botCB);
         }
      }
   }

   if (numCB)
   {
      /* Pad the wave */

      botCB = *CB + numCB - 1;
      botOOL = *BOOL + numBOOL - 1;
      topOOL = *TOOL - numTOOL;

      /* Link the last CB to end of wave */

      p->next = waveCbPOadr(botCB);

      /* Insert sentinel CB at end of DMA */

      p = rawWaveCBAdr(botCB++);
      p->info   = NORMAL_DMA | DMA_DEST_IGNORE;
      p->src    = waveOOLPOadr(botOOL++);
      p->dst    = ((GPIO_BASE + (GPSET0*4)) & 0x00ffffff) | PI_PERI_BUS;
      p->length = 4;
      p->next   = 0;
   }

   if (p != NULL)
   {
      if (wave_mode == PI_WAVE_MODE_ONE_SHOT)
           p->next = 0;
      else p->next = waveCbPOadr(repeatCB);
   }

   status = botCB - *CB;

   *CB   = botCB;
   *BOOL = botOOL;
   *TOOL = topOOL;

   return status;
}

/* ----------------------------------------------------------------------- */

static void waveRxSerial(wfRx_t *w, int level, uint32_t tick)
{
   int diffTicks, lastLevel;
   int newWritePos;

   level = level ^ w->s.invert;

   if (w->s.bit >= 0)
   {
      diffTicks = tick - w->s.startBitTick;

      if (level != PI_TIMEOUT)
      {
         w->s.level = level;
         lastLevel = !level;
      }
      else lastLevel = w->s.level;

      while ((w->s.bit <= w->s.dataBits) &&
             (diffTicks > (w->s.nextBitDiff/1000)))
      {
         if (w->s.bit)
         {
            if (lastLevel) w->s.data |= (1<<(w->s.bit-1));
         }
         else w->s.data = 0;

         ++(w->s.bit);

         w->s.nextBitDiff += w->s.fullBit;
      }

      if (w->s.bit > w->s.dataBits)
      {
         memcpy(w->s.buf + w->s.writePos, &w->s.data, w->s.bytes);

         /* don't let writePos catch readPos */

         newWritePos = (w->s.writePos + w->s.bytes) % (w->s.bufSize);

         if (newWritePos != w->s.readPos) w->s.writePos = newWritePos;

         if (level == 0)
         {
            gpioSetWatchdog(w->gpio, w->s.timeout);
            w->s.bit          = 0;
            w->s.startBitTick = tick;
            w->s.nextBitDiff  = w->s.halfBit;
         }
         else
         {
            w->s.bit = -1;
            gpioSetWatchdog(w->gpio, 0);
         }
      }
   }
   else
   {
      /* start bit if high->low */

      if (level == 0)
      {
         gpioSetWatchdog(w->gpio, w->s.timeout);
         w->s.level        = 0;
         w->s.bit          = 0;
         w->s.startBitTick = tick;
         w->s.nextBitDiff  = w->s.halfBit;
      }
   }
}


/* ----------------------------------------------------------------------- */

static void waveRxBit(int gpio, int level, uint32_t tick)
{
   switch (wfRx[gpio].mode)
   {
      case PI_WFRX_SERIAL:
         waveRxSerial(&wfRx[gpio], level, tick);
   }
}


/* ----------------------------------------------------------------------- */

int rawWaveAddGeneric(unsigned numIn1, rawWave_t *in1)
{
   unsigned inPos1=0, inPos2=0, outPos=0, level = NUM_WAVE_OOL;

   unsigned cbs=0;

   unsigned numIn2, numOut;

   uint32_t tNow, tNext1, tNext2, tDelay, tMax;

   rawWave_t *in2, *out;

   numIn2 = wfc[wfcur];
   in2    = wf[wfcur];

   numOut = PI_WAVE_MAX_PULSES;
   out   = wf[1-wfcur];

   tNow = 0;
   tMax = 0;

   if (!numIn1) tNext1 = -1; else tNext1 = 0;
   if (!numIn2) tNext2 = -1; else tNext2 = 0;

   while (((inPos1<numIn1) || (inPos2<numIn2)) && (outPos<numOut))
   {
      if (tNext1 < tNext2)
      {
         /* pulse 1 due */

         if (tNow < tNext1)
         {
            /* extend previous delay */
            out[outPos-1].usDelay += (tNext1 - tNow);
            tNow = tNext1;
         }

         out[outPos].gpioOn  = in1[inPos1].gpioOn;
         out[outPos].gpioOff = in1[inPos1].gpioOff;
         out[outPos].flags   = in1[inPos1].flags;

         tNext1 = tNow + in1[inPos1].usDelay; ++inPos1;
         if (tMax < tNext1) tMax = tNext1;
      }
      else if (tNext2 < tNext1)
      {
         /* pulse 2 due */

         if (tNow < tNext2)
         {
            /* extend previous delay */
            out[outPos-1].usDelay += (tNext2 - tNow);
            tNow = tNext2;
         }

         out[outPos].gpioOn  = in2[inPos2].gpioOn;
         out[outPos].gpioOff = in2[inPos2].gpioOff;
         out[outPos].flags   = in2[inPos2].flags;

         tNext2 = tNow + in2[inPos2].usDelay; ++inPos2;
         if (tMax < tNext2) tMax = tNext2;
      }
      else
      {
         /* pulse 1 and 2 both due */

         if (tNow < tNext1)
         {
            /* extend previous delay */
            out[outPos-1].usDelay += (tNext1 - tNow);
            tNow = tNext1;
         }

         out[outPos].gpioOn  = in1[inPos1].gpioOn  | in2[inPos2].gpioOn;
         out[outPos].gpioOff = in1[inPos1].gpioOff | in2[inPos2].gpioOff;
         out[outPos].flags   = in1[inPos1].flags   | in2[inPos2].flags;

         tNext1 = tNow + in1[inPos1].usDelay; ++inPos1;
         tNext2 = tNow + in2[inPos2].usDelay; ++inPos2;
         if (tMax < tNext1) tMax = tNext1;
         if (tMax < tNext2) tMax = tNext2;
      }

      if (tNext1 <= tNext2) { tDelay = tNext1 - tNow; tNow = tNext1; }
      else                  { tDelay = tNext2 - tNow; tNow = tNext2; }

      out[outPos].usDelay = tDelay;

      cbs += waveDelayCBs(tDelay);

      if (out[outPos].gpioOn || out[outPos].gpioOff) cbs++;

      if (out[outPos].flags & WAVE_FLAG_READ)
      {
         cbs++; /* one cb if read */
         --level;
      }

      if (out[outPos].flags & WAVE_FLAG_TICK)
      {
         cbs++; /* one cb if tick */
         --level;
      }

      outPos++;

      if (inPos1 >= numIn1) tNext1 = -1;
      if (inPos2 >= numIn2) tNext2 = -1;

   }

   if (tNow < tMax)
   {
      /* extend previous delay */
      out[outPos-1].usDelay += (tMax - tNow);
      tNow = tMax;
   }

   if ((outPos < numOut) && (outPos < level))
   {
      wfStats.micros = tNow;

      if (tNow > wfStats.highMicros) wfStats.highMicros = tNow;

      wfStats.pulses = outPos;

      if (outPos > wfStats.highPulses) wfStats.highPulses = outPos;

      wfStats.cbs    = cbs;

      if (cbs > wfStats.highCbs) wfStats.highCbs = cbs;

      wfc[1-wfcur] = outPos;
      wfcur = 1 - wfcur;

      return outPos;
   }
   else return PI_TOO_MANY_PULSES;
}

/* ======================================================================= */

int i2cWriteQuick(unsigned handle, unsigned bit)
{
   int status;

   DBG(DBG_USER, "handle=%d bit=%d", handle, bit);

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if ((i2cInfo[handle].funcs & PI_I2C_FUNC_SMBUS_QUICK) == 0)
      SOFT_ERROR(PI_BAD_SMBUS_CMD, "SMBUS command not supported by driver");

   if (bit > 1)
      SOFT_ERROR(PI_BAD_PARAM, "bad bit (%d)", bit);

   status = my_smbus_access(
      i2cInfo[handle].fd, bit, 0, PI_I2C_SMBUS_QUICK, NULL);

   if (status < 0)
   {
      DBG(DBG_USER, "error=%d (%m)", status);
      return PI_I2C_WRITE_FAILED;
   }

   return status;
}

int i2cReadByte(unsigned handle)
{
   union my_smbus_data data;
   int status;

   DBG(DBG_USER, "handle=%d", handle);

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if ((i2cInfo[handle].funcs & PI_I2C_FUNC_SMBUS_READ_BYTE) == 0)
      SOFT_ERROR(PI_BAD_SMBUS_CMD, "SMBUS command not supported by driver");

   status = my_smbus_access(
      i2cInfo[handle].fd, PI_I2C_SMBUS_READ, 0, PI_I2C_SMBUS_BYTE, &data);

   if (status < 0)
   {
      DBG(DBG_USER, "error=%d (%m)", status);
      return PI_I2C_READ_FAILED;
   }

   return 0xFF & data.byte;
}


int i2cWriteByte(unsigned handle, unsigned bVal)
{
   int status;

   DBG(DBG_USER, "handle=%d bVal=%d", handle, bVal);

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if ((i2cInfo[handle].funcs & PI_I2C_FUNC_SMBUS_WRITE_BYTE) == 0)
      SOFT_ERROR(PI_BAD_SMBUS_CMD, "SMBUS command not supported by driver");

   if (bVal > 0xFF)
      SOFT_ERROR(PI_BAD_PARAM, "bad bVal (%d)", bVal);

   status = my_smbus_access(
            i2cInfo[handle].fd,
            PI_I2C_SMBUS_WRITE,
            bVal,
            PI_I2C_SMBUS_BYTE,
            NULL);

   if (status < 0)
   {
      DBG(DBG_USER, "error=%d (%m)", status);
      return PI_I2C_WRITE_FAILED;
   }

   return status;
}


int i2cReadByteData(unsigned handle, unsigned reg)
{
   union my_smbus_data data;
   int status;

   DBG(DBG_USER, "handle=%d reg=%d", handle, reg);

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if ((i2cInfo[handle].funcs & PI_I2C_FUNC_SMBUS_READ_BYTE_DATA) == 0)
      SOFT_ERROR(PI_BAD_SMBUS_CMD, "SMBUS command not supported by driver");

   if (reg > 0xFF)
      SOFT_ERROR(PI_BAD_PARAM, "bad reg (%d)", reg);

   status = my_smbus_access(i2cInfo[handle].fd,
            PI_I2C_SMBUS_READ, reg, PI_I2C_SMBUS_BYTE_DATA, &data);

   if (status < 0)
   {
      DBG(DBG_USER, "error=%d (%m)", status);
      return PI_I2C_READ_FAILED;
   }

   return 0xFF & data.byte;
}


int i2cWriteByteData(unsigned handle, unsigned reg, unsigned bVal)
{
   union my_smbus_data data;

   int status;

   DBG(DBG_USER, "handle=%d reg=%d bVal=%d", handle, reg, bVal);

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if ((i2cInfo[handle].funcs & PI_I2C_FUNC_SMBUS_WRITE_BYTE_DATA) == 0)
      SOFT_ERROR(PI_BAD_SMBUS_CMD, "SMBUS command not supported by driver");

   if (reg > 0xFF)
      SOFT_ERROR(PI_BAD_PARAM, "bad reg (%d)", reg);

   if (bVal > 0xFF)
      SOFT_ERROR(PI_BAD_PARAM, "bad bVal (%d)", bVal);

   data.byte = bVal;

   status = my_smbus_access(
            i2cInfo[handle].fd,
            PI_I2C_SMBUS_WRITE,
            reg,
            PI_I2C_SMBUS_BYTE_DATA,
            &data);

   if (status < 0)
   {
      DBG(DBG_USER, "error=%d (%m)", status);
      return PI_I2C_WRITE_FAILED;
   }

   return status;
}


int i2cReadWordData(unsigned handle, unsigned reg)
{
   union my_smbus_data data;
   int status;

   DBG(DBG_USER, "handle=%d reg=%d", handle, reg);

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if ((i2cInfo[handle].funcs & PI_I2C_FUNC_SMBUS_READ_WORD_DATA) == 0)
      SOFT_ERROR(PI_BAD_SMBUS_CMD, "SMBUS command not supported by driver");

   if (reg > 0xFF)
      SOFT_ERROR(PI_BAD_PARAM, "bad reg (%d)", reg);

   status = (my_smbus_access(
      i2cInfo[handle].fd,
      PI_I2C_SMBUS_READ,
      reg,
      PI_I2C_SMBUS_WORD_DATA,
      &data));

   if (status < 0)
   {
      DBG(DBG_USER, "error=%d (%m)", status);
      return PI_I2C_READ_FAILED;
   }

   return 0xFFFF & data.word;
}


int i2cWriteWordData(unsigned handle, unsigned reg, unsigned wVal)
{
   union my_smbus_data data;

   int status;

   DBG(DBG_USER, "handle=%d reg=%d wVal=%d", handle, reg, wVal);

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if ((i2cInfo[handle].funcs & PI_I2C_FUNC_SMBUS_WRITE_WORD_DATA) == 0)
      SOFT_ERROR(PI_BAD_SMBUS_CMD, "SMBUS command not supported by driver");

   if (reg > 0xFF)
      SOFT_ERROR(PI_BAD_PARAM, "bad reg (%d)", reg);

   if (wVal > 0xFFFF)
      SOFT_ERROR(PI_BAD_PARAM, "bad wVal (%d)", wVal);

   data.word = wVal;

   status = my_smbus_access(
            i2cInfo[handle].fd,
            PI_I2C_SMBUS_WRITE,
            reg,
            PI_I2C_SMBUS_WORD_DATA,
            &data);

   if (status < 0)
   {
      DBG(DBG_USER, "error=%d (%m)", status);
      return PI_I2C_WRITE_FAILED;
   }

   return status;
}


int i2cProcessCall(unsigned handle, unsigned reg, unsigned wVal)
{
   union my_smbus_data data;
   int status;

   DBG(DBG_USER, "handle=%d reg=%d wVal=%d", handle, reg, wVal);

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if ((i2cInfo[handle].funcs & PI_I2C_FUNC_SMBUS_PROC_CALL) == 0)
      SOFT_ERROR(PI_BAD_SMBUS_CMD, "SMBUS command not supported by driver");

   if (reg > 0xFF)
      SOFT_ERROR(PI_BAD_PARAM, "bad reg (%d)", reg);

   if (wVal > 0xFFFF)
      SOFT_ERROR(PI_BAD_PARAM, "bad wVal (%d)", wVal);

   data.word = wVal;

   status = (my_smbus_access(
      i2cInfo[handle].fd,
      PI_I2C_SMBUS_WRITE,
      reg, PI_I2C_SMBUS_PROC_CALL,
      &data));

   if (status < 0)
   {
      DBG(DBG_USER, "error=%d (%m)", status);
      return PI_I2C_READ_FAILED;
   }

   return 0xFFFF & data.word;
}


int i2cReadBlockData(unsigned handle, unsigned reg, char *buf)
{
   union my_smbus_data data;

   int i, status;

   DBG(DBG_USER, "handle=%d reg=%d buf=%08"PRIXPTR, handle, reg, (uintptr_t)buf);

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if ((i2cInfo[handle].funcs & PI_I2C_FUNC_SMBUS_READ_BLOCK_DATA) == 0)
      SOFT_ERROR(PI_BAD_SMBUS_CMD, "SMBUS command not supported by driver");

   if (reg > 0xFF)
      SOFT_ERROR(PI_BAD_PARAM, "bad reg (%d)", reg);

   status = (my_smbus_access(
      i2cInfo[handle].fd,
      PI_I2C_SMBUS_READ,
      reg,
      PI_I2C_SMBUS_BLOCK_DATA,
      &data));

   if (status < 0)
   {
      DBG(DBG_USER, "error=%d (%m)", status);
      return PI_I2C_READ_FAILED;
   }
   else
   {
      if (data.block[0] <= PI_I2C_SMBUS_BLOCK_MAX)
      {
         for (i=0; i<data.block[0]; i++) buf[i] = data.block[i+1];
         return data.block[0];
      }
      else return PI_I2C_READ_FAILED;
   }
}


int i2cWriteBlockData(
   unsigned handle, unsigned reg, char *buf, unsigned count)
{
   union my_smbus_data data;

   int i, status;

   DBG(DBG_USER, "handle=%d reg=%d count=%d [%s]",
      handle, reg, count, myBuf2Str(count, buf));

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if ((i2cInfo[handle].funcs & PI_I2C_FUNC_SMBUS_WRITE_BLOCK_DATA) == 0)
      SOFT_ERROR(PI_BAD_SMBUS_CMD, "SMBUS command not supported by driver");

   if (reg > 0xFF)
      SOFT_ERROR(PI_BAD_PARAM, "bad reg (%d)", reg);

   if ((count < 1) || (count > 32))
      SOFT_ERROR(PI_BAD_PARAM, "bad count (%d)", count);

   for (i=1; i<=count; i++) data.block[i] = buf[i-1];
   data.block[0] = count;

   status = my_smbus_access(
            i2cInfo[handle].fd,
            PI_I2C_SMBUS_WRITE,
            reg,
            PI_I2C_SMBUS_BLOCK_DATA,
            &data);

   if (status < 0)
   {
      DBG(DBG_USER, "error=%d (%m)", status);
      return PI_I2C_WRITE_FAILED;
   }

   return status;
}


int i2cBlockProcessCall(
   unsigned handle, unsigned reg, char *buf, unsigned count)
{
   union my_smbus_data data;

   int i, status;

   DBG(DBG_USER, "handle=%d reg=%d count=%d [%s]",
      handle, reg, count, myBuf2Str(count, buf));

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if ((i2cInfo[handle].funcs & PI_I2C_FUNC_SMBUS_PROC_CALL) == 0)
      SOFT_ERROR(PI_BAD_SMBUS_CMD, "SMBUS command not supported by driver");

   if (reg > 0xFF)
      SOFT_ERROR(PI_BAD_PARAM, "bad reg (%d)", reg);

   if ((count < 1) || (count > 32))
      SOFT_ERROR(PI_BAD_PARAM, "bad count (%d)", count);

   for (i=1; i<=count; i++) data.block[i] = buf[i-1];
   data.block[0] = count;

   status = (my_smbus_access(
      i2cInfo[handle].fd, PI_I2C_SMBUS_WRITE, reg,
      PI_I2C_SMBUS_BLOCK_PROC_CALL, &data));

   if (status < 0)
   {
      DBG(DBG_USER, "error=%d (%m)", status);
      return PI_I2C_READ_FAILED;
   }
   else
   {
      if (data.block[0] <= PI_I2C_SMBUS_BLOCK_MAX)
      {
         for (i=0; i<data.block[0]; i++) buf[i] = data.block[i+1];
         return data.block[0];
      }
      else return PI_I2C_READ_FAILED;
   }
}


int i2cReadI2CBlockData(
   unsigned handle, unsigned reg, char *buf, unsigned count)
{
   union my_smbus_data data;

   int i, status;
   uint32_t size;

   DBG(DBG_USER, "handle=%d reg=%d count=%d buf=%08"PRIXPTR,
      handle, reg, count, (uintptr_t)buf);

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if ((i2cInfo[handle].funcs & PI_I2C_FUNC_SMBUS_READ_I2C_BLOCK) == 0)
      SOFT_ERROR(PI_BAD_SMBUS_CMD, "SMBUS command not supported by driver");

   if (reg > 0xFF)
      SOFT_ERROR(PI_BAD_PARAM, "bad reg (%d)", reg);

   if ((count < 1) || (count > 32))
      SOFT_ERROR(PI_BAD_PARAM, "bad count (%d)", count);

   if (count == 32)
      size = PI_I2C_SMBUS_I2C_BLOCK_BROKEN;
   else
      size = PI_I2C_SMBUS_I2C_BLOCK_DATA;

   data.block[0] = count;

   status = (my_smbus_access(
      i2cInfo[handle].fd, PI_I2C_SMBUS_READ, reg, size, &data));

   if (status < 0)
   {
      DBG(DBG_USER, "error=%d (%m)", status);
      return PI_I2C_READ_FAILED;
   }
   else
   {
      if (data.block[0] <= PI_I2C_SMBUS_I2C_BLOCK_MAX)
      {
         for (i=0; i<data.block[0]; i++) buf[i] = data.block[i+1];
         return data.block[0];
      }
      else return PI_I2C_READ_FAILED;
   }
}


int i2cWriteI2CBlockData(
   unsigned handle, unsigned reg, char *buf, unsigned count)
{
   union my_smbus_data data;

   int i, status;

   DBG(DBG_USER, "handle=%d reg=%d count=%d [%s]",
      handle, reg, count, myBuf2Str(count, buf));

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if ((i2cInfo[handle].funcs & PI_I2C_FUNC_SMBUS_WRITE_I2C_BLOCK) == 0)
      SOFT_ERROR(PI_BAD_SMBUS_CMD, "SMBUS command not supported by driver");

   if (reg > 0xFF)
      SOFT_ERROR(PI_BAD_PARAM, "bad reg (%d)", reg);

   if ((count < 1) || (count > 32))
      SOFT_ERROR(PI_BAD_PARAM, "bad count (%d)", count);

   for (i=1; i<=count; i++) data.block[i] = buf[i-1];

   data.block[0] = count;

   status = my_smbus_access(
            i2cInfo[handle].fd,
            PI_I2C_SMBUS_WRITE,
            reg,
            PI_I2C_SMBUS_I2C_BLOCK_BROKEN,
            &data);

   if (status < 0)
   {
      DBG(DBG_USER, "error=%d (%m)", status);
      return PI_I2C_WRITE_FAILED;
   }

   return status;
}

int i2cWriteDevice(unsigned handle, char *buf, unsigned count)
{
   int bytes;

   DBG(DBG_USER, "handle=%d count=%d [%s]",
      handle, count, myBuf2Str(count, buf));

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if ((count < 1) || (count > PI_MAX_I2C_DEVICE_COUNT))
      SOFT_ERROR(PI_BAD_PARAM, "bad count (%d)", count);

   bytes = write(i2cInfo[handle].fd, buf, count);

   if (bytes != count)
   {
      DBG(DBG_USER, "error=%d (%m)", bytes);
      return PI_I2C_WRITE_FAILED;
   }

   return 0;
}

int i2cReadDevice(unsigned handle, char *buf, unsigned count)
{
   int bytes;

   DBG(DBG_USER, "handle=%d count=%d buf=%08"PRIXPTR,
      handle, count, (uintptr_t)buf);

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if ((count < 1) || (count > PI_MAX_I2C_DEVICE_COUNT))
      SOFT_ERROR(PI_BAD_PARAM, "bad count (%d)", count);

   bytes = read(i2cInfo[handle].fd, buf, count);

   if (bytes != count)
   {
      DBG(DBG_USER, "error=%d (%m)", bytes);
      return PI_I2C_READ_FAILED;
   }

   return bytes;
}

int i2cOpen(unsigned i2cBus, unsigned i2cAddr, unsigned i2cFlags)
{
   static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
   char dev[32];
   int i, slot, fd;
   uint32_t funcs;

   DBG(DBG_USER, "i2cBus=%d i2cAddr=%d flags=0x%X",
      i2cBus, i2cAddr, i2cFlags);

   CHECK_INITED;

   if (i2cAddr > PI_MAX_I2C_ADDR)
      SOFT_ERROR(PI_BAD_I2C_ADDR, "bad I2C address (%d)", i2cAddr);

   if (i2cFlags)
      SOFT_ERROR(PI_BAD_FLAGS, "bad flags (0x%X)", i2cFlags);

   slot = -1;

   pthread_mutex_lock(&mutex);

   for (i=0; i<PI_I2C_SLOTS; i++)
   {
      if (i2cInfo[i].state == PI_I2C_CLOSED)
      {
         slot = i;
         i2cInfo[slot].state = PI_I2C_RESERVED;
         break;
      }
   }

   pthread_mutex_unlock(&mutex);

   if (slot < 0) SOFT_ERROR(PI_NO_HANDLE, "no I2C handles");

   sprintf(dev, "/dev/i2c-%d", i2cBus);

   if ((fd = open(dev, O_RDWR)) < 0)
   {
      /* try a modprobe */

      if (system("/sbin/modprobe i2c_dev") == -1) { /* ignore errors */}
      if (system("/sbin/modprobe i2c_bcm2835") == -1) { /* ignore errors */}

      myGpioDelay(100000);

      if ((fd = open(dev, O_RDWR)) < 0)
      {
         i2cInfo[slot].state = PI_I2C_CLOSED;
         return PI_BAD_I2C_BUS;
      }
   }

   if (ioctl(fd, PI_I2C_SLAVE, i2cAddr) < 0)
   {
      close(fd);
      i2cInfo[slot].state = PI_I2C_CLOSED;
      return PI_I2C_OPEN_FAILED;
   }

   if (ioctl(fd, PI_I2C_FUNCS, &funcs) < 0)
   {
      funcs = -1; /* assume all smbus commands allowed */
   }

   i2cInfo[slot].fd = fd;
   i2cInfo[slot].addr = i2cAddr;
   i2cInfo[slot].flags = i2cFlags;
   i2cInfo[slot].funcs = funcs;
   i2cInfo[slot].state = PI_I2C_OPENED;

   return slot;
}

int i2cClose(unsigned handle)
{
   DBG(DBG_USER, "handle=%d", handle);

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].fd >= 0) close(i2cInfo[handle].fd);

   i2cInfo[handle].fd = -1;
   i2cInfo[handle].state = PI_I2C_CLOSED;

   return 0;
}

void i2cSwitchCombined(int setting)
{
   int fd;

   DBG(DBG_USER, "setting=%d", setting);

   fd = open(PI_I2C_COMBINED, O_WRONLY);

   if (fd >= 0)
   {
      if (setting)
      {
         if (write(fd, "1\n", 2) == -1) { /* ignore errors */ }
      }
      else
      {
         if (write(fd, "0\n", 2) == -1) { /* ignore errors */ }
      }

      close(fd);
   }
}

int i2cSegments(unsigned handle, pi_i2c_msg_t *segs, unsigned numSegs)
{
   int retval;
   my_i2c_rdwr_ioctl_data_t rdwr;

   DBG(DBG_USER, "handle=%d", handle);

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (segs == NULL)
      SOFT_ERROR(PI_BAD_POINTER, "null segments");

   if (numSegs > PI_I2C_RDRW_IOCTL_MAX_MSGS)
      SOFT_ERROR(PI_TOO_MANY_SEGS, "too many segments (%d)", numSegs);

   rdwr.msgs = segs;
   rdwr.nmsgs = numSegs;

   retval = ioctl(i2cInfo[handle].fd, PI_I2C_RDWR, &rdwr);

   if (retval >= 0) return retval;
   else             return PI_BAD_I2C_SEG;
}

int i2cZip(
   unsigned handle,
   char *inBuf, unsigned inLen, char *outBuf, unsigned outLen)
{
   int numSegs, inPos, outPos, status, bytes, flags, addr;
   int esc, setesc;
   pi_i2c_msg_t segs[PI_I2C_RDRW_IOCTL_MAX_MSGS];

   DBG(DBG_USER, "handle=%d inBuf=%s outBuf=%08"PRIXPTR" len=%d",
      handle, myBuf2Str(inLen, (char *)inBuf), (uintptr_t)outBuf, outLen);

   CHECK_INITED;

   if (handle >= PI_I2C_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (i2cInfo[handle].state != PI_I2C_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (!inBuf || !inLen)
      SOFT_ERROR(PI_BAD_POINTER, "input buffer can't be NULL");

   if (!outBuf && outLen)
      SOFT_ERROR(PI_BAD_POINTER, "output buffer can't be NULL");

   numSegs = 0;

   inPos = 0;
   outPos = 0;
   status = 0;

   addr = i2cInfo[handle].addr;
   flags = 0;
   esc = 0;
   setesc = 0;

   while (!status && (inPos < inLen))
   {
      DBG(DBG_INTERNAL, "status=%d inpos=%d inlen=%d cmd=%d addr=%d flags=%x",
         status, inPos, inLen, inBuf[inPos], addr, flags);

      switch (inBuf[inPos++])
      {
         case PI_I2C_END:
            status = 1;
            break;

         case PI_I2C_COMBINED_ON:
            /* Run prior transactions before setting combined flag */
            if (numSegs)
            {
               status = i2cSegments(handle, segs, numSegs);
               if (status >= 0) status = 0; /* continue */
               numSegs = 0;
            }
            i2cSwitchCombined(1);
            break;

         case PI_I2C_COMBINED_OFF:
            /* Run prior transactions before clearing combined flag */
            if (numSegs)
            {
               status = i2cSegments(handle, segs, numSegs);
               if (status >= 0) status = 0; /* continue */
               numSegs = 0;
            }
            i2cSwitchCombined(0);
            break;

         case PI_I2C_ADDR:
            addr = myI2CGetPar(inBuf, &inPos, inLen, &esc);
            if (addr < 0) status = PI_BAD_I2C_CMD;
            break;

         case PI_I2C_FLAGS:
            /* cheat to force two byte flags */
            esc = 1;
            flags = myI2CGetPar(inBuf, &inPos, inLen, &esc);
            if (flags < 0) status = PI_BAD_I2C_CMD;
            break;

         case PI_I2C_ESC:
            setesc = 1;
            break;

         case PI_I2C_READ:

            bytes = myI2CGetPar(inBuf, &inPos, inLen, &esc);

            if (bytes >= 0)
            {
               if ((bytes + outPos) < outLen)
               {
                  segs[numSegs].addr = addr;
                  segs[numSegs].flags = (flags|1);
                  segs[numSegs].len = bytes;
                  segs[numSegs].buf = (uint8_t *)(outBuf + outPos);
                  outPos += bytes;
                  numSegs++;
                  if (numSegs >= PI_I2C_RDRW_IOCTL_MAX_MSGS)
                  {
                     status = i2cSegments(handle, segs, numSegs);
                     if (status >= 0) status = 0; /* continue */
                     numSegs = 0;
                  }
               }
               else status = PI_BAD_I2C_RLEN;
            }
            else status = PI_BAD_I2C_RLEN;
            break;

         case PI_I2C_WRITE:

            bytes = myI2CGetPar(inBuf, &inPos, inLen, &esc);

            if (bytes >= 0)
            {
               if ((bytes + inPos) < inLen)
               {
                  segs[numSegs].addr = addr;
                  segs[numSegs].flags = (flags&0xfffe);
                  segs[numSegs].len = bytes;
                  segs[numSegs].buf = (uint8_t *)(inBuf + inPos);
                  inPos += bytes;
                  numSegs++;
                  if (numSegs >= PI_I2C_RDRW_IOCTL_MAX_MSGS)
                  {
                     status = i2cSegments(handle, segs, numSegs);
                     if (status >= 0) status = 0; /* continue */
                     numSegs = 0;
                  }
               }
               else status = PI_BAD_I2C_WLEN;
            }
            else status = PI_BAD_I2C_WLEN;
            break;

         default:
            status = PI_BAD_I2C_CMD;
      }

      if (setesc) esc = 1; else esc = 0;

      setesc = 0;
   }

   if (status >= 0)
   {
      if (numSegs) status = i2cSegments(handle, segs, numSegs);
   }

   if (status >= 0) status = outPos;

   return status;
}

/* ======================================================================= */

/*SPI */

static uint32_t _spiTXBits(char *buf, int pos, int bitlen, int msbf)
{
   uint32_t bits=0;

   if (buf)
   {
      if      (bitlen <=  8) bits = *((( uint8_t*)buf)+pos);
      else if (bitlen <= 16) bits = *(((uint16_t*)buf)+pos);
      else                   bits = *(((uint32_t*)buf)+pos);

      if (msbf) bits <<= (32-bitlen);
   }

   return bits;
}

static void _spiRXBits(
   char *buf, int pos, int bitlen, int msbf, uint32_t bits)
{
   if (buf)
   {
      if (!msbf) bits >>= (32-bitlen);

      if      (bitlen <=  8) *((( uint8_t*)buf)+pos) = bits;
      else if (bitlen <= 16) *(((uint16_t*)buf)+pos) = bits;
      else                   *(((uint32_t*)buf)+pos) = bits;
   }
}

static void spiACS(int channel, int on)
{
   int gpio;

   switch (channel)
   {
       case  0: gpio = PI_ASPI_CE0; break;
       case  1: gpio = PI_ASPI_CE1; break;
       default: gpio = PI_ASPI_CE2; break;
   }
   myGpioWrite(gpio, on);
}

static void spiGoA(
   unsigned speed,    /* bits per second */
   uint32_t flags,    /* flags           */
   char     *txBuf,   /* tx buffer       */
   char     *rxBuf,   /* rx buffer       */
   unsigned count)    /* number of bytes */
{
   int cs;
   char bit_ir[4] = {1, 0, 0, 1}; /* read on rising edge */
   char bit_or[4] = {0, 1, 1, 0}; /* write on rising edge */
   char bit_ic[4] = {0, 0, 1, 1}; /* invert clock */

   int mode, bitlen, txmsbf, rxmsbf, channel;
   unsigned txCnt=0;
   unsigned rxCnt=0;
   uint32_t spiDefaults;
   uint32_t statusReg;
   int txFull, rxEmpty;

   channel = PI_SPI_FLAGS_GET_CHANNEL(flags);
   mode   =  PI_SPI_FLAGS_GET_MODE   (flags);

   bitlen =  PI_SPI_FLAGS_GET_BITLEN (flags);

   if (!bitlen) bitlen = 8;

   /* correct count for word size */

   if (bitlen >  8) count /= 2;
   if (bitlen > 16) count /= 2;

   txmsbf = !PI_SPI_FLAGS_GET_TX_LSB (flags);
   rxmsbf = !PI_SPI_FLAGS_GET_RX_LSB (flags);

   cs = PI_SPI_FLAGS_GET_CSPOLS(flags) & (1<<channel);

   spiDefaults = AUXSPI_CNTL0_SPEED((125000000/speed)-1)|
                 AUXSPI_CNTL0_IN_RISING(bit_ir[mode])  |
                 AUXSPI_CNTL0_OUT_RISING(bit_or[mode]) |
                 AUXSPI_CNTL0_INVERT_CLK(bit_ic[mode]) |
                 AUXSPI_CNTL0_MSB_FIRST(txmsbf)        |
                 AUXSPI_CNTL0_SHIFT_LEN(bitlen);

   if (!count)
   {
      auxReg[AUX_SPI0_CNTL0_REG] =
         AUXSPI_CNTL0_ENABLE | AUXSPI_CNTL0_CLR_FIFOS;

      myGpioDelay(10);

      auxReg[AUX_SPI0_CNTL0_REG] = AUXSPI_CNTL0_ENABLE  | spiDefaults;

      auxReg[AUX_SPI0_CNTL1_REG] = AUXSPI_CNTL1_MSB_FIRST(rxmsbf);

      return;
   }

   auxReg[AUX_SPI0_CNTL0_REG] = AUXSPI_CNTL0_ENABLE  | spiDefaults;

   auxReg[AUX_SPI0_CNTL1_REG] = AUXSPI_CNTL1_MSB_FIRST(rxmsbf);

   spiACS(channel, cs);

   while ((txCnt < count) || (rxCnt < count))
   {
      statusReg = auxReg[AUX_SPI0_STAT_REG];

      rxEmpty = statusReg & AUXSPI_STAT_RX_EMPTY;

      txFull = (((statusReg>>28)&15) > 2);

      if (rxCnt < count)
      {
         if (!rxEmpty)
         {
            _spiRXBits(
               rxBuf, rxCnt++, bitlen, rxmsbf, auxReg[AUX_SPI0_IO_REG]);
         }
      }

      if (txCnt < count)
      {
         if (!txFull)
         {
            if (txCnt != (count-1))
            {
               auxReg[AUX_SPI0_TX_HOLD] =
                  _spiTXBits(txBuf, txCnt++, bitlen, txmsbf);
            }
            else
            {
               auxReg[AUX_SPI0_IO_REG] =
                  _spiTXBits(txBuf, txCnt++, bitlen, txmsbf);
            }
         }
      }
   }

   while ((auxReg[AUX_SPI0_STAT_REG] & AUXSPI_STAT_BUSY)) ;

   spiACS(channel, !cs);
}

static void spiGoS(
   unsigned speed,
   uint32_t flags,
   char     *txBuf,
   char     *rxBuf,
   unsigned count)
{
   unsigned txCnt=0;
   unsigned rxCnt=0;
   unsigned cnt, cnt4w, cnt3w;
   uint32_t spiDefaults;
   unsigned mode, channel, cspol, cspols, flag3w, ren3w;

   channel = PI_SPI_FLAGS_GET_CHANNEL(flags);
   mode   =  PI_SPI_FLAGS_GET_MODE   (flags);
   cspols =  PI_SPI_FLAGS_GET_CSPOLS(flags);
   cspol  =  (cspols>>channel) & 1;
   flag3w =  PI_SPI_FLAGS_GET_3WIRE(flags);
   ren3w =   PI_SPI_FLAGS_GET_3WREN(flags);

   spiDefaults = SPI_CS_MODE(mode)     |
                 SPI_CS_CSPOLS(cspols) |
                 SPI_CS_CS(channel)    |
                 SPI_CS_CSPOL(cspol)   |
                 SPI_CS_CLEAR(3);

   spiReg[SPI_DLEN] = 2; /* undocumented, stops inter-byte gap */

   spiReg[SPI_CS] = spiDefaults; /* stop */

   if (!count) return;

   if (flag3w)
   {
      if (ren3w < count)
      {
         cnt4w = ren3w;
         cnt3w = count - ren3w;
      }
      else
      {
         cnt4w = count;
         cnt3w = 0;
      }
   }
   else
   {
      cnt4w = count;
      cnt3w = 0;
   }

   spiReg[SPI_CLK] = 250000000/speed;

   spiReg[SPI_CS] = spiDefaults | SPI_CS_TA; /* start */

   cnt = cnt4w;

   while((txCnt < cnt) || (rxCnt < cnt))
   {
      while((rxCnt < cnt) && ((spiReg[SPI_CS] & SPI_CS_RXD)))
      {
         if (rxBuf) rxBuf[rxCnt] = spiReg[SPI_FIFO];
         else       spi_dummy    = spiReg[SPI_FIFO];
         rxCnt++;
      }

      while((txCnt < cnt) && ((spiReg[SPI_CS] & SPI_CS_TXD)))
      {
         if (txBuf) spiReg[SPI_FIFO] = txBuf[txCnt];
         else       spiReg[SPI_FIFO] = 0;
         txCnt++;
      }
   }

   while (!(spiReg[SPI_CS] & SPI_CS_DONE)) ;

   /* now switch to 3-wire bus */

   cnt += cnt3w;

   spiReg[SPI_CS] |= SPI_CS_REN;

   while((txCnt < cnt) || (rxCnt < cnt))
   {
      while((rxCnt < cnt) && ((spiReg[SPI_CS] & SPI_CS_RXD)))
      {
         if (rxBuf) rxBuf[rxCnt] = spiReg[SPI_FIFO];
         else       spi_dummy    = spiReg[SPI_FIFO];
         rxCnt++;
      }

      while((txCnt < cnt) && ((spiReg[SPI_CS] & SPI_CS_TXD)))
      {
         if (txBuf) spiReg[SPI_FIFO] = txBuf[txCnt];
         else       spiReg[SPI_FIFO] = 0;
         txCnt++;
      }
   }

   while (!(spiReg[SPI_CS] & SPI_CS_DONE)) ;

   spiReg[SPI_CS] = spiDefaults; /* stop */
}

static void spiGo(
   unsigned speed,
   uint32_t flags,
   char     *txBuf,
   char     *rxBuf,
   unsigned count)
{
   static pthread_mutex_t main_mutex = PTHREAD_MUTEX_INITIALIZER;
   static pthread_mutex_t aux_mutex = PTHREAD_MUTEX_INITIALIZER;

   if (PI_SPI_FLAGS_GET_AUX_SPI(flags))
   {
      pthread_mutex_lock(&aux_mutex);
      spiGoA(speed, flags, txBuf, rxBuf, count);
      pthread_mutex_unlock(&aux_mutex);
   }
   else
   {
      pthread_mutex_lock(&main_mutex);
      spiGoS(speed, flags, txBuf, rxBuf, count);
      pthread_mutex_unlock(&main_mutex);
   }
}

static int spiAnyOpen(uint32_t flags)
{
   int i, aux;

   aux = PI_SPI_FLAGS_GET_AUX_SPI(flags);

   for (i=0; i<PI_SPI_SLOTS; i++)
   {
      if ((spiInfo[i].state == PI_SPI_OPENED) &&
         (PI_SPI_FLAGS_GET_AUX_SPI(spiInfo[i].flags) == aux))
            return 1;
   }
   return 0;
}

static void spiInit(uint32_t flags)
{
   uint32_t resvd, cspols;

   resvd  = PI_SPI_FLAGS_GET_RESVD(flags);
   cspols = PI_SPI_FLAGS_GET_CSPOLS(flags);

   if (PI_SPI_FLAGS_GET_AUX_SPI(flags))
   {
      /* enable module and access to registers */

      auxReg[AUX_ENABLES] |= AUXENB_SPI1;

      /* save original state */

      old_mode_ace0  = gpioGetMode(PI_ASPI_CE0);
      old_mode_ace1  = gpioGetMode(PI_ASPI_CE1);
      old_mode_ace2  = gpioGetMode(PI_ASPI_CE2);
      old_mode_asclk = gpioGetMode(PI_ASPI_SCLK);
      old_mode_amiso = gpioGetMode(PI_ASPI_MISO);
      old_mode_amosi = gpioGetMode(PI_ASPI_MOSI);

      old_spi_cntl0 = auxReg[AUX_SPI0_CNTL0_REG];
      old_spi_cntl1 = auxReg[AUX_SPI0_CNTL1_REG];

      /* manually control auxiliary SPI chip selects */

      if (!(resvd&1))
      {
         myGpioSetMode(PI_ASPI_CE0,  PI_OUTPUT);
         myGpioWrite(PI_ASPI_CE0, !(cspols&1));
      }

      if (!(resvd&2))
      {
         myGpioSetMode(PI_ASPI_CE1,  PI_OUTPUT);
         myGpioWrite(PI_ASPI_CE1, !(cspols&2));
      }

      if (!(resvd&4))
      {
         myGpioSetMode(PI_ASPI_CE2,  PI_OUTPUT);
         myGpioWrite(PI_ASPI_CE2, !(cspols&4));
      }

      /* set gpios to SPI mode */

      myGpioSetMode(PI_ASPI_SCLK, PI_ALT4);
      myGpioSetMode(PI_ASPI_MISO, PI_ALT4);
      myGpioSetMode(PI_ASPI_MOSI, PI_ALT4);
   }
   else
   {
      /* save original state */

      old_mode_ce0  = gpioGetMode(PI_SPI_CE0);
      old_mode_ce1  = gpioGetMode(PI_SPI_CE1);
      old_mode_sclk = gpioGetMode(PI_SPI_SCLK);
      old_mode_miso = gpioGetMode(PI_SPI_MISO);
      old_mode_mosi = gpioGetMode(PI_SPI_MOSI);

      old_spi_cs  = spiReg[SPI_CS];
      old_spi_clk = spiReg[SPI_CLK];

      /* set gpios to SPI mode */

      if (!(resvd&1)) myGpioSetMode(PI_SPI_CE0,  PI_ALT0);
      if (!(resvd&2)) myGpioSetMode(PI_SPI_CE1,  PI_ALT0);

      myGpioSetMode(PI_SPI_SCLK, PI_ALT0);
      myGpioSetMode(PI_SPI_MISO, PI_ALT0);
      myGpioSetMode(PI_SPI_MOSI, PI_ALT0);
   }
}

static void spiTerm(uint32_t flags)
{
   int resvd;

   resvd = PI_SPI_FLAGS_GET_RESVD(flags);

   if (PI_SPI_FLAGS_GET_AUX_SPI(flags))
   {
      /* disable module and access to registers */

      auxReg[AUX_ENABLES] &= (~AUXENB_SPI1);

      /* restore original state */

      if (!(resvd&1)) myGpioSetMode(PI_ASPI_CE0,  old_mode_ace0);
      if (!(resvd&2)) myGpioSetMode(PI_ASPI_CE1,  old_mode_ace1);
      if (!(resvd&4)) myGpioSetMode(PI_ASPI_CE2,  old_mode_ace2);

      myGpioSetMode(PI_ASPI_SCLK, old_mode_asclk);
      myGpioSetMode(PI_ASPI_MISO, old_mode_amiso);
      myGpioSetMode(PI_ASPI_MOSI, old_mode_amosi);

      auxReg[AUX_SPI0_CNTL0_REG] = old_spi_cntl0;
      auxReg[AUX_SPI0_CNTL1_REG] = old_spi_cntl1;
   }
   else
   {
      /* restore original state */

      if (!(resvd&1)) myGpioSetMode(PI_SPI_CE0,  old_mode_ce0);
      if (!(resvd&2)) myGpioSetMode(PI_SPI_CE1,  old_mode_ce1);

      myGpioSetMode(PI_SPI_SCLK, old_mode_sclk);
      myGpioSetMode(PI_SPI_MISO, old_mode_miso);
      myGpioSetMode(PI_SPI_MOSI, old_mode_mosi);

      spiReg[SPI_CS]  = old_spi_cs;
      spiReg[SPI_CLK] = old_spi_clk;
   }
}

int spiOpen(unsigned spiChan, unsigned baud, unsigned spiFlags)
{
   static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
   int i, slot;

   DBG(DBG_USER, "spiChan=%d baud=%d spiFlags=0x%X",
      spiChan, baud, spiFlags);

   CHECK_INITED;

   if (PI_SPI_FLAGS_GET_AUX_SPI(spiFlags))
   {
      if (gpioHardwareRevision() < 16)
         SOFT_ERROR(PI_NO_AUX_SPI, "no auxiliary SPI on Pi A or B");

      i = PI_NUM_AUX_SPI_CHANNEL;
   }
   else
      i = PI_NUM_STD_SPI_CHANNEL;

   if (spiChan >= i)
      SOFT_ERROR(PI_BAD_SPI_CHANNEL, "bad spiChan (%d)", spiChan);

   if ((baud < PI_SPI_MIN_BAUD) || (baud > PI_SPI_MAX_BAUD))
      SOFT_ERROR(PI_BAD_SPI_SPEED, "bad baud (%d)", baud);

   if (spiFlags > (1<<22))
      SOFT_ERROR(PI_BAD_FLAGS, "bad spiFlags (0x%X)", spiFlags);

   if (!spiAnyOpen(spiFlags)) /* initialise on first open */
   {
      spiInit(spiFlags);
      spiGo(baud, spiFlags, NULL, NULL, 0);
   }

   slot = -1;

   pthread_mutex_lock(&mutex);

   for (i=0; i<PI_SPI_SLOTS; i++)
   {
      if (spiInfo[i].state == PI_SPI_CLOSED)
      {
         slot = i;
         spiInfo[slot].state = PI_SPI_RESERVED;
         break;
      }
   }

   pthread_mutex_unlock(&mutex);

   if (slot < 0) SOFT_ERROR(PI_NO_HANDLE, "no SPI handles");

   spiInfo[slot].speed = baud;
   spiInfo[slot].flags = spiFlags | PI_SPI_FLAGS_CHANNEL(spiChan);
   spiInfo[slot].state = PI_SPI_OPENED;

   return slot;
}

int spiClose(unsigned handle)
{
   DBG(DBG_USER, "handle=%d", handle);

   CHECK_INITED;

   if (handle >= PI_SPI_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (spiInfo[handle].state != PI_SPI_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   spiInfo[handle].state = PI_SPI_CLOSED;

   if (!spiAnyOpen(spiInfo[handle].flags))
      spiTerm(spiInfo[handle].flags); /* terminate on last close */

   return 0;
}

int spiRead(unsigned handle, char *buf, unsigned count)
{
   DBG(DBG_USER, "handle=%d count=%d [%s]",
      handle, count, myBuf2Str(count, buf));

   CHECK_INITED;

   if (handle >= PI_SPI_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (spiInfo[handle].state != PI_SPI_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (count > PI_MAX_SPI_DEVICE_COUNT)
      SOFT_ERROR(PI_BAD_SPI_COUNT, "bad count (%d)", count);

   spiGo(spiInfo[handle].speed, spiInfo[handle].flags, NULL, buf, count);

   return count;
}

int spiWrite(unsigned handle, char *buf, unsigned count)
{
   DBG(DBG_USER, "handle=%d count=%d [%s]",
      handle, count, myBuf2Str(count, buf));

   CHECK_INITED;

   if (handle >= PI_SPI_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (spiInfo[handle].state != PI_SPI_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (count > PI_MAX_SPI_DEVICE_COUNT)
      SOFT_ERROR(PI_BAD_SPI_COUNT, "bad count (%d)", count);

   spiGo(spiInfo[handle].speed, spiInfo[handle].flags, buf, NULL, count);

   return count;
}

int spiXfer(unsigned handle, char *txBuf, char *rxBuf, unsigned count)
{
   DBG(DBG_USER, "handle=%d count=%d [%s]",
      handle, count, myBuf2Str(count, txBuf));

   CHECK_INITED;

   if (handle >= PI_SPI_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (spiInfo[handle].state != PI_SPI_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (count > PI_MAX_SPI_DEVICE_COUNT)
      SOFT_ERROR(PI_BAD_SPI_COUNT, "bad count (%d)", count);

   spiGo(spiInfo[handle].speed, spiInfo[handle].flags, txBuf, rxBuf, count);

   return count;
}

/* ======================================================================= */


int serOpen(char *tty, unsigned serBaud, unsigned serFlags)
{
   static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
   struct termios new;
   int speed;
   int fd;
   int i, slot;

   DBG(DBG_USER, "tty=%s serBaud=%d serFlags=0x%X", tty, serBaud, serFlags);

   SER_CHECK_INITED;

   if (strncmp("/dev/tty", tty, 8) && strncmp("/dev/serial", tty, 11))
      SOFT_ERROR(PI_BAD_SER_DEVICE, "bad device (%s)", tty);

   switch (serBaud)
   {
      case     50: speed =     B50; break;
      case     75: speed =     B75; break;
      case    110: speed =    B110; break;
      case    134: speed =    B134; break;
      case    150: speed =    B150; break;
      case    200: speed =    B200; break;
      case    300: speed =    B300; break;
      case    600: speed =    B600; break;
      case   1200: speed =   B1200; break;
      case   1800: speed =   B1800; break;
      case   2400: speed =   B2400; break;
      case   4800: speed =   B4800; break;
      case   9600: speed =   B9600; break;
      case  19200: speed =  B19200; break;
      case  38400: speed =  B38400; break;
      case  57600: speed =  B57600; break;
      case 115200: speed = B115200; break;
      case 230400: speed = B230400; break;

      default:
         SOFT_ERROR(PI_BAD_SER_SPEED, "bad speed (%d)", serBaud);
   }

   if (serFlags)
      SOFT_ERROR(PI_BAD_FLAGS, "bad flags (0x%X)", serFlags);

   slot = -1;

   pthread_mutex_lock(&mutex);

   for (i=0; i<PI_SER_SLOTS; i++)
   {
      if (serInfo[i].state == PI_SER_CLOSED)
      {
         slot = i;
         serInfo[slot].state = PI_SER_RESERVED;
         break;
      }
   }

   pthread_mutex_unlock(&mutex);

   if (slot < 0) SOFT_ERROR(PI_NO_HANDLE, "no serial handles");

   if ((fd = open(tty, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
   {
      serInfo[slot].state = PI_SER_CLOSED;
      return PI_SER_OPEN_FAILED;
   }

   tcgetattr(fd, &new);

   cfmakeraw(&new);

   cfsetispeed(&new, speed);
   cfsetospeed(&new, speed);

   new.c_cc [VMIN]  = 0;
   new.c_cc [VTIME] = 0;

   tcflush(fd, TCIFLUSH);
   tcsetattr(fd, TCSANOW, &new);

   //fcntl(fd, F_SETFL, O_RDWR);

   serInfo[slot].fd = fd;
   serInfo[slot].flags = serFlags;
   serInfo[slot].state = PI_SER_OPENED;

   return slot;
}

int serClose(unsigned handle)
{
   DBG(DBG_USER, "handle=%d", handle);

   SER_CHECK_INITED;

   if (handle >= PI_SER_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (serInfo[handle].state != PI_SER_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (serInfo[handle].fd >= 0) close(serInfo[handle].fd);

   serInfo[handle].fd = -1;
   serInfo[handle].state = PI_SER_CLOSED;

   return 0;
}

int serWriteByte(unsigned handle, unsigned bVal)
{
   char c;

   DBG(DBG_USER, "handle=%d bVal=%d", handle, bVal);

   SER_CHECK_INITED;

   if (handle >= PI_SER_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (serInfo[handle].state != PI_SER_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (bVal > 0xFF)
      SOFT_ERROR(PI_BAD_PARAM, "bad parameter (%d)", bVal);

   c = bVal;

   if (write(serInfo[handle].fd, &c, 1) != 1)
      return PI_SER_WRITE_FAILED;
   else
      return 0;
}

int serReadByte(unsigned handle)
{
   int r;
   char x;

   DBG(DBG_USER, "handle=%d", handle);

   SER_CHECK_INITED;

   if (handle >= PI_SER_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (serInfo[handle].state != PI_SER_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   r = read(serInfo[handle].fd, &x, 1);

   if (r == 1)
      return ((int)x) & 0xFF;

   else if (r == 0)
      return PI_SER_READ_NO_DATA;

   else if ((r == -1) && (errno == EAGAIN))
      return PI_SER_READ_NO_DATA;

   else
      return PI_SER_READ_FAILED;
}

int serWrite(unsigned handle, char *buf, unsigned count)
{
   int written=0, wrote=0;

   DBG(DBG_USER, "handle=%d count=%d [%s]",
      handle, count, myBuf2Str(count, buf));

   SER_CHECK_INITED;

   if (handle >= PI_SER_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (serInfo[handle].state != PI_SER_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (!count)
      SOFT_ERROR(PI_BAD_PARAM, "bad count (%d)", count);

   while ((written != count) && (wrote >= 0))
   {
      wrote = write(serInfo[handle].fd, buf+written, count-written);

      if (wrote >= 0)
      {
         written += wrote;

         if (written != count) time_sleep(0.05);
      }
   }

   if (written != count)
      return PI_SER_WRITE_FAILED;
   else
      return 0;
}

int serRead(unsigned handle, char *buf, unsigned count)
{
   int r;

   DBG(DBG_USER, "handle=%d count=%d buf=0x%"PRIXPTR, handle, count, (uintptr_t)buf);

   SER_CHECK_INITED;

   if (handle >= PI_SER_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (serInfo[handle].state != PI_SER_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (!count)
      SOFT_ERROR(PI_BAD_PARAM, "bad count (%d)", count);

   r = read(serInfo[handle].fd, buf, count);

   if (r == -1)
   {
      if (errno == EAGAIN)
         return PI_SER_READ_NO_DATA;
      else
         return PI_SER_READ_FAILED;
   }
   else
   {
      if (r < count) buf[r] = 0;
      return r;
   }
}

int serDataAvailable(unsigned handle)
{
   int result;

   DBG(DBG_USER, "handle=%d", handle);

   SER_CHECK_INITED;

   if (handle >= PI_SER_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (serInfo[handle].state != PI_SER_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (ioctl(serInfo[handle].fd, FIONREAD, &result) == -1) return 0;

   return result;
}

/* ======================================================================= */

static int chooseBestClock
   (clkInf_t *clkInf, unsigned f, unsigned numc, unsigned *cf)
{
   int c, valid;
   double fdiv, offby, best_offby;
   unsigned div, frac;

   valid = 0;
   best_offby = 0;

   for (c=0; c<numc; c++)
   {
      fdiv = (double)cf[c] / (double)f;
      if (f < PI_MASH_MAX_FREQ)
      {
         fdiv += (0.5 / 4096.0);
         div = fdiv;
         frac = (fdiv - div) * 4096.0;
      }
      else
      {
         fdiv += 0.5;
         div = fdiv;
         frac = 0;
      }

      if ((div > 1) && (div < 4096))
      {
         offby = f - (cf[c] / (div + (frac / 4096.0)));
         if (offby < 0) offby = - offby;
         if ((!valid) || (offby <= best_offby))
         {
            valid = 1;
            clkInf->div = div;
            clkInf->frac = frac;
            clkInf->clock = c;
            best_offby = offby;
         }
      }
   }
   return valid;
}

/* ======================================================================= */

static rawCbs_t * dmaCB2adr(int pos)
{
   int page, slot;

   page = pos/CBS_PER_IPAGE;
   slot = pos%CBS_PER_IPAGE;

   return &dmaIVirt[page]->cb[slot];
}

/* ----------------------------------------------------------------------- */

static void dmaCbPrint(int pos)
{
   rawCbs_t * p;

   p = dmaCB2adr(pos);

   fprintf(stderr, "i=%x s=%x d=%x len=%x s=%x nxt=%x\n",
      p->info, p->src, p->dst, p->length, p->stride, p->next);
}

/* ----------------------------------------------------------------------- */

static unsigned dmaNowAtICB(void)
{
   unsigned cb;
   static unsigned lastPage=0;
   unsigned page;
   uint32_t cbAddr;
   uint32_t startTick, endTick;

   startTick = systReg[SYST_CLO];

   cbAddr = dmaIn[DMA_CONBLK_AD];

   page = lastPage;

   /* which page are we dma'ing? */

   while (1)
   {
      //cast twice to suppress compiler warning, I belive this cast is ok
      //because dmaIbus contains bus addresses, not user addresses. --plugwash
      cb = (cbAddr - ((int)(uintptr_t)dmaIBus[page])) / 32;

      if (cb < CBS_PER_IPAGE)
      {
         endTick = systReg[SYST_CLO];

         if (endTick != startTick)
            gpioStats.cbTicks += (endTick - startTick);

         gpioStats.cbCalls++;

         lastPage = page;

         return (page*CBS_PER_IPAGE) + cb;
      }

      if (page++ >= DMAI_PAGES) page=0;

      if (page == lastPage) break;
   }

   return 0;
}

/* ----------------------------------------------------------------------- */

static int dmaNowAtOCB(void)
{
   unsigned cb;
   unsigned page;
   uint32_t cbAddr;

   cbAddr = dmaOut[DMA_CONBLK_AD];

   if (!cbAddr) return -PI_NO_TX_WAVE;

   page = 0;

   /* which page are we dma'ing? */

   while (1)
   {
      //cast twice to suppress compiler warning, I belive this cast is ok
      //because dmaIbus contains bus addresses, not user addresses. --plugwash
      cb = (cbAddr - ((int)(uintptr_t)dmaOBus[page])) / 32;

      if (cb < CBS_PER_OPAGE) return (page*CBS_PER_OPAGE) + cb;

      if (page++ >= DMAO_PAGES) break;
   }

   /* Try twice */

   cbAddr = dmaOut[DMA_CONBLK_AD];

   if (!cbAddr) return -PI_NO_TX_WAVE;

   page = 0;

   /* which page are we dma'ing? */

   while (1)
   {
      //cast twice to suppress compiler warning, I belive this cast is ok
      //because dmaIbus contains bus addresses, not user addresses. --plugwash
      cb = (cbAddr - ((int)(uintptr_t)dmaOBus[page])) / 32;

      if (cb < CBS_PER_OPAGE) return (page*CBS_PER_OPAGE) + cb;

      if (page++ >= DMAO_PAGES) break;
   }

   return -PI_WAVE_NOT_FOUND;
}

/* ----------------------------------------------------------------------- */

unsigned rawWaveCB(void)
{
   unsigned cb;
   static unsigned lastPage=0;
   unsigned page;
   uint32_t cbAddr;

   cbAddr = dmaOut[DMA_CONBLK_AD];

   if (!cbAddr) return -1;

   page = lastPage;

   /* which page are we dma'ing? */

   while (1)
   {
      //cast twice to suppress compiler warning, I belive this cast is ok
      //because dmaIbus contains bus addresses, not user addresses. --plugwash
      cb = (cbAddr - ((int)(uintptr_t)dmaOBus[page])) / 32;

      if (cb < CBS_PER_OPAGE)
      {
         lastPage = page;

         return (page*CBS_PER_OPAGE) + cb;
      }

      if (page++ >= DMAO_PAGES) page=0;

      if (page == lastPage) break;
   }

   return 0;
}

/* ----------------------------------------------------------------------- */

static unsigned dmaCurrentSlot(unsigned pos)
{
   unsigned cycle=0, slot=0, tmp;

   cycle = (pos/CBS_PER_CYCLE);
   tmp   = (pos%CBS_PER_CYCLE);

   if (tmp > 2) slot = ((tmp-2)/3);

   return (cycle*PULSE_PER_CYCLE)+slot;
}

/* ----------------------------------------------------------------------- */

static uint32_t dmaPwmDataAdr(int pos)
{
   //cast twice to suppress compiler warning, I belive this cast is ok
   //because dmaIbus contains bus addresses, not user addresses. --plugwash
   return (uint32_t)(uintptr_t) &dmaIBus[pos]->periphData;
}

/* ----------------------------------------------------------------------- */

static uint32_t dmaGpioOnAdr(int pos)
{
   int page, slot;

   page = pos/ON_PER_IPAGE;
   slot = pos%ON_PER_IPAGE;

   //cast twice to suppress compiler warning, I belive this cast is ok
   //because dmaIbus contains bus addresses, not user addresses. --plugwash
   return (uint32_t)(uintptr_t) &dmaIBus[page]->gpioOn[slot];
}

/* ----------------------------------------------------------------------- */

static uint32_t dmaGpioOffAdr(int pos)
{
   int page, slot;

   myOffPageSlot(pos, &page, &slot);

   //cast twice to suppress compiler warning, I belive this cast is ok
   //because dmaIbus contains bus addresses, not user addresses. --plugwash
   return (uint32_t)(uintptr_t) &dmaIBus[page]->gpioOff[slot];
}

/* ----------------------------------------------------------------------- */

static uint32_t dmaTickAdr(int pos)
{
   int page, slot;

   myTckPageSlot(pos, &page, &slot);

   //cast twice to suppress compiler warning, I belive this cast is ok
   //because dmaIbus contains bus addresses, not user addresses. --plugwash
   return (uint32_t)(uintptr_t) &dmaIBus[page]->tick[slot];
}

/* ----------------------------------------------------------------------- */

static uint32_t dmaReadLevelsAdr(int pos)
{
   int page, slot;

   myLvsPageSlot(pos, &page, &slot);

   //cast twice to suppress compiler warning, I belive this cast is ok
   //because dmaIbus contains bus addresses, not user addresses. --plugwash
   return (uint32_t)(uintptr_t) &dmaIBus[page]->level[slot];
}

/* ----------------------------------------------------------------------- */

static uint32_t dmaCbAdr(int pos)
{
   int page, slot;

   page = (pos/CBS_PER_IPAGE);
   slot = (pos%CBS_PER_IPAGE);

   //cast twice to suppress compiler warning, I belive this cast is ok
   //because dmaIbus contains bus addresses, not user addresses. --plugwash
   return (uint32_t)(uintptr_t) &dmaIBus[page]->cb[slot];
}

/* ----------------------------------------------------------------------- */

static void dmaGpioOnCb(int b, int pos)
{
   rawCbs_t * p;

   p = dmaCB2adr(b);

   p->info   = NORMAL_DMA;
   p->src    = dmaGpioOnAdr(pos);
   p->dst    = ((GPIO_BASE + (GPSET0*4)) & 0x00ffffff) | PI_PERI_BUS;
   p->length = 4;
   p->next   = dmaCbAdr(b+1);
}

/* ----------------------------------------------------------------------- */

static void dmaTickCb(int b, int pos)
{
   rawCbs_t * p;

   p = dmaCB2adr(b);

   p->info   = NORMAL_DMA;
   p->src    = ((SYST_BASE + (SYST_CLO*4)) & 0x00ffffff) | PI_PERI_BUS;
   p->dst    = dmaTickAdr(pos);
   p->length = 4;
   p->next   = dmaCbAdr(b+1);
}

/* ----------------------------------------------------------------------- */

static void dmaGpioOffCb(int b, int pos)
{
   rawCbs_t * p;

   p = dmaCB2adr(b);

   p->info   = NORMAL_DMA;
   p->src    = dmaGpioOffAdr(pos);
   p->dst    = ((GPIO_BASE + (GPCLR0*4)) & 0x00ffffff) | PI_PERI_BUS;
   p->length = 4;
   p->next   = dmaCbAdr(b+1);
}

/* ----------------------------------------------------------------------- */

static void dmaReadLevelsCb(int b, int pos)
{
   rawCbs_t * p;

   p = dmaCB2adr(b);

   p->info   = NORMAL_DMA;
   p->src    = ((GPIO_BASE + (GPLEV0*4)) & 0x00ffffff) | PI_PERI_BUS;
   p->dst    = dmaReadLevelsAdr(pos);
   p->length = 4;
   p->next   = dmaCbAdr(b+1);
}

/* ----------------------------------------------------------------------- */

static void dmaDelayCb(int b)
{
   rawCbs_t * p;

   p = dmaCB2adr(b);

   if (gpioCfg.clockPeriph == PI_CLOCK_PCM)
   {
      p->info = NORMAL_DMA | TIMED_DMA(2);
      p->dst  = PCM_TIMER;
   }
   else
   {
      p->info = NORMAL_DMA | TIMED_DMA(5);
      p->dst  = PWM_TIMER;
   }

   p->src    = dmaPwmDataAdr(b%DMAI_PAGES);
   p->length = 4;
   p->next   = dmaCbAdr(b+1);
}

/* ----------------------------------------------------------------------- */

static void dmaInitCbs(void)
{
   int b, pulse, level, cycle;

   rawCbs_t * p;

   /* set up the DMA control blocks */

   DBG(DBG_STARTUP, "");

   gpioStats.dmaInitCbsCount++;

   b = -1;
   level = 0;

   for (cycle=0; cycle<bufferCycles; cycle++)
   {
      b++; dmaGpioOnCb(b, cycle%SUPERCYCLE); /* gpio on slot */

      b++; dmaTickCb(b, cycle);              /* tick slot */

      for (pulse=0; pulse<PULSE_PER_CYCLE; pulse++)
      {
         b++; dmaReadLevelsCb(b, level);               /* read levels slot */

         b++; dmaDelayCb(b);                           /* delay slot */

         b++; dmaGpioOffCb(b, (level%SUPERLEVEL)+1);   /* gpio off slot */

         ++level;
      }
   }

   /* point last cb back to first for continuous loop */

   p = dmaCB2adr(b);

   p->next = dmaCbAdr(0);

   DBG(DBG_STARTUP, "DMA page type count = %zd", sizeof(dmaIPage_t));

   DBG(DBG_STARTUP, "%d control blocks (exp=%d)", b+1, NUM_CBS);
}

/* ======================================================================= */


static void sigHandler(int signum)
{
   if ((signum >= PI_MIN_SIGNUM) && (signum <= PI_MAX_SIGNUM))
   {
      if (gpioSignal[signum].func)
      {
         if (gpioSignal[signum].ex)
         {
            (gpioSignal[signum].func)(signum, gpioSignal[signum].userdata);
         }
         else
         {
            (gpioSignal[signum].func)(signum);
         }
      }
      else
      {
         switch(signum)
         {
            case SIGUSR1:

               if (gpioCfg.dbgLevel > DBG_MIN_LEVEL) --gpioCfg.dbgLevel;
               else gpioCfg.dbgLevel = DBG_MIN_LEVEL;
               DBG(DBG_USER, "Debug level %d\n", gpioCfg.dbgLevel);
               break;

            case SIGUSR2:
               if (gpioCfg.dbgLevel < DBG_MAX_LEVEL) ++gpioCfg.dbgLevel;
               else gpioCfg.dbgLevel = DBG_MAX_LEVEL;
               DBG(DBG_USER, "Debug level %d\n", gpioCfg.dbgLevel);
               break;

            case SIGPIPE:
            case SIGWINCH:
               DBG(DBG_USER, "signal %d ignored", signum);
               break;

            case SIGCHLD:
               /* Used to notify threads of events */
               break;

            default:
               DBG(DBG_ALWAYS, "Unhandled signal %d, terminating\n", signum);
               gpioTerminate();
               exit(-1);
         }
      }
   }
   else
   {
      /* exit */

      DBG(DBG_ALWAYS, "Unhandled signal %d, terminating\n", signum);
      gpioTerminate();
      exit(-1);
   }
}

/* ----------------------------------------------------------------------- */

static void sigSetHandler(void)
{
   int i;
   struct sigaction new;

   for (i=PI_MIN_SIGNUM; i<=PI_MAX_SIGNUM; i++)
   {

      memset(&new, 0, sizeof(new));
      new.sa_handler = sigHandler;

      sigaction(i, &new, NULL);
   }
}

/*
   freq mics  net
 0 1000 1000  900
 1 4000  250  225
 2 3750  266  240
 3 3500  285  257
 4 3250  307  276
 5 3000  333  300
 6 2750  363  327
 7 2500  400  360
 8 2250  444  400
 9 2000  500  450
10 1750  571  514
11 1500  666  600
12 1250  800  720
13 1000 1000  900
14 750  1333 1200
15 500  2000 1800
*/

unsigned alert_delays[]=
{
   900000, 225000, 240000, 257142, 276923, 300000,  327272,  360000,
   400000, 450000, 514285, 600000, 720000, 900000, 1200000, 1800000
};

/* ======================================================================= */

static void alertGlitchFilter(gpioSample_t *sample, int numSamples)
{
   int i, j, diff;
   uint32_t steadyUs, changedTick, RBitV, LBitV, initialised;
   uint32_t bit, bitV;

   for (i=0; i<=PI_MAX_USER_GPIO; i++)
   {
      bit = (1<<i);

      if (monitorBits & bit & gFilterBits)
      {
         initialised = gpioAlert[i].gfInitialised;
         if (!initialised && numSamples > 0)
         {
           /* Initialise filter with first sample */
           bitV = sample[0].level & bit;
           gpioAlert[i].gfRBitV = bitV;
           gpioAlert[i].gfLBitV = bitV;
           gpioAlert[i].gfTick = sample[0].tick;
           gpioAlert[i].gfInitialised = 1;
         }

         steadyUs    = gpioAlert[i].gfSteadyUs;
         RBitV       = gpioAlert[i].gfRBitV;
         LBitV       = gpioAlert[i].gfLBitV;
         changedTick = gpioAlert[i].gfTick;

         for (j=0; j<numSamples; j++)
         {
            bitV = sample[j].level & bit;

            if (bitV != LBitV)
            {
               /* Difference between level and last level.
                  Restart steady timer. */

               changedTick = sample[j].tick;
               LBitV = bitV;
            }

            if (bitV != RBitV)
            {
               /* Difference between level and reported level. */

               diff = sample[j].tick - changedTick;

               if (diff >= steadyUs)
               {
                  /* Level stable for steady period. */
                  RBitV = bitV;
               }
               else
               {
                  /* Keep reporting old level. */

                  sample[j].level ^= bit;
               }
            }

         }

         gpioAlert[i].gfRBitV = RBitV;
         gpioAlert[i].gfLBitV = LBitV;
         gpioAlert[i].gfTick  = changedTick;
      }
   }
}

static void alertNoiseFilter(gpioSample_t *sample, int numSamples)
{
   int i, j, diff;
   uint32_t LBitV;
   uint32_t bit, bitV;
   uint32_t nowTick;

   for (i=0; i<=PI_MAX_USER_GPIO; i++)
   {
      bit = (1<<i);

      if (monitorBits & bit & nFilterBits)
      {
         LBitV = gpioAlert[i].nfLBitV;

         for (j=0; j<numSamples; j++)
         {
            bitV = sample[j].level & bit;
            nowTick = sample[j].tick;

            if (gpioAlert[i].nfActive) /* reporting events */
            {
               diff = nowTick - gpioAlert[i].nfTick2;

               if (diff >= 0)
               {
                  /* Stop reporting gpio changes */

                  gpioAlert[i].nfActive = 0;
                  gpioAlert[i].nfTick1 = nowTick;
               }
            }
            else /* waiting for steady us */
            {
               if (bitV != LBitV)
               {
                  diff = nowTick - gpioAlert[i].nfTick1;
                  gpioAlert[i].nfTick1 = nowTick;

                  if (diff >= gpioAlert[i].nfSteadyUs)
                  {
                     /* Start reporting gpio changes */

                     gpioAlert[i].nfRBitV = LBitV;
                     gpioAlert[i].nfActive = 1;
                     gpioAlert[i].nfTick2 =
                        nowTick + gpioAlert[i].nfActiveUs;
                  }
               }
            }

            if (!gpioAlert[i].nfActive)
            {
               if (bitV != gpioAlert[i].nfRBitV)
                  sample[j].level ^= bit;
            }

            LBitV = bitV;
         }

         gpioAlert[i].nfLBitV = LBitV;

      }
   }
}

static void alertEmit(
   gpioSample_t *sample, int numSamples, uint32_t changedBits, uint32_t eTick)
{
   uint32_t oldLevel, newLevel;
   int32_t diff;
   int emit, seqno, emitted;
   uint32_t changes, bits, timeoutBits, eventBits;
   int d;
   int b, n, v;
   int err;
   int max_emits;
   char fifo[32];
   /* ensure space for maximum number of watchdog and event notifications */
   gpioReport_t report[MAX_REPORT+PI_MAX_USER_GPIO+1+PI_MAX_EVENT+1];

   if (changedBits)
   {
      if (gpioGetSamples.func)
      {
         if (gpioGetSamples.ex)
         {
            (gpioGetSamples.func)
               (sample, numSamples, gpioGetSamples.userdata);
         }
         else
         {
            (gpioGetSamples.func)(sample, numSamples);
         }
      }
   }

   eventBits = 0;

   if (bscFR != (bscsReg[BSC_FR]&0xffff))
   {
      bscFR = bscsReg[BSC_FR]&0xffff;
      eventAlert[PI_EVENT_BSC].fired = 1;
   }

   for (b=0; b<=PI_MAX_EVENT; b++)
   {
      if (eventAlert[b].fired && (!eventAlert[b].ignore))
      {
         eventBits |= (1<<b);

         if (eventAlert[b].func)
         {
            if (eventAlert[b].ex)
            {
               (eventAlert[b].func)(b, eTick, eventAlert[b].userdata);
            }
            else
            {
               (eventAlert[b].func)(b, eTick);
            }
         }
      }

      eventAlert[b].fired = 0;
   }

   /* call alert callbacks for each bit transition */

   if (changedBits & alertBits)
   {
      oldLevel = (reportedLevel & alertBits);

      for (d=0; d<numSamples; d++)
      {
         newLevel = (sample[d].level & alertBits);

         if (newLevel != oldLevel)
         {
            changes = (newLevel ^ oldLevel);

            for (b=0; b<=PI_MAX_USER_GPIO; b++)
            {
               if (changes & (1<<b))
               {
                  if (newLevel & (1<<b)) v = 1; else v = 0;

                  if (gpioAlert[b].func)
                  {
                     if (gpioAlert[b].ex)
                     {
                        (gpioAlert[b].func)
                           (b, v, sample[d].tick,
                            gpioAlert[b].userdata);
                     }
                     else
                     {
                        (gpioAlert[b].func)(b, v, sample[d].tick);
                     }
                  }
               }
            }
            oldLevel = newLevel;
         }
      }
   }

   /* check for watchdog timeouts */

   timeoutBits = 0;

   if (wdogBits)
   {
      for (b=0; b<=PI_MAX_USER_GPIO; b++)
      {
         if (gpioAlert[b].wdSteadyUs)
         {
            diff = eTick - gpioAlert[b].wdTick;

            if (diff >= gpioAlert[b].wdSteadyUs)
            {
               timeoutBits |= (1<<b);

               gpioAlert[b].wdTick = eTick;

               if (gpioAlert[b].func)
               {
                  if (gpioAlert[b].ex)
                  {
                     (gpioAlert[b].func)(b, PI_TIMEOUT, eTick,
                                            gpioAlert[b].userdata);
                  }
                  else
                  {
                     (gpioAlert[b].func)(b, PI_TIMEOUT, eTick);
                  }
               }
            }
         }
      }
   }

   for (n=0; n<PI_NOTIFY_SLOTS; n++)
   {
      if (gpioNotify[n].state == PI_NOTIFY_CLOSING)
      {
         if (gpioNotify[n].pipe)
         {
            DBG(DBG_INTERNAL, "close notify pipe %d", gpioNotify[n].fd);
            close(gpioNotify[n].fd);

            sprintf(fifo, "/dev/pigpio%d", n);

            unlink(fifo);
         }

         gpioNotify[n].state = PI_NOTIFY_CLOSED;
      }
      else if (gpioNotify[n].state >= PI_NOTIFY_OPENED)
      {
         bits = gpioNotify[n].bits;

         emit = 0;

         seqno = gpioNotify[n].seqno;

         if (gpioNotify[n].state == PI_NOTIFY_RUNNING)
         {
            /* check to see if any bits have changed for this
               notification.

               bits         is the set of notification bits
               changedBits is the set of changed bits
            */

            if (changedBits & bits)
            {
               oldLevel = reportedLevel & bits;

               for (d=0; d<numSamples; d++)
               {
                  newLevel = sample[d].level & bits;

                  if (newLevel != oldLevel)
                  {
                     report[emit].seqno = seqno;
                     report[emit].flags = 0;
                     report[emit].tick  = sample[d].tick;
                     report[emit].level = sample[d].level;

                     oldLevel = newLevel;

                     emit++;
                     seqno++;
                  }
               }
            }

            /* check to see if any watchdogs are due for this
               notification.

               bits        is the set of notification bits
               timeoutBits is the set of timed out bits
            */

            bits = gpioNotify[n].bits;

            if (timeoutBits & bits)
            {
               /* at least one watchdog has fired for this
                  notification.
               */

               for (b=0; b<=PI_MAX_USER_GPIO; b++)
               {
                  if (timeoutBits & bits & (1<<b))
                  {
                     if (numSamples)
                        newLevel = sample[numSamples-1].level;
                     else
                        newLevel = reportedLevel;

                     report[emit].seqno = seqno;
                     report[emit].flags =
                        PI_NTFY_FLAGS_WDOG | PI_NTFY_FLAGS_BIT(b);
                     report[emit].tick  = eTick;
                     report[emit].level = newLevel;

                     emit++;
                     seqno++;
                  }
               }
            }
         }

         /* check to see if any events are due

            eventBits is the set of events
         */

         if (eventBits & gpioNotify[n].eventBits)
         {
            for (b=0; b<=PI_MAX_EVENT; b++)
            {
               if (eventBits & gpioNotify[n].eventBits & (1<<b))
               {
                  if (numSamples)
                     newLevel = sample[numSamples-1].level;
                  else
                     newLevel = reportedLevel;

                  report[emit].seqno = seqno;
                  report[emit].flags = 
                     PI_NTFY_FLAGS_EVENT | PI_NTFY_FLAGS_BIT(b);
                  report[emit].tick  = eTick;
                  report[emit].level = newLevel;

                  emit++;
                  seqno++;
               }
            }
         }

         if (!emit)
         {
            if ((int)(eTick - gpioNotify[n].lastReportTick) > 60000000)
            {
               if (numSamples)
                  newLevel = sample[numSamples-1].level;
               else
                  newLevel = reportedLevel;

               report[emit].seqno = seqno;
               report[emit].flags = PI_NTFY_FLAGS_ALIVE;
               report[emit].tick  = eTick;
               report[emit].level = newLevel;

               emit++;
               seqno++;
            }
         }

         if (emit)
         {
            DBG(DBG_FAST_TICK, "notification %d (%d reports, %x-%x)",
               n, emit, report[0].seqno,  report[emit-1].seqno);
            gpioNotify[n].lastReportTick = eTick;
            max_emits = gpioNotify[n].max_emits;

            if (emit > gpioStats.maxEmit) gpioStats.maxEmit = emit;

            emitted = 0;

            while (emit > 0)
            {
               if (emit > max_emits)
               {
                  gpioStats.emitFrags++;

                  err = write(gpioNotify[n].fd,
                           report+emitted,
                           max_emits*sizeof(gpioReport_t));

                  if (err != (max_emits*sizeof(gpioReport_t)))
                  {
                     if (err < 0)
                     {
                        if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
                        {
                           /* serious error, no point continuing */

                           DBG(DBG_ALWAYS, "fd=%d err=%d errno=%d",
                              gpioNotify[n].fd, err, errno);

                           DBG(DBG_ALWAYS, "%s", strerror(errno));

                           gpioNotify[n].bits  = 0;
                           gpioNotify[n].state = PI_NOTIFY_CLOSING;
                           intNotifyBits();
                           break;
                        }
                        else gpioStats.wouldBlockPipeWrite++;
                     }
                     else
                     {
                        gpioStats.shortPipeWrite++;
                        DBG(DBG_ALWAYS, "emitted %zd, asked for %d",
                           err/sizeof(gpioReport_t), max_emits);
                     }
                  }
                  else
                  {
                     gpioStats.goodPipeWrite++;
                  }

                  emitted += max_emits;
                  emit    -= max_emits;
               }
               else
               {
                  err = write(gpioNotify[n].fd,
                           report+emitted,
                           emit*sizeof(gpioReport_t));

                  if (err != (emit*sizeof(gpioReport_t)))
                  {
                     if (err < 0)
                     {
                        if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
                        {
                           DBG(DBG_ALWAYS, "fd=%d err=%d errno=%d",
                              gpioNotify[n].fd, err, errno);

                           DBG(DBG_ALWAYS, "%s", strerror(errno));

                           /* serious error, no point continuing */
                           gpioNotify[n].bits  = 0;
                           gpioNotify[n].state = PI_NOTIFY_CLOSING;
                           intNotifyBits();
                           break;
                        }
                        else gpioStats.wouldBlockPipeWrite++;
                     }
                     else
                     {
                        gpioStats.shortPipeWrite++;
                        DBG(DBG_ALWAYS, "emitted %zd, asked for %d",
                           err/sizeof(gpioReport_t), emit);
                     }
                  }
                  else
                  {
                     gpioStats.goodPipeWrite++;
                  }

                  emitted += emit;
                  emit = 0;
               }
            }

            gpioNotify[n].seqno = seqno;
         }
      }
   }

   if (changedBits & scriptBits)
   {
      for (n=0; n<PI_MAX_SCRIPTS; n++)
      {
         if ((gpioScript[n].state     == PI_SCRIPT_IN_USE)  &&
             (gpioScript[n].run_state == PI_SCRIPT_WAITING) &&
             (gpioScript[n].waitBits & changedBits))
         {
            pthread_mutex_lock(&gpioScript[n].pthMutex);

            if (gpioScript[n].run_state == PI_SCRIPT_WAITING)
            {
               gpioScript[n].changedBits =
                  gpioScript[n].waitBits & changedBits;
               pthread_cond_signal(&gpioScript[n].pthCond);
            }

            pthread_mutex_unlock(&gpioScript[n].pthMutex);
         }
      }
   }

   if (eventBits & scriptEventBits)
   {
      for (n=0; n<PI_MAX_SCRIPTS; n++)
      {
         if ((gpioScript[n].state     == PI_SCRIPT_IN_USE)  &&
             (gpioScript[n].run_state == PI_SCRIPT_WAITING) &&
             (gpioScript[n].eventBits & eventBits))
         {
            pthread_mutex_lock(&gpioScript[n].pthMutex);

            if (gpioScript[n].run_state == PI_SCRIPT_WAITING)
            {
               gpioScript[n].changedBits =
                  gpioScript[n].eventBits & eventBits;
               pthread_cond_signal(&gpioScript[n].pthCond);
            }

            pthread_mutex_unlock(&gpioScript[n].pthMutex);
         }
      }
   }

   if (numSamples) reportedLevel = sample[numSamples-1].level;
}

static void alertWdogCheck(gpioSample_t *sample, int numSamples)
{
   /*
   Go through and set the last time each GPIO with a watchdog changed state.
   */

   int i, j;
   uint32_t LBitV;
   uint32_t bit;

   for (i=0; i<=PI_MAX_USER_GPIO; i++)
   {
      bit = (1<<i);

      if (monitorBits & bit & wdogBits)
      {
         LBitV = gpioAlert[i].wdLBitV;

         for (j=0; j<numSamples; j++)
         {
            if ((sample[j].level & bit) != LBitV)
            {
               LBitV = sample[j].level & bit;
               gpioAlert[i].wdTick = sample[j].tick;
            }
         }

         gpioAlert[i].wdLBitV = LBitV;
      }
   }
}

static void * pthAlertThread(void *x)
{
   struct timespec req, rem;
   uint32_t oldLevel, newLevel, level;
   uint32_t oldSlot,  newSlot;
   uint32_t expected, ft, sTick;
   uint32_t changedBits;
   int32_t diff, minDiff, stickInited;
   int cycle, pulse;
   int numSamples, ticks, i;
   int rp, reports, totalSamples;
   int stopped;
   int moreToDo;
   gpioSample_t sample[MAX_SAMPLE];

   req.tv_sec = 0;

   /* don't start until DMA started */

   spinWhileStarting();

   reportedLevel = gpioReg[GPLEV0];

   oldLevel = reportedLevel;

   oldSlot = dmaCurrentSlot(dmaNowAtICB());

   oldSlot = (oldSlot / PULSE_PER_CYCLE) * PULSE_PER_CYCLE;

   cycle = (oldSlot/PULSE_PER_CYCLE);

   pulse = 0;

   stopped = 0;

   moreToDo = 0;

   stickInited = 0;

   sTick = 0;

   minDiff = gpioCfg.clockMicros / 2;

   while (1)
   {
      /* Check that DMA is running okay */

      if (dmaIn[DMA_CONBLK_AD])
      {
         if (stopped)
         {
            DBG(DBG_STARTUP, "****** GOING ******");
            stopped = 0;
         }
      }
      else
      {
         stopped = 1;

         myGpioDelay(5000);

         if (runState == PI_RUNNING)
         {
            /* should never be executed, leave code just in case */

            gpioCfg.internals |= PI_CFG_STATS;

            dmaInitCbs();
            flushMemory();
            //cast twice to suppress compiler warning, I belive this cast is ok
            //because dmaIbus contains bus addresses, not user addresses. --plugwash
            initDMAgo((uint32_t *)dmaIn, (uint32_t)(uintptr_t)dmaIBus[0]);
            myGpioDelay(5000); /* let DMA run for a while */
            oldSlot = dmaCurrentSlot(dmaNowAtICB());
            gpioStats.DMARestarts++;
         }
      }

      newSlot = dmaCurrentSlot(dmaNowAtICB());

      newSlot = (newSlot / PULSE_PER_CYCLE) * PULSE_PER_CYCLE;

      numSamples = 0;

      /*
      Extract samples from DMA ring buffer.
      */

      while ((oldSlot != newSlot) && (numSamples < MAX_SAMPLE))
      {
         level = myGetLevel(oldSlot++);

         sample[numSamples].tick  = sTick;
         sample[numSamples].level = level;

         numSamples++;

         sTick += gpioCfg.clockMicros;

         if (++pulse >= PULSE_PER_CYCLE)
         {
            pulse = 0;

            if (++cycle >= bufferCycles)
            {
               cycle = 0;
               oldSlot = 0;
            }

            expected = sTick;

            sTick = myGetTick(cycle);

            if (stickInited)
            {
               diff = sTick - expected;

               if (abs(diff) > minDiff)
               {
                  ft = sample[numSamples-PULSE_PER_CYCLE].tick;

                  ticks = sTick - ft;

                  for (i=1; i<PULSE_PER_CYCLE; i++)
                  {
                     sample[numSamples-PULSE_PER_CYCLE+i].tick =
                        ((i*ticks)/PULSE_PER_CYCLE) + ft;
                  }
               }

               diff += (TICKSLOTS/2);

               if (diff < 0)
               {
                  gpioStats.diffTick[0]++;
               }

               else if (diff >= TICKSLOTS)
               {
                  gpioStats.diffTick[TICKSLOTS-1]++;
               }

               else gpioStats.diffTick[diff]++;
            }
            else
            {
               stickInited = 1;
               numSamples = 0;
               if (!(gpioCfg.ifFlags & PI_DISABLE_ALERT))
               {
                  pthAlertRunning = PI_THREAD_RUNNING;
               }
            }
         }
      }

      if (oldSlot == newSlot) moreToDo = 0; else moreToDo = 1;

      /* Apply glitch filter */

      if (numSamples && gFilterBits) alertGlitchFilter(sample, numSamples);

      /* Apply noise filter */

      if (numSamples && nFilterBits) alertNoiseFilter(sample, numSamples);

      /* Compact samples */

      changedBits = 0;
      oldLevel &= monitorBits;
      reports = 0;
      totalSamples = 0;

      for (rp=0; rp<numSamples; rp++)
      {
         newLevel = (sample[rp].level & monitorBits);

         if (newLevel != oldLevel)
         {
            sample[reports].tick  = sample[rp].tick;
            sample[reports].level = sample[rp].level;
            changedBits |= (newLevel ^ oldLevel);
            oldLevel = newLevel;

            reports++;

            if (reports >= MAX_REPORT)
            {
               totalSamples += reports;

               /* Rebase watchdog timeouts */
               if (wdogBits) alertWdogCheck(sample, reports);

               gpioStats.numSamples += reports;

               alertEmit(sample, reports, changedBits, sample[rp].tick);

               changedBits = 0;
               reports = 0;
            }
         }
      }

      if (reports)
      {
         totalSamples += reports;

         /* Rebase watchdog timeouts */
         if (wdogBits) alertWdogCheck(sample, reports);

         gpioStats.numSamples += reports;
      }

      alertEmit(sample, reports, changedBits, sTick);
      reportedLevel = sample[numSamples -1].level;

      if (totalSamples > gpioStats.maxSamples)
         gpioStats.maxSamples = numSamples;

      req.tv_sec = 0;
      req.tv_nsec = alert_delays[(gpioCfg.internals>>PI_CFG_ALERT_FREQ)&15];

      if (moreToDo)
      {
         gpioStats.moreToDo++;
      }
      else
      {
         gpioStats.alertTicks++;

         while (nanosleep(&req, &rem))
         {
            req.tv_sec  = rem.tv_sec;
            req.tv_nsec = rem.tv_nsec;
         }
      }
   }

   return 0;
}

/* ======================================================================= */

static int scrPop(gpioScript_t *s, int *SP, int *S)
{
   if ((*SP) > 0)
   {
      return S[--(*SP)];
   }
   else
   {
      s->run_state = PI_SCRIPT_FAILED;
      DBG(DBG_ALWAYS, "script %d too many pops", s->id);
      return 0;
   }
}

/* ----------------------------------------------------------------------- */

static void scrPush(gpioScript_t *s, int *SP, int *S, int val)
{
   if ((*SP) < PI_SCRIPT_STACK_SIZE)
   {
      S[(*SP)++] = val;
   }
   else
   {
      s->run_state = PI_SCRIPT_FAILED;
      DBG(DBG_ALWAYS, "script %d too many pushes", s->id);
   }
}

/* ----------------------------------------------------------------------- */

static void scrSwap(int *v1, int *v2)
{
   int t;

   t=*v1; *v1=*v2; *v2= t;
}

/* ----------------------------------------------------------------------- */

static int scrEvtWait(gpioScript_t *s, uint32_t bits)
{
   pthread_mutex_lock(&s->pthMutex);

   if (s->request == PI_SCRIPT_RUN)
   {
      s->run_state = PI_SCRIPT_WAITING;
      s->eventBits = bits;
      intScriptEventBits();

      pthread_cond_wait(&s->pthCond, &s->pthMutex);

      s->waitBits = 0;
      intScriptEventBits();
      s->run_state = PI_SCRIPT_RUNNING;
   }

   pthread_mutex_unlock(&s->pthMutex);

   return s->changedBits;
}

/* ----------------------------------------------------------------------- */

static int scrWait(gpioScript_t *s, uint32_t bits)
{
   pthread_mutex_lock(&s->pthMutex);

   if (s->request == PI_SCRIPT_RUN)
   {
      s->run_state = PI_SCRIPT_WAITING;
      s->waitBits = bits;
      intScriptBits();

      pthread_cond_wait(&s->pthCond, &s->pthMutex);

      s->waitBits = 0;
      intScriptBits();
      s->run_state = PI_SCRIPT_RUNNING;
   }

   pthread_mutex_unlock(&s->pthMutex);

   return s->changedBits;
}

/* ----------------------------------------------------------------------- */

static int scrSys(char *cmd, uint32_t p1, uint32_t p2)
{
   char buf[1024];
   int status;

   if (!myScriptNameValid(cmd))
      SOFT_ERROR(PI_BAD_SCRIPT_NAME, "bad script name (%s)", cmd);

   snprintf(buf, sizeof(buf), "/opt/pigpio/cgi/%s %u %u", cmd, p1, p2);

   DBG(DBG_USER, "%s", buf);

   status = system(buf);

   if (status < 0) status = PI_BAD_SHELL_STATUS;

   return status;
}

/* ----------------------------------------------------------------------- */

static void *pthScript(void *x)
{
   gpioScript_t *s;
   cmdInstr_t instr;
   int p1, p2, p1o, p2o, p3o, *t1, *t2;
   int PC, A, F, SP;
   int S[PI_SCRIPT_STACK_SIZE];
   char buf[CMD_MAX_EXTENSION];


   S[0] = 0; /* to prevent compiler warning */

   s = x;

   while ((volatile int)s->request != PI_SCRIPT_DELETE)
   {
      pthread_mutex_lock(&s->pthMutex);
      s->run_state = PI_SCRIPT_HALTED;
      pthread_cond_wait(&s->pthCond, &s->pthMutex);
      pthread_mutex_unlock(&s->pthMutex);

      s->run_state = PI_SCRIPT_RUNNING;

      A  = 0;
      F  = 0;
      PC = 0;
      SP = 0;

      while (((volatile int)s->request   == PI_SCRIPT_RUN    ) &&
                           (s->run_state == PI_SCRIPT_RUNNING))
      {
         instr = s->script.instr[PC];

         p1o = instr.p[1];
         p2o = instr.p[2];

         if      (instr.opt[1] == CMD_VAR) instr.p[1] = s->script.var[p1o];
         else if (instr.opt[1] == CMD_PAR) instr.p[1] = s->script.par[p1o];

         if      (instr.opt[2] == CMD_VAR) instr.p[2] = s->script.var[p2o];
         else if (instr.opt[2] == CMD_PAR) instr.p[2] = s->script.par[p2o];
/*
         fprintf(stderr, "PC=%d cmd=%d p1o=%d p1=%d p2o=%d p2=%d\n",
            PC, instr.p[0], p1o, instr.p[1], p2o, instr.p[2]);
         fflush(stderr);
*/
         if (instr.p[0] < PI_CMD_SCRIPT)
         {
            if (instr.p[3])
            {
               if ((instr.p[3] == sizeof(int)) && ((instr.opt[3] == CMD_VAR) || (instr.opt[3] == CMD_PAR)))
               {
                  /* Hack to allow register use in 3rd parameter */
                  memcpy((char*)&p3o, (char *)instr.p[4], sizeof(int));
                  if (instr.opt[3] == CMD_VAR) memcpy(buf, (char *)&(s->script.var[p3o]), sizeof(int));
                  else                         memcpy(buf, (char *)&(s->script.par[p3o]), sizeof(int));
               }
               else
               {
                  memcpy(buf, (char *)instr.p[4], instr.p[3]);
               }
            }

            A = myDoCommand(instr.p, sizeof(buf)-1, buf);

            F = A;

            PC++;
         }
         else
         {
            p1 = instr.p[1];
            p2 = instr.p[2];

            switch (instr.p[0])
            {
               case PI_CMD_ADD:   A+=p1; F=A;                     PC++; break;

               case PI_CMD_AND:   A&=p1; F=A;                     PC++; break;

               case PI_CMD_CALL:  scrPush(s, &SP, S, PC+1);    PC = p1; break;

               case PI_CMD_CMP:   F=A-p1;                         PC++; break;

               case PI_CMD_DCR:
                  if (instr.opt[1] == CMD_PAR)
                     {--s->script.par[p1o]; F=s->script.par[p1o];}
                  else
                     {--s->script.var[p1o]; F=s->script.var[p1o];}
                  PC++;
                  break;

               case PI_CMD_DCRA:  --A; F=A;                       PC++; break;

               case PI_CMD_DIV:   A/=p1; F=A;                     PC++; break;

               case PI_CMD_HALT:  s->run_state = PI_SCRIPT_HALTED;      break;

               case PI_CMD_EVTWT: A=scrEvtWait(s, p1); F=A;       PC++; break;

               case PI_CMD_INR:
                  if (instr.opt[1] == CMD_PAR)
                     {++s->script.par[p1o]; F=s->script.par[p1o];}
                  else
                     {++s->script.var[p1o]; F=s->script.var[p1o];}
                  PC++;
                  break;

               case PI_CMD_INRA:  ++A; F=A;                       PC++; break;

               case PI_CMD_JM:    if (F<0)  PC=p1; else PC++;           break;

               case PI_CMD_JMP:   PC=p1;                                break;

               case PI_CMD_JNZ:   if (F)    PC=p1; else PC++;           break;

               case PI_CMD_JP:    if (F>=0) PC=p1; else PC++;           break;

               case PI_CMD_JZ:    if (!F)   PC=p1; else PC++;           break;

               case PI_CMD_LD:
                  if (instr.opt[1] == CMD_PAR) s->script.par[p1o]=p2;
                  else                         s->script.var[p1o]=p2;
                  PC++;
                  break;

               case PI_CMD_LDA:   A=p1;                           PC++; break;

               case PI_CMD_LDAB:
                  if ((p1 >= 0) && (p1 < sizeof(buf))) A = buf[p1];
                  PC++;
                  break;

               case PI_CMD_MLT:   A*=p1; F=A;                     PC++; break;

               case PI_CMD_MOD:   A%=p1; F=A;                     PC++; break;

               case PI_CMD_OR:    A|=p1; F=A;                     PC++; break;

               case PI_CMD_POP:
                  if (instr.opt[1] == CMD_PAR)
                     s->script.par[p1o]=scrPop(s, &SP, S);
                  else
                     s->script.var[p1o]=scrPop(s, &SP, S);
                  PC++;
                  break;

               case PI_CMD_POPA:  A=scrPop(s, &SP, S);            PC++; break;

               case PI_CMD_PUSH:
                  if (instr.opt[1] == CMD_PAR)
                     scrPush(s, &SP, S, s->script.par[p1o]);
                  else
                     scrPush(s, &SP, S, s->script.var[p1o]);
                  PC++;
                  break;

               case PI_CMD_PUSHA: scrPush(s, &SP, S, A);          PC++; break;

               case PI_CMD_RET:   PC=scrPop(s, &SP, S);                 break;

               case PI_CMD_RL:
                  if (instr.opt[1] == CMD_PAR)
                     {s->script.par[p1o]<<=p2; F=s->script.par[p1o];}
                  else
                     {s->script.var[p1o]<<=p2; F=s->script.var[p1o];}
                  PC++;
                  break;

               case PI_CMD_RLA:   A<<=p1; F=A;                    PC++; break;

               case PI_CMD_RR:
                  if (instr.opt[1] == CMD_PAR)
                     {s->script.par[p1o]>>=p2; F=s->script.par[p1o];}
                  else
                     {s->script.var[p1o]>>=p2; F=s->script.var[p1o];}
                  PC++;
                  break;

               case PI_CMD_RRA:   A>>=p1; F=A;                    PC++; break;

               case PI_CMD_STA:
                  if (instr.opt[1] == CMD_PAR) s->script.par[p1o]=A;
                  else                         s->script.var[p1o]=A;
                  PC++;
                  break;

               case PI_CMD_STAB:
                  if ((p1 >= 0) && (p1 < sizeof(buf))) buf[p1] = A;
                  PC++;
                  break;

               case PI_CMD_SUB:   A-=p1; F=A;                     PC++; break;

               case PI_CMD_SYS:
                  A=scrSys((char*)instr.p[4], A, *(gpioReg + GPLEV0));
                  F=A;
                  PC++;
                  break;

               case PI_CMD_WAIT:  A=scrWait(s, p1); F=A;          PC++; break;

               case PI_CMD_X:
                  if (instr.opt[1] == CMD_PAR) t1 = &s->script.par[p1o];
                  else                         t1 = &s->script.var[p1o];

                  if (instr.opt[2] == CMD_PAR) t2 = &s->script.par[p2o];
                  else                         t2 = &s->script.var[p2o];

                  scrSwap(t1, t2);
                  PC++;
                  break;

               case PI_CMD_XA:
                  if (instr.opt[1] == CMD_PAR)
                     scrSwap(&s->script.par[p1o], &A);
                  else
                     scrSwap(&s->script.var[p1o], &A);
                  PC++;
                  break;

               case PI_CMD_XOR:   A^=p1; F=A;                     PC++; break;

            }
         }

         if (PC >= s->script.instrs) s->run_state = PI_SCRIPT_HALTED;

      }

      if ((volatile int)s->request == PI_SCRIPT_HALT)
         s->run_state = PI_SCRIPT_HALTED;

   }

   return 0;
}

/* ----------------------------------------------------------------------- */

static void * pthTimerTick(void *x)
{
   gpioTimer_t *tp;
   struct timespec req, rem;

   tp = x;

   while (1)
   {
      req.tv_sec  = tp->millis / THOUSAND;
      req.tv_nsec = (tp->millis % THOUSAND) * THOUSAND * THOUSAND;

      while (nanosleep(&req, &rem))
      {
         req.tv_sec  = rem.tv_sec;
         req.tv_nsec = rem.tv_nsec;
      }

      if (tp->ex) (tp->func)(tp->userdata);
      else        (tp->func)();
   }

   return 0;
}

/* ----------------------------------------------------------------------- */


static void * pthFifoThread(void *x)
{
   char buf[CMD_MAX_EXTENSION];
   int idx, flags, len, res, i;
   uintptr_t p[CMD_P_ARR];
   cmdCtlParse_t ctl;
   uint32_t *param;
   char v[CMD_MAX_EXTENSION];

   myCreatePipe(PI_INPFIFO, 0662);

   if ((inpFifo = fopen(PI_INPFIFO, "r+")) == NULL)
      SOFT_ERROR((void*)PI_INIT_FAILED, "fopen %s failed(%m)", PI_INPFIFO);

   myCreatePipe(PI_OUTFIFO, 0664);

   if ((outFifo = fopen(PI_OUTFIFO, "w+")) == NULL)
      SOFT_ERROR((void*)PI_INIT_FAILED, "fopen %s failed (%m)", PI_OUTFIFO);

   /* set outFifo non-blocking */

   flags = fcntl(fileno(outFifo), F_GETFL, 0);
   fcntl(fileno(outFifo), F_SETFL, flags | O_NONBLOCK);

   /* don't start until DMA started */

   spinWhileStarting();

   while (1)
   {
      if (fgets(buf, sizeof(buf), inpFifo) == NULL)
         SOFT_ERROR((void*)PI_INIT_FAILED, "fifo fgets failed (%m)");

      len = strlen(buf);

      if (len)
      {
        --len;
        buf[len] = 0; /* replace terminating */
      }

      ctl.eaten = 0;
      idx = 0;

      while (((ctl.eaten)<len) && (idx >= 0))
      {
         if ((idx=cmdParse(buf, p, CMD_MAX_EXTENSION, v, &ctl)) >= 0)
         {
            /* make sure extensions are null terminated */

            v[p[3]] = 0;

            res = myDoCommand(p, sizeof(v)-1, v);

            switch (cmdInfo[idx].rv)
            {
               case 0:
                  fprintf(outFifo, "%d\n", res);
                  break;

               case 1:
                  fprintf(outFifo, "%d\n", res);
                  break;

               case 2:
                  fprintf(outFifo, "%d\n", res);
                  break;

               case 3:
                  fprintf(outFifo, "%08X\n", res);
                  break;

               case 4:
                  fprintf(outFifo, "%u\n", res);
                  break;

               case 5:
                  fprintf(outFifo, "%s", cmdUsage);
                  break;

               case 6:
                  fprintf(outFifo, "%d", res);
                  if (res > 0)
                  {
                     for (i=0; i<res; i++)
                     {
                        fprintf(outFifo, " %d", v[i]);
                     }
                  }
                  fprintf(outFifo, "\n");
                  break;

               case 7:
                  if (res < 0) fprintf(outFifo, "%d\n", res);
                  else
                  {
                     fprintf(outFifo, "%d", res);
                     param = (uint32_t *)v;
                     for (i=0; i<PI_MAX_SCRIPT_PARAMS; i++)
                     {
                        fprintf(outFifo, " %d", param[i]);
                     }
                     fprintf(outFifo, "\n");
                  }
                  break;
            }
         }
         else fprintf(outFifo, "%d\n", PI_BAD_FIFO_COMMAND);
      }

      fflush(outFifo);
   }

   return 0;
}

/* ----------------------------------------------------------------------- */

static void *pthSocketThreadHandler(void *fdC)
{
   int sock = *(int*)fdC;
   uintptr_t p[10];
   uint32_t tmp, response[4];
   int i;
   int opt;
   char buf[CMD_MAX_EXTENSION];

   free(fdC);

   /* Disable the Nagle algorithm. */
   opt = 1;
   setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char*)&opt, sizeof(int));

   while (1)
   {
      if (sizeof(uintptr_t) == 8)
      {
         if (recv(sock, &tmp, 4, MSG_WAITALL) != 4) break;
         p[0] = (uintptr_t)tmp;
         if (recv(sock, &tmp, 4, MSG_WAITALL) != 4) break;
         p[1] = (uintptr_t)tmp;
         if (recv(sock, &tmp, 4, MSG_WAITALL) != 4) break;
         p[2] = (uintptr_t)tmp;
         if (recv(sock, &tmp, 4, MSG_WAITALL) != 4) break;
         p[3] = (uintptr_t)tmp;
      }
      else
      {
         if (recv(sock, p, 16, MSG_WAITALL) != 16) break;
      }

      if (p[3])
      {
         if (p[3] < sizeof(buf))
         {
            /* read extension into buf */
            if (recv(sock, buf, p[3], MSG_WAITALL) != p[3])
            {
               /* Serious error.  No point continuing. */
               DBG(DBG_ALWAYS,
                  "recv failed for %"PRIdPTR" bytes, sock=%d", p[3], sock);

               closeOrphanedNotifications(-1, sock);

               close(sock);

               return 0;
            }
         }
         else
         {
            /* Serious error.  No point continuing. */
            DBG(DBG_ALWAYS, "ext too large %"PRIdPTR"(%zd), sock=%d",
               p[3], sizeof(buf), sock);

            closeOrphanedNotifications(-1, sock);

            close(sock);

            return 0;
         }
      }

      /* add null terminator in case it's a string */

      buf[p[3]] = 0;

      switch (p[0])
      {
         case PI_CMD_NOIB:

            p[3] = gpioNotifyOpenInBand(sock);

           /* Enable the Nagle algorithm. */
            opt = 0;
            setsockopt(
               sock, IPPROTO_TCP, TCP_NODELAY, (char*)&opt, sizeof(int));

            break;

         case PI_CMD_PROCP:
            p[3] = myDoCommand(p, sizeof(buf)-1, buf+sizeof(int));
            if (((int)p[3]) >= 0)
            {
               memcpy(buf, &p[3], 4);
               p[3] = 4 + (4*PI_MAX_SCRIPT_PARAMS);
            }
            break;

         default:
            p[3] = myDoCommand(p, sizeof(buf)-1, buf);
      }

      if (sizeof(uintptr_t) == 8) // 64-bit system
      {
         for (i = 0; i < 4; i++)
            response[i] = (uint32_t)p[i];
         if (write(sock, response, 16) == -1) { /* ignore errors */ }
      }
      else // 32-bit system
      {
         if (write(sock, p, 16) == -1) { /* ignore errors */ }
      }

      switch (p[0])
      {
         /* extensions */

         case PI_CMD_BI2CZ:
         case PI_CMD_BSCX:
         case PI_CMD_CF2:
         case PI_CMD_FL:
         case PI_CMD_FR:
         case PI_CMD_I2CPK:
         case PI_CMD_I2CRD:
         case PI_CMD_I2CRI:
         case PI_CMD_I2CRK:
         case PI_CMD_I2CZ:
         case PI_CMD_PROCP:
         case PI_CMD_SERR:
         case PI_CMD_SLR:
         case PI_CMD_SPIX:
         case PI_CMD_SPIR:
         case PI_CMD_BSPIX:

            if (((int)p[3]) > 0)
            {
               if (write(sock, buf, p[3]) == 1) { /* ignore errors */ }
            }
            break;

         default:
           break;
      }
   }

   closeOrphanedNotifications(-1, sock);

   close(sock);

   DBG(DBG_USER, "Socket %d closed", sock);

   return 0;
}

static int addrAllowed(struct sockaddr *saddr)
{
   int i;
   uint32_t addr;

   if (!numSockNetAddr) return 1;

   // FIXME: add IPv6 whitelisting support
   if (saddr->sa_family != AF_INET) return 0;

   addr = ((struct sockaddr_in *) saddr)->sin_addr.s_addr;

   for (i=0; i<numSockNetAddr; i++)
   {
      if (addr == sockNetAddr[i]) return 1;
   }
   return 0;
}

/* ----------------------------------------------------------------------- */

static void * pthSocketThread(void *x)
{
   int fdC=0, c, *sock;
   struct sockaddr_storage client;
   pthread_attr_t attr;

   if (pthread_attr_init(&attr))
      SOFT_ERROR((void*)PI_INIT_FAILED,
         "pthread_attr_init failed (%m)");

   if (pthread_attr_setstacksize(&attr, STACK_SIZE))
      SOFT_ERROR((void*)PI_INIT_FAILED,
         "pthread_attr_setstacksize failed (%m)");

   if (pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED))
      SOFT_ERROR((void*)PI_INIT_FAILED,
         "pthread_attr_setdetachstate failed (%m)");

   /* fdSock opened in gpioInitialise so that we can treat
      failure to bind as fatal. */

   listen(fdSock, 100);

   c = sizeof(client);

   /* don't start until DMA started */

   spinWhileStarting();

   while (fdC >= 0)
   {
      pthread_t thr;

      fdC = accept(fdSock, (struct sockaddr *)&client, (socklen_t*)&c);

      closeOrphanedNotifications(-1, fdC);

      if (addrAllowed((struct sockaddr *)&client))
      {
         DBG(DBG_USER, "Connection accepted on socket %d", fdC);

         sock = malloc(sizeof(int));

         *sock = fdC;

         /* Enable tcp_keepalive */
         int optval = 1;
         socklen_t optlen = sizeof(optval);

         if (setsockopt(fdC, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0)
         {
           DBG(DBG_ALWAYS, "setsockopt() fail, closing socket %d", fdC);
           close(fdC);
         }

         DBG(DBG_USER, "SO_KEEPALIVE enabled on socket %d\n", fdC);

         if (pthread_create
            (&thr, &attr, pthSocketThreadHandler, (void*) sock) < 0)
            SOFT_ERROR((void*)PI_INIT_FAILED,
               "socket pthread_create failed (%m)");
      }
      else
      {
         DBG(DBG_ALWAYS, "Connection rejected, closing");
         close(fdC);
      }
   }

   if (fdC < 0)
      SOFT_ERROR((void*)PI_INIT_FAILED, "accept failed (%m)");

   return 0;
}

/* ======================================================================= */

static void initCheckLockFile(void)
{
   int fd;
   int count;
   int pid;
   int err;
   int delete;
   char str[20];

   fd = open(PI_LOCKFILE, O_RDONLY);

   if (fd != -1)
   {
      DBG(DBG_STARTUP, "lock file exists");
      delete = 1;

      count = read(fd, str, sizeof(str)-1);

      if (count)
      {
         pid = atoi(str);
         err = kill(pid, 0);
         if (!err) delete = 0; /* process still exists */
         DBG(DBG_STARTUP, "lock file pid=%d err=%d", pid, err);
      }

      close(fd);
      DBG(DBG_STARTUP, "lock file delete=%d", delete);

      if (delete) unlink(PI_LOCKFILE);
   }
}

static int initGrabLockFile(void)
{
   int fd;
   int lockResult;
   char pidStr[20];

   initCheckLockFile();

   /* try to grab the lock file */

   fd = open(PI_LOCKFILE, O_WRONLY|O_CREAT|O_EXCL|O_TRUNC, 0644);

   if (fd != -1)
   {
      lockResult = flock(fd, LOCK_EX|LOCK_NB);

      if(lockResult == 0)
      {
         sprintf(pidStr, "%d\n", (int)getpid());

         if (write(fd, pidStr, strlen(pidStr)) == -1)
         {
            /* ignore errors */
         }
      }
      else
      {
         close(fd);
         return -1;
      }
   }

   return fd;
}

/* ----------------------------------------------------------------------- */

static uint32_t * initMapMem(int fd, uint32_t addr, uint32_t len)
{
    return (uint32_t *) mmap(0, len,
       PROT_READ|PROT_WRITE,
       MAP_SHARED|MAP_LOCKED,
       fd, addr);
}

/* ----------------------------------------------------------------------- */

static int initCheckPermitted(void)
{
   DBG(DBG_STARTUP, "");

   if (!pi_ispi)
   {
      DBG(DBG_ALWAYS,
         "\n" \
         "+---------------------------------------------------------+\n" \
         "|Sorry, this system does not appear to be a raspberry pi. |\n" \
         "|aborting.                                                |\n" \
         "+---------------------------------------------------------+\n\n");
      return -1;
   }

   if ((fdMem = open("/dev/mem", O_RDWR | O_SYNC) ) < 0)
   {
      DBG(DBG_ALWAYS,
         "\n" \
         "+---------------------------------------------------------+\n" \
         "|Sorry, you don't have permission to run this program.    |\n" \
         "|Try running as root, e.g. precede the command with sudo. |\n" \
         "+---------------------------------------------------------+\n\n");
      return -1;
   }
   return 0;
}

/* ----------------------------------------------------------------------- */

static int initPeripherals(void)
{
   DBG(DBG_STARTUP, "");

   gpioReg = initMapMem(fdMem, GPIO_BASE, GPIO_LEN);

   if (gpioReg == MAP_FAILED)
      SOFT_ERROR(PI_INIT_FAILED, "mmap gpio failed (%m)");

   dmaReg = initMapMem(fdMem, DMA_BASE, DMA_LEN);

   if (dmaReg == MAP_FAILED)
      SOFT_ERROR(PI_INIT_FAILED, "mmap dma failed (%m)");

   /* we should know if we are running on a BCM2711 by now */

   if (gpioCfg.DMAprimaryChannel == PI_DEFAULT_DMA_NOT_SET)
   {
      if (pi_is_2711)
         gpioCfg.DMAprimaryChannel = PI_DEFAULT_DMA_PRIMARY_CH_2711;
      else
         gpioCfg.DMAprimaryChannel = PI_DEFAULT_DMA_PRIMARY_CHANNEL;
   }
      
   if (gpioCfg.DMAsecondaryChannel == PI_DEFAULT_DMA_NOT_SET)
   {
      if (pi_is_2711)
         gpioCfg.DMAsecondaryChannel = PI_DEFAULT_DMA_SECONDARY_CH_2711;
      else
         gpioCfg.DMAsecondaryChannel = PI_DEFAULT_DMA_SECONDARY_CHANNEL;
   }
      
   dmaIn =  dmaReg + (gpioCfg.DMAprimaryChannel   * 0x40);
   dmaOut = dmaReg + (gpioCfg.DMAsecondaryChannel * 0x40);

   DBG(DBG_STARTUP, "DMA #%d @ %08"PRIXPTR,
      gpioCfg.DMAprimaryChannel, (uintptr_t)dmaIn);

   DBG(DBG_STARTUP, "debug reg is %08X", dmaIn[DMA_DEBUG]);

   clkReg  = initMapMem(fdMem, CLK_BASE,  CLK_LEN);

   if (clkReg == MAP_FAILED)
      SOFT_ERROR(PI_INIT_FAILED, "mmap clk failed (%m)");

   systReg  = initMapMem(fdMem, SYST_BASE,  SYST_LEN);

   if (systReg == MAP_FAILED)
      SOFT_ERROR(PI_INIT_FAILED, "mmap syst failed (%m)");

   spiReg  = initMapMem(fdMem, SPI_BASE,  SPI_LEN);

   if (spiReg == MAP_FAILED)
      SOFT_ERROR(PI_INIT_FAILED, "mmap spi failed (%m)");

   pwmReg  = initMapMem(fdMem, PWM_BASE,  PWM_LEN);

   if (pwmReg == MAP_FAILED)
      SOFT_ERROR(PI_INIT_FAILED, "mmap pwm failed (%m)");

   pcmReg  = initMapMem(fdMem, PCM_BASE,  PCM_LEN);

   if (pcmReg == MAP_FAILED)
      SOFT_ERROR(PI_INIT_FAILED, "mmap pcm failed (%m)");

   auxReg  = initMapMem(fdMem, AUX_BASE,  AUX_LEN);

   if (auxReg == MAP_FAILED)
      SOFT_ERROR(PI_INIT_FAILED, "mmap aux failed (%m)");

   padsReg  = initMapMem(fdMem, PADS_BASE,  PADS_LEN);

   if (padsReg == MAP_FAILED)
      SOFT_ERROR(PI_INIT_FAILED, "mmap pads failed (%m)");

   bscsReg  = initMapMem(fdMem, BSCS_BASE,  BSCS_LEN);

   if (bscsReg == MAP_FAILED)
      SOFT_ERROR(PI_INIT_FAILED, "mmap bscs failed (%m)");

   return 0;
}

/* ----------------------------------------------------------------------- */

static int initZaps
   (int  pmapFd, void *virtualBase, int  basePage, int  pages)
{
   int n;
   uintptr_t index;
   off_t offset;
   ssize_t t;
   uint32_t physical;
   int status;
   uintptr_t pageAdr;
   unsigned long long pa;

   DBG(DBG_STARTUP, "");

   status = 0;

   pageAdr = (uintptr_t) dmaVirt[basePage];

   index  = ((uintptr_t)virtualBase / PAGE_SIZE) * 8;

   offset = lseek(pmapFd, index, SEEK_SET);

   if (offset != index)
      SOFT_ERROR(PI_INIT_FAILED, "lseek pagemap failed (%m)");

   for (n=0; n<pages; n++)
   {
      t = read(pmapFd, &pa, sizeof(pa));

      if (t != sizeof(pa))
         SOFT_ERROR(PI_INIT_FAILED, "read pagemap failed (%m)");

      DBG(DBG_STARTUP, "pf%d=%016llX", n, pa);

      physical = 0x3FFFFFFF & (PAGE_SIZE * (pa & 0xFFFFFFFF));

      if (physical)
      {
         //cast twice to suppress warning, I belive this is ok as these
         //are bus addresses, not virtual addresses. --plugwash
         dmaBus[basePage+n] = (dmaPage_t *)(uintptr_t) (physical | pi_dram_bus);

         dmaVirt[basePage+n] = mmap
         (
            (void *)pageAdr,
            PAGE_SIZE,
            PROT_READ|PROT_WRITE,
            MAP_SHARED|MAP_FIXED|MAP_LOCKED|MAP_NORESERVE,
            fdMem,
            physical
         );
      }
      else status = 1;

      pageAdr += PAGE_SIZE;
   }

   return status;
}

/* ----------------------------------------------------------------------- */

static int initPagemapBlock(int block)
{
   int trys, ok;
   unsigned pageNum;

   DBG(DBG_STARTUP, "block=%d", block);

   dmaPMapBlk[block] = mmap(
       0, (PAGES_PER_BLOCK*PAGE_SIZE),
       PROT_READ|PROT_WRITE,
       MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
       -1, 0);

   if (dmaPMapBlk[block] == MAP_FAILED)
      SOFT_ERROR(PI_INIT_FAILED, "mmap dma block %d failed (%m)", block);

   /* force allocation of physical memory */

   memset((void *)dmaPMapBlk[block], 0xAA, (PAGES_PER_BLOCK*PAGE_SIZE));

   memset((void *)dmaPMapBlk[block], 0xFF, (PAGES_PER_BLOCK*PAGE_SIZE));

   memset((void *)dmaPMapBlk[block], 0, (PAGES_PER_BLOCK*PAGE_SIZE));

   pageNum = block * PAGES_PER_BLOCK;

   dmaVirt[pageNum] = mmap(
       0, (PAGES_PER_BLOCK*PAGE_SIZE),
       PROT_READ|PROT_WRITE,
       MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
       -1, 0);

   if (dmaVirt[pageNum] == MAP_FAILED)
      SOFT_ERROR(PI_INIT_FAILED, "mmap dma block %d failed (%m)", block);

   munmap(dmaVirt[pageNum], PAGES_PER_BLOCK*PAGE_SIZE);

   trys = 0;
   ok = 0;

   while ((trys < 10) && !ok)
   {
      if (initZaps(fdPmap,
                    dmaPMapBlk[block],
                    pageNum,
                    PAGES_PER_BLOCK) == 0) ok = 1;
      else myGpioDelay(50000);

      ++trys;
   }

   if (!ok) SOFT_ERROR(PI_INIT_FAILED, "initZaps failed");

   return 0;
}

static int initMboxBlock(int block)
{
   int n, ok;
   unsigned page;
   uintptr_t virtualAdr;
   uintptr_t busAdr;

   DBG(DBG_STARTUP, "block=%d", block);

   ok = mbDMAAlloc
      (&dmaMboxBlk[block], PAGES_PER_BLOCK * PAGE_SIZE, pi_mem_flag);

   if (!ok) SOFT_ERROR(PI_INIT_FAILED, "init mbox zaps failed");

   page = block * PAGES_PER_BLOCK;

   virtualAdr = (uintptr_t) dmaMboxBlk[block].virtual_addr;
   busAdr = dmaMboxBlk[block].bus_addr;

   for (n=0; n<PAGES_PER_BLOCK; n++)
   {
      dmaVirt[page+n] = (dmaPage_t *) virtualAdr;
      dmaBus[page+n] = (dmaPage_t *) busAdr;
      virtualAdr += PAGE_SIZE;
      busAdr += PAGE_SIZE;
   }

   return 0;
}

/* ----------------------------------------------------------------------- */

static int initAllocDMAMem(void)
{
   int i, servoCycles, superCycles;
   int status;

   DBG(DBG_STARTUP, "");

   /* Calculate the number of blocks needed for buffers.  The number
      of blocks must be a multiple of the 20ms servo cycle.
   */

   servoCycles = gpioCfg.bufferMilliseconds / 20;
   if           (gpioCfg.bufferMilliseconds % 20) servoCycles++;

   bufferCycles = (SUPERCYCLE * servoCycles) / gpioCfg.clockMicros;

   superCycles = bufferCycles / SUPERCYCLE;
   if           (bufferCycles % SUPERCYCLE) superCycles++;

   bufferCycles = SUPERCYCLE * superCycles;

   bufferBlocks = bufferCycles / CYCLES_PER_BLOCK;

   DBG(DBG_STARTUP, "bmillis=%d mics=%d bblk=%d bcyc=%d",
      gpioCfg.bufferMilliseconds, gpioCfg.clockMicros,
      bufferBlocks, bufferCycles);

   /* allocate memory for pointers to virtual and bus memory pages */

   dmaVirt = mmap(
       0, PAGES_PER_BLOCK*(bufferBlocks+PI_WAVE_BLOCKS)*sizeof(dmaPage_t *),
       PROT_READ|PROT_WRITE,
       MAP_PRIVATE|MAP_ANONYMOUS|MAP_LOCKED,
       -1, 0);

   if (dmaVirt == MAP_FAILED)
      SOFT_ERROR(PI_INIT_FAILED, "mmap dma virtual failed (%m)");

   dmaBus = mmap(
       0, PAGES_PER_BLOCK*(bufferBlocks+PI_WAVE_BLOCKS)*sizeof(dmaPage_t *),
       PROT_READ|PROT_WRITE,
       MAP_PRIVATE|MAP_ANONYMOUS|MAP_LOCKED,
       -1, 0);

   if (dmaBus == MAP_FAILED)
      SOFT_ERROR(PI_INIT_FAILED, "mmap dma bus failed (%m)");

   dmaIVirt = (dmaIPage_t **) dmaVirt;
   dmaIBus  = (dmaIPage_t **) dmaBus;

   dmaOVirt = (dmaOPage_t **)(dmaVirt + (PAGES_PER_BLOCK*bufferBlocks));
   dmaOBus  = (dmaOPage_t **)(dmaBus  + (PAGES_PER_BLOCK*bufferBlocks));

   if ((gpioCfg.memAllocMode == PI_MEM_ALLOC_PAGEMAP) ||
       ((gpioCfg.memAllocMode == PI_MEM_ALLOC_AUTO) &&
        (gpioCfg.bufferMilliseconds > PI_DEFAULT_BUFFER_MILLIS)))
   {
      /* pagemap allocation of DMA memory */

      dmaPMapBlk = mmap(
          0, (bufferBlocks+PI_WAVE_BLOCKS)*sizeof(dmaPage_t *),
          PROT_READ|PROT_WRITE,
          MAP_PRIVATE|MAP_ANONYMOUS|MAP_LOCKED,
          -1, 0);

      if (dmaPMapBlk == MAP_FAILED)
         SOFT_ERROR(PI_INIT_FAILED, "pagemap mmap block failed (%m)");

      fdPmap = open("/proc/self/pagemap", O_RDONLY);

      if (fdPmap < 0)
         SOFT_ERROR(PI_INIT_FAILED, "pagemap open failed(%m)");

      for (i=0; i<(bufferBlocks+PI_WAVE_BLOCKS); i++)
      {
         status = initPagemapBlock(i);
         if (status < 0)
         {
            close(fdPmap);
            return status;
         }
      }

      close(fdPmap);

      DBG(DBG_STARTUP, "dmaPMapBlk=%08"PRIXPTR" dmaIn=%08"PRIXPTR,
         (uintptr_t)dmaPMapBlk, (uintptr_t)dmaIn);
   }
   else
   {
      /* mailbox allocation of DMA memory */

      dmaMboxBlk = mmap(
          0, (bufferBlocks+PI_WAVE_BLOCKS)*sizeof(DMAMem_t),
          PROT_READ|PROT_WRITE,
          MAP_PRIVATE|MAP_ANONYMOUS|MAP_LOCKED,
          -1, 0);

      if (dmaMboxBlk == MAP_FAILED)
         SOFT_ERROR(PI_INIT_FAILED, "mmap mbox block failed (%m)");

      fdMbox = mbOpen();

      if (fdMbox < 0)
         SOFT_ERROR(PI_INIT_FAILED, "mbox open failed(%m)");

      for (i=0; i<(bufferBlocks+PI_WAVE_BLOCKS); i++)
      {
         status = initMboxBlock(i);
         if (status < 0)
         {
            mbClose(fdMbox);
            return status;
         }
      }

      mbClose(fdMbox);

      DBG(DBG_STARTUP, "dmaMboxBlk=%08"PRIXPTR" dmaIn=%08"PRIXPTR,
         (uintptr_t)dmaMboxBlk, (uintptr_t)dmaIn);
   }

   DBG(DBG_STARTUP,
      "gpioReg=%08"PRIXPTR" pwmReg=%08"PRIXPTR" pcmReg=%08"PRIXPTR" clkReg=%08"PRIXPTR" auxReg=%08"PRIXPTR,
      (uintptr_t)gpioReg, (uintptr_t)pwmReg,
      (uintptr_t)pcmReg,  (uintptr_t)clkReg, (uintptr_t)auxReg);

   for (i=0; i<DMAI_PAGES; i++)
      DBG(DBG_STARTUP, "dmaIBus[%d]=%08"PRIXPTR, i, (uintptr_t)dmaIBus[i]);

   if (gpioCfg.dbgLevel >= DBG_DMACBS)
   {
      fprintf(stderr, "*** INPUT DMA CONTROL BLOCKS ***\n");
      for (i=0; i<NUM_CBS; i++) dmaCbPrint(i);
   }

   return 0;
}

/* ----------------------------------------------------------------------- */

static void initPWM(unsigned bits)
{
   DBG(DBG_STARTUP, "bits=%d", bits);

   /* reset PWM */

   pwmReg[PWM_CTL] = 0;

   myGpioDelay(10);

   pwmReg[PWM_STA] = -1;

   myGpioDelay(10);

   /* set number of bits to transmit */

   pwmReg[PWM_RNG1] = bits;

   myGpioDelay(10);

   dmaIVirt[0]->periphData = 1;

   /* enable PWM DMA, raise panic and dreq thresholds to 15 */

   pwmReg[PWM_DMAC] = PWM_DMAC_ENAB      |
                      PWM_DMAC_PANIC(15) |
                      PWM_DMAC_DREQ(15);

   myGpioDelay(10);

   /* clear PWM fifo */

   pwmReg[PWM_CTL] = PWM_CTL_CLRF1;

   myGpioDelay(10);

   /* enable PWM channel 1 and use fifo */

   pwmReg[PWM_CTL] = PWM_CTL_USEF1 | PWM_CTL_MODE1 | PWM_CTL_PWEN1;
}

/* ----------------------------------------------------------------------- */

static void initPCM(unsigned bits)
{
   DBG(DBG_STARTUP, "bits=%d", bits);

   /* disable PCM so we can modify the regs */

   pcmReg[PCM_CS] = 0;

   myGpioDelay(1000);

   pcmReg[PCM_FIFO]   = 0;
   pcmReg[PCM_MODE]   = 0;
   pcmReg[PCM_RXC]    = 0;
   pcmReg[PCM_TXC]    = 0;
   pcmReg[PCM_DREQ]   = 0;
   pcmReg[PCM_INTEN]  = 0;
   pcmReg[PCM_INTSTC] = 0;
   pcmReg[PCM_GRAY]   = 0;

   myGpioDelay(1000);

   pcmReg[PCM_MODE] = PCM_MODE_FLEN(bits-1); /* # bits in frame */

   /* enable channel 1 with # bits width */

   pcmReg[PCM_TXC] = PCM_TXC_CH1EN | PCM_TXC_CH1WID(bits-8);

   pcmReg[PCM_CS] |= PCM_CS_STBY; /* clear standby */

   myGpioDelay(1000);

   pcmReg[PCM_CS] |= PCM_CS_TXCLR; /* clear TX FIFO */

   pcmReg[PCM_CS] |= PCM_CS_DMAEN; /* enable DREQ */

   pcmReg[PCM_DREQ] = PCM_DREQ_TX_PANIC(16) | PCM_DREQ_TX_REQ_L(30);

   pcmReg[PCM_INTSTC] = 0b1111; /* clear status bits */

   /* enable PCM */

   pcmReg[PCM_CS] |= PCM_CS_EN;

   /* enable tx */

   pcmReg[PCM_CS] |= PCM_CS_TXON;

   dmaIVirt[0]->periphData = 0x0F;
}

/* ----------------------------------------------------------------------- */

static void initHWClk
   (int clkCtl, int clkDiv, int clkSrc, int divI, int divF, int MASH)
{
   DBG(DBG_INTERNAL, "ctl=%d div=%d src=%d /I=%d /f=%d M=%d",
      clkCtl, clkDiv, clkSrc, divI, divF, MASH);

   /* kill the clock if busy, anything else isn't reliable */

   if (clkReg[clkCtl] & CLK_CTL_BUSY)
   {
      do
      {
         clkReg[clkCtl] = BCM_PASSWD | CLK_CTL_KILL;
      }
      while (clkReg[clkCtl] & CLK_CTL_BUSY);
   }

   clkReg[clkDiv] = (BCM_PASSWD | CLK_DIV_DIVI(divI) | CLK_DIV_DIVF(divF));

   usleep(10);

   clkReg[clkCtl] = (BCM_PASSWD | CLK_CTL_MASH(MASH) | CLK_CTL_SRC(clkSrc));

   usleep(10);

   clkReg[clkCtl] |= (BCM_PASSWD | CLK_CTL_ENAB);
}

static void initClock(int mainClock)
{
   const unsigned BITS=10;
   int clockPWM;
   unsigned clkCtl, clkDiv, clkSrc, clkDivI, clkDivF, clkMash, clkBits;
   char *per;
   unsigned micros;

   DBG(DBG_STARTUP, "mainClock=%d", mainClock);

   if (mainClock) micros = gpioCfg.clockMicros;
   else           micros = PI_WF_MICROS;

   clockPWM = mainClock ^ (gpioCfg.clockPeriph == PI_CLOCK_PCM);

   if (clockPWM)
   {
      clkCtl = CLK_PWMCTL;
      clkDiv = CLK_PWMDIV;
      per = "PWM";
   }
   else
   {
      clkCtl = CLK_PCMCTL;
      clkDiv = CLK_PCMDIV;
      per = "PCM";
   }

   clkSrc  = CLK_CTL_SRC_PLLD;
   clkDivI = clk_plld_freq / (10000000 / micros); /* 10 MHz - 1 MHz */ 
   clkBits = BITS;        /* 10/BITS MHz - 1/BITS MHz */
   clkDivF = 0;
   clkMash = 0;

   DBG(DBG_STARTUP, "%s PLLD divi=%d divf=%d mash=%d bits=%d",
      per, clkDivI, clkDivF, clkMash, clkBits);

   initHWClk(clkCtl, clkDiv, clkSrc, clkDivI, clkDivF, clkMash);

   if (clockPWM) initPWM(BITS);
   else          initPCM(BITS);

   myGpioDelay(2000);
}

static void initKillDMA(volatile uint32_t *dmaAddr)
{
   dmaAddr[DMA_CS] = DMA_CHANNEL_ABORT;
   dmaAddr[DMA_CS] = 0;
   dmaAddr[DMA_CS] = DMA_CHANNEL_RESET;

   dmaAddr[DMA_CONBLK_AD] = 0;
}

/* ----------------------------------------------------------------------- */

static void initDMAgo(volatile uint32_t  *dmaAddr, uint32_t cbAddr)
{
   DBG(DBG_STARTUP, "");

   initKillDMA(dmaAddr);

   dmaAddr[DMA_CS] = DMA_INTERRUPT_STATUS | DMA_END_FLAG;

   dmaAddr[DMA_CONBLK_AD] = cbAddr;

   /* clear READ/FIFO/READ_LAST_NOT_SET error bits */

   dmaAddr[DMA_DEBUG] = DMA_DEBUG_READ_ERR            |
                        DMA_DEBUG_FIFO_ERR            |
                        DMA_DEBUG_RD_LST_NOT_SET_ERR;


   dmaAddr[DMA_CS] = DMA_WAIT_ON_WRITES    |
                     DMA_PANIC_PRIORITY(8) |
                     DMA_PRIORITY(8)       |
                     DMA_ACTIVE;
}

/* ----------------------------------------------------------------------- */

static void initClearGlobals(void)
{
   int i;

   DBG(DBG_STARTUP, "");

   alertBits   = 0;
   monitorBits = 0;
   notifyBits  = 0;
   scriptBits  = 0;
   gFilterBits = 0;
   nFilterBits = 0;
   wdogBits    = 0;

   pthAlertRunning  = PI_THREAD_NONE;
   pthFifoRunning   = PI_THREAD_NONE;
   pthSocketRunning = PI_THREAD_NONE;

   wfc[0] = 0;
   wfc[1] = 0;
   wfc[2] = 0;

   wfcur=0;

   wfStats.micros     = 0;
   wfStats.highMicros = 0;
   wfStats.maxMicros  = PI_WAVE_MAX_MICROS;

   wfStats.pulses     = 0;
   wfStats.highPulses = 0;
   wfStats.maxPulses  = PI_WAVE_MAX_PULSES;

   wfStats.cbs        = 0;
   wfStats.highCbs    = 0;
   wfStats.maxCbs     = (PI_WAVE_BLOCKS * PAGES_PER_BLOCK * CBS_PER_OPAGE);

   gpioGetSamples.func     = NULL;
   gpioGetSamples.ex       = 0;
   gpioGetSamples.userdata = NULL;
   gpioGetSamples.bits     = 0;

   for (i=0; i<=PI_MAX_USER_GPIO; i++)
   {
      wfRx[i].mode      = PI_WFRX_NONE;
      pthread_mutex_init(&wfRx[i].mutex, NULL);
      gpioAlert[i].func = NULL;
   }

   for (i=0; i<=PI_MAX_GPIO; i++)
   {
      gpioInfo [i].is      = GPIO_UNDEFINED;
      gpioInfo [i].width   = 0;
      gpioInfo [i].range   = PI_DEFAULT_DUTYCYCLE_RANGE;
      gpioInfo [i].freqIdx = DEFAULT_PWM_IDX;
   }

   for (i=0; i<PI_NOTIFY_SLOTS; i++)
   {
      gpioNotify[i].seqno = 0;
      gpioNotify[i].state = PI_NOTIFY_CLOSED;
   }

   for (i=0; i<=PI_MAX_SIGNUM; i++)
   {
      gpioSignal[i].func     = NULL;
      gpioSignal[i].ex       = 0;
      gpioSignal[i].userdata = NULL;
   }

   for (i=0; i<=PI_MAX_TIMER; i++)
   {
      gpioTimer[i].running = 0;
      gpioTimer[i].func    = NULL;
   }

   for (i=0; i<=PI_MAX_EVENT; i++)
   {
      eventAlert[i].func      = NULL;
      eventAlert[i].ignore    = 0;
      eventAlert[i].fired     = 0;
   }

   /* calculate the usable PWM frequencies */

   for (i=0; i<PWM_FREQS; i++)
   {
      pwmFreq[i]=
         (1000000.0/
            ((float)PULSE_PER_CYCLE*gpioCfg.clockMicros*pwmCycles[i]))+0.5;

      DBG(DBG_STARTUP, "f%d is %d", i, pwmFreq[i]);
   }

   inpFifo = NULL;
   outFifo = NULL;

   fdLock       = -1;
   fdMem        = -1;
   fdSock       = -1;

   dmaMboxBlk = MAP_FAILED;
   dmaPMapBlk = MAP_FAILED;
   dmaVirt = MAP_FAILED;
   dmaBus  = MAP_FAILED;

   auxReg  = MAP_FAILED;
   clkReg  = MAP_FAILED;
   dmaReg  = MAP_FAILED;
   gpioReg = MAP_FAILED;
   pcmReg  = MAP_FAILED;
   pwmReg  = MAP_FAILED;
   systReg = MAP_FAILED;
   spiReg  = MAP_FAILED;
}

/* ----------------------------------------------------------------------- */

static void initReleaseResources(void)
{
   int i;

   DBG(DBG_STARTUP, "");

   /* shut down running threads */

   for (i=0; i<=PI_MAX_GPIO; i++)
   {
      if (gpioISR[i].pth)
      {
         /* destroy thread, unexport GPIO */

         gpioSetISRFunc(i, 0, 0, NULL);
      }
   }

   for (i=0; i<=PI_MAX_TIMER; i++)
   {
      if (gpioTimer[i].running)
      {
         /* destroy thread */

         pthread_cancel(gpioTimer[i].pthId);
         pthread_join(gpioTimer[i].pthId, NULL);
         gpioTimer[i].running = 0;
      }
   }

   if (pthAlertRunning != PI_THREAD_NONE)
   {
      pthread_cancel(pthAlert);
      pthread_join(pthAlert, NULL);
      pthAlertRunning = PI_THREAD_NONE;
   }

   if (pthFifoRunning != PI_THREAD_NONE)
   {
      pthread_cancel(pthFifo);
      pthread_join(pthFifo, NULL);
      pthFifoRunning = PI_THREAD_NONE;
   }

   if (pthSocketRunning != PI_THREAD_NONE)
   {
      pthread_cancel(pthSocket);
      pthread_join(pthSocket, NULL);
      pthSocketRunning = PI_THREAD_NONE;
   }

   /* release mmap'd memory */

   if (auxReg  != MAP_FAILED) munmap((void *)auxReg,  AUX_LEN);
   if (bscsReg != MAP_FAILED) munmap((void *)bscsReg, BSCS_LEN);
   if (clkReg  != MAP_FAILED) munmap((void *)clkReg,  CLK_LEN);
   if (dmaReg  != MAP_FAILED) munmap((void *)dmaReg,  DMA_LEN);
   if (gpioReg != MAP_FAILED) munmap((void *)gpioReg, GPIO_LEN);
   if (pcmReg  != MAP_FAILED) munmap((void *)pcmReg,  PCM_LEN);
   if (pwmReg  != MAP_FAILED) munmap((void *)pwmReg,  PWM_LEN);
   if (systReg != MAP_FAILED) munmap((void *)systReg, SYST_LEN);
   if (spiReg  != MAP_FAILED) munmap((void *)spiReg,  SPI_LEN);

   auxReg  = MAP_FAILED;
   bscsReg = MAP_FAILED;
   clkReg  = MAP_FAILED;
   dmaReg  = MAP_FAILED;
   gpioReg = MAP_FAILED;
   pcmReg  = MAP_FAILED;
   pwmReg  = MAP_FAILED;
   systReg = MAP_FAILED;
   spiReg  = MAP_FAILED;

   if (dmaBus != MAP_FAILED)
   {
      munmap(dmaBus,
         PAGES_PER_BLOCK*(bufferBlocks+PI_WAVE_BLOCKS)*sizeof(dmaPage_t *));
   }

   dmaBus = MAP_FAILED;

   if (dmaVirt != MAP_FAILED)
   {
      for (i=0; i<PAGES_PER_BLOCK*(bufferBlocks+PI_WAVE_BLOCKS); i++)
      {
         munmap(dmaVirt[i], PAGE_SIZE);
      }

      munmap(dmaVirt,
         PAGES_PER_BLOCK*(bufferBlocks+PI_WAVE_BLOCKS)*sizeof(dmaPage_t *));
   }

   dmaVirt = MAP_FAILED;

   if (dmaPMapBlk != MAP_FAILED)
   {
      for (i=0; i<(bufferBlocks+PI_WAVE_BLOCKS); i++)
      {
         munmap(dmaPMapBlk[i], PAGES_PER_BLOCK*PAGE_SIZE);
      }

      munmap(dmaPMapBlk, (bufferBlocks+PI_WAVE_BLOCKS)*sizeof(dmaPage_t *));
   }

   dmaPMapBlk = MAP_FAILED;

   if (dmaMboxBlk != MAP_FAILED)
   {
      fdMbox = mbOpen();

      for (i=0; i<(bufferBlocks+PI_WAVE_BLOCKS); i++)
      {
         mbDMAFree(&dmaMboxBlk[bufferBlocks+PI_WAVE_BLOCKS-i-1]);
      }

      mbClose(fdMbox);

      munmap(dmaMboxBlk, (bufferBlocks+PI_WAVE_BLOCKS)*sizeof(DMAMem_t));
   }

   dmaMboxBlk = MAP_FAILED;

   if (inpFifo != NULL)
   {
      fclose(inpFifo);
      unlink(PI_INPFIFO);
      inpFifo = NULL;
   }

   if (outFifo != NULL)
   {
      fclose(outFifo);
      unlink(PI_OUTFIFO);
      outFifo = NULL;
   }

   if (fdMem != -1)
   {
      close(fdMem);
      fdMem = -1;
   }

   if (fdLock != -1)
   {
      close(fdLock);
      unlink(PI_LOCKFILE);
      fdLock = -1;
   }

   if (fdSock != -1)
   {
      close(fdSock);
      fdSock = -1;
   }

   if (fdPmap != -1)
   {
      close(fdPmap);
      fdPmap = -1;
   }

   if (fdMbox != -1)
   {
      close(fdMbox);
      fdMbox = -1;
   }

   gpioStats.DMARestarts = 0;
   gpioStats.dmaInitCbsCount = 0;

   numSockNetAddr = 0;
}

int initInitialise(void)
{
   int i;
   unsigned rev, model;
   struct sockaddr_in server;
   struct sockaddr_in6 server6;
   char * portStr;
   unsigned port;
   struct sched_param param;
   pthread_attr_t pthAttr;

   DBG(DBG_STARTUP, "");

   waveClockInited = 0;
   PWMClockInited = 0;

   clock_gettime(CLOCK_REALTIME, &libStarted);

   rev = gpioHardwareRevision();

   initClearGlobals();

   if (initCheckPermitted() < 0) return PI_INIT_FAILED;

   fdLock = initGrabLockFile();

   if (fdLock < 0)
      SOFT_ERROR(PI_INIT_FAILED, "Can't lock %s", PI_LOCKFILE);

   if (!gpioMaskSet)
   {
      if      (rev ==  0) gpioMask = PI_DEFAULT_UPDATE_MASK_UNKNOWN;
      else if (rev <   4) gpioMask = PI_DEFAULT_UPDATE_MASK_B1;
      else if (rev <  16) gpioMask = PI_DEFAULT_UPDATE_MASK_A_B2;
      else if (rev == 17) gpioMask = PI_DEFAULT_UPDATE_MASK_COMPUTE;
      else if (rev  < 20) gpioMask = PI_DEFAULT_UPDATE_MASK_APLUS_BPLUS;
      else if (rev == 20) gpioMask = PI_DEFAULT_UPDATE_MASK_COMPUTE;
      else if (rev == 21) gpioMask = PI_DEFAULT_UPDATE_MASK_APLUS_BPLUS;
      else
      {
         model = (rev >> 4) & 0xFF;

         /* model
         0=A 1=B
         2=A+ 3=B+
         4=Pi2B
         5=Alpha
         6=Compute Module
         7=Unknown
         8=Pi3B
         9=Zero
         12=Zero W
         13=Pi3B+
         14=Pi3A+
         17=Pi4B
         */
         if      (model <  2) gpioMask = PI_DEFAULT_UPDATE_MASK_A_B2;
         else if (model <  4) gpioMask = PI_DEFAULT_UPDATE_MASK_APLUS_BPLUS;
         else if (model == 4) gpioMask = PI_DEFAULT_UPDATE_MASK_PI2B;

         else if (model == 6
               || model ==10
               || model ==16) gpioMask = PI_DEFAULT_UPDATE_MASK_COMPUTE;

         else if (model == 8
               || model ==13
               || model ==14) gpioMask = PI_DEFAULT_UPDATE_MASK_PI3B;

         else if (model == 9
               || model ==12) gpioMask = PI_DEFAULT_UPDATE_MASK_ZERO;

         else if (model ==17) gpioMask = PI_DEFAULT_UPDATE_MASK_PI4B;
         else                 gpioMask = PI_DEFAULT_UPDATE_MASK_UNKNOWN;
      }

      gpioMaskSet = 1;
   }

#ifndef EMBEDDED_IN_VM
   if (!(gpioCfg.internals & PI_CFG_NOSIGHANDLER))
      sigSetHandler();
#endif

   if (initPeripherals() < 0) return PI_INIT_FAILED;

   if (initAllocDMAMem() < 0) return PI_INIT_FAILED;

   /* done with /dev/mem */

   if (fdMem != -1)
   {
      close(fdMem);
      fdMem = -1;
   }

   param.sched_priority = sched_get_priority_max(SCHED_FIFO);

   if (gpioCfg.internals & PI_CFG_RT_PRIORITY)
      sched_setscheduler(0, SCHED_FIFO, &param);

   initClock(1); /* initialise main clock */

   atexit(gpioTerminate);

   if (pthread_attr_init(&pthAttr))
      SOFT_ERROR(PI_INIT_FAILED, "pthread_attr_init failed (%m)");

   if (pthread_attr_setstacksize(&pthAttr, STACK_SIZE))
      SOFT_ERROR(PI_INIT_FAILED, "pthread_attr_setstacksize failed (%m)");

   if (!(gpioCfg.ifFlags & PI_DISABLE_ALERT))
   {
      if (pthread_create(&pthAlert, &pthAttr, pthAlertThread, &i))
         SOFT_ERROR(PI_INIT_FAILED, "pthread_create alert failed (%m)");

      pthAlertRunning = PI_THREAD_STARTED;
   }

   if (!(gpioCfg.ifFlags & PI_DISABLE_FIFO_IF))
   {
      if (pthread_create(&pthFifo, &pthAttr, pthFifoThread, &i))
         SOFT_ERROR(PI_INIT_FAILED, "pthread_create fifo failed (%m)");

      pthFifoRunning = PI_THREAD_STARTED;
   }

   if (!(gpioCfg.ifFlags & PI_DISABLE_SOCK_IF))
   {
      portStr = getenv(PI_ENVPORT);
      if (portStr) port = atoi(portStr); else port = gpioCfg.socketPort;

      // Accept connections on IPv6, unless we have an IPv4-only whitelist
      if (!numSockNetAddr)
      {
         fdSock = socket(AF_INET6, SOCK_STREAM , 0);

         if (fdSock != -1)
         {
            bzero((char *)&server6, sizeof(server6));
            server6.sin6_family = AF_INET6;
            if (gpioCfg.ifFlags & PI_LOCALHOST_SOCK_IF)
            {
               server6.sin6_addr = in6addr_loopback;
            }
            else
            {
               server6.sin6_addr = in6addr_any;
            }
            server6.sin6_port = htons(port);

            int opt = 1;
            setsockopt(fdSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            if (bind(fdSock,(struct sockaddr *)&server6, sizeof(server6)) < 0)
               SOFT_ERROR(PI_INIT_FAILED, "bind to port %d failed (%m)", port);
         }
      }

      if (numSockNetAddr || fdSock == -1)
      {
         fdSock = socket(AF_INET , SOCK_STREAM , 0);

         if (fdSock == -1)
            SOFT_ERROR(PI_INIT_FAILED, "socket failed (%m)");
         else
         {
           int opt = 1;
           setsockopt(fdSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
         }
         server.sin_family = AF_INET;
         if (gpioCfg.ifFlags & PI_LOCALHOST_SOCK_IF)
         {
            server.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
         }
         else
         {
            server.sin_addr.s_addr = htonl(INADDR_ANY);
         }
         server.sin_port = htons(port);

         if (bind(fdSock,(struct sockaddr *)&server , sizeof(server)) < 0)
            SOFT_ERROR(PI_INIT_FAILED, "bind to port %d failed (%m)", port);
      }

      if (pthread_create(&pthSocket, &pthAttr, pthSocketThread, &i))
         SOFT_ERROR(PI_INIT_FAILED, "pthread_create socket failed (%m)");

      pthSocketRunning = PI_THREAD_STARTED;
   }

   myGpioDelay(1000);

   dmaInitCbs();

   flushMemory();

   //cast twice to suppress compiler warning, I belive this cast
   //is ok because dmaIBus contains bus addresses, not virtual
   //addresses.
   initDMAgo((uint32_t *)dmaIn, (uint32_t)(uintptr_t)dmaIBus[0]);

   return PIGPIO_VERSION;
}


/* ======================================================================= */

int getBitInBytes(int bitPos, char *buf, int numBits)
{
   int bitp, bufp;

   if (bitPos < numBits)
   {
      bufp =      bitPos / 8;
      bitp = 7 - (bitPos % 8);
      if (buf[bufp] & (1<<bitp)) return 1;
   }

   return 0;
}

/* ----------------------------------------------------------------------- */

void putBitInBytes(int bitPos, char *buf, int bit)
{
   int bitp, bufp;

   bufp =      bitPos / 8;
   bitp = 7 - (bitPos % 8);

   if (bit) buf[bufp] |=   (1<<bitp);
   else     buf[bufp] &= (~(1<<bitp));
}

/* ----------------------------------------------------------------------- */

uint32_t rawWaveGetOOL(int pos)
{
   int page, slot;

   if ((pos >= 0) && (pos < NUM_WAVE_OOL))
   {
      waveOOLPageSlot(pos, &page, &slot);
      return (dmaOVirt[page]->OOL[slot]);
   }

   return -1;
}

/* ----------------------------------------------------------------------- */

void rawWaveSetOOL(int pos, uint32_t value)
{
   int page, slot;

   if ((pos >= 0) && (pos < NUM_WAVE_OOL))
   {
      waveOOLPageSlot(pos, &page, &slot);
      dmaOVirt[page]->OOL[slot] = value;
   }
}


/* ----------------------------------------------------------------------- */

uint32_t rawWaveGetOut(int pos)
{
   int page, slot;

   if ((pos >= 0) && (pos < NUM_WAVE_OOL))
   {
      waveOOLPageSlot(pos, &page, &slot);
      return (dmaOVirt[page]->OOL[slot]);
   }

   return -1;
}

/* ----------------------------------------------------------------------- */

void rawWaveSetOut(int pos, uint32_t value)
{
   int page, slot;

   if ((pos >= 0) && (pos < NUM_WAVE_OOL))
   {
      waveOOLPageSlot(pos, &page, &slot);
      dmaOVirt[page]->OOL[slot] = value;
   }
}


/* ----------------------------------------------------------------------- */

uint32_t rawWaveGetIn(int pos)
{
   int page, slot;

   if ((pos >= 0) && (pos < NUM_WAVE_OOL))
   {
      waveOOLPageSlot((NUM_WAVE_OOL-1)-pos, &page, &slot);
      return (dmaOVirt[page]->OOL[slot]);
   }

   return -1;
}

/* ----------------------------------------------------------------------- */

void rawWaveSetIn(int pos, uint32_t value)
{
   int page, slot;

   if ((pos >= 0) && (pos < NUM_WAVE_OOL))
   {
      waveOOLPageSlot((NUM_WAVE_OOL-1)-pos, &page, &slot);
      dmaOVirt[page]->OOL[slot] = value;
   }
}

/* ----------------------------------------------------------------------- */

rawWaveInfo_t rawWaveInfo(int wave_id)
{
   rawWaveInfo_t dummy = {0, 0, 0, 0, 0, 0, 0, 0};

   if ((wave_id >=0) && (wave_id < PI_MAX_WAVES)) return waveInfo[wave_id];
   else                                           return dummy;
}

/* ----------------------------------------------------------------------- */

double time_time(void)
{
   struct timeval tv;
   double t;

   gettimeofday(&tv, 0);

   t = (double)tv.tv_sec + ((double)tv.tv_usec / 1E6);

   return t;
}

/* ----------------------------------------------------------------------- */

void time_sleep(double seconds)
{
   struct timespec ts, rem;

   if (seconds > 0.0)
   {
      ts.tv_sec = seconds;
      ts.tv_nsec = (seconds-(double)ts.tv_sec) * 1E9;

      while (clock_nanosleep(CLOCK_REALTIME, 0, &ts, &rem))
      {
         /* copy remaining time to ts */
         ts.tv_sec  = rem.tv_sec;
         ts.tv_nsec = rem.tv_nsec;
      }
   }
}

/* ----------------------------------------------------------------------- */

void rawDumpWave(void)
{
   int i;

   unsigned numWaves, t;

   rawWave_t *waves;

   numWaves = wfc[wfcur];
   waves    = wf [wfcur];

   t = 0;

   for (i=0; i<numWaves; i++)
   {
      fprintf(stderr, "%10u %08X %08X %08X %10u\n",
         t, waves[i].gpioOn, waves[i].gpioOff,
         waves[i].flags, waves[i].usDelay);
      t += waves[i].usDelay;
   }
}

/* ----------------------------------------------------------------------- */

void rawDumpScript(unsigned script_id)
{
   int i;

   if (script_id >= PI_MAX_SCRIPTS) return;

   if (gpioScript[script_id].state == PI_SCRIPT_IN_USE)
   {

      for (i=0; i<PI_MAX_SCRIPT_PARAMS; i++)
      {
         fprintf(stderr, "p%d=%d ", i, gpioScript[script_id].script.par[i]);
      }

      fprintf(stderr, "\n");

      for (i=0; i<PI_MAX_SCRIPT_VARS; i++)
      {
         fprintf(stderr, "v%d=%d ", i, gpioScript[script_id].script.var[i]);
      }

      fprintf(stderr, "\n");

      for (i=0; i<gpioScript[script_id].script.instrs; i++)
      {
         fprintf(stderr,
            "c%d=[%"PRIdPTR", %"PRIdPTR"(%d), %"PRIdPTR"(%d), %"PRIdPTR", %"PRIdPTR"]\n",
            i,
            gpioScript[script_id].script.instr[i].p[0],
            gpioScript[script_id].script.instr[i].p[1],
            gpioScript[script_id].script.instr[i].opt[1],
            gpioScript[script_id].script.instr[i].p[2],
            gpioScript[script_id].script.instr[i].opt[2],
            gpioScript[script_id].script.instr[i].p[3],
            gpioScript[script_id].script.instr[i].p[4]);
      }
   }
}

/* ======================================================================= */

int gpioInitialise(void)
{
   int status;

   if (libInitialised) return PIGPIO_VERSION;

   DBG(DBG_STARTUP, "not initialised, initialising");

   runState = PI_STARTING;

   status = initInitialise();

   if (status < 0)
   {
      runState = PI_ENDING;
      initReleaseResources();
   }
   else
   {
      libInitialised = 1;

      runState = PI_RUNNING;

      if (!(gpioCfg.ifFlags & PI_DISABLE_ALERT))
      {
         while (pthAlertRunning != PI_THREAD_RUNNING) myGpioDelay(1000);
      }

   }

   return status;
}


/* ----------------------------------------------------------------------- */

void gpioTerminate(void)
{
   int i;

   DBG(DBG_USER, "");

   if (!libInitialised) return;

   DBG(DBG_STARTUP, "initialised, terminating");

   runState = PI_ENDING;

   gpioMaskSet = 0;

   /* reset DMA */

   if (dmaReg != MAP_FAILED)
   {
      initKillDMA(dmaIn);
      initKillDMA(dmaOut);
   }

#ifndef EMBEDDED_IN_VM
   if ((gpioCfg.internals & PI_CFG_STATS) &&
       (!(gpioCfg.internals & PI_CFG_NOSIGHANDLER)))
   {
      fprintf(stderr,
         "\n#####################################################\n");
      fprintf(stderr, "pigpio version=%d internals=%X\n",
         PIGPIO_VERSION, gpioCfg.internals);

      fprintf(stderr,
         "micros=%d allocMode=%d dmaInitCbs=%d DMARestarts=%d\n",
         gpioCfg.clockMicros, gpioCfg.memAllocMode,
         gpioStats.dmaInitCbsCount, gpioStats.DMARestarts);

      fprintf(stderr,
         "samples %u maxSamples %u maxEmit %u emitFrags %u\n",
         gpioStats.numSamples, gpioStats.maxSamples,
         gpioStats.maxEmit, gpioStats.emitFrags);

      fprintf(stderr, "cbTicks %d, cbCalls %u\n",
         gpioStats.cbTicks, gpioStats.cbCalls);

      fprintf(stderr, "pipe: good %u, short %u, would block %u\n",
         gpioStats.goodPipeWrite, gpioStats.shortPipeWrite,
         gpioStats.wouldBlockPipeWrite);

      fprintf(stderr, "alertTicks %u, lateTicks %u, moreToDo %u\n",
         gpioStats.alertTicks, gpioStats.lateTicks, gpioStats.moreToDo);

      for (i=0; i< TICKSLOTS; i++)
         fprintf(stderr, "%9u ", gpioStats.diffTick[i]);

      fprintf(stderr,
         "\n#####################################################\n\n\n");
   }

#endif
   initReleaseResources();

   fflush(NULL);

   libInitialised = 0;
}

static void switchFunctionOff(unsigned gpio)
{
   switch (gpioInfo[gpio].is)
   {
      case GPIO_SERVO:
         /* switch servo off */
         myGpioSetServo(gpio, gpioInfo[gpio].width, 0);
         gpioInfo[gpio].width = 0;
         break;

      case GPIO_PWM:
         /* switch pwm off */
         myGpioSetPwm(gpio, gpioInfo[gpio].width, 0);
         gpioInfo[gpio].width = 0;
         break;

      case GPIO_HW_CLK:
         /* No longer disable clock hardware, doing that was a bug. */
         gpioInfo[gpio].width = 0;
         break;

      case GPIO_HW_PWM:
         /* No longer disable PWM hardware, doing that was a bug. */
         gpioInfo[gpio].width = 0;
         break;
   }
}

static void stopHardwarePWM(void)
{
   unsigned i, pwm;

   for (i=0; i<= PI_MAX_GPIO; i++)
   {
      if (gpioInfo[i].is == GPIO_HW_PWM)
      {
         pwm = (PWMDef[i] >> 4) & 3;

         if (pwm == 0) pwmReg[PWM_CTL] &= (~PWM_CTL_PWEN1);
         else          pwmReg[PWM_CTL] &= (~PWM_CTL_PWEN2);

         gpioInfo[i].width = 0;
         gpioInfo[i].is = GPIO_UNDEFINED;
      }
   }
}

/* ----------------------------------------------------------------------- */

int gpioSetMode(unsigned gpio, unsigned mode)
{
   int reg, shift, old_mode;

   DBG(DBG_USER, "gpio=%d mode=%d", gpio, mode);

   CHECK_INITED;

   if (gpio > PI_MAX_GPIO)
      SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);

   if (mode > PI_ALT3)
      SOFT_ERROR(PI_BAD_MODE, "gpio %d, bad mode (%d)", gpio, mode);

   reg   =  gpio/10;
   shift = (gpio%10) * 3;

   old_mode = (gpioReg[reg] >> shift) & 7;

   if (mode != old_mode)
   {
      switchFunctionOff(gpio);

      gpioInfo[gpio].is = GPIO_UNDEFINED;
   }

   gpioReg[reg] = (gpioReg[reg] & ~(7<<shift)) | (mode<<shift);

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioGetMode(unsigned gpio)
{
   int reg, shift;

   DBG(DBG_USER, "gpio=%d", gpio);

   CHECK_INITED;

   if (gpio > PI_MAX_GPIO)
      SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);

   reg   =  gpio/10;
   shift = (gpio%10) * 3;

   return (gpioReg[reg] >> shift) & 7;
}


/* ----------------------------------------------------------------------- */

int gpioSetPullUpDown(unsigned gpio, unsigned pud)
{
   int shift = (gpio & 0xf) << 1;
   uint32_t bits;
   uint32_t pull;

   DBG(DBG_USER, "gpio=%d pud=%d", gpio, pud);

   CHECK_INITED;

   if (gpio > PI_MAX_GPIO)
      SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);

   if (pud > PI_PUD_UP)
      SOFT_ERROR(PI_BAD_PUD, "gpio %d, bad pud (%d)", gpio, pud);

   if (pi_is_2711)
   {
      switch (pud)
      {
         case PI_PUD_OFF:  pull = 0; break;
         case PI_PUD_UP:   pull = 1; break;
         case PI_PUD_DOWN: pull = 2; break;
      }

      bits = *(gpioReg + GPPUPPDN0 + (gpio>>4));
      bits &= ~(3 << shift);
      bits |= (pull << shift);
      *(gpioReg + GPPUPPDN0 + (gpio>>4)) = bits;
   }
   else
   {
      *(gpioReg + GPPUD) = pud;

      myGpioDelay(1);

      *(gpioReg + GPPUDCLK0 + BANK) = BIT;

      myGpioDelay(1);

      *(gpioReg + GPPUD) = 0;

      *(gpioReg + GPPUDCLK0 + BANK) = 0;
   }

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioRead(unsigned gpio)
{
   DBG(DBG_USER, "gpio=%d", gpio);

   CHECK_INITED;

   if (gpio > PI_MAX_GPIO)
      SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);

   if ((*(gpioReg + GPLEV0 + BANK) & BIT) != 0) return PI_ON;
   else                                         return PI_OFF;
}


/* ----------------------------------------------------------------------- */

int gpioWrite(unsigned gpio, unsigned level)
{
   DBG(DBG_USER, "gpio=%d level=%d", gpio, level);

   CHECK_INITED;

   if (gpio > PI_MAX_GPIO)
      SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);

   if (level > PI_ON)
      SOFT_ERROR(PI_BAD_LEVEL, "gpio %d, bad level (%d)", gpio, level);

   if (gpio <= PI_MAX_GPIO)
   {
      if (gpioInfo[gpio].is != GPIO_WRITE)
      {
         /* stop a glitch between setting mode then level */
         if (level == PI_OFF) *(gpioReg + GPCLR0 + BANK) = BIT;
         else                 *(gpioReg + GPSET0 + BANK) = BIT;

         switchFunctionOff(gpio);

         gpioInfo[gpio].is = GPIO_WRITE;
      }
   }

   myGpioSetMode(gpio, PI_OUTPUT);

   if (level == PI_OFF) *(gpioReg + GPCLR0 + BANK) = BIT;
   else                 *(gpioReg + GPSET0 + BANK) = BIT;

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioPWM(unsigned gpio, unsigned val)
{
   DBG(DBG_USER, "gpio=%d dutycycle=%d", gpio, val);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   if (val > gpioInfo[gpio].range)
      SOFT_ERROR(PI_BAD_DUTYCYCLE, "gpio %d, bad dutycycle (%d)", gpio, val);

   if (gpioInfo[gpio].is != GPIO_PWM)
   {
      switchFunctionOff(gpio);

      gpioInfo[gpio].is = GPIO_PWM;

      if (!val) myGpioWrite(gpio, 0);
   }

   myGpioSetMode(gpio, PI_OUTPUT);

   myGpioSetPwm(gpio, gpioInfo[gpio].width, val);

   gpioInfo[gpio].width=val;

   return 0;
}

/* ----------------------------------------------------------------------- */

int gpioGetPWMdutycycle(unsigned gpio)
{
   unsigned pwm;

   DBG(DBG_USER, "gpio=%d", gpio);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   switch (gpioInfo[gpio].is)
   {
      case GPIO_PWM:
         return gpioInfo[gpio].width;

      case GPIO_HW_PWM:
         pwm = (PWMDef[gpio] >> 4) & 3;
         return hw_pwm_duty[pwm];

      case GPIO_HW_CLK:
         return PI_HW_PWM_RANGE/2;

      default:
         SOFT_ERROR(PI_NOT_PWM_GPIO, "not a PWM gpio (%d)", gpio);
   }
}


/* ----------------------------------------------------------------------- */

int gpioSetPWMrange(unsigned gpio, unsigned range)
{
   int oldWidth, newWidth;

   DBG(DBG_USER, "gpio=%d range=%d", gpio, range);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   if ((range < PI_MIN_DUTYCYCLE_RANGE)  || (range > PI_MAX_DUTYCYCLE_RANGE))
      SOFT_ERROR(PI_BAD_DUTYRANGE, "gpio %d, bad range (%d)", gpio, range);

   oldWidth = gpioInfo[gpio].width;

   if (oldWidth)
   {
      if (gpioInfo[gpio].is == GPIO_PWM)
      {
         newWidth = (range * oldWidth) / gpioInfo[gpio].range;

         myGpioSetPwm(gpio, oldWidth, 0);
         gpioInfo[gpio].range = range;
         gpioInfo[gpio].width = newWidth;
         myGpioSetPwm(gpio, 0, newWidth);
      }
   }

   gpioInfo[gpio].range = range;

   /* return the actual range for the current gpio frequency */

   return pwmRealRange[gpioInfo[gpio].freqIdx];
}


/* ----------------------------------------------------------------------- */

int gpioGetPWMrange(unsigned gpio)
{
   DBG(DBG_USER, "gpio=%d", gpio);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   switch (gpioInfo[gpio].is)
   {
      case GPIO_HW_PWM:
      case GPIO_HW_CLK:
         return PI_HW_PWM_RANGE;

      default:
         return gpioInfo[gpio].range;
   }
}


/* ----------------------------------------------------------------------- */

int gpioGetPWMrealRange(unsigned gpio)
{
   unsigned pwm;

   DBG(DBG_USER, "gpio=%d", gpio);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   switch (gpioInfo[gpio].is)
   {
      case GPIO_HW_PWM:
         pwm = (PWMDef[gpio] >> 4) & 3;
         return hw_pwm_real_range[pwm];

      case GPIO_HW_CLK:
         return PI_HW_PWM_RANGE;

      default:
         return pwmRealRange[gpioInfo[gpio].freqIdx];
   }
}


/* ----------------------------------------------------------------------- */

int gpioSetPWMfrequency(unsigned gpio, unsigned frequency)
{
   int i, width;
   unsigned diff, best, idx;

   DBG(DBG_USER, "gpio=%d frequency=%d", gpio, frequency);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   if      (frequency > pwmFreq[0])           idx = 0;
   else if (frequency < pwmFreq[PWM_FREQS-1]) idx = PWM_FREQS-1;
   else
   {
      best = 100000; /* impossibly high frequency difference */
      idx = 0;

      for (i=0; i<PWM_FREQS; i++)
      {
         if (frequency > pwmFreq[i]) diff = frequency - pwmFreq[i];
         else                        diff = pwmFreq[i] - frequency;

         if (diff < best)
         {
            best = diff;
            idx = i;
         }
      }
   }

   width = gpioInfo[gpio].width;

   if (width)
   {
      if (gpioInfo[gpio].is == GPIO_PWM)
      {
         myGpioSetPwm(gpio, width, 0);
         gpioInfo[gpio].freqIdx = idx;
         myGpioSetPwm(gpio, 0, width);
      }
   }

   gpioInfo[gpio].freqIdx = idx;

   return pwmFreq[idx];
}


/* ----------------------------------------------------------------------- */

int gpioGetPWMfrequency(unsigned gpio)
{
   unsigned pwm, clock;

   DBG(DBG_USER, "gpio=%d", gpio);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   switch (gpioInfo[gpio].is)
   {
      case GPIO_HW_PWM:
         pwm = (PWMDef[gpio] >> 4) & 3;
         return hw_pwm_freq[pwm];

      case GPIO_HW_CLK:
         clock = (clkDef[gpio] >> 4) & 3;
         return hw_clk_freq[clock];

      default:
         return pwmFreq[gpioInfo[gpio].freqIdx];
   }
}


/* ----------------------------------------------------------------------- */

int gpioServo(unsigned gpio, unsigned val)
{
   DBG(DBG_USER, "gpio=%d pulsewidth=%d", gpio, val);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   if ((val!=PI_SERVO_OFF) && (val<PI_MIN_SERVO_PULSEWIDTH))
      SOFT_ERROR(PI_BAD_PULSEWIDTH,
         "gpio %d, bad pulsewidth (%d)", gpio, val);

   if (val>PI_MAX_SERVO_PULSEWIDTH)
      SOFT_ERROR(PI_BAD_PULSEWIDTH,
         "gpio %d, bad pulsewidth (%d)", gpio, val);

   if (gpioInfo[gpio].is != GPIO_SERVO)
   {
      switchFunctionOff(gpio);

      gpioInfo[gpio].is = GPIO_SERVO;

      if (!val) myGpioWrite(gpio, 0);
   }

   myGpioSetMode(gpio, PI_OUTPUT);

   myGpioSetServo(gpio, gpioInfo[gpio].width, val);

   gpioInfo[gpio].width=val;

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioGetServoPulsewidth(unsigned gpio)
{
   DBG(DBG_USER, "gpio=%d", gpio);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   if (gpioInfo[gpio].is != GPIO_SERVO)
      SOFT_ERROR(PI_NOT_SERVO_GPIO, "not a servo gpio (%d)", gpio);

   return gpioInfo[gpio].width;
}


/* ----------------------------------------------------------------------- */

int gpioWaveClear(void)
{
   DBG(DBG_USER, "");

   CHECK_INITED;

   wfc[0] = 0;
   wfc[1] = 0;
   wfc[2] = 0;

   wfcur = 0;

   wfStats.micros = 0;
   wfStats.pulses = 0;
   wfStats.cbs    = 0;

   waveOutBotCB  = PI_WAVE_COUNT_PAGES*CBS_PER_OPAGE;
   waveOutBotOOL = PI_WAVE_COUNT_PAGES*OOL_PER_OPAGE;
   waveOutTopOOL = NUM_WAVE_OOL;

   waveOutCount = 0;

   waveEndPtr = NULL;

   return 0;
}

/* ----------------------------------------------------------------------- */

int gpioWaveAddNew(void)
{
   DBG(DBG_USER, "");

   CHECK_INITED;

   wfc[0] = 0;
   wfc[1] = 0;
   wfc[2] = 0;

   wfcur = 0;

   wfStats.micros = 0;
   wfStats.pulses = 0;
   wfStats.cbs    = 0;

   return 0;
}

/* ----------------------------------------------------------------------- */

int gpioWaveAddGeneric(unsigned numPulses, gpioPulse_t *pulses)
{
   int p;

   DBG(DBG_USER, "numPulses=%u pulses=%08"PRIXPTR, numPulses, (uintptr_t)pulses);

   CHECK_INITED;

   if (numPulses > PI_WAVE_MAX_PULSES)
      SOFT_ERROR(PI_TOO_MANY_PULSES, "bad number of pulses (%d)", numPulses);

   if (!pulses) SOFT_ERROR(PI_BAD_POINTER, "bad (NULL) pulses pointer");

   for (p=0; p<numPulses; p++)
   {
      wf[2][p].gpioOff = pulses[p].gpioOff;
      wf[2][p].gpioOn  = pulses[p].gpioOn;
      wf[2][p].usDelay = pulses[p].usDelay;
      wf[2][p].flags   = 0;
   }

   return rawWaveAddGeneric(numPulses, wf[2]);
}

/* ----------------------------------------------------------------------- */

int gpioWaveAddSerial
   (unsigned gpio,
    unsigned baud,
    unsigned data_bits,
    unsigned stop_bits,
    unsigned offset,
    unsigned numBytes,
    char     *bstr)
{
   int i, b, p, lev, c, v;

   uint16_t *wstr = (uint16_t *)bstr;
   uint32_t *lstr = (uint32_t *)bstr;

   unsigned bitDelay[32];

   DBG(DBG_USER,
      "gpio=%d baud=%d bits=%d stops=%d offset=%d numBytes=%d str=[%s]",
      gpio, baud, data_bits, stop_bits, offset,
      numBytes, myBuf2Str(numBytes, (char *)bstr));

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   if ((baud < PI_WAVE_MIN_BAUD) || (baud > PI_WAVE_MAX_BAUD))
      SOFT_ERROR(PI_BAD_WAVE_BAUD, "bad baud rate (%d)", baud);

   if ((data_bits < PI_MIN_WAVE_DATABITS) ||
       (data_bits > PI_MAX_WAVE_DATABITS))
      SOFT_ERROR(PI_BAD_DATABITS, "bad number of databits (%d)", data_bits);

   if ((stop_bits < PI_MIN_WAVE_HALFSTOPBITS) ||
       (stop_bits > PI_MAX_WAVE_HALFSTOPBITS))
      SOFT_ERROR(PI_BAD_STOPBITS,
         "bad number of (half) stop bits (%d)", stop_bits);

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   if (numBytes > PI_WAVE_MAX_CHARS)
      SOFT_ERROR(PI_TOO_MANY_CHARS, "too many chars (%d)", numBytes);

   if (offset > PI_WAVE_MAX_MICROS)
      SOFT_ERROR(PI_BAD_SER_OFFSET, "offset too large (%d)", offset);

   if (data_bits > 8) numBytes /= 2;
   if (data_bits > 16) numBytes /= 2;

   if (!numBytes) return 0;

   waveBitDelay(baud, data_bits, stop_bits, bitDelay);

   p = 0;

   wf[2][p].gpioOn  = (1<<gpio);
   wf[2][p].gpioOff = 0;
   wf[2][p].flags   = 0;

   if (offset > bitDelay[0]) wf[2][p].usDelay = offset;
   else                      wf[2][p].usDelay = bitDelay[0];

   for (i=0; i<numBytes; i++)
   {
      p++;

      /* start bit */

      wf[2][p].gpioOn = 0;
      wf[2][p].gpioOff = (1<<gpio);
      wf[2][p].usDelay = bitDelay[0];
      wf[2][p].flags   = 0;

      lev = 0;

      if      (data_bits <  9) c = bstr[i];
      else if (data_bits < 17) c = wstr[i];
      else                  c = lstr[i];

      for (b=0; b<data_bits; b++)
      {
         if (c & (1<<b)) v=1; else v=0;

         if (v == lev) wf[2][p].usDelay += bitDelay[b+1];
         else
         {
            p++;

            lev = v;

            if (lev)
            {
               wf[2][p].gpioOn  = (1<<gpio);
               wf[2][p].gpioOff = 0;
               wf[2][p].flags   = 0;
            }
            else
            {
               wf[2][p].gpioOn  = 0;
               wf[2][p].gpioOff = (1<<gpio);
               wf[2][p].flags   = 0;
            }

            wf[2][p].usDelay = bitDelay[b+1];
         }
      }

      /* stop bit */

      if (lev) wf[2][p].usDelay += bitDelay[data_bits+1];
      else
      {
         p++;

         wf[2][p].gpioOn  = (1<<gpio);
         wf[2][p].gpioOff = 0;
         wf[2][p].usDelay = bitDelay[data_bits+1];
         wf[2][p].flags   = 0;
      }
   }

   p++;

   wf[2][p].gpioOn  = (1<<gpio);
   wf[2][p].gpioOff = 0;
   wf[2][p].usDelay = bitDelay[0];
   wf[2][p].flags   = 0;

   return rawWaveAddGeneric(p, wf[2]);
}

/* ----------------------------------------------------------------------- */

int rawWaveAddSPI(
   rawSPI_t *spi,
   unsigned offset,
   unsigned spiSS,
   char *buf,
   unsigned spiTxBits,
   unsigned spiBitFirst,
   unsigned spiBitLast,
   unsigned spiBits)
{
   int p, bit, dbv, halfbit;
   int rising_edge[2], read_cycle[2];
   uint32_t on_bits, off_bits;
   int tx_bit_pos;

   DBG(DBG_USER,
      "spi=%08"PRIXPTR" off=%d spiSS=%d tx=%08"PRIXPTR", num=%d fb=%d lb=%d spiBits=%d",
      (uintptr_t)spi, offset, spiSS, (uintptr_t)buf, spiTxBits,
      spiBitFirst, spiBitLast, spiBits);

   CHECK_INITED;

   if (spiSS > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", spiSS);

   /*
   CPOL CPHA
    0    0   read rising/write falling
    0    1   read falling/write rising
    1    0   read falling/write rising
    1    1   read rising/write falling
   */

   if (spi->clk_pol) {rising_edge[0] = 0; rising_edge[1] = 1;}
   else              {rising_edge[0] = 1; rising_edge[1] = 0;}

   if (spi->clk_pha) {read_cycle[0] = 0; read_cycle[1] = 1;}
   else              {read_cycle[0] = 1; read_cycle[1] = 0;}

   p = 0;

   if (offset)
   {
      wf[2][p].gpioOn  = 0;
      wf[2][p].gpioOff = 0;
      wf[2][p].flags   = 0;
      wf[2][p].usDelay = offset;
      p++;
   }

   on_bits = 0;
   off_bits = 0;

   tx_bit_pos = 0;

   /* preset initial mosi bit */

   if (getBitInBytes(tx_bit_pos, buf, spiTxBits))
   {
      on_bits  |= (1<<(spi->mosi));
      dbv = 1;
   }
   else
   {
      off_bits |= (1<<(spi->mosi));
      dbv = 0;
   }

   if (!spi->clk_pha) tx_bit_pos ++;

   if (spi->ss_pol) off_bits |= (1<<spiSS);
   else             on_bits  |= (1<<spiSS);

   if (spi->clk_pol) on_bits  |= (1<<(spi->clk));
   else              off_bits |= (1<<(spi->clk));

   wf[2][p].gpioOn  = on_bits;
   wf[2][p].gpioOff = off_bits;
   wf[2][p].flags   = 0;

   if (spi->clk_us > spi->ss_us) wf[2][p].usDelay = spi->clk_us;
   else                          wf[2][p].usDelay = spi->ss_us;

   p++;

   for (bit=1; bit<=spiBits; bit++)
   {
      for (halfbit=0; halfbit<2; halfbit++)
      {
         wf[2][p].usDelay = spi->clk_us;
         wf[2][p].flags = 0;

         on_bits = 0;
         off_bits = 0;

         if (read_cycle[halfbit])
         {
            if ((bit>=spiBitFirst) && (bit<=spiBitLast))
               wf[2][p].flags = WAVE_FLAG_READ;
         }
         else
         {
            if (getBitInBytes(tx_bit_pos, buf, spiTxBits))
            {
               if (!dbv) on_bits  |= (1<<(spi->mosi));
               dbv = 1;
            }
            else
            {
               if (dbv) off_bits |= (1<<(spi->mosi));
               dbv = 0;
            }

            ++tx_bit_pos;
         }

         if (rising_edge[halfbit]) on_bits  |= (1<<(spi->clk));
         else                      off_bits |= (1<<(spi->clk));

         wf[2][p].gpioOn = on_bits;
         wf[2][p].gpioOff = off_bits;

         p++;
      }
   }

   on_bits = 0;
   off_bits = 0;

   if (spi->ss_pol) on_bits  |= (1<<spiSS);
   else             off_bits |= (1<<spiSS);

   wf[2][p].gpioOn  = on_bits;
   wf[2][p].gpioOff = off_bits;
   wf[2][p].flags   = 0;
   wf[2][p].usDelay = 0;

   p++;

   return rawWaveAddGeneric(p, wf[2]);
}

/* ----------------------------------------------------------------------- */

int gpioWaveCreate(void)
{
   int i, wid;
   int numCB, numBOOL, numTOOL;
   int CB, BOOL, TOOL;

   DBG(DBG_USER, "");

   CHECK_INITED;

   if (wfc[wfcur] == 0) return PI_EMPTY_WAVEFORM;

   /* What resources are needed? */

      waveCBsOOLs(&numCB, &numBOOL, &numTOOL);

   wid = -1;

   /* Is there an exact fit with a deleted wave. */

   for (i=0; i<waveOutCount; i++)
   {
      if (waveInfo[i].deleted             &&
         (waveInfo[i].numCB   == numCB)   &&
         (waveInfo[i].numBOOL == numBOOL) &&
         (waveInfo[i].numTOOL == numTOOL))
      {
         /* Reuse the deleted waves resources. */
         wid = i;
         break;
      }
   }

   if (wid == -1)
   {
      /* Are there enough spare resources? */

      if ((numCB+waveOutBotCB) > NUM_WAVE_CBS)
         return PI_TOO_MANY_CBS;

      if ((numBOOL+waveOutBotOOL) > (waveOutTopOOL-numTOOL))
         return PI_TOO_MANY_OOL;

      if (wid >= PI_MAX_WAVES)
         return PI_NO_WAVEFORM_ID;

      wid = waveOutCount++;

      waveInfo[wid].botCB  = waveOutBotCB;
      waveInfo[wid].topCB  = waveOutBotCB + numCB -1;
      waveInfo[wid].botOOL = waveOutBotOOL;
      waveInfo[wid].topOOL = waveOutTopOOL;
      waveInfo[wid].numCB = numCB;
      waveInfo[wid].numBOOL = numBOOL;
      waveInfo[wid].numTOOL = numTOOL;

      waveOutBotCB += numCB;
      waveOutBotOOL += numBOOL;
      waveOutTopOOL -= numTOOL;
   }

   /* Must be room if got this far. */

   CB   = waveInfo[wid].botCB;
   BOOL = waveInfo[wid].botOOL;
   TOOL = waveInfo[wid].topOOL;

   wave2Cbs(PI_WAVE_MODE_ONE_SHOT, &CB, &BOOL, &TOOL, 0, 0, 0);

   /* Sanity check. */

   if ( (numCB   != (CB-waveInfo[wid].botCB))    ||
        (numBOOL != (BOOL-waveInfo[wid].botOOL)) ||
        (numTOOL != (waveInfo[wid].topOOL-TOOL)) )
   {
      DBG(DBG_ALWAYS, "ERROR wid=%d CBs %d=%d BOOL %d=%d TOOL %d=%d", wid,
         numCB,   CB-waveInfo[wid].botCB,
         numBOOL, BOOL-waveInfo[wid].botOOL,
         numTOOL, waveInfo[wid].topOOL-TOOL);
   }

   DBG(DBG_USER, "Wave Stats: wid=%d CBs %d BOOL %d TOOL %d", wid,
      numCB, numBOOL, numTOOL);

   waveInfo[wid].deleted = 0;

   /* Consume waves. */

   wfc[0] = 0;
   wfc[1] = 0;
   wfc[2] = 0;

   wfcur = 0;

   return wid;
}

int gpioWaveCreatePad(int pctCB, int pctBOOL, int pctTOOL)
{
   int i, wid;
   int numCB, numBOOL, numTOOL;
   int CB, BOOL, TOOL;

   DBG(DBG_USER, "%d, %d, %d", pctCB, pctBOOL, pctTOOL);

   CHECK_INITED;

   if (pctCB < 0 || pctCB > 100)
      SOFT_ERROR(PI_BAD_PARAM, "bad wave param, pctCB=(%d)", pctCB);
   if (pctBOOL < 0 || pctBOOL > 100)
      SOFT_ERROR(PI_BAD_PARAM, "bad wave param, pctBOOL=(%d)", pctBOOL);
   if (pctTOOL < 0 || pctTOOL > 100)
      SOFT_ERROR(PI_BAD_PARAM, "bad wave param, pctTOOL=(%d)", pctTOOL);

   if (wfc[wfcur] == 0) return PI_EMPTY_WAVEFORM;

   /* What resources are needed? */
   waveCBsOOLs(&numCB, &numBOOL, &numTOOL);

   /* Amount of pad required */
   CB = (NUM_WAVE_CBS - PI_WAVE_COUNT_PAGES*CBS_PER_OPAGE) * pctCB / 100;
   BOOL = (NUM_WAVE_OOL - PI_WAVE_COUNT_PAGES*OOL_PER_OPAGE) * pctBOOL /100;
   TOOL = (NUM_WAVE_OOL - PI_WAVE_COUNT_PAGES*OOL_PER_OPAGE) * pctTOOL /100;

   /* Reject if wave is too big */
   if (numCB > CB) return PI_TOO_MANY_CBS;
   if (numBOOL > BOOL) return PI_TOO_MANY_OOL;
   if (numTOOL > TOOL) return PI_TOO_MANY_OOL;

   /* Set the padding */
   numCB = CB;
   numBOOL = BOOL;
   numTOOL = TOOL;


   wid = -1;

   /* Is there an exact fit with a deleted wave. */

   for (i=0; i<waveOutCount; i++)
   {
      if (waveInfo[i].deleted             &&
         (waveInfo[i].numCB   == numCB)   &&
         (waveInfo[i].numBOOL == numBOOL) &&
         (waveInfo[i].numTOOL == numTOOL))
      {
         /* Reuse the deleted waves resources. */
         wid = i;
         break;
      }
   }

   if (wid == -1)
   {
      /* Are there enough spare resources? */

      if ((numCB+waveOutBotCB) > NUM_WAVE_CBS)
         return PI_TOO_MANY_CBS;

      if ((numBOOL+waveOutBotOOL) > (waveOutTopOOL-numTOOL))
         return PI_TOO_MANY_OOL;

      if (wid >= PI_MAX_WAVES)
         return PI_NO_WAVEFORM_ID;

      wid = waveOutCount++;

      waveInfo[wid].botCB  = waveOutBotCB;
      waveInfo[wid].topCB  = waveOutBotCB + numCB -1;
      waveInfo[wid].botOOL = waveOutBotOOL;
      waveInfo[wid].topOOL = waveOutTopOOL;
      waveInfo[wid].numCB = numCB;
      waveInfo[wid].numBOOL = numBOOL;
      waveInfo[wid].numTOOL = numTOOL;

      waveOutBotCB += numCB;
      waveOutBotOOL += numBOOL;
      waveOutTopOOL -= numTOOL;
   }

   /* Must be room if got this far. */

   CB   = waveInfo[wid].botCB;
   BOOL = waveInfo[wid].botOOL;
   TOOL = waveInfo[wid].topOOL;

   wave2Cbs(PI_WAVE_MODE_ONE_SHOT, &CB, &BOOL, &TOOL, numCB, numBOOL, numTOOL);

   /* Sanity check. */

   if ( (numCB   != (CB-waveInfo[wid].botCB))    ||
        (numBOOL != (BOOL-waveInfo[wid].botOOL)) ||
        (numTOOL != (waveInfo[wid].topOOL-TOOL)) )
   {
      DBG(DBG_ALWAYS, "ERROR wid=%d CBs %d=%d BOOL %d=%d TOOL %d=%d", wid,
         numCB,   CB-waveInfo[wid].botCB,
         numBOOL, BOOL-waveInfo[wid].botOOL,
         numTOOL, waveInfo[wid].topOOL-TOOL);
   }

   DBG(DBG_USER, "Wave padding: wid=%d CBs %d BOOL %d TOOL %d", wid,
      numCB, numBOOL, numTOOL);

   waveInfo[wid].deleted = 0;

   /* Consume waves. */

   wfc[0] = 0;
   wfc[1] = 0;
   wfc[2] = 0;

   wfcur = 0;

   return wid;
}
/* ----------------------------------------------------------------------- */

int gpioWaveDelete(unsigned wave_id)
{
   DBG(DBG_USER, "wave id=%d", wave_id);

   CHECK_INITED;

   if ((wave_id >= waveOutCount) || waveInfo[wave_id].deleted)
      SOFT_ERROR(PI_BAD_WAVE_ID, "bad wave id (%d)", wave_id);

   waveInfo[wave_id].deleted = 1;

   if (wave_id == (waveOutCount-1))
   {
      /* top wave deleted, garbage collect any other deleted waves */

      while ((wave_id > 0) && (waveInfo[wave_id-1].deleted)) --wave_id;

      waveOutBotCB  = waveInfo[wave_id].botCB;
      waveOutBotOOL = waveInfo[wave_id].botOOL;
      waveOutTopOOL = waveInfo[wave_id].topOOL;

      waveOutCount = wave_id;
   }

   return 0;
}

/* ----------------------------------------------------------------------- */

int gpioWaveTxStart(unsigned wave_mode)
{
   /* This function is deprecated and has been removed. */

   CHECK_INITED;

   SOFT_ERROR(PI_DEPRECATED, "deprected function removed");
}

/* ----------------------------------------------------------------------- */

int gpioWaveTxSend(unsigned wave_id, unsigned wave_mode)
{
   rawCbs_t *p=NULL;

   DBG(DBG_USER, "wave_id=%d wave_mode=%d", wave_id, wave_mode);

   CHECK_INITED;

   if ((wave_id >= waveOutCount) || waveInfo[wave_id].deleted)
      SOFT_ERROR(PI_BAD_WAVE_ID, "bad wave id (%d)", wave_id);

   if (wave_mode > PI_WAVE_MODE_REPEAT_SYNC)
      SOFT_ERROR(PI_BAD_WAVE_MODE, "bad wave mode (%d)", wave_mode);

   if (!waveClockInited)
   {
      stopHardwarePWM();
      initClock(0); /* initialise secondary clock */
      waveClockInited = 1;
      PWMClockInited = 0;
   }

   if (wave_mode < PI_WAVE_MODE_ONE_SHOT_SYNC) initKillDMA(dmaOut);

   p = rawWaveCBAdr(waveInfo[wave_id].topCB);

   if ((wave_mode & 1) == PI_WAVE_MODE_ONE_SHOT)
      p->next = 0;
   else
      p->next = waveCbPOadr(waveInfo[wave_id].botCB+1);

   if (waveEndPtr && (wave_mode > PI_WAVE_MODE_REPEAT))
   {
      *waveEndPtr = waveCbPOadr(waveInfo[wave_id].botCB+1);

      if (!dmaOut[DMA_CONBLK_AD])
      {
         initDMAgo((uint32_t *)dmaOut, waveCbPOadr(waveInfo[wave_id].botCB));
      }
   }
   else
   {
      initDMAgo((uint32_t *)dmaOut, waveCbPOadr(waveInfo[wave_id].botCB));
   }

   waveEndPtr = &p->next;

   /* for compatability with the deprecated gpioWaveTxStart return the
      number of cbs
   */
   return (waveInfo[wave_id].topCB - waveInfo[wave_id].botCB) + 1;
}


/* ----------------------------------------------------------------------- */

static int chainGetCB(int n)
{
   int block, index;

   if (n < (WCB_CHAIN_CBS * PI_WAVE_COUNT_PAGES))
   {
      block = n / WCB_CHAIN_CBS;
      index = n % WCB_CHAIN_CBS;
      return (block*CBS_PER_OPAGE) + WCB_COUNTER_CBS + index;
   }
   return -1;
}

static void chainSetVal(int n, uint32_t val)
{
   int block, index;
   uint32_t *p;

   if (n < (WCB_CHAIN_OOL * PI_WAVE_COUNT_PAGES))
   {
      block = n / WCB_CHAIN_OOL;
      index = n % WCB_CHAIN_OOL;
      p = (uint32_t *) dmaOVirt[block] + (WCB_COUNTER_CBS+WCB_CHAIN_CBS) * 8;
      p[index] = val;
   }
}

static uint32_t chainGetValPadr(int n)
{
   int block, index;
   uint32_t *p;

   if (n < (WCB_CHAIN_OOL * PI_WAVE_COUNT_PAGES))
   {
      block = n / WCB_CHAIN_OOL;
      index = n % WCB_CHAIN_OOL;
      p = (uint32_t *) dmaOBus[block] + (WCB_COUNTER_CBS+WCB_CHAIN_CBS) * 8;
      //cast twice to suppress warning, I belive this is ok as dmaOBus
      //contains bus addresses not virtual addresses.
      return (uint32_t)(uintptr_t) (p + index);
   }
   return 0;
}

static uint32_t chainGetCntVal(int counter, int slot)
{
   uint32_t *p;
   int page, offset;
   page = counter / 2;
   offset = (counter % 2 ? WCB_COUNTER_OOL : 0);
   p = (uint32_t *) dmaOVirt[page] + (WCB_COUNTER_CBS+WCB_CHAIN_CBS) * 8;
   return p[WCB_CHAIN_OOL+ offset + slot];
}

static void chainSetCntVal(int counter, int slot, uint32_t value)
{
   uint32_t *p;
   int page, offset;
   page = counter / 2;
   offset = (counter % 2 ? WCB_COUNTER_OOL : 0);
   p = (uint32_t *) dmaOVirt[page] + (WCB_COUNTER_CBS+WCB_CHAIN_CBS) * 8;
   p[WCB_CHAIN_OOL + offset + slot] = value;
}

static uint32_t chainGetCntValPadr(int counter, int slot)
{
   uint32_t *p;
   int page, offset;
   page = counter / 2;
   offset = (counter % 2 ? WCB_COUNTER_OOL : 0);
   p = (uint32_t *) dmaOBus[page] + (WCB_COUNTER_CBS+WCB_CHAIN_CBS) * 8;
   //cast twice to suppress warning, I belive this is ok as dmaOBus
   //contains bus addresses not virtual addresses. --plugwash
   return (uint32_t)(uintptr_t)(p + WCB_CHAIN_OOL + offset + slot);
}

static int chainGetCntCB(int counter)
{
   int page, offset;
   page = counter / 2;
   offset = (counter % 2 ? WCB_CNT_CBS : 0);
   return ((page * CBS_PER_OPAGE) + offset);
}

static void chainMakeCounter(
   unsigned counter,
   unsigned blklen,
   unsigned blocks,
   unsigned count,
   uint32_t repeat,
   uint32_t next)
{
   rawCbs_t *p=NULL;

   int b, baseCB, dig;
   uint32_t nxt;

   int botCB;

   botCB  = chainGetCntCB(counter);

   baseCB = botCB;

   /* set up all the OOLs */
   for (b=0; b < (blocks*(blklen+1)); b++) chainSetCntVal(counter, b, repeat);

   for (b=0; b<blocks; b++)
      chainSetCntVal(counter,
         ((b*(blklen+1))+blklen),
         waveCbPOadr(baseCB+((b*3)+3)));

   for (b=0; b<blocks; b++)
   {
      /* copy BOTTOM to NEXT */

      p = rawWaveCBAdr(botCB++);

      p->info = NORMAL_DMA;

      p->src = chainGetCntValPadr(counter, b*(blklen+1));
      p->dst = (waveCbPOadr(botCB+1) + 20);

      p->length = 4;
      p->next   = waveCbPOadr(botCB);

      /* copy BOTTOM to TOP */

      p = rawWaveCBAdr(botCB++);

      p->info   = NORMAL_DMA;

      p->src = chainGetCntValPadr(counter, b*(blklen+1));
      p->dst = chainGetCntValPadr(counter, (b*(blklen+1))+blklen);

      p->length = 4;
      p->next   = waveCbPOadr(botCB);

      /* shift all down one */

      p = rawWaveCBAdr(botCB++);

      p->info   = NORMAL_DMA|DMA_SRC_INC|DMA_DEST_INC;

      p->src = chainGetCntValPadr(counter, ((b*(blklen+1))+1));
      p->dst = chainGetCntValPadr(counter, ((b*(blklen+1))+0));

      p->length = blklen*4;
      p->next   = repeat;
   }

   /* reset the counter */

   p = rawWaveCBAdr(botCB);

   p->info = NORMAL_DMA|DMA_SRC_INC|DMA_DEST_INC;

   p->src = chainGetCntValPadr(counter, blocks*(blklen+1));
   p->dst = chainGetCntValPadr(counter, 0);

   p->length = blocks*(blklen+1)*4;
   p->next   = next;

   b = 0;

   while (count && (b<blocks))
   {
      dig = count % blklen;
      count /= blklen;

      if (count) nxt = chainGetCntVal(counter, (b*(blklen+1))+blklen);
      else       nxt = waveCbPOadr(botCB);

      chainSetCntVal(counter, b*(blklen+1)+dig, nxt);

      b++;
   }

   /* copy all the OOLs */
   for (b=0; b < (blocks*(blklen+1)); b++)
      chainSetCntVal(
         counter, b+(blocks*(blklen+1)), chainGetCntVal(counter, b));
}


int gpioWaveChain(char *buf, unsigned bufSize)
{
   unsigned blklen=16, blocks=4;
   int cb, chaincb;
   rawCbs_t *p;
   int i, wid, cmd, loop, counters;
   unsigned cycles, delayCBs, dcb, delayLeft;
   uint32_t repeat, next, *endPtr;
   int stk_pos[10], stk_lev=0;

   cb = 0;
   loop = -1;

   DBG(DBG_USER, "bufSize=%d [%s]", bufSize, myBuf2Str(bufSize, buf));

   CHECK_INITED;

   if (!waveClockInited)
   {
      stopHardwarePWM();
      initClock(0); /* initialise secondary clock */
      waveClockInited = 1;
      PWMClockInited = 0;
   }

   initKillDMA(dmaOut);

   waveEndPtr = NULL;
   endPtr = NULL;

   /* add delay cb at start of DMA */

   p = rawWaveCBAdr(chainGetCB(cb++));

   /* use the secondary clock */

   if (gpioCfg.clockPeriph != PI_CLOCK_PCM)
   {
      p->info = NORMAL_DMA | TIMED_DMA(2);
      p->dst  = PCM_TIMER;
   }
   else
   {
      p->info = NORMAL_DMA | TIMED_DMA(5);
      p->dst  = PWM_TIMER;
   }

   //cast twice to suppress warning, I belive this is ok as dmaOBus
   //contains bus addresses not virtual addresses. --plugwash
   p->src    = (uint32_t)(uintptr_t) (&dmaOBus[0]->periphData);
   p->length = BPD * 20 / PI_WF_MICROS; /* 20 micros delay */
   p->next   = waveCbPOadr(chainGetCB(cb));

   counters = 0;
   wid = -1;

   i = 0;

   while (i<bufSize)
   {
      wid = (unsigned)buf[i];

      if (wid == 255) /* wave command */
      {
         if ((i+2) > bufSize)
            SOFT_ERROR(PI_BAD_CHAIN_CMD,
               "incomplete chain command (at %d)", i);

         cmd = buf[i+1];

         if (cmd == 0) /* loop begin */
         {
            if (stk_lev >= (sizeof(stk_pos)/sizeof(int)))
               SOFT_ERROR(PI_CHAIN_NESTING,
                  "chain counters nested too deep (at %d)", i);

            stk_pos[stk_lev++] = cb;

            i += 2;
         }
         else if (cmd == 1) /* loop end */
         {
            if (counters >= WCB_COUNTERS)
               SOFT_ERROR(PI_CHAIN_COUNTER,
                  "too many chain counters (at %d)", i);

            if ((i+4) > bufSize)
               SOFT_ERROR(PI_BAD_CHAIN_CMD,
                  "incomplete chain command (at %d)", i);

            loop = 0;
            if (--stk_lev >= 0) loop = stk_pos[stk_lev];

            if ((loop < 1) || (loop == cb))
               SOFT_ERROR(PI_BAD_CHAIN_LOOP,
                  "empty chain loop (at %d)", i);

            cycles = ((unsigned)buf[i+3] <<  8) + (unsigned)buf[i+2];

            i += 4;

            if (cycles > PI_MAX_WAVE_CYCLES)
               SOFT_ERROR(PI_CHAIN_LOOP_CNT,
                  "bad chain loop count (%d)", cycles);

            if (cycles == 0)
            {
               /* Skip the complete loop block.  Change
                  the next pointing to the start of the
                  loop block to the current cb.
               */
               p = rawWaveCBAdr(chainGetCB(loop));
               p->next = waveCbPOadr(chainGetCB(cb));
            }
            else if (cycles == 1)
            {
               /* Nothing to do, no need for a counter. */
            }
            else
            {
               chaincb = chainGetCB(cb++);
               if (chaincb < 0)
                  SOFT_ERROR(PI_CHAIN_TOO_BIG, "chain is too long (%d)", cb);

               p = rawWaveCBAdr(chaincb);

               repeat = waveCbPOadr(chainGetCB(loop));

                /* Need to check next cb as well. */

               chaincb = chainGetCB(cb);

               if (chaincb < 0)
                  SOFT_ERROR(PI_CHAIN_TOO_BIG, "chain is too long (%d)", cb);

               next = waveCbPOadr(chainGetCB(cb));

               /* dummy src and dest */
               p->info = NORMAL_DMA;
               //cast twice to suppress warning, I belive this is ok as dmaOBus
               //contains bus addresses not virtual addresses. --plugwash
               p->src = (uint32_t)(uintptr_t) (&dmaOBus[0]->periphData);
               p->dst = (uint32_t)(uintptr_t) (&dmaOBus[0]->periphData);
               p->length = 4;
               p->next = waveCbPOadr(chainGetCntCB(counters));

               chainMakeCounter(counters, blklen, blocks,
                            cycles-1, repeat, next);

               counters++;
            }
         }
         else if (cmd == 2) /* delay us */
         {
            if ((i+4) > bufSize)
               SOFT_ERROR(PI_BAD_CHAIN_CMD,
                  "incomplete chain command (at %d)", i);

            cycles = ((unsigned)buf[i+3] <<  8) + (unsigned)buf[i+2];

            i += 4;

            if (cycles > PI_MAX_WAVE_DELAY)
               SOFT_ERROR(PI_BAD_CHAIN_DELAY,
                  "bad chain delay micros (%d)", cycles);

            if (cycles)
            {
               delayLeft = cycles;
               delayCBs = waveDelayCBs(delayLeft);
               for (dcb=0; dcb<delayCBs; dcb++)
               {
                  chaincb = chainGetCB(cb++);

                  if (chaincb < 0)
                     SOFT_ERROR(
                        PI_CHAIN_TOO_BIG, "chain is too long (%d)", cb);

                  p = rawWaveCBAdr(chaincb);

                  /* use the secondary clock */

                  if (gpioCfg.clockPeriph != PI_CLOCK_PCM)
                  {
                     p->info = NORMAL_DMA | TIMED_DMA(2);
                     p->dst  = PCM_TIMER;
                  }
                  else
                  {
                     p->info = NORMAL_DMA | TIMED_DMA(5);
                     p->dst  = PWM_TIMER;
                  }

                  //cast twice to suppress warning, I belive this is ok as dmaOBus
                  //contains bus addresses not virtual addresses. --plugwash
                  p->src = (uint32_t)(uintptr_t) (&dmaOBus[0]->periphData);

                  p->length = BPD * delayLeft / PI_WF_MICROS;

                  if ((gpioCfg.DMAsecondaryChannel >= DMA_LITE_FIRST) &&
                      (p->length > DMA_LITE_MAX))
                  {
                     p->length = DMA_LITE_MAX;
                  }

                  delayLeft -= (p->length / BPD);

                  p->next = waveCbPOadr(chainGetCB(cb));
               }
            }
         }
         else if (cmd == 3) /* repeat loop forever */
         {
            i += 2;

            loop = 0;
            if (--stk_lev >= 0) loop = stk_pos[stk_lev];

            if ((loop < 1) || (loop == cb))
               SOFT_ERROR(PI_BAD_CHAIN_LOOP,
                  "empty chain loop (at %d)", i);

            chaincb = chainGetCB(cb++);
            if (chaincb < 0)
               SOFT_ERROR(PI_CHAIN_TOO_BIG, "chain is too long (%d)", cb);

            if (i < bufSize)
               SOFT_ERROR(PI_BAD_FOREVER,
                  "loop forever must be last command");

            p = rawWaveCBAdr(chaincb);

            /* dummy src and dest */
            p->info = NORMAL_DMA;
            //cast twice to suppress warning, I belive this is ok as dmaOBus
            //contains bus addresses not virtual addresses. --plugwash
            p->src = (uint32_t)(uintptr_t) (&dmaOBus[0]->periphData);
            p->dst = (uint32_t)(uintptr_t) (&dmaOBus[0]->periphData);
            p->length = 4;
            p->next = waveCbPOadr(chainGetCB(loop));
            endPtr = &p->next;
         }
         else
            SOFT_ERROR(PI_BAD_CHAIN_CMD,
               "unknown chain command (255 %d)", cmd);
      }
      else if ((wid >= waveOutCount) || waveInfo[wid].deleted)
         SOFT_ERROR(PI_BAD_WAVE_ID, "undefined wave (%d)", wid);
      else
      {
         chaincb = chainGetCB(cb++);

         if (chaincb < 0)
            SOFT_ERROR(PI_CHAIN_TOO_BIG, "chain is too long (%d)", cb);

         p = rawWaveCBAdr(chaincb);

         chaincb = chainGetCB(cb);

         if (chaincb < 0)
            SOFT_ERROR(PI_CHAIN_TOO_BIG, "chain is too long (%d)", cb);

         chainSetVal(cb-1, waveCbPOadr(chaincb));

         /* patch next of wid topCB to next cb */

         p->info   = NORMAL_DMA;
         p->src    = chainGetValPadr(cb-1); /* this next */
         p->dst    = waveCbPOadr(waveInfo[wid].topCB) + 20; /* wid next */
         p->length = 4;
         p->next   = waveCbPOadr(waveInfo[wid].botCB+1);

         i += 1;
      }
   }

   chaincb = chainGetCB(cb++);

   if (chaincb < 0)
      SOFT_ERROR(PI_CHAIN_TOO_BIG, "chain is too long (%d)", cb);

   p = rawWaveCBAdr(chaincb);

   p->info   = NORMAL_DMA;

   //cast twice to suppress warning, I belive this is ok as dmaOBus
   //contains bus addresses not virtual addresses. --plugwash
   p->src    = (uint32_t)(uintptr_t) (&dmaOBus[0]->periphData);
   p->dst    = (uint32_t)(uintptr_t) (&dmaOBus[0]->periphData);
   p->length = 4;
   p->next = 0;

   if (!endPtr) endPtr = &p->next;

   initDMAgo((uint32_t *)dmaOut, waveCbPOadr(chainGetCB(0)));

   waveEndPtr = endPtr;

   return 0;
}

/*-------------------------------------------------------------------------*/

int gpioWaveTxBusy(void)
{
   DBG(DBG_USER, "");

   CHECK_INITED;

   if (dmaOut[DMA_CONBLK_AD])
      return 1;
   else
      return 0;
}

/*-------------------------------------------------------------------------*/

int gpioWaveTxAt(void)
{
   int i, cb;

   DBG(DBG_USER, "");

   CHECK_INITED;

   cb = dmaNowAtOCB();

   if (cb < 0) return -cb;

   for (i=0; i<PI_MAX_WAVES; i++)
   {
      if ( !waveInfo[i].deleted &&
          (cb >= waveInfo[i].botCB) &&
          (cb <= waveInfo[i].topCB) ) return i;
   }

   return PI_WAVE_NOT_FOUND;
}

/* ----------------------------------------------------------------------- */

int gpioWaveTxStop(void)
{
   DBG(DBG_USER, "");

   CHECK_INITED;

   initKillDMA(dmaOut);

   waveEndPtr = NULL;

   return 0;
}

/* ----------------------------------------------------------------------- */

int gpioWaveGetMicros(void)
{
   DBG(DBG_USER, "");

   CHECK_INITED;

   return wfStats.micros;
}

/* ----------------------------------------------------------------------- */

int gpioWaveGetHighMicros(void)
{
   DBG(DBG_USER, "");

   CHECK_INITED;

   return wfStats.highMicros;
}

/* ----------------------------------------------------------------------- */

int gpioWaveGetMaxMicros(void)
{
   DBG(DBG_USER, "");

   CHECK_INITED;

   return wfStats.maxMicros;
}

/* ----------------------------------------------------------------------- */

int gpioWaveGetPulses(void)
{
   DBG(DBG_USER, "");

   CHECK_INITED;

   return wfStats.pulses;
}

/* ----------------------------------------------------------------------- */

int gpioWaveGetHighPulses(void)
{
   DBG(DBG_USER, "");

   CHECK_INITED;

   return wfStats.highPulses;
}

/* ----------------------------------------------------------------------- */

int gpioWaveGetMaxPulses(void)
{
   DBG(DBG_USER, "");

   CHECK_INITED;

   return wfStats.maxPulses;
}

/* ----------------------------------------------------------------------- */

int gpioWaveGetCbs(void)
{
   DBG(DBG_USER, "");

   CHECK_INITED;

   return wfStats.cbs;
}

/* ----------------------------------------------------------------------- */

int gpioWaveGetHighCbs(void)
{
   DBG(DBG_USER, "");

   CHECK_INITED;

   return wfStats.highCbs;
}

/* ----------------------------------------------------------------------- */

int gpioWaveGetMaxCbs(void)
{
   DBG(DBG_USER, "");

   CHECK_INITED;

   return wfStats.maxCbs;
}

/* ----------------------------------------------------------------------- */

static int read_SDA(wfRx_t *w)
{
   myGpioSetMode(w->I.SDA, PI_INPUT);
   return myGpioRead(w->I.SDA);
}

static void set_SDA(wfRx_t *w)
{
   myGpioSetMode(w->I.SDA, PI_INPUT);
}

static void clear_SDA(wfRx_t *w)
{
   myGpioSetMode(w->I.SDA, PI_OUTPUT);
   myGpioWrite(w->I.SDA, 0);
}

static void clear_SCL(wfRx_t *w)
{
   myGpioSetMode(w->I.SCL, PI_OUTPUT);
   myGpioWrite(w->I.SCL, 0);
}

static void I2C_delay(wfRx_t *w)
{
   myGpioDelay(w->I.delay);
}

static void I2C_clock_stretch(wfRx_t *w)
{
   uint32_t now, max_stretch=100000;

   myGpioSetMode(w->I.SCL, PI_INPUT);
   now = gpioTick();
   while ((myGpioRead(w->I.SCL) == 0) && ((gpioTick()-now) < max_stretch));
}

static void I2CStart(wfRx_t *w)
{
   if (w->I.started)
   {
      set_SDA(w);
      I2C_delay(w);
      I2C_clock_stretch(w);
      I2C_delay(w);
   }

   clear_SDA(w);
   I2C_delay(w);
   clear_SCL(w);
   I2C_delay(w);

   w->I.started = 1;
}

static void I2CStop(wfRx_t *w)
{
   clear_SDA(w);
   I2C_delay(w);
   I2C_clock_stretch(w);
   I2C_delay(w);
   set_SDA(w);
   I2C_delay(w);

   w->I.started = 0;
}

static void I2CPutBit(wfRx_t *w, int bit)
{
   if (bit) set_SDA(w);
   else     clear_SDA(w);

   I2C_delay(w);
   I2C_clock_stretch(w);
   I2C_delay(w);
   clear_SCL(w);
}

static int I2CGetBit(wfRx_t *w)
{
   int bit;

   set_SDA(w); /* let SDA float */
   I2C_delay(w);
   I2C_clock_stretch(w);
   bit = read_SDA(w);
   I2C_delay(w);
   clear_SCL(w);

   return bit;
}

static int I2CPutByte(wfRx_t *w, int byte)
{
   int bit, nack;

   for(bit=0; bit<8; bit++)
   {
      I2CPutBit(w, byte & 0x80);
      byte <<= 1;
   }

   nack = I2CGetBit(w);

   return nack;
}

static uint8_t I2CGetByte(wfRx_t *w, int nack)
{
   int bit, byte=0;

   for (bit=0; bit<8; bit++)
   {
      byte = (byte << 1) | I2CGetBit(w);
   }

   I2CPutBit(w, nack);

   return byte;
}

/*-------------------------------------------------------------------------*/

int bbI2COpen(unsigned SDA, unsigned SCL, unsigned baud)
{
   DBG(DBG_USER, "SDA=%d SCL=%d baud=%d", SDA, SCL, baud);

   CHECK_INITED;

   if (SDA > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad SDA (%d)", SDA);

   if (SCL > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad SCL (%d)", SCL);

   if ((baud < PI_BB_I2C_MIN_BAUD) || (baud > PI_BB_I2C_MAX_BAUD))
      SOFT_ERROR(PI_BAD_I2C_BAUD,
         "SDA %d, bad baud rate (%d)", SDA, baud);

   if (wfRx[SDA].mode != PI_WFRX_NONE)
      SOFT_ERROR(PI_GPIO_IN_USE, "gpio %d is already being used", SDA);

   if ((wfRx[SCL].mode != PI_WFRX_NONE)  || (SCL == SDA))
      SOFT_ERROR(PI_GPIO_IN_USE, "gpio %d is already being used", SCL);

   wfRx[SDA].gpio = SDA;
   wfRx[SDA].mode = PI_WFRX_I2C_SDA;
   wfRx[SDA].baud = baud;

   wfRx[SDA].I.started = 0;
   wfRx[SDA].I.SDA = SDA;
   wfRx[SDA].I.SCL = SCL;
   wfRx[SDA].I.delay = 500000 / baud;
   wfRx[SDA].I.SDAMode = gpioGetMode(SDA);
   wfRx[SDA].I.SCLMode = gpioGetMode(SCL);

   wfRx[SCL].gpio = SCL;
   wfRx[SCL].mode = PI_WFRX_I2C_SCL;

   myGpioSetMode(SDA, PI_INPUT);
   myGpioSetMode(SCL, PI_INPUT);

   return 0;
}

/* ----------------------------------------------------------------------- */

int bbI2CClose(unsigned SDA)
{
   DBG(DBG_USER, "SDA=%d", SDA);

   CHECK_INITED;

   if (SDA > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", SDA);

   switch(wfRx[SDA].mode)
   {
      case PI_WFRX_I2C_SDA:

         myGpioSetMode(wfRx[SDA].I.SDA, wfRx[SDA].I.SDAMode);
         myGpioSetMode(wfRx[SDA].I.SCL, wfRx[SDA].I.SCLMode);

         wfRx[wfRx[SDA].I.SDA].mode = PI_WFRX_NONE;
         wfRx[wfRx[SDA].I.SCL].mode = PI_WFRX_NONE;

         break;

      default:

         SOFT_ERROR(PI_NOT_I2C_GPIO, "no I2C on gpio (%d)", SDA);

         break;

   }

   return 0;
}

/*-------------------------------------------------------------------------*/

int bbI2CZip(
   unsigned SDA,
   char *inBuf,
   unsigned inLen,
   char *outBuf,
   unsigned outLen)
{
   int i, ack, inPos, outPos, status, bytes;
   int addr, flags, esc, setesc;
   wfRx_t *w;

   DBG(DBG_USER, "gpio=%d inBuf=%s outBuf=%08"PRIXPTR" len=%d",
      SDA, myBuf2Str(inLen, (char *)inBuf), (uintptr_t)outBuf, outLen);

   CHECK_INITED;

   if (SDA > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", SDA);

   if (wfRx[SDA].mode != PI_WFRX_I2C_SDA)
      SOFT_ERROR(PI_NOT_I2C_GPIO, "no I2C on gpio (%d)", SDA);

   if (!inBuf || !inLen)
      SOFT_ERROR(PI_BAD_POINTER, "input buffer can't be NULL");

   if (!outBuf && outLen)
      SOFT_ERROR(PI_BAD_POINTER, "output buffer can't be NULL");

   w = &wfRx[SDA];

   inPos = 0;
   outPos = 0;
   status = 0;

   addr = 0;
   flags = 0;
   esc = 0;
   setesc = 0;

   wfRx_lock(SDA);

   while (!status && (inPos < inLen))
   {
      DBG(DBG_INTERNAL, "status=%d inpos=%d inlen=%d cmd=%d addr=%d flags=%x",
         status, inPos, inLen, inBuf[inPos], addr, flags);

      switch (inBuf[inPos++])
      {
         case PI_I2C_END:
            status = 1;
            break;

         case PI_I2C_START:
            I2CStart(w);
            break;

         case PI_I2C_STOP:
            I2CStop(w);
            break;

         case PI_I2C_ADDR:
            addr = myI2CGetPar(inBuf, &inPos, inLen, &esc);
            if (addr < 0) status = PI_BAD_I2C_CMD;
            break;

         case PI_I2C_FLAGS:
            /* cheat to force two byte flags */
            esc = 1;
            flags = myI2CGetPar(inBuf, &inPos, inLen, &esc);
            if (flags < 0) status = PI_BAD_I2C_CMD;
            break;

         case PI_I2C_ESC:
            setesc = 1;
            break;

         case PI_I2C_READ:

            bytes = myI2CGetPar(inBuf, &inPos, inLen, &esc);

            if (bytes >= 0) ack = I2CPutByte(w, (addr<<1)|1);

            if (bytes > 0)
            {
               if (!ack)
               {
                  if ((bytes + outPos) <= outLen)
                  {
                     for (i=0; i<(bytes-1); i++)
                     {
                        outBuf[outPos++] = I2CGetByte(w, 0);
                     }
                     outBuf[outPos++] = I2CGetByte(w, 1);
                  }
                  else status = PI_BAD_I2C_RLEN;
               }
               else status = PI_I2C_READ_FAILED;
            }
            else status = PI_BAD_I2C_CMD;
            break;

         case PI_I2C_WRITE:

            bytes = myI2CGetPar(inBuf, &inPos, inLen, &esc);

            if (bytes >= 0) ack = I2CPutByte(w, addr<<1);

            if (bytes > 0)
            {
               if (!ack)
               {
                  if ((bytes + inPos) <= inLen)
                  {
                     for (i=0; i<(bytes-1); i++)
                     {
                        ack = I2CPutByte(w, inBuf[inPos++]);
                        if (ack) status = PI_I2C_WRITE_FAILED;
                     }
                     ack = I2CPutByte(w, inBuf[inPos++]);
                  }
                  else status = PI_BAD_I2C_WLEN;
               } else status = PI_I2C_WRITE_FAILED;
            }
            else status = PI_BAD_I2C_CMD;
            break;

         default:
            status = PI_BAD_I2C_CMD;
      }

      if (setesc) esc = 1; else esc = 0;

      setesc = 0;
   }

   wfRx_unlock(SDA);

   if (status >= 0) status = outPos;

   return status;
}

/* ----------------------------------------------------------------------- */

void bscInit(int mode)
{
   int sda, scl, mosi, miso, ce;

   bscsReg[BSC_CR]=0; /* clear device */
   bscsReg[BSC_RSR]=0; /* clear underrun and overrun errors */
   bscsReg[BSC_SLV]=0; /* clear I2C slave address */
   bscsReg[BSC_IMSC]=0xf; /* mask off all interrupts */
   bscsReg[BSC_ICR]=0x0f; /* clear all interrupts */

   if (pi_is_2711)
   {
      sda = BSC_SDA_2711;
      scl = BSC_SCL_SCLK_2711;
      mosi = BSC_MOSI_2711;
      miso = BSC_MISO_2711;
      ce = BSC_CE_N_2711;
   }
   else
   {
      sda = BSC_SDA;
      scl = BSC_SCL_SCLK;
      mosi = BSC_MOSI;
      miso = BSC_MISO;
      ce = BSC_CE_N;
   }


   if (mode > 1) /* SPI uses all GPIO */
   {
      gpioSetMode(scl, PI_ALT3);
      gpioSetMode(mosi, PI_ALT3);
      gpioSetMode(miso, PI_ALT3);
      gpioSetMode(ce, PI_ALT3);
   }
   else
   {
      gpioSetMode(scl, PI_ALT3);
      gpioSetMode(sda, PI_ALT3);
   }
}

void bscTerm(int mode)
{
   int sda, scl, mosi, miso, ce;

   bscsReg[BSC_CR] = 0; /* clear device */
   bscsReg[BSC_RSR]=0; /* clear underrun and overrun errors */
   bscsReg[BSC_SLV]=0; /* clear I2C slave address */

   if (pi_is_2711)
   {
      sda = BSC_SDA_2711;
      scl = BSC_SCL_SCLK_2711;
      mosi = BSC_MOSI_2711;
      miso = BSC_MISO_2711;
      ce = BSC_CE_N_2711;
   }
   else
   {
      sda = BSC_SDA;
      scl = BSC_SCL_SCLK;
      mosi = BSC_MOSI;
      miso = BSC_MISO;
      ce = BSC_CE_N;
   }


   if (mode > 1)
   {
      gpioSetMode(scl, PI_INPUT);
      gpioSetMode(mosi, PI_INPUT);
      gpioSetMode(miso, PI_INPUT);
      gpioSetMode(ce, PI_INPUT);
   }
   else
   {
      gpioSetMode(sda, PI_INPUT);
      gpioSetMode(scl, PI_INPUT);

   }
}

int bscXfer(bsc_xfer_t *xfer)
{
   static int bscMode = 0;

   int copied=0;
   int active, mode;

   DBG(DBG_USER, "control=0x%X (sa=0x%X, cr=0x%X) tx=%d [%s]",
      xfer->control,
      ((xfer->control)>>16) & 127,
      (xfer->control) & 0x3fff,
      xfer->txCnt,
      myBuf2Str(xfer->txCnt, (char *)xfer->txBuf));

   CHECK_INITED;

   eventAlert[PI_EVENT_BSC].ignore = 1;

   if (xfer->control)
   {
      /*
         bscMode (0=None, 1=I2C, 2=SPI) tracks which GPIO have been
         set to BSC mode
      */
      if (xfer->control & 2) mode = 2; /* SPI */
      else                   mode = 1; /* assume I2C */

      if (mode > bscMode)
      {
         bscInit(mode);
         bscMode = mode;
      }
   }
   else
   {
      if (bscMode) bscTerm(bscMode);
      bscMode = 0;
      return 0; /* leave ignore set */
   }

   xfer->rxCnt = 0;

   bscsReg[BSC_SLV] = ((xfer->control)>>16) & 127;
   bscsReg[BSC_CR] = (xfer->control) & 0x3fff;
   bscsReg[BSC_RSR]=0; /* clear underrun and overrun errors */

   active = 1;

   while (active)
   {
      active = 0;

      while ((copied < xfer->txCnt) &&
             (!(bscsReg[BSC_FR] & BSC_FR_TXFF)))
      {
         bscsReg[BSC_DR] = xfer->txBuf[copied++];
         active = 1;
      }

      while ((xfer->rxCnt < BSC_FIFO_SIZE) &&
             (!(bscsReg[BSC_FR] & BSC_FR_RXFE)))
      {
         xfer->rxBuf[xfer->rxCnt++] = bscsReg[BSC_DR];
         active = 1;
      }

      if (!active)
      {
         active = bscsReg[BSC_FR] & (BSC_FR_RXBUSY | BSC_FR_TXBUSY);
      }

      if (active) myGpioSleep(0, 20);
   }

   bscFR = bscsReg[BSC_FR] & 0xffff;

   eventAlert[PI_EVENT_BSC].ignore = 0;

   return (copied<<16) | bscFR;
}

/* ----------------------------------------------------------------------- */

static void set_CS(wfRx_t *w)
{
   myGpioWrite(w->S.CS, PI_SPI_FLAGS_GET_CSPOL(w->S.spiFlags));
}

static void clear_CS(wfRx_t *w)
{
   myGpioWrite(w->S.CS, !PI_SPI_FLAGS_GET_CSPOL(w->S.spiFlags));
}

static void set_SCLK(wfRx_t *w)
{
   myGpioWrite(w->S.SCLK, !PI_SPI_FLAGS_GET_CPOL(w->S.spiFlags));
}

static void clear_SCLK(wfRx_t *w)
{
   myGpioWrite(w->S.SCLK, PI_SPI_FLAGS_GET_CPOL(w->S.spiFlags));
}

static void SPI_delay(wfRx_t *w)
{
   myGpioDelay(w->S.delay);
}

static void bbSPIStart(wfRx_t *w)
{
   clear_SCLK(w);

   SPI_delay(w);

   set_CS(w);

   SPI_delay(w);
}

static void bbSPIStop(wfRx_t *w)
{
   SPI_delay(w);

   clear_CS(w);

   SPI_delay(w);

   clear_SCLK(w);
}

static uint8_t bbSPIXferByte(wfRx_t *w, char txByte)
{
   uint8_t bit, rxByte=0;

   if (PI_SPI_FLAGS_GET_CPHA(w->S.spiFlags))
   {
      /*
      CPHA = 1
      write on set clock
      read on clear clock
      */

      for (bit=0; bit<8; bit++)
      {
         set_SCLK(w);

         if (PI_SPI_FLAGS_GET_TX_LSB(w->S.spiFlags))
         {
            myGpioWrite(w->S.MOSI, txByte & 0x01);
            txByte >>= 1;
         }
         else
         {
            myGpioWrite(w->S.MOSI, txByte & 0x80);
            txByte <<= 1;
         }

         SPI_delay(w);

         clear_SCLK(w);

         if (PI_SPI_FLAGS_GET_RX_LSB(w->S.spiFlags))
         {
            rxByte = (rxByte >> 1) | myGpioRead(w->S.MISO) << 7;
         }
         else
         {
            rxByte = (rxByte << 1) | myGpioRead(w->S.MISO);
         }

         SPI_delay(w);
      }
   }
   else
   {
      /*
      CPHA = 0
      read on set clock
      write on clear clock
      */

      for (bit=0; bit<8; bit++)
      {
         if (PI_SPI_FLAGS_GET_TX_LSB(w->S.spiFlags))
         {
            myGpioWrite(w->S.MOSI, txByte & 0x01);
            txByte >>= 1;
         }
         else
         {
            myGpioWrite(w->S.MOSI, txByte & 0x80);
            txByte <<= 1;
         }

         SPI_delay(w);

         set_SCLK(w);

         if (PI_SPI_FLAGS_GET_RX_LSB(w->S.spiFlags))
         {
            rxByte = (rxByte >> 1) | myGpioRead(w->S.MISO) << 7;
         }
         else
         {
            rxByte = (rxByte << 1) | myGpioRead(w->S.MISO);
         }

         SPI_delay(w);

         clear_SCLK(w);
      }
   }

   return rxByte;
}

/*-------------------------------------------------------------------------*/

int bbSPIOpen(
   unsigned CS, unsigned MISO, unsigned MOSI, unsigned SCLK,
   unsigned baud, unsigned spiFlags)
{
   int valid;
   uint32_t bits;

   DBG(DBG_USER, "CS=%d MISO=%d MOSI=%d SCLK=%d baud=%d flags=%d",
      CS, MISO, MOSI, SCLK, baud, spiFlags);

   CHECK_INITED;

   if (CS > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad CS (%d)", CS);

   if (MISO > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad MISO (%d)", MISO);

   if (MOSI > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad MOSI (%d)", MOSI);

   if (SCLK > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad SCLK (%d)", SCLK);

   if ((baud < PI_BB_SPI_MIN_BAUD) || (baud > PI_BB_SPI_MAX_BAUD))
      SOFT_ERROR(PI_BAD_SPI_BAUD, "CS %d, bad baud (%d)", CS, baud);

   if (wfRx[CS].mode != PI_WFRX_NONE)
      SOFT_ERROR(PI_GPIO_IN_USE,
         "CS %d is already being used, mode %d", CS, wfRx[CS].mode);

   valid = 0;

   /* check all GPIO unique */

   bits = (1<<CS) | (1<<MISO) | (1<<MOSI) | (1<<SCLK);

   if (__builtin_popcount(bits) == 4)
   {
      if ((wfRx[MISO].mode == PI_WFRX_NONE) &&
          (wfRx[MOSI].mode == PI_WFRX_NONE) &&
          (wfRx[SCLK].mode == PI_WFRX_NONE))
      {
         valid = 1; /* first time GPIO used for SPI */
      }
      else
      {
         if ((wfRx[MISO].mode == PI_WFRX_SPI_MISO) &&
             (wfRx[MOSI].mode == PI_WFRX_SPI_MOSI) &&
             (wfRx[SCLK].mode == PI_WFRX_SPI_SCLK))
         {
            valid = 2; /* new CS for existing SPI GPIO */
         }
      }
   }

   if (!valid)
   {
      SOFT_ERROR(PI_GPIO_IN_USE,
         "GPIO already being used (%d=%d %d=%d, %d=%d %d=%d)",
          CS,   wfRx[CS].mode,
          MISO, wfRx[MISO].mode,
          MOSI, wfRx[MOSI].mode,
          SCLK, wfRx[SCLK].mode);
   }

   wfRx[CS].mode = PI_WFRX_SPI_CS;
   wfRx[CS].baud = baud;

   wfRx[CS].S.CS = CS;
   wfRx[CS].S.SCLK = SCLK;

   wfRx[CS].S.CSMode = gpioGetMode(CS);
   wfRx[CS].S.delay = (500000 / baud) - 1;
   wfRx[CS].S.spiFlags = spiFlags;

   /* preset CS to off */

   if (PI_SPI_FLAGS_GET_CSPOL(spiFlags))
      gpioWrite(CS, 0); /* active high */
   else
      gpioWrite(CS, 1); /* active low */

   /* The SCLK entry is used to store full information */

   if (valid == 1) /* first time GPIO for SPI */
   {
      wfRx[SCLK].S.usage = 1;

      wfRx[SCLK].S.SCLKMode = gpioGetMode(SCLK);
      wfRx[SCLK].S.MISOMode = gpioGetMode(MISO);
      wfRx[SCLK].S.MOSIMode = gpioGetMode(MOSI);

      wfRx[SCLK].mode = PI_WFRX_SPI_SCLK;
      wfRx[MISO].mode = PI_WFRX_SPI_MISO;
      wfRx[MOSI].mode = PI_WFRX_SPI_MOSI;

      wfRx[SCLK].S.SCLK = SCLK;
      wfRx[SCLK].S.MISO = MISO;
      wfRx[SCLK].S.MOSI = MOSI;

      myGpioSetMode(MISO, PI_INPUT);
      myGpioSetMode(SCLK, PI_OUTPUT);
      gpioWrite(MOSI, 0); /* low output */
   }
   else
   {
      wfRx[SCLK].S.usage++;
   }

   return 0;
}

/*-------------------------------------------------------------------------*/

int bbSPIClose(unsigned CS)
{
   int SCLK;

   DBG(DBG_USER, "CS=%d", CS);

   CHECK_INITED;

   if (CS > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", CS);

   switch(wfRx[CS].mode)
   {
      case PI_WFRX_SPI_CS:

         myGpioSetMode(wfRx[CS].S.CS, wfRx[CS].S.CSMode);
         wfRx[CS].mode = PI_WFRX_NONE;

         SCLK = wfRx[CS].S.SCLK;

         if (--wfRx[SCLK].S.usage <= 0)
         {
            myGpioSetMode(wfRx[SCLK].S.MISO, wfRx[SCLK].S.MISOMode);
            myGpioSetMode(wfRx[SCLK].S.MOSI, wfRx[SCLK].S.MOSIMode);
            myGpioSetMode(wfRx[SCLK].S.SCLK, wfRx[SCLK].S.SCLKMode);

            wfRx[wfRx[SCLK].S.MISO].mode = PI_WFRX_NONE;
            wfRx[wfRx[SCLK].S.MOSI].mode = PI_WFRX_NONE;
            wfRx[wfRx[SCLK].S.SCLK].mode = PI_WFRX_NONE;
         }

         break;

      default:

         SOFT_ERROR(PI_NOT_SPI_GPIO, "no SPI on gpio (%d)", CS);

         break;

   }

   return 0;
}

/*-------------------------------------------------------------------------*/

int bbSPIXfer(
   unsigned CS,
   char *inBuf,
   char *outBuf,
   unsigned count)
{
   int SCLK;
   int pos;
   wfRx_t *w;

   DBG(DBG_USER, "CS=%d inBuf=%s outBuf=%08"PRIXPTR" count=%d",
      CS, myBuf2Str(count, (char *)inBuf), (uintptr_t)outBuf, count);

   CHECK_INITED;

   if (CS > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", CS);

   if (wfRx[CS].mode != PI_WFRX_SPI_CS)
      SOFT_ERROR(PI_NOT_SPI_GPIO, "no SPI on gpio (%d)", CS);

   if (!inBuf || !count)
      SOFT_ERROR(PI_BAD_POINTER, "input buffer can't be NULL");

   if (!outBuf && count)
      SOFT_ERROR(PI_BAD_POINTER, "output buffer can't be NULL");

   SCLK = wfRx[CS].S.SCLK;

   wfRx[SCLK].S.CS = CS;
   wfRx[SCLK].baud = wfRx[CS].baud;
   wfRx[SCLK].S.delay = wfRx[CS].S.delay;
   wfRx[SCLK].S.spiFlags = wfRx[CS].S.spiFlags;

   w = &wfRx[SCLK];

   wfRx_lock(SCLK);

   bbSPIStart(w);
     
   for (pos=0; pos < count; pos++)
   {
      outBuf[pos] = bbSPIXferByte(w, inBuf[pos]);
   }

   bbSPIStop(w);

   wfRx_unlock(SCLK);

   return count;
}

/*-------------------------------------------------------------------------*/

int gpioSerialReadOpen(unsigned gpio, unsigned baud, unsigned data_bits)
{
   int bitTime, timeout;

   DBG(DBG_USER, "gpio=%d baud=%d data_bits=%d", gpio, baud, data_bits);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   if ((baud < PI_BB_SER_MIN_BAUD) || (baud > PI_BB_SER_MAX_BAUD))
      SOFT_ERROR(PI_BAD_WAVE_BAUD,
         "gpio %d, bad baud rate (%d)", gpio, baud);

   if ((data_bits < PI_MIN_WAVE_DATABITS) ||
       (data_bits > PI_MAX_WAVE_DATABITS))
      SOFT_ERROR(PI_BAD_DATABITS,
         "gpio %d, bad data bits (%d)", gpio, data_bits);

   if (wfRx[gpio].mode != PI_WFRX_NONE)
      SOFT_ERROR(PI_GPIO_IN_USE, "gpio %d is already being used", gpio);

   bitTime = (1000 * MILLION) / baud; /* nanos */

   timeout  = ((data_bits+2) * bitTime)/MILLION; /* millis */

   if (timeout < 1) timeout = 1;

   wfRx[gpio].gpio = gpio;
   wfRx[gpio].mode = PI_WFRX_SERIAL;
   wfRx[gpio].baud = baud;

   wfRx[gpio].s.buf      = malloc(SRX_BUF_SIZE);
   wfRx[gpio].s.bufSize  = SRX_BUF_SIZE;
   wfRx[gpio].s.timeout  = timeout;
   wfRx[gpio].s.fullBit  = bitTime;         /* nanos */
   wfRx[gpio].s.halfBit  = (bitTime/2)+500; /* nanos (500 for rounding) */
   wfRx[gpio].s.readPos  = 0;
   wfRx[gpio].s.writePos = 0;
   wfRx[gpio].s.bit      = -1;
   wfRx[gpio].s.dataBits = data_bits;
   wfRx[gpio].s.invert   = PI_BB_SER_NORMAL;

   if      (data_bits <  9) wfRx[gpio].s.bytes = 1;
   else if (data_bits < 17) wfRx[gpio].s.bytes = 2;
   else                  wfRx[gpio].s.bytes = 4;

   gpioSetAlertFunc(gpio, waveRxBit);

   return 0;
}

/*-------------------------------------------------------------------------*/

int gpioSerialReadInvert(unsigned gpio, unsigned invert)
{
   DBG(DBG_USER, "gpio=%d invert=%d", gpio, invert);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   if (wfRx[gpio].mode != PI_WFRX_SERIAL)
      SOFT_ERROR(PI_NOT_SERIAL_GPIO, "no serial read on gpio (%d)", gpio);

   if ((invert < PI_BB_SER_NORMAL) ||
       (invert > PI_BB_SER_INVERT))
      SOFT_ERROR(PI_BAD_SER_INVERT,
         "bad invert level for gpio %d (%d)", gpio, invert);

   wfRx[gpio].s.invert = invert;

   return 0;
}

/*-------------------------------------------------------------------------*/

int gpioSerialRead(unsigned gpio, void *buf, size_t bufSize)
{
   unsigned bytes=0, wpos;
   volatile wfRx_t *w;

   DBG(DBG_USER, "gpio=%d buf=%08"PRIXPTR" bufSize=%zd", gpio, (uintptr_t)buf, bufSize);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   if (bufSize == 0)
      SOFT_ERROR(PI_BAD_SERIAL_COUNT, "buffer size can't be zero");

   if (wfRx[gpio].mode != PI_WFRX_SERIAL)
      SOFT_ERROR(PI_NOT_SERIAL_GPIO, "no serial read on gpio (%d)", gpio);

   w = &wfRx[gpio];

   if (w->s.readPos != w->s.writePos)
   {
      wpos = w->s.writePos;

      if (wpos > w->s.readPos) bytes = wpos - w->s.readPos;
      else                     bytes = w->s.bufSize - w->s.readPos;

      if (bytes > bufSize) bytes = bufSize;

      /* copy in multiples of the data size in bytes */

      bytes = (bytes / w->s.bytes) * w->s.bytes;

      if (buf) memcpy(buf, w->s.buf+w->s.readPos, bytes);

      w->s.readPos += bytes;

      if (w->s.readPos >= w->s.bufSize) w->s.readPos = 0;
   }
   return bytes;
}


/*-------------------------------------------------------------------------*/

int gpioSerialReadClose(unsigned gpio)
{
   DBG(DBG_USER, "gpio=%d", gpio);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   switch(wfRx[gpio].mode)
   {
      case PI_WFRX_NONE:

         SOFT_ERROR(PI_NOT_SERIAL_GPIO, "no serial read on gpio (%d)", gpio);

         break;

      case PI_WFRX_SERIAL:

         free(wfRx[gpio].s.buf);

         gpioSetWatchdog(gpio, 0); /* switch off timeouts */

         gpioSetAlertFunc(gpio, NULL); /* cancel alert */

         wfRx[gpio].mode = PI_WFRX_NONE;

         break;
   }

   return 0;
}


/* ----------------------------------------------------------------------- */

static int intEventSetFunc(
   unsigned event,
   void *   f,
   int      user,
   void *   userdata)
{
   DBG(DBG_INTERNAL, "event=%d function=%08"PRIXPTR", user=%d, userdata=%08"PRIXPTR,
      event, (uintptr_t)f, user, (uintptr_t)userdata);

   eventAlert[event].ex = user;
   eventAlert[event].userdata = userdata;

   eventAlert[event].func = f;

   return 0;
}


/* ----------------------------------------------------------------------- */

int eventSetFunc(unsigned event, eventFunc_t f)
{
   DBG(DBG_USER, "event=%d function=%08"PRIXPTR, event, (uintptr_t)f);

   CHECK_INITED;

   if (event > PI_MAX_EVENT)
      SOFT_ERROR(PI_BAD_EVENT_ID, "bad event (%d)", event);

   intEventSetFunc(event, f, 0, NULL);

   return 0;
}


/* ----------------------------------------------------------------------- */

int eventSetFuncEx(unsigned event, eventFuncEx_t f, void *userdata)
{
   DBG(DBG_USER, "event=%d function=%08"PRIxPTR" userdata=%08"PRIxPTR,
      event, (uintptr_t)f, (uintptr_t)userdata);

   CHECK_INITED;

   if (event > PI_MAX_EVENT)
      SOFT_ERROR(PI_BAD_EVENT_ID, "bad event (%d)", event);

   intEventSetFunc(event, f, 1, userdata);

   return 0;
}


/* ----------------------------------------------------------------------- */

int eventMonitor(unsigned handle, uint32_t bits)
{
   DBG(DBG_USER, "handle=%d bits=%08X", handle, bits);

   CHECK_INITED;

   if (handle >= PI_NOTIFY_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (gpioNotify[handle].state <= PI_NOTIFY_CLOSING)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   gpioNotify[handle].eventBits  = bits;

   return 0;
}


/* ----------------------------------------------------------------------- */

int eventTrigger(unsigned event)
{
   DBG(DBG_USER, "event=%d", event);

   CHECK_INITED;

   if (event > PI_MAX_EVENT)
      SOFT_ERROR(PI_BAD_EVENT_ID, "bad event (%d)", event);

   eventAlert[event].fired = 1;

   return 0;
}


/* ----------------------------------------------------------------------- */

static int intGpioSetAlertFunc(
   unsigned gpio,
   void *   f,
   int      user,
   void *   userdata)
{
   DBG(DBG_INTERNAL, "gpio=%d function=%08"PRIXPTR", user=%d, userdata=%08"PRIXPTR,
      gpio, (uintptr_t)f, user, (uintptr_t)userdata);

   gpioAlert[gpio].ex = user;
   gpioAlert[gpio].userdata = userdata;

   gpioAlert[gpio].func = f;

   if (f)
   {
      alertBits |= BIT;
   }
   else
   {
      alertBits &= ~BIT;
   }

   monitorBits = alertBits | notifyBits | scriptBits | gpioGetSamples.bits;

   return 0;
}

/* ----------------------------------------------------------------------- */

int gpioSetAlertFunc(unsigned gpio, gpioAlertFunc_t f)
{
   DBG(DBG_USER, "gpio=%d function=%08"PRIXPTR, gpio, (uintptr_t)f);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   intGpioSetAlertFunc(gpio, f, 0, NULL);

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioSetAlertFuncEx(unsigned gpio, gpioAlertFuncEx_t f, void *userdata)
{
   DBG(DBG_USER, "gpio=%d function=%08"PRIXPTR" userdata=%08"PRIXPTR,
      gpio, (uintptr_t)f, (uintptr_t)userdata);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   intGpioSetAlertFunc(gpio, f, 1, userdata);

   return 0;
}

static void *pthISRThread(void *x)
{
   gpioISR_t *isr = x;
   int fd;
   int retval;
   uint32_t tick;
   int level;
   uint32_t levels;
   struct pollfd pfd;
   char buf[64];

   DBG(DBG_USER, "gpio=%d edge=%d timeout=%d f=%"PRIxPTR" u=%d data=%"PRIxPTR,
      isr->gpio, isr->edge, isr->timeout, (uintptr_t)isr->func,
      isr->ex, (uintptr_t)isr->userdata);

   sprintf(buf, "/sys/class/gpio/gpio%d/value", isr->gpio);

   isr->fd = -1; /* no fd assigned */

   if ((fd = open(buf, O_RDONLY)) < 0)
   {
      DBG(DBG_ALWAYS, "gpio %d not exported", isr->gpio);
      return NULL;
   }

   isr->fd = fd; /* store fd so it can be closed */

   pfd.fd = fd;

   pfd.events = POLLPRI;

   lseek(fd, 0, SEEK_SET);    /* consume any prior interrupt */
   if (read(fd, buf, sizeof buf) == -1) { /* ignore errors */ }

   while (1)
   {
      retval = poll(&pfd, 1, isr->timeout); /* wait for interrupt */

      tick = systReg[SYST_CLO];

      levels = *(gpioReg + GPLEV0);

      if (retval >= 0)
      {
         lseek(fd, 0, SEEK_SET);    /* consume interrupt */
         if (read(fd, buf, sizeof buf) == -1) { /* ignore errors */ }

         if (retval)
         {
            if (levels & (1<<isr->gpio)) level = PI_ON; else level = PI_OFF;
         }
         else level = PI_TIMEOUT;

         if (isr->ex) (isr->func)(isr->gpio, level, tick, isr->userdata);
         else         (isr->func)(isr->gpio, level, tick);
      }
   }

   return NULL;
}


/* ----------------------------------------------------------------------- */

static int intGpioSetISRFunc(
   unsigned gpio,
   unsigned edge,
   int timeout,
   void *f,
   int user,
   void *userdata)
{
   char buf[64];

   char *edge_str[]={"rising\n", "falling\n", "both\n"};
   int fd;
   int err;

   DBG(DBG_INTERNAL,
      "gpio=%d edge=%d timeout=%d function=%08"PRIXPTR" user=%d userdata=%08"PRIXPTR,
      gpio, edge, timeout, (uintptr_t)f, user, (uintptr_t)userdata);

   if (f)
   {
      if (!gpioISR[gpio].inited) /* export gpio if unexported */
      {
         fd = open("/sys/class/gpio/export", O_WRONLY);
         if (fd < 0) return PI_BAD_ISR_INIT;

         /* ignore write fail if already exported */
         sprintf(buf, "%d\n", gpio);
         err = write(fd, buf, strlen(buf));
         close(fd);

         sprintf(buf, "/sys/class/gpio/gpio%d/direction", gpio);
         fd = open(buf, O_WRONLY);
         if (fd < 0) return PI_BAD_ISR_INIT;

         err = write(fd, "in\n", 3);
         close(fd);
         if (err != 3) return PI_BAD_ISR_INIT;

         gpioISR[gpio].gpio = gpio;
         gpioISR[gpio].edge = -1;
         gpioISR[gpio].timeout = -1;

         gpioISR[gpio].inited = 1;
      }

      if (gpioISR[gpio].edge != edge)
      {
         sprintf(buf, "/sys/class/gpio/gpio%d/edge", gpio);
         fd = open(buf, O_WRONLY);
         if (fd < 0) return PI_BAD_ISR_INIT;

         err = write(fd, edge_str[edge], strlen(edge_str[edge]));
         close(fd);
         if (err != strlen(edge_str[edge])) return PI_BAD_ISR_INIT;

         gpioISR[gpio].edge = edge;

         if (gpioISR[gpio].pth != NULL)
            pthread_kill(*gpioISR[gpio].pth, SIGCHLD);
      }

      if (timeout <= 0) timeout = -1;
      if (gpioISR[gpio].timeout != timeout)
      {
         gpioISR[gpio].timeout = timeout;

         if (gpioISR[gpio].pth != NULL)
            pthread_kill(*gpioISR[gpio].pth, SIGCHLD);
      }

      gpioISR[gpio].func = f;
      gpioISR[gpio].ex = user;
      gpioISR[gpio].userdata = userdata;

      if (gpioISR[gpio].pth == NULL)
         gpioISR[gpio].pth = gpioStartThread(pthISRThread, &gpioISR[gpio]);
   }
   else /* null function, delete ISR, unexport gpio */
   {
      if (gpioISR[gpio].pth) /* delete any existing ISR */
      {
         gpioStopThread(gpioISR[gpio].pth);

         if (gpioISR[gpio].fd >= 0)
         {
            close(gpioISR[gpio].fd);
            gpioISR[gpio].fd = -1;
         }

         gpioISR[gpio].func = NULL;
         gpioISR[gpio].pth = NULL;
      }

      if (gpioISR[gpio].inited) /* unexport the gpio */
      {
         fd = open("/sys/class/gpio/unexport", O_WRONLY);
         if (fd < 0) return PI_BAD_ISR_INIT;
         sprintf(buf, "%d\n", gpio);
         err = write(fd, buf, strlen(buf));
         close(fd);
         if (err != strlen(buf)) return PI_BAD_ISR_INIT;
         gpioISR[gpio].inited = 0;
      }
   }

   return 0;
}

/* ----------------------------------------------------------------------- */

int gpioSetISRFunc(
   unsigned gpio,
   unsigned edge,
   int timeout,
   gpioISRFunc_t f)
{
   DBG(DBG_USER, "gpio=%d edge=%d timeout=%d function=%08"PRIXPTR,
      gpio, edge, timeout, (uintptr_t)f);

   CHECK_INITED;

   if (gpio > PI_MAX_GPIO)
      SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);

   if (edge > EITHER_EDGE)
      SOFT_ERROR(PI_BAD_EDGE, "bad ISR edge (%d)", edge);

   return intGpioSetISRFunc(gpio, edge, timeout, f, 0, NULL);
}


/* ----------------------------------------------------------------------- */

int gpioSetISRFuncEx(
   unsigned gpio,
   unsigned edge,
   int timeout,
   gpioAlertFuncEx_t f,
   void *userdata)
{
   DBG(DBG_USER, "gpio=%d edge=%d timeout=%d function=%08"PRIXPTR" userdata=%08"PRIXPTR,
      gpio, edge, timeout, (uintptr_t)f, (uintptr_t)userdata);

   CHECK_INITED;

   if (gpio > PI_MAX_GPIO)
      SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);

   if (edge > EITHER_EDGE)
      SOFT_ERROR(PI_BAD_EDGE, "bad ISR edge (%d)", edge);

   return intGpioSetISRFunc(gpio, edge, timeout, f, 1, userdata);
}

static void closeOrphanedNotifications(int slot, int fd)
{
   int i;

   /* Check for and close any orphaned notifications. */

   for (i=0; i<PI_NOTIFY_SLOTS; i++)
   {
      if ((i != slot) &&
          (gpioNotify[i].state >= PI_NOTIFY_OPENED) &&
          (gpioNotify[i].fd == fd))
      {
         DBG(DBG_USER, "closed orphaned fd=%d (handle=%d)", fd, i);
         gpioNotify[i].state = PI_NOTIFY_CLOSED;
         intNotifyBits();
      }
   }
}

/* ----------------------------------------------------------------------- */

static void notifyMutex(int lock)
{
   static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
   if (lock) pthread_mutex_lock(&mutex);
   else      pthread_mutex_unlock(&mutex);
}

/* ----------------------------------------------------------------------- */

int gpioNotifyOpenWithSize(int bufSize)
{
   int i, slot, fd;
   char name[32];

   DBG(DBG_USER, "bufSize=%d", bufSize);

   CHECK_INITED;

   slot = -1;

   notifyMutex(1);

   for (i=0; i<PI_NOTIFY_SLOTS; i++)
   {
      if (gpioNotify[i].state == PI_NOTIFY_CLOSED)
      {
         slot = i;
         gpioNotify[slot].state = PI_NOTIFY_RESERVED;
         break;
      }
   }

   notifyMutex(0);

   if (slot < 0) SOFT_ERROR(PI_NO_HANDLE, "no handle");

   sprintf(name, "/dev/pigpio%d", slot);

   myCreatePipe(name, 0664);

   fd = open(name, O_RDWR|O_NONBLOCK);

   if (fd < 0)
   {
      gpioNotify[slot].state = PI_NOTIFY_CLOSED;
      SOFT_ERROR(PI_BAD_PATHNAME, "open %s failed (%m)", name);
   }

   if (bufSize != 0)
   {
      i = fcntl(fd, F_SETPIPE_SZ, bufSize);
      if (i != bufSize)
      {
         gpioNotify[slot].state = PI_NOTIFY_CLOSED;
         SOFT_ERROR(PI_BAD_PATHNAME,
            "fcntl %s size %d failed (%m)", name, bufSize);
      }
   }

   gpioNotify[slot].seqno = 0;
   gpioNotify[slot].bits  = 0;
   gpioNotify[slot].fd    = fd;
   gpioNotify[slot].pipe  = 1;
   gpioNotify[slot].max_emits  = MAX_EMITS;
   gpioNotify[slot].lastReportTick = gpioTick();
   gpioNotify[i].state = PI_NOTIFY_OPENED;

   closeOrphanedNotifications(slot, fd);

   return slot;
}

int gpioNotifyOpen(void)
{
   return gpioNotifyOpenWithSize(0);
}

/* ----------------------------------------------------------------------- */

static int gpioNotifyOpenInBand(int fd)
{
   int i, slot;

   DBG(DBG_USER, "fd=%d", fd);

   CHECK_INITED;

   slot = -1;

   notifyMutex(1);

   for (i=0; i<PI_NOTIFY_SLOTS; i++)
   {
      if (gpioNotify[i].state == PI_NOTIFY_CLOSED)
      {
         slot = i;
         gpioNotify[slot].state = PI_NOTIFY_RESERVED;
         break;
      }
   }

   notifyMutex(0);

   if (slot < 0) SOFT_ERROR(PI_NO_HANDLE, "no handle");

   gpioNotify[slot].seqno = 0;
   gpioNotify[slot].bits  = 0;
   gpioNotify[slot].fd    = fd;
   gpioNotify[slot].pipe  = 0;
   gpioNotify[slot].max_emits  = MAX_EMITS;
   gpioNotify[slot].lastReportTick = gpioTick();
   gpioNotify[slot].state = PI_NOTIFY_OPENED;

   closeOrphanedNotifications(slot, fd);

   return slot;
}


/* ----------------------------------------------------------------------- */

static void intScriptBits(void)
{
   int i;
   uint32_t bits;

   bits = 0;

   for (i=0; i<PI_MAX_SCRIPTS; i++)
   {
      if (gpioScript[i].state == PI_SCRIPT_IN_USE)
      {
         bits |= gpioScript[i].waitBits;
      }
   }

   scriptBits = bits;

   monitorBits = alertBits | notifyBits | scriptBits | gpioGetSamples.bits;
}


static void intScriptEventBits(void)
{
   int i;
   uint32_t bits;

   bits = 0;

   for (i=0; i<PI_MAX_SCRIPTS; i++)
   {
      if (gpioScript[i].state == PI_SCRIPT_IN_USE)
      {
         bits |= gpioScript[i].eventBits;
      }
   }

   scriptEventBits = bits;
}


static void intNotifyBits(void)
{
   int i;
   uint32_t bits;

   bits = 0;

   for (i=0; i<PI_NOTIFY_SLOTS; i++)
   {
      if (gpioNotify[i].state == PI_NOTIFY_RUNNING)
      {
         bits |= gpioNotify[i].bits;
      }
   }

   notifyBits = bits;

   monitorBits = alertBits | notifyBits | scriptBits | gpioGetSamples.bits;
}


/* ----------------------------------------------------------------------- */

int gpioNotifyBegin(unsigned handle, uint32_t bits)
{
   DBG(DBG_USER, "handle=%d bits=%08X", handle, bits);

   CHECK_INITED;

   if (handle >= PI_NOTIFY_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (gpioNotify[handle].state <= PI_NOTIFY_CLOSING)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   gpioNotify[handle].bits  = bits;

   gpioNotify[handle].state = PI_NOTIFY_RUNNING;

   intNotifyBits();

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioNotifyPause (unsigned handle)
{
   DBG(DBG_USER, "handle=%d", handle);

   CHECK_INITED;

   if (handle >= PI_NOTIFY_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (gpioNotify[handle].state <= PI_NOTIFY_CLOSING)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   gpioNotify[handle].bits  = 0;

   gpioNotify[handle].state = PI_NOTIFY_PAUSED;

   intNotifyBits();

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioNotifyClose(unsigned handle)
{
   char fifo[32];

   DBG(DBG_USER, "handle=%d", handle);

   CHECK_INITED;

   if (handle >= PI_NOTIFY_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (gpioNotify[handle].state <= PI_NOTIFY_CLOSING)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   gpioNotify[handle].bits  = 0;

   gpioNotify[handle].state = PI_NOTIFY_CLOSING;

   intNotifyBits();

   if (gpioCfg.ifFlags & PI_DISABLE_ALERT)
   {
      if (gpioNotify[handle].pipe)
      {
         DBG(DBG_INTERNAL, "close notify pipe %d", gpioNotify[handle].fd);
         close(gpioNotify[handle].fd);

         sprintf(fifo, "/dev/pigpio%d", handle);

         unlink(fifo);
      }

      gpioNotify[handle].state = PI_NOTIFY_CLOSED;
   }
   else
   {
      /* actual close done in alert thread */
   }

   return 0;
}

/* ----------------------------------------------------------------------- */

int gpioTrigger(unsigned gpio, unsigned pulseLen, unsigned level)
{
   DBG(DBG_USER, "gpio=%d pulseLen=%d level=%d", gpio, pulseLen, level);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   if (level > PI_ON)
      SOFT_ERROR(PI_BAD_LEVEL, "gpio %d, bad level (%d)", gpio, level);

   if ((pulseLen > PI_MAX_BUSY_DELAY) || (!pulseLen))
      SOFT_ERROR(PI_BAD_PULSELEN,
         "gpio %d, bad pulseLen (%d)", gpio, pulseLen);

   if (level == PI_OFF) *(gpioReg + GPCLR0 + BANK) = BIT;
   else                 *(gpioReg + GPSET0 + BANK) = BIT;

   myGpioDelay(pulseLen);

   if (level != PI_OFF) *(gpioReg + GPCLR0 + BANK) = BIT;
   else                 *(gpioReg + GPSET0 + BANK) = BIT;

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioSetWatchdog(unsigned gpio, unsigned timeout)
{
   DBG(DBG_USER, "gpio=%d timeout=%d", gpio, timeout);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   if (timeout > PI_MAX_WDOG_TIMEOUT)
      SOFT_ERROR(PI_BAD_WDOG_TIMEOUT,
         "gpio %d, bad timeout (%d)", gpio, timeout);

   gpioAlert[gpio].wdTick   = systReg[SYST_CLO];
   gpioAlert[gpio].wdSteadyUs = timeout*1000;

   if (timeout) wdogBits |= (1<<gpio);
   else         wdogBits &= (~(1<<gpio));

   return 0;
}

/* ----------------------------------------------------------------------- */

int gpioNoiseFilter(unsigned gpio, unsigned steady, unsigned active)
{
   DBG(DBG_USER, "gpio=%d steady=%d active=%d", gpio, steady, active);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   if (steady > PI_MAX_STEADY)
      SOFT_ERROR(PI_BAD_FILTER, "bad steady (%d)", steady);

   if (active > PI_MAX_ACTIVE)
      SOFT_ERROR(PI_BAD_FILTER, "bad active (%d)", active);

   gpioAlert[gpio].nfTick1  = systReg[SYST_CLO];
   gpioAlert[gpio].nfTick2  = gpioAlert[gpio].nfTick1;
   gpioAlert[gpio].nfSteadyUs = steady;
   gpioAlert[gpio].nfActiveUs = active;
   gpioAlert[gpio].nfActive   = 0;

   if (steady) nFilterBits |= (1<<gpio);
   else        nFilterBits &= (~(1<<gpio));

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioGlitchFilter(unsigned gpio, unsigned steady)
{
   DBG(DBG_USER, "gpio=%d steady=%d", gpio, steady);

   CHECK_INITED;

   if (gpio > PI_MAX_USER_GPIO)
      SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

   if (steady > PI_MAX_STEADY)
      SOFT_ERROR(PI_BAD_FILTER, "bad steady (%d)", steady);

   if (steady)
   {
      /* Initialise values next time we process alerts */
      gpioAlert[gpio].gfInitialised = 0;
   }

   gpioAlert[gpio].gfSteadyUs = steady;

   if (steady) gFilterBits |= (1<<gpio);
   else        gFilterBits &= (~(1<<gpio));

   return 0;
}

/* ----------------------------------------------------------------------- */

int gpioSetGetSamplesFunc(gpioGetSamplesFunc_t f, uint32_t bits)
{
   DBG(DBG_USER, "function=%08"PRIXPTR" bits=%08X", (uintptr_t)f, bits);

   CHECK_INITED;

   gpioGetSamples.ex       = 0;
   gpioGetSamples.userdata = NULL;
   gpioGetSamples.func     = f;

   if (f) gpioGetSamples.bits = bits;
   else   gpioGetSamples.bits = 0;

   monitorBits = alertBits | notifyBits | scriptBits | gpioGetSamples.bits;

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioSetGetSamplesFuncEx(gpioGetSamplesFuncEx_t f,
                            uint32_t bits,
                            void * userdata)
{
   DBG(DBG_USER, "function=%08"PRIXPTR" bits=%08X", (uintptr_t)f, bits);

   CHECK_INITED;

   gpioGetSamples.ex       = 1;
   gpioGetSamples.userdata = userdata;
   gpioGetSamples.func     = f;

   if (f) gpioGetSamples.bits = bits;
   else   gpioGetSamples.bits = 0;

   monitorBits = alertBits | notifyBits | scriptBits | gpioGetSamples.bits;

   return 0;
}


/* ----------------------------------------------------------------------- */

static int intGpioSetTimerFunc(unsigned id,
                               unsigned millis,
                               void *f,
                               int user,
                               void *userdata)
{
   pthread_attr_t pthAttr;

   DBG(DBG_INTERNAL, "id=%d millis=%d function=%08"PRIXPTR" user=%d userdata=%08"PRIXPTR,
      id, millis, (uintptr_t)f, user, (uintptr_t)userdata);

   gpioTimer[id].id   = id;

   if (f)
   {
      gpioTimer[id].func     = f;
      gpioTimer[id].ex       = user;
      gpioTimer[id].userdata = userdata;
      gpioTimer[id].millis   = millis;

      if (!gpioTimer[id].running)
      {
         if (pthread_attr_init(&pthAttr))
            SOFT_ERROR(PI_TIMER_FAILED,
               "pthread_attr_init failed (%m)");

         if (pthread_attr_setstacksize(&pthAttr, STACK_SIZE))
            SOFT_ERROR(PI_TIMER_FAILED,
               "pthread_attr_setstacksize failed (%m)");

         if (pthread_create(
            &gpioTimer[id].pthId, &pthAttr, pthTimerTick, &gpioTimer[id]))
               SOFT_ERROR(PI_TIMER_FAILED,
                  "timer %d, create failed (%m)", id);

         gpioTimer[id].running = 1;
      }
   }
   else
   {
      if (gpioTimer[id].running)
      {

         /* destroy thread */

         if (pthread_self() == gpioTimer[id].pthId)
         {
            gpioTimer[id].running = 0;
            gpioTimer[id].func    = 0;
            pthread_exit(NULL);
         }
         else
         {
            if (pthread_cancel(gpioTimer[id].pthId))
               SOFT_ERROR(PI_TIMER_FAILED, "timer %d, cancel failed (%m)", id);

            if (pthread_join(gpioTimer[id].pthId, NULL))
               SOFT_ERROR(PI_TIMER_FAILED, "timer %d, join failed (%m)", id);

            gpioTimer[id].running = 0;
            gpioTimer[id].func    = 0;
         }
      }
   }

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioSetTimerFunc(unsigned id, unsigned millis, gpioTimerFunc_t f)
{
   DBG(DBG_USER, "id=%d millis=%d function=%08"PRIXPTR, id, millis, (uintptr_t)f);

   CHECK_INITED;

   if (id > PI_MAX_TIMER)
      SOFT_ERROR(PI_BAD_TIMER, "bad timer id (%d)", id);

   if (f)
   {
      if ((millis < PI_MIN_MS) || (millis > PI_MAX_MS))
         SOFT_ERROR(PI_BAD_MS, "timer %d, bad millis (%d)", id, millis);
   }

   intGpioSetTimerFunc(id, millis, f, 0, NULL);

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioSetTimerFuncEx(unsigned id, unsigned millis, gpioTimerFuncEx_t f,
                       void * userdata)
{
   DBG(DBG_USER, "id=%d millis=%d function=%08"PRIXPTR", userdata=%08"PRIXPTR,
      id, millis, (uintptr_t)f, (uintptr_t)userdata);

   CHECK_INITED;

   if (id > PI_MAX_TIMER)
      SOFT_ERROR(PI_BAD_TIMER, "bad timer id (%d)", id);

   if ((millis < PI_MIN_MS) || (millis > PI_MAX_MS))
      SOFT_ERROR(PI_BAD_MS, "timer %d, bad millis (%d)", id, millis);

   intGpioSetTimerFunc(id, millis, f, 1, userdata);

   return 0;
}

/* ----------------------------------------------------------------------- */

pthread_t *gpioStartThread(gpioThreadFunc_t f, void *userdata)
{
   pthread_t *pth;
   pthread_attr_t pthAttr;

   DBG(DBG_USER, "f=%08"PRIXPTR", userdata=%08"PRIXPTR, (uintptr_t)f, (uintptr_t)userdata);

   CHECK_INITED_RET_NULL_PTR;

   pth = malloc(sizeof(pthread_t));

   if (pth)
   {
      if (pthread_attr_init(&pthAttr))
      {
         free(pth);
         SOFT_ERROR(NULL, "pthread_attr_init failed");
      }

      if (pthread_attr_setstacksize(&pthAttr, STACK_SIZE))
      {
         free(pth);
         SOFT_ERROR(NULL, "pthread_attr_setstacksize failed");
      }

      if (pthread_create(pth, &pthAttr, f, userdata))
      {
         free(pth);
         SOFT_ERROR(NULL, "pthread_create failed");
      }
   }
   return pth;
}

/* ----------------------------------------------------------------------- */

void gpioStopThread(pthread_t *pth)
{
   DBG(DBG_USER, "pth=%08"PRIXPTR, (uintptr_t)pth);

   CHECK_INITED_RET_NIL;

   if (pth)
   {
      if (pthread_self() == *pth)
      {
         free(pth);
         pthread_exit(NULL);
      }
      else
      {
         pthread_cancel(*pth);
         pthread_join(*pth, NULL);
         free(pth);
      }
   }
}

/* ----------------------------------------------------------------------- */

int gpioStoreScript(char *script)
{
   static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
   gpioScript_t *s;
   int status, slot, i;

   DBG(DBG_USER, "script=[%s]", script);

   CHECK_INITED;

   slot = -1;

   pthread_mutex_lock(&mutex);

   for (i=0; i<PI_MAX_SCRIPTS; i++)
   {
      if (gpioScript[i].state == PI_SCRIPT_FREE)
      {
         slot = i;
         gpioScript[slot].state = PI_SCRIPT_RESERVED;
         break;
      }
   }

   pthread_mutex_unlock(&mutex);

   if (slot < 0) SOFT_ERROR(PI_NO_SCRIPT_ROOM, "no room for scripts");

   s = &gpioScript[slot];

   status = cmdParseScript(script, &s->script, 0);

   if (status == 0)
   {
      s->request   = PI_SCRIPT_HALT;
      s->run_state = PI_SCRIPT_INITING;

      pthread_cond_init(&s->pthCond, NULL);
      pthread_mutex_init(&s->pthMutex, NULL);

      s->id = slot;

      gpioScript[slot].state = PI_SCRIPT_IN_USE;

      s->pthIdp = gpioStartThread(pthScript, s);

      status = slot;
   }
   else
   {
      if (s->script.par) free(s->script.par);
      s->script.par = NULL;
      gpioScript[slot].state = PI_SCRIPT_FREE;
   }

   return status;
}


/* ----------------------------------------------------------------------- */

int gpioRunScript(unsigned script_id, unsigned numParam, uint32_t *param)
{
   int status = 0;

   DBG(DBG_USER, "script_id=%d numParam=%d param=%08"PRIXPTR,
      script_id, numParam, (uintptr_t)param);

   CHECK_INITED;

   if (script_id >= PI_MAX_SCRIPTS)
      SOFT_ERROR(PI_BAD_SCRIPT_ID, "bad script id(%d)", script_id);

   if (numParam > PI_MAX_SCRIPT_PARAMS)
      SOFT_ERROR(PI_TOO_MANY_PARAM, "bad number of parameters(%d)", numParam);

   if (gpioScript[script_id].state == PI_SCRIPT_IN_USE)
   {
      pthread_mutex_lock(&gpioScript[script_id].pthMutex);

      if (gpioScript[script_id].run_state != PI_SCRIPT_INITING)
      {
         if ((numParam > 0) && (param != 0))
         {
            memcpy(gpioScript[script_id].script.par, param,
               sizeof(uint32_t) * numParam);
         }

         gpioScript[script_id].request = PI_SCRIPT_RUN;

         pthread_cond_signal(&gpioScript[script_id].pthCond);
      }
      else
      {
         status = PI_SCRIPT_NOT_READY;
      }

      pthread_mutex_unlock(&gpioScript[script_id].pthMutex);

      return status;
   }
   else
   {
      return PI_BAD_SCRIPT_ID;
   }
}


/* ----------------------------------------------------------------------- */

int gpioUpdateScript(unsigned script_id, unsigned numParam, uint32_t *param)
{
   DBG(DBG_USER, "script_id=%d numParam=%d param=%08"PRIXPTR,
      script_id, numParam, (uintptr_t)param);

   CHECK_INITED;

   if (script_id >= PI_MAX_SCRIPTS)
      SOFT_ERROR(PI_BAD_SCRIPT_ID, "bad script id(%d)", script_id);

   if (numParam > PI_MAX_SCRIPT_PARAMS)
      SOFT_ERROR(PI_TOO_MANY_PARAM, "bad number of parameters(%d)", numParam);

   if (gpioScript[script_id].state == PI_SCRIPT_IN_USE)
   {
      if ((numParam > 0) && (param != 0))
      {
         memcpy(gpioScript[script_id].script.par, param,
            sizeof(uint32_t) * numParam);
      }
   }
   else
   {
      return PI_BAD_SCRIPT_ID;
   }

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioScriptStatus(unsigned script_id, uint32_t *param)
{
   DBG(DBG_USER, "script_id=%d param=%08"PRIXPTR, script_id, (uintptr_t)param);

   CHECK_INITED;

   if (script_id >= PI_MAX_SCRIPTS)
      SOFT_ERROR(PI_BAD_SCRIPT_ID, "bad script id(%d)", script_id);

   if (gpioScript[script_id].state == PI_SCRIPT_IN_USE)
   {
      if (param != NULL)
      {
         memcpy(param, gpioScript[script_id].script.par,
            sizeof(uint32_t) * PI_MAX_SCRIPT_PARAMS);
      }

      return gpioScript[script_id].run_state;
   }
   else return PI_BAD_SCRIPT_ID;
}


/* ----------------------------------------------------------------------- */

int gpioStopScript(unsigned script_id)
{
   DBG(DBG_USER, "script_id=%d", script_id);

   CHECK_INITED;

   if (script_id >= PI_MAX_SCRIPTS)
      SOFT_ERROR(PI_BAD_SCRIPT_ID, "bad script id(%d)", script_id);

   if (gpioScript[script_id].state == PI_SCRIPT_IN_USE)
   {
      pthread_mutex_lock(&gpioScript[script_id].pthMutex);

      gpioScript[script_id].request = PI_SCRIPT_HALT;

      if (gpioScript[script_id].run_state == PI_SCRIPT_WAITING)
      {
         pthread_cond_signal(&gpioScript[script_id].pthCond);
      }

      pthread_mutex_unlock(&gpioScript[script_id].pthMutex);

      return 0;
   }
   else return PI_BAD_SCRIPT_ID;
}

/* ----------------------------------------------------------------------- */

int gpioDeleteScript(unsigned script_id)
{
   DBG(DBG_USER, "script_id=%d", script_id);

   CHECK_INITED;

   if (script_id >= PI_MAX_SCRIPTS)
      SOFT_ERROR(PI_BAD_SCRIPT_ID, "bad script id(%d)", script_id);

   if (gpioScript[script_id].state == PI_SCRIPT_IN_USE)
   {
      gpioScript[script_id].state = PI_SCRIPT_DYING;

      pthread_mutex_lock(&gpioScript[script_id].pthMutex);

      gpioScript[script_id].request = PI_SCRIPT_HALT;

      if (gpioScript[script_id].run_state == PI_SCRIPT_WAITING)
      {
         pthread_cond_signal(&gpioScript[script_id].pthCond);
      }

      pthread_mutex_unlock(&gpioScript[script_id].pthMutex);

      while (gpioScript[script_id].run_state == PI_SCRIPT_RUNNING)
      {
         myGpioSleep(0, 5000); /* give script time to halt */
      }

      gpioStopThread(gpioScript[script_id].pthIdp);

      if (gpioScript[script_id].script.par)
         free(gpioScript[script_id].script.par);

      gpioScript[script_id].script.par = NULL;

      gpioScript[script_id].state = PI_SCRIPT_FREE;

      return 0;
   }
   else return PI_BAD_SCRIPT_ID;
}



/* ----------------------------------------------------------------------- */

int gpioSetSignalFunc(unsigned signum, gpioSignalFunc_t f)
{
   DBG(DBG_USER, "signum=%d function=%08"PRIXPTR, signum, (uintptr_t)f);

   CHECK_INITED;

   if (signum > PI_MAX_SIGNUM)
      SOFT_ERROR(PI_BAD_SIGNUM, "bad signum (%d)", signum);

   gpioSignal[signum].ex = 0;
   gpioSignal[signum].userdata = NULL;

   gpioSignal[signum].func = f;

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioSetSignalFuncEx(unsigned signum, gpioSignalFuncEx_t f,
                        void *userdata)
{
   DBG(DBG_USER, "signum=%d function=%08"PRIXPTR" userdata=%08"PRIXPTR,
      signum, (uintptr_t)f, (uintptr_t)userdata);

   CHECK_INITED;

   if (signum > PI_MAX_SIGNUM)
      SOFT_ERROR(PI_BAD_SIGNUM, "bad signum (%d)", signum);

   gpioSignal[signum].ex = 1;
   gpioSignal[signum].userdata = userdata;

   gpioSignal[signum].func = f;

   return 0;
}


/* ----------------------------------------------------------------------- */

uint32_t gpioRead_Bits_0_31(void)
{
   DBG(DBG_USER, "");

   CHECK_INITED;

   return (*(gpioReg + GPLEV0));
}


/* ----------------------------------------------------------------------- */

uint32_t gpioRead_Bits_32_53(void)
{
   DBG(DBG_USER, "");

   CHECK_INITED;

   return (*(gpioReg + GPLEV1));
}


/* ----------------------------------------------------------------------- */

int gpioWrite_Bits_0_31_Clear(uint32_t bits)
{
   DBG(DBG_USER, "bits=%08X", bits);

   CHECK_INITED;

   *(gpioReg + GPCLR0) = bits;

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioWrite_Bits_32_53_Clear(uint32_t bits)
{
   DBG(DBG_USER, "bits=%08X", bits);

   CHECK_INITED;

   *(gpioReg + GPCLR1) = bits;

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioWrite_Bits_0_31_Set(uint32_t bits)
{
   DBG(DBG_USER, "bits=%08X", bits);

   CHECK_INITED;

   *(gpioReg + GPSET0) = bits;

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioWrite_Bits_32_53_Set(uint32_t bits)
{
   DBG(DBG_USER, "bits=%08X", bits);

   CHECK_INITED;

   *(gpioReg + GPSET1) = bits;

   return 0;
}

/* ----------------------------------------------------------------------- */

int gpioHardwareClock(unsigned gpio, unsigned frequency)
{
   int cctl[] = {CLK_GP0_CTL, CLK_GP1_CTL, CLK_GP2_CTL};
   int cdiv[] = {CLK_GP0_DIV, CLK_GP1_DIV, CLK_GP2_DIV};
   int csrc[CLK_SRCS] = {CLK_CTL_SRC_OSC, CLK_CTL_SRC_PLLD};
   uint32_t cfreq[CLK_SRCS]={clk_osc_freq, clk_plld_freq};
   unsigned clock, mode, mash;
   int password = 0;
   double f;
   clkInf_t clkInf={0,0,0};

   DBG(DBG_USER, "gpio=%d frequency=%d", gpio, frequency);

   CHECK_INITED;

   if ((gpio >> 24) == 0x5A) password = 1;

   gpio &= 0xFFFFFF;

   if (gpio > PI_MAX_GPIO)
      SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);

   if (!clkDef[gpio])
      SOFT_ERROR(PI_NOT_HCLK_GPIO, "bad gpio for clock (%d)", gpio);

   if (((frequency < hw_clk_min_freq) ||
        (frequency > hw_clk_max_freq)) &&
        (frequency))
      SOFT_ERROR(PI_BAD_HCLK_FREQ,
         "bad hardware clock frequency %d-%d: (%d)",
            hw_clk_min_freq, hw_clk_max_freq, frequency);

   clock = (clkDef[gpio] >> 4) & 3;

   if ((clock == 1) && (!password))
      SOFT_ERROR(PI_BAD_HCLK_PASS,
         "Need password to use clock 1 (%d)", gpio);

   mode  = clkDef[gpio] & 7;
   mash = frequency < PI_MASH_MAX_FREQ ? 1 : 0;

   if (frequency)
   {
      if (chooseBestClock(&clkInf, frequency, CLK_SRCS, cfreq))
      {
         if (clkInf.frac == 0) mash = 0;

         initHWClk(cctl[clock], cdiv[clock],
            csrc[clkInf.clock], clkInf.div, clkInf.frac, mash);

         myGpioSetMode(gpio, mode);

         gpioInfo[gpio].is = GPIO_HW_CLK;

         f = (double) cfreq[clkInf.clock] /
           ((double)clkInf.div + ((double)clkInf.frac / 4096.0));

         hw_clk_freq[clock] = (f + 0.5);

         DBG(DBG_USER, "cf=%d div=%d frac=%d mash=%d",
            cfreq[clkInf.clock], clkInf.div, clkInf.frac, mash);
      }
      else
      {
         SOFT_ERROR(PI_BAD_HCLK_FREQ,
            "bad hardware clock frequency %d-%d: (%d)",
               hw_clk_min_freq, hw_clk_max_freq, frequency);
      }
   }
   else
   {
      /* frequency 0, stop clock */
      clkReg[cctl[clock]] = BCM_PASSWD | CLK_CTL_KILL;

      if (gpioInfo[gpio].is == GPIO_HW_CLK)
         gpioInfo[gpio].is = GPIO_UNDEFINED;
   }

   return 0;
}

/* ----------------------------------------------------------------------- */

int gpioHardwarePWM(
   unsigned gpio, unsigned frequency, unsigned dutycycle)
{
   uint32_t old_PWM_CTL;
   unsigned pwm, mode;
   uint32_t real_range, real_dutycycle;

   DBG(DBG_USER, "gpio=%d  frequency=%d dutycycle=%d",
      gpio, frequency, dutycycle);

   CHECK_INITED;

   if (gpio > PI_MAX_GPIO)
      SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);

   if (!PWMDef[gpio])
      SOFT_ERROR(PI_NOT_HPWM_GPIO, "bad gpio for PWM (%d)", gpio);

   if (dutycycle > PI_HW_PWM_RANGE)
      SOFT_ERROR(PI_BAD_HPWM_DUTY, "bad PWM dutycycle (%d)", dutycycle);

   if (((frequency < PI_HW_PWM_MIN_FREQ) ||
        (frequency > hw_pwm_max_freq)) &&
        (frequency))
      SOFT_ERROR(PI_BAD_HPWM_FREQ,
         "bad hardware PWM frequency %d-%d: (%d)",
            PI_HW_PWM_MIN_FREQ, hw_pwm_max_freq, frequency);


   if (gpioCfg.clockPeriph == PI_CLOCK_PWM)
      SOFT_ERROR(PI_HPWM_ILLEGAL, "illegal, PWM in use for main clock");

   pwm = (PWMDef[gpio] >> 4) & 3;
   mode  = PWMDef[gpio] & 7;

   if (frequency)
   {
      real_range = ((double)clk_plld_freq / (2.0 * frequency)) + 0.5;
      real_dutycycle = ((uint64_t)dutycycle * real_range) / PI_HW_PWM_RANGE;

      /* record the set PWM frequency and dutycycle */

      hw_pwm_freq[pwm] =
         ((double)clk_plld_freq / ( 2.0 * real_range)) + 0.5;

      hw_pwm_duty[pwm]  = dutycycle;

      hw_pwm_real_range[pwm] = real_range;

      /* Abort any waveform transmission in progress */

      if (gpioWaveTxBusy()) gpioWaveTxStop();

      waveClockInited = 0;

      /* preserve channel enable only and mark space mode */

      old_PWM_CTL = pwmReg[PWM_CTL] &
         (PWM_CTL_PWEN1 | PWM_CTL_MSEN1 | PWM_CTL_PWEN2 | PWM_CTL_MSEN2);

      if (!PWMClockInited)
      {
         pwmReg[PWM_CTL] = 0;

         myGpioDelay(10);

         initHWClk(CLK_PWMCTL, CLK_PWMDIV, CLK_CTL_SRC_PLLD, 2, 0, 0);

         PWMClockInited = 1;
      }

      if (pwm == 0)
      {
         pwmReg[PWM_RNG1] = real_range;
         myGpioDelay(10);
         pwmReg[PWM_DAT1] = real_dutycycle;
         myGpioDelay(10);

         pwmReg[PWM_CTL] = (old_PWM_CTL | PWM_CTL_PWEN1 | PWM_CTL_MSEN1);
      }
      else
      {
         pwmReg[PWM_RNG2] = real_range;
         myGpioDelay(10);
         pwmReg[PWM_DAT2] = real_dutycycle;
         myGpioDelay(10);

         pwmReg[PWM_CTL] = (old_PWM_CTL | PWM_CTL_PWEN2 | PWM_CTL_MSEN2);
      }

      if (gpioInfo[gpio].is != GPIO_HW_PWM)
      {
         switchFunctionOff(gpio);

         myGpioSetMode(gpio, mode);

         gpioInfo[gpio].is = GPIO_HW_PWM;
      }
   }
   else
   {
      /* frequency 0, stop PWM */

      if (gpioInfo[gpio].is == GPIO_HW_PWM)
      {
         if (pwm == 0) pwmReg[PWM_CTL] &= (~PWM_CTL_PWEN1);
         else          pwmReg[PWM_CTL] &= (~PWM_CTL_PWEN2);

         gpioInfo[gpio].is = GPIO_UNDEFINED;
      }
   }

   return 0;
}


int gpioSetPad(unsigned pad, unsigned padStrength)
{
   DBG(DBG_USER, "pad=%d  padStrength=%d", pad, padStrength);

   CHECK_INITED;

   if (pad > PI_MAX_PAD)
      SOFT_ERROR(PI_BAD_PAD, "bad pad number (%d)", pad);

   if ((padStrength < PI_MIN_PAD_STRENGTH) ||
       (padStrength > PI_MAX_PAD_STRENGTH))
      SOFT_ERROR(PI_BAD_STRENGTH, "bad pad drive strength (%d)", pad);

   /* 1-16 -> 0-7 */

   padStrength += 1;
   padStrength /= 2;
   padStrength -= 1;

   padsReg[11+pad] = BCM_PASSWD | 0x18 | (padStrength & 7) ;

   return 0;
}

int gpioGetPad(unsigned pad)
{
   int strength;

   DBG(DBG_USER, "pad=%d", pad);

   CHECK_INITED;

   if (pad > PI_MAX_PAD)
      SOFT_ERROR(PI_BAD_PAD, "bad pad (%d)", pad);

   strength = padsReg[11+pad] & 7;

   strength *= 2;
   strength += 2;

   return strength;
}

int shell(char *scriptName, char *scriptString)
{
   int status;
   char buf[4096];

   DBG(DBG_USER, "name=%s string=%s", scriptName, scriptString);

   CHECK_INITED;

   if (!myScriptNameValid(scriptName))
      SOFT_ERROR(PI_BAD_SCRIPT_NAME, "bad script name (%s)", scriptName);

   snprintf(buf, sizeof(buf),
      "/opt/pigpio/cgi/%s %s", scriptName, scriptString);

   DBG(DBG_USER, "%s", buf);

   status = system(buf);

   if (status < 0) status = PI_BAD_SHELL_STATUS;

   return status;
}


int fileApprove(char *filename)
{
   char match[PI_MAX_PATH];
   char buffer[PI_MAX_PATH];
   char line[PI_MAX_PATH];
   char mperm=0;
   char perm;
   char term;
   FILE *f;

   buffer[0] = 0;
   match[0] = 0;

   if (myPathBad(filename)) return PI_FILE_NONE;

   f = fopen("/opt/pigpio/access", "r");

   if (!f) return PI_FILE_NONE;

   while (!feof(f))
   {
      buffer[0] = 0;
      perm = 0;
      term = 0;
      if (fgets(line, sizeof(line), f))
      {
         sscanf(line, " %511s %c%c", buffer, &perm, &term);
         if (term == 10)
         {
            if (myPathBad(buffer)) continue; /* disallow risky lines */

            if (fnmatch(buffer, filename, 0) == 0)
            {
               if (match[0])
               {
                  if (fnmatch(match, buffer, 0) == 0)
                  {
                     strcpy(match, buffer);
                     mperm = perm;
                  }
               }
               else
               {
                  strcpy(match, buffer);
                  mperm = perm;
               }
            }
         }
      }
   }

   fclose(f);

   if (match[0])
   {
      switch (toupper(mperm))
      {
         case 'R': return PI_FILE_READ;
         case 'W': return PI_FILE_WRITE;
         case 'U': return PI_FILE_RW;
         default : return PI_FILE_NONE;
      }
   }

   return PI_FILE_NONE;
}

int fileOpen(char *file, unsigned mode)
{
   static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
   int fd=-1;
   int i, slot, oflag, omode, pmode, rmode;
   struct stat statbuf;

   DBG(DBG_USER, "file=%s mode=%d", file, mode);

   CHECK_INITED;

   if ( (mode < PI_FILE_MIN) ||
        (mode > PI_FILE_MAX) ||
        ((mode & PI_FILE_RW) == 0) )
      SOFT_ERROR(PI_BAD_FILE_MODE, "bad mode (%d)", mode);

   pmode = fileApprove(file); // 0=NONE, 1=READ, 2=WRITE, 3=RW
   rmode = mode & PI_FILE_RW; // 0=NONE, 1=READ, 2=WRITE, 3=RW

   if (((pmode & rmode) != rmode) || (rmode == PI_FILE_NONE))
      SOFT_ERROR(PI_NO_FILE_ACCESS, "no permission to access file (%s)", file);

   if ((mode > 3) && ((mode & PI_FILE_WRITE) == 0))
      SOFT_ERROR(PI_NO_FILE_ACCESS, "no permission to write file (%s)", file);

   slot = -1;

   pthread_mutex_lock(&mutex);

   for (i=0; i<PI_FILE_SLOTS; i++)
   {
      if (fileInfo[i].state == PI_FILE_CLOSED)
      {
         slot = i;
         fileInfo[slot].state = PI_FILE_RESERVED;
         break;
      }
   }

   pthread_mutex_unlock(&mutex);

   if (slot < 0) SOFT_ERROR(PI_NO_HANDLE, "no file handles");

   omode = 0;
   oflag = 0;

   if (mode & PI_FILE_APPEND)
   {
      oflag |= O_APPEND;
   }

   if (mode & PI_FILE_CREATE)
   {
      oflag |= O_CREAT;
      omode |= (S_IRUSR|S_IWUSR);
   }

   if (mode & PI_FILE_TRUNC)
   {
      oflag |= O_TRUNC;
   }

   switch(mode&PI_FILE_RW)
   {
      case PI_FILE_READ:
         fd = open(file, O_RDONLY|oflag, omode);
         break;

      case PI_FILE_WRITE:
         fd = open(file, O_WRONLY|oflag, omode);
         break;

      case PI_FILE_RW:
         fd = open(file, O_RDWR|oflag, omode);
         break;
   }

   if (fd == -1)
   {
      fileInfo[slot].state = PI_FILE_CLOSED;
      return PI_FIL_OPEN_FAILED;
   }
   else
   {
      if (stat(file, &statbuf) == 0)
      {
         if (S_ISDIR(statbuf.st_mode))
         {
            close(fd);
            fileInfo[slot].state = PI_FILE_CLOSED;
            SOFT_ERROR(PI_FILE_IS_A_DIR, "file is a directory (%s)", file);
         }
      }
   }

   fileInfo[slot].fd = fd;
   fileInfo[slot].mode = mode;
   fileInfo[slot].state = PI_FILE_OPENED;

   return slot;
}

int fileClose(unsigned handle)
{
   DBG(DBG_USER, "handle=%d", handle);

   CHECK_INITED;

   if (handle >= PI_FILE_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (fileInfo[handle].state != PI_FILE_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (fileInfo[handle].fd >= 0) close(fileInfo[handle].fd);

   fileInfo[handle].fd = -1;
   fileInfo[handle].state = PI_FILE_CLOSED;

   return 0;
}

int fileWrite(unsigned handle, char *buf, unsigned count)
{
   int w;

   DBG(DBG_USER, "handle=%d count=%d [%s]",
      handle, count, myBuf2Str(count, buf));

   CHECK_INITED;

   if (handle >= PI_FILE_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (fileInfo[handle].state != PI_FILE_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (!count)
      SOFT_ERROR(PI_BAD_PARAM, "bad count (%d)", count);

   if (!(fileInfo[handle].mode & PI_FILE_WRITE))
      SOFT_ERROR(PI_FILE_NOT_WOPEN, "file not opened for write");

   w = write(fileInfo[handle].fd, buf, count);

   if (w != count)
   {
      if (w == -1) DBG(DBG_USER, "write failed with errno %d", errno);

      return PI_BAD_FILE_WRITE;
   }
   return 0;
}

int fileRead(unsigned handle, char *buf, unsigned count)
{
   int r;

   DBG(DBG_USER, "handle=%d count=%d buf=0x%"PRIXPTR, handle, count, (uintptr_t)buf);

   CHECK_INITED;

   if (handle >= PI_FILE_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (fileInfo[handle].state != PI_FILE_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (!count)
      SOFT_ERROR(PI_BAD_PARAM, "bad count (%d)", count);

   if (!(fileInfo[handle].mode & PI_FILE_READ))
      SOFT_ERROR(PI_FILE_NOT_ROPEN, "file not opened for read");

   r = read(fileInfo[handle].fd, buf, count);

   if (r == -1)
   {
      DBG(DBG_USER, "read failed with errno %d", errno);
      return PI_BAD_FILE_READ;
   }
   else
   {
      buf[r] = 0;
      return r;
   }
}


int fileSeek(unsigned handle, int32_t seekOffset, int seekFrom)
{
   int whence, s;

   DBG(DBG_USER, "handle=%d offset=%d from=%d",
      handle, seekOffset, seekFrom);

   CHECK_INITED;

   if (handle >= PI_FILE_SLOTS)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   if (fileInfo[handle].state != PI_FILE_OPENED)
      SOFT_ERROR(PI_BAD_HANDLE, "bad handle (%d)", handle);

   switch (seekFrom)
   {
      case PI_FROM_START:
         whence = SEEK_SET;
         break;

      case PI_FROM_CURRENT:
         whence = SEEK_CUR;
         break;

      case PI_FROM_END:
         whence = SEEK_END;
         break;

      default:
         SOFT_ERROR(PI_BAD_FILE_SEEK, "bad seek from (%d)", seekFrom);
   }

   s = lseek(fileInfo[handle].fd, seekOffset, whence);

   if (s == -1)
   {
      DBG(DBG_USER, "seek failed with errno %d", errno);
      return PI_BAD_FILE_SEEK;
   }

   return s;
}

int fileList(char *fpat,  char *buf, unsigned count)
{
   int len, bufpos;
   glob_t pglob;
   int i;

   DBG(DBG_USER, "fpat=%s count=%d buf=%"PRIxPTR, fpat, count, (uintptr_t)buf);

   CHECK_INITED;

   if ((fileApprove(fpat) & PI_FILE_READ) != PI_FILE_READ)
      SOFT_ERROR(PI_NO_FILE_ACCESS, "no permission to access file (%s)", fpat);

   bufpos = 0;

   if (glob(fpat, GLOB_MARK, NULL, &pglob) == 0)
   {
      for (i=0; i<pglob.gl_pathc; i++)
      {
         len = strlen(pglob.gl_pathv[i]);
         if ((bufpos + len + 1) < count)
         {
            strcpy(buf+bufpos, pglob.gl_pathv[i]);
            bufpos += len;
            buf[bufpos++] = '\n';
         }
      }
   }
   else
   {
      bufpos = PI_NO_FILE_MATCH;
   }

   globfree(&pglob);

   return bufpos;
}



/* ----------------------------------------------------------------------- */

int gpioTime(unsigned timetype, int *seconds, int *micros)
{
   struct timespec ts;

   DBG(DBG_USER, "timetype=%d &seconds=%08"PRIXPTR" &micros=%08"PRIXPTR,
      timetype, (uintptr_t)seconds, (uintptr_t)micros);

   CHECK_INITED;

   if (timetype > PI_TIME_ABSOLUTE)
      SOFT_ERROR(PI_BAD_TIMETYPE, "bad timetype (%d)", timetype);

   if (timetype == PI_TIME_ABSOLUTE)
   {
      clock_gettime(CLOCK_REALTIME, &ts);
      *seconds = ts.tv_sec;
      *micros  = ts.tv_nsec/1000;
   }
   else
   {
      clock_gettime(CLOCK_REALTIME, &ts);

      TIMER_SUB(&ts, &libStarted, &ts);

      *seconds = ts.tv_sec;
      *micros  = ts.tv_nsec/1000;
   }

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioSleep(unsigned timetype, int seconds, int micros)
{
   struct timespec ts, rem;

   DBG(DBG_USER, "timetype=%d seconds=%d micros=%d",
      timetype, seconds, micros);

   CHECK_INITED;

   if (timetype > PI_TIME_ABSOLUTE)
      SOFT_ERROR(PI_BAD_TIMETYPE, "bad timetype (%d)", timetype);

   if (seconds < 0)
      SOFT_ERROR(PI_BAD_SECONDS, "bad seconds (%d)", seconds);

   if ((micros < 0) || (micros > 999999))
      SOFT_ERROR(PI_BAD_MICROS, "bad micros (%d)", micros);

   ts.tv_sec  = seconds;
   ts.tv_nsec = micros * 1000;

   if (timetype == PI_TIME_ABSOLUTE)
   {
      while (clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, &rem));
   }
   else
   {
      while (clock_nanosleep(CLOCK_REALTIME, 0, &ts, &rem))
      {
         /* copy remaining time to ts */
         ts.tv_sec  = rem.tv_sec;
         ts.tv_nsec = rem.tv_nsec;
      }
   }

   return 0;
}


/* ----------------------------------------------------------------------- */

uint32_t gpioDelay(uint32_t micros)
{
   uint32_t start;

   DBG(DBG_USER, "microseconds=%u", micros);

   CHECK_INITED;

   start = systReg[SYST_CLO];

   if (micros <= PI_MAX_BUSY_DELAY)
      while ((systReg[SYST_CLO] - start) <= micros);
   else
      gpioSleep(PI_TIME_RELATIVE, (micros/MILLION), (micros%MILLION));

   return (systReg[SYST_CLO] - start);
}


/* ----------------------------------------------------------------------- */

uint32_t gpioTick(void)
{
   CHECK_INITED;

   return systReg[SYST_CLO];
}


/* ----------------------------------------------------------------------- */

unsigned gpioVersion(void)
{
   DBG(DBG_USER, "");

   return PIGPIO_VERSION;
}


/* ----------------------------------------------------------------------- */

/*
2 2  2  2 2 2  1 1 1 1  1 1 1 1  1 1 0 0 0 0 0 0  0 0 0 0
5 4  3  2 1 0  9 8 7 6  5 4 3 2  1 0 9 8 7 6 5 4  3 2 1 0

W W  S  M M M  B B B B  P P P P  T T T T T T T T  R R R R

W  warranty void if either bit is set

S  0=old (bits 0-22 are revision number) 1=new (following fields apply)

M  0=256 1=512 2=1024 3=2GB 4=4GB

B  0=Sony 1=Egoman 2=Embest 3=Sony Japan 4=Embest 5=Stadium

P  0=2835, 1=2836, 2=2837 3=2711

T  0=A 1=B 2=A+ 3=B+ 4=Pi2B 5=Alpha 6=CM1 8=Pi3B 9=Zero a=CM3 c=Zero W
   d=3B+ e=3A+ 10=CM3+ 11=4B

R  PCB board revision

*/

unsigned gpioHardwareRevision(void)
{
   static unsigned rev = 0;

   FILE * filp;
   char buf[512];
   char term;

   DBG(DBG_USER, "");

   if (rev) return rev;

   filp = fopen ("/proc/cpuinfo", "r");


   if (filp != NULL)
   {
      while (fgets(buf, sizeof(buf), filp) != NULL)
      {
         if (!strncasecmp("revision\t:", buf, 10))
         {
            if (sscanf(buf+10, "%x%c", &rev, &term) == 2)
            {
               if (term != '\n') rev = 0;
            }
         }
      }
      fclose(filp);
   }

   /* (some) arm64 operating systems get revision number here  */

   if (rev == 0)
   {
      DBG(DBG_USER, "searching /proc/device-tree for revision");
      filp = fopen ("/proc/device-tree/system/linux,revision", "r");

      if (filp != NULL)
      {
         uint32_t tmp;
         if (fread(&tmp,1 , 4, filp) == 4)
         {
            /*
               for some reason the value returned by reading
               this /proc entry seems to be big endian,
               convert it.
            */
            rev = ntohl(tmp);
            rev &= 0xFFFFFF; /* mask out warranty bit */
         }
         fclose(filp);
      }
   }

   piCores = 0;
   pi_ispi = 0;
   rev &= 0xFFFFFF; /* mask out warranty bit */

   /* Decode revision code */

   if ((rev & 0x800000) == 0) /* old rev code */
   {
      if ((rev > 0) && (rev < 0x0016)) /* all BCM2835 */
      {
         pi_ispi = 1;
         piCores = 1;
         pi_peri_phys = 0x20000000;
         pi_dram_bus  = 0x40000000;
         pi_mem_flag  = 0x0C;
      }
      else
      {
         DBG(DBG_ALWAYS, "unknown revision=%x", rev);
         rev = 0;
      }
   }
   else /* new rev code */
   {
      switch ((rev >> 12) & 0xF)  /* just interested in BCM model */
      {

         case 0x0:   /* BCM2835 */
            pi_ispi = 1;
            piCores = 1;
            pi_peri_phys = 0x20000000;
            pi_dram_bus  = 0x40000000;
            pi_mem_flag  = 0x0C;
            break;

         case 0x1:   /* BCM2836 */
         case 0x2:   /* BCM2837 */
            pi_ispi = 1;
            piCores = 4;
            pi_peri_phys = 0x3F000000;
            pi_dram_bus  = 0xC0000000;
            pi_mem_flag  = 0x04;
            break;

         case 0x3:   /* BCM2711 */
            pi_ispi = 1;
            piCores = 4;
            pi_peri_phys = 0xFE000000;
            pi_dram_bus  = 0xC0000000;
            pi_mem_flag  = 0x04;
            pi_is_2711   = 1;
            clk_osc_freq = CLK_OSC_FREQ_2711;
            clk_plld_freq = CLK_PLLD_FREQ_2711;
            hw_pwm_max_freq = PI_HW_PWM_MAX_FREQ_2711;
            hw_clk_min_freq = PI_HW_CLK_MIN_FREQ_2711;
            hw_clk_max_freq = PI_HW_CLK_MAX_FREQ_2711;
            break;

         default:
            DBG(DBG_ALWAYS, "unknown rev code (%x)", rev);
            rev=0;
            pi_ispi = 0;
            break;
      }
   }

   DBG(DBG_USER, "revision=%x", rev);
   DBG(DBG_USER, "pi_peri_phys=%x", pi_peri_phys);
   DBG(DBG_USER, "pi_dram_bus=%x", pi_dram_bus);
   DBG(DBG_USER, "pi_mem_flag=%x", pi_mem_flag);

   return rev;
}


/* ----------------------------------------------------------------------- */

int gpioCfgBufferSize(unsigned millis)
{
   DBG(DBG_USER, "millis=%d", millis);

   CHECK_NOT_INITED;

   if ((millis < PI_BUF_MILLIS_MIN) || (millis > PI_BUF_MILLIS_MAX))
      SOFT_ERROR(PI_BAD_BUF_MILLIS, "bad millis (%d)", millis);

   gpioCfg.bufferMilliseconds = millis;

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioCfgClock(unsigned micros, unsigned peripheral, unsigned source)
{
   DBG(DBG_USER, "micros=%d peripheral=%d", micros, peripheral);

   CHECK_NOT_INITED;

   if ((micros < 1) || (micros > 10))
      SOFT_ERROR(PI_BAD_CLK_MICROS, "bad micros (%d)", micros);

   if (!clkCfg[micros].valid)
      SOFT_ERROR(PI_BAD_CLK_MICROS, "bad micros (%d)", micros);

   if (peripheral > PI_CLOCK_PCM)
      SOFT_ERROR(PI_BAD_CLK_PERIPH, "bad peripheral (%d)", peripheral);

   gpioCfg.clockMicros = micros;
   gpioCfg.clockPeriph = peripheral;

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioCfgDMAchannel(unsigned DMAchannel)
{
   DBG(DBG_USER, "channel=%d", DMAchannel);

   CHECK_NOT_INITED;

   if ((DMAchannel < PI_MIN_DMA_CHANNEL) || (DMAchannel > PI_MAX_DMA_CHANNEL))
      SOFT_ERROR(PI_BAD_CHANNEL, "bad channel (%d)", DMAchannel);

   gpioCfg.DMAprimaryChannel = DMAchannel;

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioCfgDMAchannels(unsigned primaryChannel, unsigned secondaryChannel)
{
   DBG(DBG_USER, "primary channel=%d, secondary channel=%d",
      primaryChannel, secondaryChannel);

   CHECK_NOT_INITED;

   if (primaryChannel > PI_MAX_DMA_CHANNEL)
      SOFT_ERROR(PI_BAD_PRIM_CHANNEL, "bad primary channel (%d)",
         primaryChannel);

   if ((secondaryChannel > PI_MAX_DMA_CHANNEL) ||
         ((secondaryChannel == primaryChannel) &&
            (secondaryChannel != PI_DEFAULT_DMA_NOT_SET)))
      SOFT_ERROR(PI_BAD_SECO_CHANNEL, "bad secondary channel (%d)",
         secondaryChannel);

   gpioCfg.DMAprimaryChannel   = primaryChannel;
   gpioCfg.DMAsecondaryChannel = secondaryChannel;

   return 0;
}


/*-------------------------------------------------------------------------*/

int gpioCfgPermissions(uint64_t updateMask)
{
   DBG(DBG_USER, "gpio update mask=%"PRIX64, updateMask);

   CHECK_NOT_INITED;

   gpioMask = updateMask;

   gpioMaskSet = 1;

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioCfgInterfaces(unsigned ifFlags)
{
   DBG(DBG_USER, "ifFlags=%X", ifFlags);

   CHECK_NOT_INITED;

   if (ifFlags > 15)
      SOFT_ERROR(PI_BAD_IF_FLAGS, "bad ifFlags (%X)", ifFlags);

   gpioCfg.ifFlags = ifFlags;

   return 0;
}

/* ----------------------------------------------------------------------- */

int gpioCfgSocketPort(unsigned port)
{
   DBG(DBG_USER, "port=%d", port);

   CHECK_NOT_INITED;

   if ((port < PI_MIN_SOCKET_PORT) || (port > PI_MAX_SOCKET_PORT))
      SOFT_ERROR(PI_BAD_SOCKET_PORT, "bad port (%d)", port);

   gpioCfg.socketPort = port;

   return 0;
}


/* ----------------------------------------------------------------------- */

int gpioCfgMemAlloc(unsigned memAllocMode)
{
   DBG(DBG_USER, "memAllocMode=%d", memAllocMode);

   CHECK_NOT_INITED;

   if (memAllocMode > PI_MEM_ALLOC_MAILBOX)
      SOFT_ERROR(
         PI_BAD_MALLOC_MODE, "bad mem alloc mode (%d)", memAllocMode);

   gpioCfg.memAllocMode = memAllocMode;

   return 0;
}

/* ----------------------------------------------------------------------- */

int gpioCfgNetAddr(int numSockAddr, uint32_t *sockAddr)
{
   int i;

   DBG(DBG_USER, "numSockAddr=%d sockAddr=%08"PRIXPTR,
      numSockAddr, (uintptr_t)sockAddr);

   CHECK_NOT_INITED;

   if (numSockAddr <= 0) numSockNetAddr = 0;
   else
   {
      if (numSockAddr >= MAX_CONNECT_ADDRESSES)
         numSockAddr = MAX_CONNECT_ADDRESSES;

      for (i=0; i<numSockAddr; i++) sockNetAddr[i] = sockAddr[i];

      numSockNetAddr = numSockAddr;
   }
   return 0;
}


/* ----------------------------------------------------------------------- */

uint32_t gpioCfgGetInternals(void)
{
   return gpioCfg.internals;
}

int gpioCfgSetInternals(uint32_t cfgVal)
{
   gpioCfg.internals = cfgVal;
   gpioCfg.dbgLevel = cfgVal & 0xF;
   gpioCfg.alertFreq = (cfgVal>>4) & 0xF;
   return 0;
}


/* include any user customisations */

#include "custom.cext"

