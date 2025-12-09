#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED

#define ENAGAL
// #define ENAGLO
// #define ENAQZS
// #define ENACMP
// #define ENASBS

#define NUMSATSOL   20      // Max number of observations used in the solution
#define MAXOBS      46      // Max number of observations used in pre-buffer
#define NFREQ       2       // Number of carrier frequencies
#define NEXOBS      0       // Number of extended obs codes

#if defined(GPX_GNSS_F9P)
    #error "F9P defined. Are you sure want to do this?  If so comment this line out in rtk_defines.h"
    #define L1_L5_RTK   0        // Use second slot for L2 
#else
    #define L1_L5_RTK   1        // Use second slot for L5 
#endif

// #if defined(RTK_EMBEDDED)
// #include "data_sets.h"
// #else
// #include "ISConstants.h"
// #endif

// #define RTK_MALLOC MALLOC
// // #define RTK_CALLOC calloc    // not supported
// #define RTK_FREE FREE

#define NFREQ2 (NFREQ == 1 ? 2 : NFREQ) // DO NOT CHANGE
#define RTK_INPUT_COUNT 2 // rover, base station - DO NOT CHANGE

#define HALF_MAXOBS (MAXOBS/2)

#define MAXSUBFRMLEN 152
#define MAXRAWLEN 2048

#ifdef ENAGLO
#define NFREQGLO 2
#else
#define NFREQGLO 0
#endif

#ifdef ENAGAL
#define NFREQGAL 2
#else
#define NFREQGAL 0
#endif

#define MINPRNGPS   1                   /* min satellite PRN number of GPS */
#define MAXPRNGPS   32                  /* max satellite PRN number of GPS */
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) /* number of GPS satellites */
#define NSYSGPS     1

#ifdef ENAGLO
#define MINPRNGLO   1                   /* min satellite slot number of GLONASS */
#define MAXPRNGLO   27                  /* max satellite slot number of GLONASS */
#define NSATGLO     (MAXPRNGLO-MINPRNGLO+1) /* number of GLONASS satellites */
#define NSYSGLO     1
#else
#define MINPRNGLO   0
#define MAXPRNGLO   0
#define NSATGLO     0
#define NSYSGLO     0
#endif

#ifdef ENAGAL
#define MINPRNGAL   1                   /* min satellite PRN number of Galileo */
#define MAXPRNGAL   36                  /* max satellite PRN number of Galileo */
#define NSATGAL    (MAXPRNGAL-MINPRNGAL+1) /* number of Galileo satellites */
#define NSYSGAL     1
#else
#define MINPRNGAL   0
#define MAXPRNGAL   0
#define NSATGAL     0
#define NSYSGAL     0
#endif

#ifdef ENAQZS
#define MINPRNQZS   193                 /* min satellite PRN number of QZSS */
#define MAXPRNQZS   202                 /* max satellite PRN number of QZSS */
#define MINPRNQZS_S 183                 /* min satellite PRN number of QZSS SAIF */
#define MAXPRNQZS_S 191                 /* max satellite PRN number of QZSS SAIF */
#define NSATQZS     (MAXPRNQZS-MINPRNQZS+1) /* number of QZSS satellites */
#define NSYSQZS     1
#else
#define MINPRNQZS   0
#define MAXPRNQZS   0
#define MINPRNQZS_S 0
#define MAXPRNQZS_S 0
#define NSATQZS     0
#define NSYSQZS     0
#endif

#ifdef ENACMP
#define MINPRNCMP   1                   /* min satellite sat number of BeiDou */
#define MAXPRNCMP   63                  /* max satellite sat number of BeiDou */
#define NSATCMP     (MAXPRNCMP-MINPRNCMP+1) /* number of BeiDou satellites */
#define NSYSCMP     1
#else
#define MINPRNCMP   0
#define MAXPRNCMP   0
#define NSATCMP     0
#define NSYSCMP     0
#endif

#ifdef ENAIRN
#define MINPRNIRN   1                   /* min satellite sat number of IRNSS */
#define MAXPRNIRN   14                  /* max satellite sat number of IRNSS */
#define NSATIRN     (MAXPRNIRN-MINPRNIRN+1) /* number of IRNSS satellites */
#define NSYSIRN     1
#else
#define MINPRNIRN   0
#define MAXPRNIRN   0
#define NSATIRN     0
#define NSYSIRN     0
#endif

#ifdef ENALEO
#define MINPRNLEO   1                   /* min satellite sat number of LEO */
#define MAXPRNLEO   10                  /* max satellite sat number of LEO */
#define NSATLEO     (MAXPRNLEO-MINPRNLEO+1) /* number of LEO satellites */
#define NSYSLEO     1
#else
#define MINPRNLEO   0
#define MAXPRNLEO   0
#define NSATLEO     0
#define NSYSLEO     0
#endif

#ifdef ENASBS
#define MINPRNSBS   120                 /* min satellite PRN number of SBAS */
#define MAXPRNSBS   158                 /* max satellite PRN number of SBAS */
#define NSATSBS     (MAXPRNSBS-MINPRNSBS+1) /* number of SBAS satellites */
#define SBAS_EPHEMERIS_ARRAY_SIZE NSATSBS
#else
#define MINPRNSBS   0
#define MAXPRNSBS   0
#define NSATSBS     0
#define SBAS_EPHEMERIS_ARRAY_SIZE 0
#endif

#define NSYS        (NSYSGPS+NSYSGLO+NSYSGAL+NSYSQZS+NSYSCMP+NSYSIRN+NSYSLEO) /* number of systems */

#define NX 7       /* Number of estimated parameters for point-positioning: 
                   (3) ECEF receiver position (m) + 
                   (1) GPS receiver clock bias (expresed in meters, i.e. multiplied by speed of light) + 
                   (1) glo-gps time offset (expresed in meters) +
                   (1) gal-gps time offset (expresed in meters) +
                   (1) bds-gps time offset (expresed in meters) */

#define MAXERRMSG 0

#define GPS_EPHEMERIS_ARRAY_SIZE (NSATGPS + NSATGAL + NSATQZS + NSATCMP)
#define GLONASS_EPHEMERIS_ARRAY_SIZE (NSATGLO)

#define DATA_TYPE_NONE 0
#define DATA_TYPE_OBSERVATION 1
#define DATA_TYPE_EPHEMERIS 2
#define DATA_TYPE_SBS 3
#define DATA_TYPE_ANTENNA_POSITION 5
#define DATA_TYPE_DGPS 7
#define DATA_TYPE_ION_UTC_ALMANAC 9
#define DATA_TYPE_SSR 10
#define DATA_TYPE_LEX 31
#define DATA_TYPE_CONTINUE 999
#define DATA_TYPE_ERROR -1

#define SYS_NONE    0x00                /* navigation system: none */
#define SYS_GPS     0x01                /* navigation system: GPS */
#define SYS_SBS     0x02                /* navigation system: SBAS */
#define SYS_GLO     0x04                /* navigation system: GLONASS */
#define SYS_GAL     0x08                /* navigation system: Galileo */
#define SYS_QZS     0x10                /* navigation system: QZSS */
#define SYS_CMP     0x20                /* navigation system: BeiDou */
#define SYS_IRN     0x40                /* navigation system: IRNS */
#define SYS_LEO     0x80                /* navigation system: LEO */
#define SYS_ALL     0xFF                /* navigation system: all */


#if defined(PLATFORM_IS_ARM) && PLATFORM_IS_ARM
#ifndef __ZEPHYR__

// typedef out linux types not used, this saves having to #if the RTKLib code everywhere
typedef uint32_t pthread_t;
typedef uint32_t pthread_mutex_t;
typedef int lock_t;
typedef int DIR;
struct dirent { char* d_name; };

#define pthread_mutex_init(f, f2)
#define pthread_mutex_lock(f)
#define pthread_mutex_unlock(f)

#endif
#endif // ARM

#endif // AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED
