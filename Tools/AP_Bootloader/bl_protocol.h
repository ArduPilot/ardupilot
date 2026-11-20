// keep back 32 bytes at the front of flash. This is long enough to allow for aligned
// access on STM32H7
#define RESERVE_LEAD_WORDS 8

void jump_to_app(void);
void bootloader(unsigned timeout);
// stop the bootloader timing out and jumping to the application, e.g.
// because a firmware upload has started on an interface other than serial
void bootloader_hold(void);
bool bootloader_held(void);

/*****************************************************************************
 * Chip/board functions.
 */

/* LEDs */

#ifdef BOOT_DELAY_ADDRESS
# define BOOT_DELAY_SIGNATURE1	0x92c2ecff
# define BOOT_DELAY_SIGNATURE2	0xc5057d5d
# define BOOT_DELAY_MAX		30
#endif

#define MAX_DES_LENGTH 20
#define MAX_VERSION_LENGTH 32

#define arraySize(a) (sizeof((a))/sizeof(((a)[0])))

enum led_state {LED_BLINK, LED_ON, LED_OFF, LED_BAD_FW};
void led_set(enum led_state state);
