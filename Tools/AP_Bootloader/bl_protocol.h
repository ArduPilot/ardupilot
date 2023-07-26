void jump_to_app(void);
void bootloader(unsigned timeout);

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

#define arraySize(a) (sizeof((a))/sizeof(((a)[0])))

enum led_state {LED_BLINK, LED_ON, LED_OFF, LED_BAD_FW};
void led_set(enum led_state state);
