
#define LED_ACTIVITY	1
#define LED_BOOTLOADER	2

/* board info forwarded from board-specific code to booloader */
struct boardinfo {
	uint32_t	board_type;
	uint32_t	board_rev;
	uint32_t	fw_size;
} __attribute__((packed));

extern struct boardinfo board_info;

int16_t cin(unsigned timeout_ms);
void cout(uint8_t *data, uint32_t len);
void cfini(void);

void flash_init();

uint32_t flash_func_read_word(uint32_t offset);
void flash_func_write_word(uint32_t offset, uint32_t v);
uint32_t flash_func_sector_size(uint32_t sector);
void flash_func_erase_sector(uint32_t sector);
uint32_t flash_func_read_otp(uint32_t idx);
uint32_t flash_func_read_sn(uint32_t idx);

uint32_t get_mcu_id(void);
uint32_t get_mcu_desc(uint32_t len, uint8_t *buf);

void led_on(unsigned led);
void led_off(unsigned led);
void led_toggle(unsigned led);

// printf to USB
void uprintf(const char *fmt, ...);

// generate a LED sequence forever
void led_pulses(uint8_t npulses);

