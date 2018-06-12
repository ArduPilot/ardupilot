#pragma once

typedef void (*cb_putc)(uint8_t c);

bool read_mavlink();

void On100ms();
void On20ms();

void delay_150();


void NOINLINE eeprom_read_len(byte *p, uint16_t e, uint16_t l);
NOINLINE void eeprom_write_len(byte *p, uint16_t e, uint16_t l);

void print_eeprom_string(byte n, cb_putc cb);

void NOINLINE set_data_got();

void NOINLINE delay_telem();
void serial_hex_dump(byte *p, uint16_t len);
byte get_alt_num(point p);
byte get_alt_filter(point p);
point NOINLINE readPanel(byte n);
void NOINLINE mav_message_start(byte len, byte time);
int NOINLINE normalize_angle(int a);
byte get_switch_time(byte n);
void doScreenSwitch();
uint8_t grad_to_sect(int grad);
void NOINLINE filter( float &dst, float val, const byte k);
void filter( float &dst, float val);
void setFdataVars();
NOINLINE void logo();
bool parse_osd_packet(byte *p);
void request_mavlink_rates();
void heartBeat();
byte NOINLINE radar_char();
void renew();
void setup_horiz();
uint16_t uidiff(uint16_t, uint16_t);

void unplugSlaves();
void delay_15();
void MAX_mode(byte mode);

#define GPS_MUL 10000000.0f
