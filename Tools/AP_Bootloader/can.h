void can_start();
void can_update();
void can_set_node_id(uint8_t node_id);
bool can_check_update(void);
extern "C" {
void can_vprintf(const char *fmt, va_list ap);
void can_printf(const char *fmt, ...);
void can_printf_severity(uint8_t severity, const char *fmt, ...);
};
