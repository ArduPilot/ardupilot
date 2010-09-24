
#include <FastSerial.h>
#include <AP_Common.h>

#include <avr/pgmspace.h>

FastSerialPort0(Serial);

int
menu_test(uint8_t argc, const Menu::arg *argv)
{
	int	i;

	Serial.printf("This is a test with %d arguments\n", argc);
	for (i = 1; i < argc; i++) {
		Serial.printf("%d: int %ld  float ", i, argv[i].i);
		Serial.println(argv[i].f, 6);    // gross
	}
}

int
menu_auto(uint8_t argc, const Menu::arg *argv)
{
	Serial.println("auto text");
}

const struct Menu::command top_menu_commands[] PROGMEM = {
        {"*",                           menu_auto},
	{"test",			menu_test},
};

MENU(top, "menu", top_menu_commands);

void
setup(void)
{
	Serial.begin(38400);
	top.run();
}

void
loop(void)
{
}

