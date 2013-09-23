
static void delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

static void mavlink_delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

static uint32_t millis()
{
    return hal.scheduler->millis();
}

static uint32_t micros()
{
    return hal.scheduler->micros();
}

static void pinMode(uint8_t pin, uint8_t output)
{
    hal.gpio->pinMode(pin, output);
}

static void digitalWrite(uint8_t pin, uint8_t out)
{
    hal.gpio->write(pin,out);
}

static uint8_t digitalRead(uint8_t pin)
{
    return hal.gpio->read(pin);
}

