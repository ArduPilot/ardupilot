

void delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

void mavlink_delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

uint32_t millis()
{
    return hal.scheduler->millis();
}

uint32_t micros()
{
    return hal.scheduler->micros();
}

void pinMode(uint8_t pin, uint8_t output)
{
    hal.gpio->pinMode(pin, output);
}

void digitalWriteFast(uint8_t pin, uint8_t out)
{
    hal.gpio->write(pin,out);
}

void digitalWrite(uint8_t pin, uint8_t out)
{
    hal.gpio->write(pin,out);
}

uint8_t digitalReadFast(uint8_t pin)
{
    return hal.gpio->read(pin);
}

uint8_t digitalRead(uint8_t pin)
{
    return hal.gpio->read(pin);
}

