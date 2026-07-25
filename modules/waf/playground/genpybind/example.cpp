#include "example.h"

constexpr int Example::not_exposed;

int Example::calculate(int some_argument) const { return _value + some_argument; }

int Example::getSomething() const { return _value; }

void Example::setSomething(int value) { _value = value; }
