/*
  conversion of JSON numbers to double and float
 */

#pragma once

namespace AP_JSON_Number {

/*
  convert a nul terminated number that has already been checked against
  the JSON grammar. The result is exactly what strtod / strtof return:
  common short numbers take an exact fast path that avoids them, which
  matters on microcontrollers where they are slow
 */
double to_double(const char *s);
float to_float(const char *s);

}
