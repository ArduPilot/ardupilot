#pragma once

/*
  strtod() and strtof() replacements that don't allocate memory. newlib's
  versions assert, and so abort, when an internal allocation fails.
 */
double ap_strtod(const char *str, char **endptr);
float ap_strtof(const char *str, char **endptr);
