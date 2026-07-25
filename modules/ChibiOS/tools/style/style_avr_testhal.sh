#!/bin/bash
find ../../testhal/AVR -name "*.[ch]" -exec perl stylecheck.pl "{}" \;
