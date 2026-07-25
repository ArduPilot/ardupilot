#!/bin/bash
find ../../os/hal/ports/AVR/ -name "*.[ch]" -exec perl stylecheck.pl "{}" \;
