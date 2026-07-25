#!/bin/bash
find ../../demos/AVR -name "*.[ch]" -exec perl stylecheck.pl "{}" \;
