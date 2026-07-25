#!/bin/bash
find ../../os/ex -name "*.[ch]" -exec perl stylecheck.pl "{}" \;

