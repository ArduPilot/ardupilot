#!/bin/bash
find ../../os/nil -name "*.[ch]" -exec perl stylecheck.pl "{}" \;
find ../../os/oslib -name "*.[ch]" -exec perl stylecheck.pl "{}" \;
find ../../os/license -name "*.[ch]" -exec perl stylecheck.pl "{}" \;

