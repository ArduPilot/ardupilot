#!/bin/bash
rm ../../manuals/html/ex/*
doxygen Doxyfile_html
rm ../../manuals/html/ex/*.md5
rm ../../manuals/html/ex/*.map

