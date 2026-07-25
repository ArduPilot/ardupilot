#!/bin/bash
rm ../../manuals/html/hal/*
doxygen Doxyfile_html
rm ../../manuals/html/hal/*.md5
rm ../../manuals/html/hal/*.map

