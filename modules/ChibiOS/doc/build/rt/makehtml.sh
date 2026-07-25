#!/bin/bash
rm ../../manuals/html/rt/*
doxygen Doxyfile_html
rm ../../manuals/html/rt/*.md5
rm ../../manuals/html/rt/*.map

