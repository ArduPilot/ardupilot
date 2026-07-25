#!/bin/bash
rm ../../manuals/html/nil/*
doxygen Doxyfile_html
rm ../../manuals/html/nil/*.md5
rm ../../manuals/html/nil/*.map

