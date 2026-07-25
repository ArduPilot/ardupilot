#!/bin/bash
rm ../../manuals/html/full_rm/*
doxygen Doxyfile_html
rm ../../manuals/html/full_rm/*.md5
rm ../../manuals/html/full_rm/*.map

