#!/usr/bin/gnuplot -persist
set y2tics 0,10
set ytics nomirror
set style data linespoints
set key autotitle
set datafile separator ","
set key autotitle columnhead
set xlabel "Freq(Hz)"
set ylabel "Attenuation"
#set ylabel2 "PhaseLag(deg)"
plot "harmonicnotch_test.csv" using 1:2 axis x1y1, "harmonicnotch_test.csv" using 1:3 axis x1y2
