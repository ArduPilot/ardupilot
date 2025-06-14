#!/usr/bin/gnuplot -persist
set y2tics 0,10
set ytics nomirror
set style data linespoints
set key autotitle
set datafile separator ","
set key autotitle columnhead
set xlabel "Freq(Hz)"
set ylabel "Attenuation(dB)"
#set ylabel2 "PhaseLag(deg)"
plot "harmonicnotch_test3.csv" using 1:2 axis x1y1, "harmonicnotch_test3.csv" using 1:3 axis x1y2, "harmonicnotch_test3.csv" using 1:4 axis x1y1, "harmonicnotch_test3.csv" using 1:5 axis x1y2

