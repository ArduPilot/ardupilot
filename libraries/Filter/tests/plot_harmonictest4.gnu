#!/usr/bin/env -S gnuplot -persist
set y2tics 0,10
set ytics nomirror
set style data linespoints
set key autotitle
set datafile separator ","
set key autotitle columnhead
set xlabel "Freq(Hz)"
set ylabel "Attenuation(dB)"
set key left bottom
plot "harmonicnotch_test4.csv" using 1:2, "harmonicnotch_test4.csv" using 1:3, "harmonicnotch_test4.csv" using 1:4, "harmonicnotch_test4.csv" using 1:5, "harmonicnotch_test4.csv" using 1:6, "harmonicnotch_test4.csv" using 1:7, "harmonicnotch_test4.csv" using 1:8

