#!/usr/bin/env -S gnuplot -persist
set y2tics 0,10
set ytics nomirror
set style data linespoints
set key autotitle
set datafile separator ","
set key autotitle columnhead
set xlabel "Attenuation(dB)"
set ylabel "PhaseLag(deg)"
set key left bottom
set yrange [0:60]
plot "harmonicnotch_test5.csv" using 1:2, "harmonicnotch_test5.csv" using 1:3, "harmonicnotch_test5.csv" using 1:4, "harmonicnotch_test5.csv" using 1:5, "harmonicnotch_test5.csv" using 1:6

