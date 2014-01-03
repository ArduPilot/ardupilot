set style data lines
plot "plot.dat" using "ATT.Roll", "" using "AHRS.Roll", "" using "EKF.Roll"
pause -1 "hit return to exit"
