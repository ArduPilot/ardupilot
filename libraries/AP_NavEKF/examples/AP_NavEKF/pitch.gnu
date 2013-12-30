set style data lines
plot "plot.dat" using "ATT.Pitch", "" using "AHRS.Pitch", "" using "EKF.Pitch"
pause -1 "hit return to exit"
