set style data lines
plot "plot.dat" using "SIM.Pitch", "" using "DCM.Pitch", "" using "EKF.Pitch"
pause -1 "hit return to exit"
