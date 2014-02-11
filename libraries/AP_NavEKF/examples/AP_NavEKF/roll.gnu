set style data lines
plot 'plot.dat' using "SIM.Roll", "" using "DCM.Roll", "" using "EKF.Roll"
pause -1 "hit return to exit"
