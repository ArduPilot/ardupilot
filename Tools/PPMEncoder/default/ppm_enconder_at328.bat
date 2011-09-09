@echo *********************************
@echo Visit DIYdrones.com!
@echo This Bat is for PPM_encoder!
@echo *********************************
@echo Press enter to do some Magic!
pause
:A
@echo Programming Arduino...

"C:\Program Files\Atmel\AVR Tools\STK500\Stk500.exe" -cUSB -dATmega328P -fD9E2 -EFD -FD9E2 -GFD -e -ifap_ppm_encoder.hex -pf -lCF -LCF

@echo ****************************************************************
@echo Jordi: CHECK FOR PROBLEMS ABOVE before closing this window!
@echo Press enter to do the Magic again!
@echo ****************************************************************
pause
Goto A
