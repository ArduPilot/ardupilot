# Turbine Telemetry 

Allows logging turbine values from ECU S.Port telemetry adapters, ECU status is displayed as GCS message (viewable through Yaapu or MP)
Currently implemented adapters are Vspeak and Xicoy 
For Xicoy adapter set ECU_TYPE to 8

## Parameters

ECU_TYPE 

 1 = Jakadofsky - Vspeak adapter
 2 = EvoJet - Vspeak adapter
 3 = PBS - Vspeak adapter
 4 = Hornet - Vspeak adapter
 5 = JetCat - Vspeak adapter
 6 = KingTech - Vspeak adapter
 7 = AMT - Vspeak adapter
 8 = Xicoy - Xicoy adapter
 9 = JetCentral - Vspeak adapter
 10 = Kolibri NG - Vspeak adapter
 11 = Swiwin - Vspeak adapter
 12 = Linton - Vspeak adapter

## How To Use

Connect S.Port signal to serialX TX pin, GND to serialX GND pin

SERIALx_BAUD:57

SERIALx_PROTOCOL:28

SERIALx_OPTIONS: ”invertTX”/”HalfDuplex”