# Battery State of Charge Estimator

This script implements a battery state of charge estimator based on
resting voltage and a simple cell model.

This allows the remaining battery percentage to be automatically set
based on the resting voltage when disarmed.

# Parameters

You will need to start by setting BATT_SOC_COUNT to the number of
estimators you want (how many batteries you want to do SoC estimation
for).

Then you should restart scripting or reboot and set the following
parameters per SoC estimator.

## BATT_SOCn_IDX

The IDX is the battery index, starting at 1.

## BATT_SOCn_NCELL

Set the number of cells in your battery in the NCELL parameter

## BATT_SOCn_C1

C1 is the first coefficient from your fit of your battery

## BATT_SOCn_C2

C2 is the second coefficient from your fit of your battery

## BATT_SOCn_C3

C3 is the third coefficient from your fit of your battery

## BATT_SOCn_C4

C4 is the fourth coefficient from your fit of your battery

# Usage

You need to start by working out the coefficients C1, C2, C3 and C4 for
your battery. You can do this by starting with a fully charged battery
and slowly discharging it with LOG_DISARMED set to 1. Alternatively you
can provide a CSV file with battery percentage in the first column and
voltage in the 2nd column.

Then run the resulting log or csv file through the script at
Tools/scripts/battery_fit.py. The fitting process is designed to work
reliably with LiPo and Li-ion batteries and is generally effective for
other battery chemistries as well, but users should verify that the fit
is accurate for their specific case.

You will need to tell the script the following:

 - the number of cells
 - the final percentage charge your log stops at
 - the battery index you want to fit to (1 is the first battery)

That will produce a graph and a set of coefficients like this:
 - Coefficients C1=111.5629 C2=3.6577 C3=0.2048 C4=80.0000

Use the C1, C2, C3 and C4 parameters in the parameters for this script.

The remaining battery percentage is only set when disarmed, and won't
be set till 10 seconds after you disarm from a flight.


