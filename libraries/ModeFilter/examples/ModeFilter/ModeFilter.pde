/*
	Example of APM_RC library.
	Code by Jordi MuÒoz and Jose Julio. DIYDrones.com

	Print Input values and send Output to the servos
	(Works with last PPM_encoder firmware)
*/

#include <ModeFilter.h> // ArduPilot Mega RC Library

int rangevalue[] = {31000, 31000, 50, 55, 60, 55, 10, 0, 31000};

ModeFilter mfilter;
byte i = 0;

void setup()
{
	//Open up a serial connection
	Serial.begin(115200);
	//Wait for the serial connection
	delay(500);
}

//Main loop where the action takes place
void loop()
{
	while(i < 9){
		printArray(mfilter._samples, 6);
		int modE = mfilter.get_filtered_with_sample(rangevalue[i]);
		i++;

		Serial.print("The mode/median is: ");
		Serial.print(modE);
		Serial.println();
	}
	delay(100000);
}

/*-----------Functions------------*/
//Function to print the arrays.
void printArray(int *a, int n)
{
	for (int i = 0; i < n; i++)
	{
		Serial.print(a[i], DEC);
		Serial.print(' ');
	}

	Serial.println();
}

