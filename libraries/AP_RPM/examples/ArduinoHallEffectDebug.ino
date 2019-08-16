/*
INSTRUCTIONS
============
*** *** *** For testing the hall effect sensor and correct magnet polarity *** *** ***
Connect the signal pin of the sensor to pin 2 on the Arduino (an interrupt pin). Connect the 
Ground to Gnd and Vcc to 5V. Set the detect_on variable to true. Upload the code to your Arduino.
Open the serial monitor and set the Baud rate to 9600. All being well, when you bring the 
magnet close to the sensor, with the correct polarity, the serial monitor will print ‘detect’.

*** *** *** To output hall effect RPM to a graph *** *** ***
Connect the signal pin of the sensor to pin 2 on the Arduino (an interrupt pin). Connect the
Ground to Gnd and Vcc to 5V. Set the detect_on variable to false.  Set the number of magnets
on the shaft using n_magnets.  Upload the code to your Arduino. Open the serial plotter. When
you spin the shaft with magnet(s) attached the RPM will be plotted.

*** *** *** To output commutation RPM to a graph *** *** ***
Connect up the commutation RPM sensor to the motor as per the manufacturers instructions. 
Connect the signal pin of the sensor to pin 2 on the Arduino (an interrupt pin). Connect the
Ground to Gnd and Vcc to 5V. Set the detect_on variable to false.  Set the number of poles in
the motor using n_magnets.  Upload the code to your Arduino. Open the serial plotter. When
you power the shaft with magnet(s) attached the RPM will be plotted.
*/

//Conversion factors
#define MILLIS_TO_SEC 1000.0f
#define SEC_TO_MIN 60.0f

/* 
 If you want to use the hall effect sensor to print 'detect' to the serial monitor when the magnet is
 near and the correct polarity is presented, set this to true.  If you want to output an rpm value 
 set this to false.
*/
bool detect_on = true;


/* Set the number of magnets on autorotation wheel / shaft or the number of poles on the motor, 
depending on the type of sensor being used. */
int n_magnets = 1;

volatile byte n_rev;
float rpm;
float prev_time;


 void setup()
 {
   Serial.begin(9600);
   attachInterrupt(0, detect, RISING);//Initialize the intterrupt pin (Arduino digital pin 2)
   n_rev = 0;
   rpm = 0.0f;
   prev_time = 0.0f;
 }

 
 void loop()//Measure RPM
 {
   //Only do rpm calculation after 20 rotations has past.
   if (n_rev >= 20) {
     rpm = SEC_TO_MIN*MILLIS_TO_SEC/(millis() - prev_time)*n_rev/n_magnets;

     //reset prev_time to now
     prev_time = millis();

     //reset rotation counter
     n_rev = 0;

     //print rpm value to serial console
     Serial.println(rpm,DEC);
   }
 }


 //This function is called whenever a magnet/interrupt is detected by the arduino
 void detect()
 {
   n_rev++;

   if (detect_on == true) {
      Serial.println("Magnet Detected");
   }
 }