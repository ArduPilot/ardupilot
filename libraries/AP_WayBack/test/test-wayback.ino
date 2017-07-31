/*
*/

#define SERIAL_USB 1

#define LED_PIN PB1

#include "Arduino.h"

#define HAL_BOARD_SITL     3
#define HAL_BOARD_SMACCM   4  // unused
#define HAL_BOARD_PX4      5
#define HAL_BOARD_LINUX    7
#define HAL_BOARD_VRBRAIN  8
#define HAL_BOARD_QURT     9
#define HAL_BOARD_REVOMINI 10

#define CONFIG_HAL_BOARD HAL_BOARD_SITL

#include "AP_WayBack.h"
#include "AP_Math.h"

AP_WayBack track;


#define SERIAL_BUFSIZE 128
char buffer[SERIAL_BUFSIZE];


void getSerialLine(char *cp ){      // получение строки
    byte cnt=0; // строка не длиннее 256 байт

    while(true){
        if(!Serial.available()){
            delay(1); // ничего хорошего мы тут сделать не можем
            continue;
        }
    
        char c=Serial.read();

        if(c==0x0d || (cnt && c==0x0a)){
            cp[cnt]=0;
            return;
        }
        if(c==0x0a) continue; // skip unneeded LF
    
        cp[cnt]=c;
        if(cnt<SERIAL_BUFSIZE) cnt++;
    
    }
    
}


void blinks(uint8_t n){
    digitalWrite(LED_PIN, LOW);
    delay(500);
    for(int i=0; i<n;i++){
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
        delay(200);
    }    
    delay(500);
}

void setup() {
  // initialize the digital pin as an output.
  pinMode(LED_PIN, OUTPUT);
  Serial.begin();  // USB does not require BAUD

  // wait for serial monitor to be connected.
//  while (!(Serial.isConnected() && (Serial.getDTR() || Serial.getRTS())))
/*  while (!Serial.isConnected())
  {
    digitalWrite(LED_PIN,!digitalRead(LED_PIN));// Turn the LED from off to on, or on to off
    delay(50);         // fast blink
    digitalWrite(LED_PIN,!digitalRead(LED_PIN));// Turn the LED from off to on, or on to off
    delay(200);         // fast blink
    digitalWrite(LED_PIN,!digitalRead(LED_PIN));// Turn the LED from off to on, or on to off
    delay(100);         // fast blink
  }
*/
/*
    while(!Serial.available()){
        digitalWrite(LED_PIN,!digitalRead(LED_PIN));// Turn the LED from off to on, or on to off
        delay(50);         // fast blink
        digitalWrite(LED_PIN,!digitalRead(LED_PIN));// Turn the LED from off to on, or on to off
        delay(200);         // fast blink
        digitalWrite(LED_PIN,!digitalRead(LED_PIN));// Turn the LED from off to on, or on to off
        delay(100);         // fast blink
    };
    
    while(Serial.available()){
        volatile char c=Serial.read();
    }
*/

    for(int i=0;i<12;i++){
        digitalWrite(LED_PIN,!digitalRead(LED_PIN));// Turn the LED from off to on, or on to off
        delay(50);         // fast blink
        digitalWrite(LED_PIN,!digitalRead(LED_PIN));// Turn the LED from off to on, or on to off
        delay(200);         // fast blink
        digitalWrite(LED_PIN,!digitalRead(LED_PIN));// Turn the LED from off to on, or on to off
        delay(100);         // fast blink
    }

    digitalWrite(LED_PIN, HIGH);

    track.init();
    track.start();

    Serial.println("send pairs 'lat,lon'");
    Serial.println("send G to get point");

    Serial.println("send S to show track point");
    delay(300);

    digitalWrite(LED_PIN, LOW);
}


void loop() {

  digitalWrite(LED_PIN, LOW);

    getSerialLine(buffer);

    digitalWrite(LED_PIN, HIGH);

    
    if(buffer[0] == 'G' && buffer[1]==0){ // return by track
        // get point
        float x,y;
        
        track.stop();
        
        while(track.get_point(x,y)){
            Serial.print(x);
            Serial.print(",");
            Serial.println(y);
        }
        Serial.println(".");
    } else if(buffer[0] == 'R' && buffer[1]==0){ // Reset
        track.stop();
        track.start();
    } else if(buffer[0] == 'S' && buffer[1]==0){ // show current state
        float x,y;
        uint16_t i=0;
        while(true){
            uint16_t k=i;
            if(!track.show_track(i, x, y )) break;
            Serial.print(k);
            Serial.print(",");
            Serial.print(x);
            Serial.print(",");
            Serial.println(y);
        }
        Serial.println(".");

    } else {
        float x,y;
        char *bp=buffer;
        
        while(*bp) {
            if(*bp++ == ',') break;
        }
        x=atof(buffer);
        y=atof(bp);

        uint32_t t=micros();

        track.add_point(x,y);
        t=micros() - t;
    
        Serial.print("# time=");
        Serial.println(t);
    }



}
