/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

#define SERVO_COUNT 4
#define DELAY 5//15
Servo myservos[SERVO_COUNT];  // create servo object to control a servo

int pos;    // variable to store the servo position

void setup() {
  for(int i=0;i<SERVO_COUNT;++i){
    myservos[i].attach(10+i); 
   }
  
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    for(int i=0;i<SERVO_COUNT;++i){
    myservos[i].write(pos); 
   }
    delay(DELAY);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    for(int i=0;i<SERVO_COUNT;++i){
      myservos[i].write(pos); 
     }
    delay(DELAY);                       // waits 15ms for the servo to reach the position
  }
}
