#define SERVO_COUNT 2 // number of motors and sensors

#define MAX_DISTANCE 100 // Maximum distance (in cm) to ping.

#include <NewPing.h>




// Sensor object array.
NewPing sonar[SERVO_COUNT] = {  
  NewPing(7, 6, MAX_DISTANCE), 
  NewPing(5, 4, MAX_DISTANCE), 
//  NewPing(8, 9, MAX_DISTANCE)
};

#include <Servo.h>

Servo servos[SERVO_COUNT];  // create servo object to control a servo
// twelve servo objects can be created on most boards

int servo_positions[SERVO_COUNT] = {0};    // variable to store the servo position

const int servo_pins[SERVO_COUNT] = {9,10};

void setup() {
  //Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  for(int i=0;i<SERVO_COUNT;++i){
    servos[i].attach(servo_pins[i]);
  }
}

void loop() { 
//  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
//    delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
//    //Serial.print(i);
//    //Serial.print("=");
//    Serial.print(sonar[i].ping_cm());
//    //Serial.print("cm ");
//  }
//  Serial.println();


  int distances[SERVO_COUNT];

  //get distances
  for(int i=0;i<SERVO_COUNT;++i){
    distances[i] = sonar[i].ping_cm() ;
  }

  for(int i=0;i<SERVO_COUNT;++i){
    if(distances[i] > 0){ // something is detected
    if(distances[i] <50){
        for (servo_positions[i] = 0; servo_positions[i] <= 90; servo_positions[i] += 1) { // goes from 0 degrees to 90 degrees
          // in steps of 1 degree
          servos[i].write(servo_positions[i]);              // tell servo to go to position in variable 'pos'
          delay(5);                       // waits 15ms for the servo to reach the position
        }
        for (servo_positions[i] = 90; servo_positions[i] >= 0; servo_positions[i] -= 1) { // goes from 90 degrees to 0 degrees
          servos[i].write(servo_positions[i]);              // tell servo to go to position in variable 'pos'
          delay(5);                       // waits 15ms for the servo to reach the position
        }
      }
   }
  }




}
