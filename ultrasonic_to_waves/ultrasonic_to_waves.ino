// Based on the "15 Sensors Example Sketch":  https://bitbucket.org/teckel12/arduino-new-ping/wiki/Help%20with%2015%20Sensors%20Example%20Sketch

#include <NewPing.h>
#include <Servo.h>

#define SENSOR_ACTUATOR_COUNT 4

#define SONAR_NUM     SENSOR_ACTUATOR_COUNT // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

#define SERVO_COUNT SENSOR_ACTUATOR_COUNT
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 90
#define SERVO_SPEED 0


static NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(2, 3, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(4, 5, MAX_DISTANCE),
  NewPing(6, 7, MAX_DISTANCE),
  NewPing(8, 9, MAX_DISTANCE)
};

static uint16_t pingTimer;
static volatile uint32_t distance_interrupt[SONAR_NUM];         // Where the ping distances are stored.
static volatile bool busy[SONAR_NUM];
static volatile uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
static volatile uint16_t distance_copy[SONAR_NUM];         // Where the ping distances are stored.

static Servo servos[SERVO_COUNT];  // create servo object to control a servo
static int servo_positions[SERVO_COUNT] = {0};    // variable to store the servo position
static const int servo_pins[SERVO_COUNT] = {0,1,2,3};


void setup() {
  Serial.begin(115200);

  for(int i=0;i<SERVO_COUNT;++i){
    //servos[i].attach(servo_pins[i]);
  }
  
  uint16_t t = millis();
  pingTimer = t + 75;
  busy[currentSensor] = true;
  sonar[currentSensor].ping_timer(echoCheck); 
}


static void print_sensors() { 

  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(distance_copy[i]);
    Serial.print(",");
  }
  Serial.println();
}

void loop() {

  uint16_t t = millis();

  //read sensors:

  int16_t dt = t - pingTimer; // use dt variable to account for overflow
  
  if(dt>0){
    pingTimer += PING_INTERVAL;

    sonar[currentSensor].timer_stop();//stop previous sensor
    if(busy[currentSensor]){ // measurement failed
        distance_copy[currentSensor] = MAX_DISTANCE+2;//set invalid value
    }
    else{
      distance_copy[currentSensor] = distance_interrupt[currentSensor]/ US_ROUNDTRIP_CM;
    }
      
    currentSensor = (currentSensor+1) %SONAR_NUM; //next sensor
    busy[currentSensor] = true;
    sonar[currentSensor].ping_timer(echoCheck); 
  }
 
  //process inputs, determine outputs

  if( (dt>0) && (currentSensor == 0) ){  // print every cycle
    print_sensors();
  }

//  if( dt>0 ){
//    print_sensors();
//  }
  

  //write outputs:

  for(int i=0;i<SERVO_COUNT;++i){
    //servos[i].write(servo_positions[i]); 
  }  
}

static void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer()){
    distance_interrupt[currentSensor] = sonar[currentSensor].ping_result;
  }
  else{//timeout: distance too long
     distance_interrupt[currentSensor] = MAX_DISTANCE+1;
  }
  busy[currentSensor] = false;
}
