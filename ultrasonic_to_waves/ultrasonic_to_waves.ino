// Based on the "15 Sensors Example Sketch":  https://bitbucket.org/teckel12/arduino-new-ping/wiki/Help%20with%2015%20Sensors%20Example%20Sketch

#include <NewPing.h>
#include <Servo.h>

#define SENSOR_ACTUATOR_COUNT 4

#define SONAR_NUM     SENSOR_ACTUATOR_COUNT // Number of sensors.
#define MAX_DISTANCE 150 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

#define PRINT_INTERVAL 100

#define SERVO_COUNT SENSOR_ACTUATOR_COUNT
#define SERVO_CENTER_POSITION 90L
#define SERVO_AMPLITUDE 30L
#define SERVO_PERIOD_ms 1000L
#define SERVO_PERIOD_DIV2_ms (SERVO_PERIOD_ms/2)
#define SERVO_PERIOD_DIV4_ms (SERVO_PERIOD_ms/4)
//#define SERVO_SPEED_Hz 0.5
//#define SERVO_SPEED_RAD_per_s (SERVO_SPEED_Hz*m_PI*2)
//#define SERVO_SPEED_RAD_per_ms (SERVO_SPEED_RAD_per_s *1000)

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
static bool distance_triggered[SERVO_COUNT];
static uint16_t servo_t0[SERVO_COUNT];
static int servo_positions[SERVO_COUNT] = {0};    // variable to store the servo position
static const int servo_pins[SERVO_COUNT] = {10,11,12,13};

 
static int16_t squared_sine(int32_t dt_ms){
    int32_t y;

    if(dt_ms < SERVO_PERIOD_DIV2_ms){
      y = SERVO_AMPLITUDE - SERVO_AMPLITUDE*(dt_ms-SERVO_PERIOD_DIV4_ms)*(dt_ms-SERVO_PERIOD_DIV4_ms)/(SERVO_PERIOD_DIV4_ms*SERVO_PERIOD_DIV4_ms);
    }
    else{
      dt_ms -= SERVO_PERIOD_DIV2_ms;
      y = -SERVO_AMPLITUDE + SERVO_AMPLITUDE*(dt_ms-SERVO_PERIOD_DIV4_ms)*(dt_ms-SERVO_PERIOD_DIV4_ms)/(SERVO_PERIOD_DIV4_ms*SERVO_PERIOD_DIV4_ms);
    }

    y += SERVO_CENTER_POSITION;

    return y;
}


void setup() {
  Serial.begin(115200);

  for(int i=0;i<SERVO_COUNT;++i){
    servos[i].attach(servo_pins[i]);
    servo_positions[i] = SERVO_CENTER_POSITION;
    servos[i].write(servo_positions[i]);     
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

static void print_servos() { 

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    Serial.print(servo_positions[i]);
    Serial.print(",");
  }
  Serial.println();
}


void loop() {

  uint16_t t = millis();

  //read sensors:

  int16_t dt = t - pingTimer; // use dt variable to account for overflow

  bool pingTimer_triggered = dt>0;
  
  if(pingTimer_triggered){
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
 


  if( pingTimer_triggered && (currentSensor == 0) ){  // print every cycle
    //print_sensors();
  }

//  if( pingTimer_triggered ){
//    print_sensors();
//  }

  //process inputs, determine outputs:
  
   for(int i=0;i<SERVO_COUNT;++i){
    uint16_t servo_dt = t-servo_t0[i];  
    if(distance_copy[i] < 50){ //triggered
      if(!distance_triggered[i]){ // new trigger
        distance_triggered[i] = true;
        servo_t0[i] = t;//start here
        servo_dt = 0; 
      }
      else{//old trigger        
        //prevent overflow
        if(servo_dt >= SERVO_PERIOD_ms){ 
            servo_t0[i] += SERVO_PERIOD_ms;  
            servo_dt = t-servo_t0[i]; 
        }
      }      
    }
    else{
      if(distance_triggered[i]){
        //only stop at whole period
        if(servo_dt >= SERVO_PERIOD_ms){ 
          distance_triggered[i] = false;
        }
      }
    }

    if(distance_triggered[i]){
      servo_positions[i] = squared_sine(servo_dt);
    }
    else{
      servo_positions[i] = SERVO_CENTER_POSITION;
    }


  }

   

  //write outputs:

  //print_servos();
  //delay(100);

  for(int i=0;i<SERVO_COUNT;++i){
    servos[i].write(servo_positions[i]); 
  }  
}

static void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer()){
    distance_interrupt[currentSensor] = sonar[currentSensor].ping_result;
  }
  else{//timeout: distance too long
     distance_interrupt[currentSensor] = (MAX_DISTANCE+1)*US_ROUNDTRIP_CM;
  }
  busy[currentSensor] = false;
}
