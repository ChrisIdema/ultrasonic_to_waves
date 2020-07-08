// Based on the "15 Sensors Example Sketch":  https://bitbucket.org/teckel12/arduino-new-ping/wiki/Help%20with%2015%20Sensors%20Example%20Sketch

#include <NewPing.h>
#include <Servo.h>

#define SENSOR_ACTUATOR_COUNT 4

#define SENSOR_ACTUATOR_COUNT_MAX 4
#if (SENSOR_ACTUATOR_COUNT < 1) || (SENSOR_ACTUATOR_COUNT >SENSOR_ACTUATOR_COUNT_MAX)
#error "Invalid SENSOR_ACTUATOR_COUNT not between 1 and SENSOR_ACTUATOR_COUNT_MAX"
#endif

// change these settings to change wave shape:
#define SERVO_CENTER_ANGLE 90L
#define SERVO_ANGLE_AMPLITUDE 30L
#define SERVO_PERIOD_ms 1000L

#define TRIGGER_DISTANCE_cm 90

#define SONAR_NUM     SENSOR_ACTUATOR_COUNT // Number of sensors.
#define MAX_DISTANCE_cm 150 // Maximum distance in cm to be allowed to be detected ( will return MAX_DISTANCE_cm + 1 in case distance too long and MAX_DISTANCE_cm +2 in case of timeout)
#define PING_INTERVAL_ms 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define SONAR_ERROR_TOO_FAR_cm (MAX_DISTANCE_cm+1)
#define SONAR_ERROR_TOO_FAR_us (SONAR_ERROR_TOO_FAR_cm * US_ROUNDTRIP_CM)//simplifies processing of distance
#define SONAR_ERROR_NO_ECHO_cm (MAX_DISTANCE_cm+2)

#define SERVO_COUNT SENSOR_ACTUATOR_COUNT
#define SERVO_PERIOD_DIV2_ms (SERVO_PERIOD_ms/2)
#define SERVO_PERIOD_DIV4_ms (SERVO_PERIOD_ms/4)

#define MEASUREMENT_STARTUP_DELAY_ms 0//100


//#define PRINT_SENSORS
//#define PRINT_SERVOS

static NewPing sonar[SENSOR_ACTUATOR_COUNT_MAX] = {     // Sensor object array.
  NewPing(2, 3, MAX_DISTANCE_cm), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(4, 5, MAX_DISTANCE_cm),
  NewPing(6, 7, MAX_DISTANCE_cm),
  NewPing(8, 9, MAX_DISTANCE_cm)
};

static const int servo_pins[SENSOR_ACTUATOR_COUNT_MAX] = {10,11,12,13};


//static volatile bool busy[SONAR_NUM];
static volatile bool busy;
static uint8_t current_sensor_index = 0;          // Keeps track of which sensor is active.
static volatile uint32_t received_echo_us;

static void sonar_echo_callback();

// approximate sine wave with a x squared 
static int16_t squared_sine(int32_t dt_ms){
    int32_t y;

    if(dt_ms < SERVO_PERIOD_DIV2_ms){
      y = SERVO_ANGLE_AMPLITUDE - SERVO_ANGLE_AMPLITUDE*(dt_ms-SERVO_PERIOD_DIV4_ms)*(dt_ms-SERVO_PERIOD_DIV4_ms)/(SERVO_PERIOD_DIV4_ms*SERVO_PERIOD_DIV4_ms);
    }
    else{
      dt_ms -= SERVO_PERIOD_DIV2_ms;
      y = -SERVO_ANGLE_AMPLITUDE + SERVO_ANGLE_AMPLITUDE*(dt_ms-SERVO_PERIOD_DIV4_ms)*(dt_ms-SERVO_PERIOD_DIV4_ms)/(SERVO_PERIOD_DIV4_ms*SERVO_PERIOD_DIV4_ms);
    }

    y += SERVO_CENTER_ANGLE;

    return y;
}


void setup() {
  Serial.begin(115200);
}


static void print_sensors(const uint16_t* distance_cm, bool print_line = true) { 
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(distance_cm[i]);
    Serial.print(",");
  }
  if(print_line){
    Serial.println();
  }
}

static void print_servos(const int* servo_angles, bool print_line = true) { 

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    Serial.print(servo_angles[i]);
    Serial.print(",");
  }
  if(print_line){
    Serial.println();
  }
}



void loop() {

  static bool first_time = true; // avoid using setup so less global variables can be used

  static Servo servos[SERVO_COUNT];  // create servo object to control a servo
  static bool distance_triggered[SERVO_COUNT]={false};
  static uint16_t servo_t0[SERVO_COUNT];
  int servo_angles[SERVO_COUNT];    // variable to store the servo position
  static uint16_t sonar_result_check_timer_ms;
  
  static uint16_t distance_cm[SONAR_NUM];         // Where the ping distances are stored.
  
  uint16_t t_ms = millis();

  //local setup
  if(first_time){
    first_time = false;

    for(int i=0;i<SERVO_COUNT;++i){
      servos[i].attach(servo_pins[i]);
      servo_angles[i] = SERVO_CENTER_ANGLE;
      servos[i].write(servo_angles[i]);     
    }

    for(int i=0;i<SONAR_NUM;++i){
      distance_cm[i] = SONAR_ERROR_NO_ECHO_cm;  
    }

    #if MEASUREMENT_STARTUP_DELAY_ms >0
      delay(MEASUREMENT_STARTUP_DELAY_ms);
      t_ms = millis();
    #endif

    sonar_result_check_timer_ms = t_ms + PING_INTERVAL_ms;
    busy = true;
    //busy[current_sensor_index] = true;
    sonar[0].ping_timer(sonar_echo_callback);//start first measurement
      
  }
  


  //read sensors:

  int16_t dt_ms = t_ms - sonar_result_check_timer_ms; // use dt variable to account for overflow

  bool sonar_timer_triggered = dt_ms>0;
  
  if(sonar_timer_triggered){
    sonar_result_check_timer_ms += PING_INTERVAL_ms;

    sonar[current_sensor_index].timer_stop();//stop previous sensor, prevents racing condition
    if(busy){ // measurement failed, no echo received        
      distance_cm[current_sensor_index] = SONAR_ERROR_NO_ECHO_cm;
    }
    else{//an echo pulse is received, measurement succeeded, calculate distance
      distance_cm[current_sensor_index] = received_echo_us/ US_ROUNDTRIP_CM;
    }
      
    current_sensor_index = (current_sensor_index+1) %SONAR_NUM; //next sensor
    //busy[current_sensor_index] = true;
    busy = true;
    sonar[current_sensor_index].ping_timer(sonar_echo_callback);//start measurement 
  }







  //process inputs, determine outputs:

  //
  for(int i=0;i<SERVO_COUNT;++i){
    uint16_t servo_dt = t_ms-servo_t0[i];  
    if(distance_cm[i] < TRIGGER_DISTANCE_cm){ //trigger level reached
      if(!distance_triggered[i]){ // new trigger
        distance_triggered[i] = true;
        servo_t0[i] = t_ms;//now = t0
        servo_dt = 0; 
      }
      else{//old trigger        
        //prevent overflow
        if(servo_dt >= SERVO_PERIOD_ms){ 
            servo_t0[i] += SERVO_PERIOD_ms;  
            servo_dt = t_ms-servo_t0[i]; 
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
      servo_angles[i] = squared_sine(servo_dt);
    }
    else{
      servo_angles[i] = SERVO_CENTER_ANGLE;
    }
  }
   

  
  for(int i=0;i<SERVO_COUNT;++i){
    servos[i].write(servo_angles[i]); 
  }  

  //print stuff:

  #ifdef PRINT_SENSORS 
    #ifdef PRINT_SERVOS 
        print_sensors(distance_cm,false);    
    #else
    if( sonar_timer_triggered && (current_sensor_index == 0) ){  // print every full sensor cycle    
        print_sensors(distance_cm);
    }
    #endif
  #endif

  #ifdef PRINT_SERVOS
    print_servos(servo_angles);
  #endif  
}

//this callback is called if an echo pulse is received:
static void sonar_echo_callback() { 
  
  //check timer and read value (has to be read in callback or it doesn't work! So not possible to use flag alone to signal finished measurement)
  if(sonar[current_sensor_index].check_timer()){  //distance <= MAX_DISTANCE_cm 
    received_echo_us = sonar[current_sensor_index].ping_result;
  }
  else{
    received_echo_us = SONAR_ERROR_TOO_FAR_us;
  }

  // no longer busy, allows checking for timeouts in case no echo was detected:
  //busy[current_sensor_index] = false;
  busy = false;
}
