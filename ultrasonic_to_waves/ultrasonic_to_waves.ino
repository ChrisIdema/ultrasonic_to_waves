// Based on the "15 Sensors Example Sketch":  https://bitbucket.org/teckel12/arduino-new-ping/wiki/Help%20with%2015%20Sensors%20Example%20Sketch

#include <NewPing.h>
#include <Servo.h>

#define SENSOR_ACTUATOR_COUNT 4

// change these settings to change wave shape:
#define SERVO_CENTER_ANGLE 90L
#define SERVO_ANGLE_AMPLITUDE 30L
#define SERVO_PERIOD_ms 1000L

#define TRIGGER_DISTANCE_cm 50

#define SONAR_NUM     SENSOR_ACTUATOR_COUNT // Number of sensors.
#define MAX_DISTANCE_cm 150 // Maximum distance in cm to be allowed to be detected ( will return MAX_DISTANCE_cm + 1 in case distance too long and MAX_DISTANCE_cm +2 in case of timeout)
#define PING_INTERVAL_ms 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define SERVO_COUNT SENSOR_ACTUATOR_COUNT
#define SERVO_PERIOD_DIV2_ms (SERVO_PERIOD_ms/2)
#define SERVO_PERIOD_DIV4_ms (SERVO_PERIOD_ms/4)

#define MEASUREMENT_STARTUP_DELAY_ms 75


//#define PRINT_SENSORS
//#define PRINT_SERVOS

static NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(2, 3, MAX_DISTANCE_cm), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(4, 5, MAX_DISTANCE_cm),
  NewPing(6, 7, MAX_DISTANCE_cm),
  NewPing(8, 9, MAX_DISTANCE_cm)
};

static uint16_t ping_timer_ms;
static volatile uint32_t received_echo_us[SONAR_NUM];         // Where the ping distances are stored.
static volatile bool busy[SONAR_NUM];
static volatile uint8_t current_sensor_index = 0;          // Keeps track of which sensor is active.
static volatile uint16_t distance_cm[SONAR_NUM];         // Where the ping distances are stored.

static Servo servos[SERVO_COUNT];  // create servo object to control a servo
static bool distance_triggered[SERVO_COUNT];
static uint16_t servo_t0[SERVO_COUNT];
static int servo_angles[SERVO_COUNT] = {0};    // variable to store the servo position
static const int servo_pins[SERVO_COUNT] = {10,11,12,13};

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

  //initialize servos:
  for(int i=0;i<SERVO_COUNT;++i){
    servos[i].attach(servo_pins[i]);
    servo_angles[i] = SERVO_CENTER_ANGLE;
    servos[i].write(servo_angles[i]);     
  }
  
  uint16_t t = millis();
  ping_timer_ms = t + MEASUREMENT_STARTUP_DELAY_ms;//wait MEASUREMENT_STARTUP_DELAY_ms before starting first measurement
  busy[current_sensor_index] = true;
  sonar[current_sensor_index].ping_timer(sonar_echo_callback);//setup callback
}


static void print_sensors() { 

  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(distance_cm[i]);
    Serial.print(",");
  }
  Serial.println();
}

static void print_servos() { 

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    Serial.print(servo_angles[i]);
    Serial.print(",");
  }
  Serial.println();
}


void loop() {

  uint16_t t_ms = millis();

  //read sensors:

  int16_t dt_ms = t_ms - ping_timer_ms; // use dt variable to account for overflow

  bool ping_timer_ms_triggered = dt_ms>0;
  
  if(ping_timer_ms_triggered){
    ping_timer_ms += PING_INTERVAL_ms;

    sonar[current_sensor_index].timer_stop();//stop previous sensor, prevents racing condition
    if(busy[current_sensor_index]){ // measurement failed, no echo received 
        distance_cm[current_sensor_index] = MAX_DISTANCE_cm+2;//set invalid value
    }
    else{//measurement succeeded, calculate distance
      distance_cm[current_sensor_index] = received_echo_us[current_sensor_index]/ US_ROUNDTRIP_CM;
    }
      
    current_sensor_index = (current_sensor_index+1) %SONAR_NUM; //next sensor
    busy[current_sensor_index] = true;
    sonar[current_sensor_index].ping_timer(sonar_echo_callback); 
  }
 


  if( ping_timer_ms_triggered && (current_sensor_index == 0) ){  // print every full sensor cycle
    #ifdef PRINT_SENSORS
      print_sensors();
    #endif
  }

//  if( ping_timer_ms_triggered ){
//    print_sensors();
//  }

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
   

  //write outputs:
  #ifdef PRINT_SERVOS
  print_servos();
  #endif
  
  for(int i=0;i<SERVO_COUNT;++i){
    servos[i].write(servo_angles[i]); 
  }  
}

//this callback is called if an echo pulse is received:
static void sonar_echo_callback() { 
  if (sonar[current_sensor_index].check_timer()){ // distance <= MAX_DISTANCE_cm 
    received_echo_us[current_sensor_index] = sonar[current_sensor_index].ping_result;
  }
  else{// distance > MAX_DISTANCE_cm
     received_echo_us[current_sensor_index] = (MAX_DISTANCE_cm+1)*US_ROUNDTRIP_CM;
  }
  busy[current_sensor_index] = false;  // no longer busy, allows checking for timeouts in case no echo was detected
}
