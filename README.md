# ultrasonic_to_waves

This ultrasonic_to_waves repo contains the project ultrasonic_to_waves and several test projects.
The goal is to create waves in an oil bath at different corners after aproaching the bath.
This is achieved using 4 ultrasonic distance sensors and 4 servos.
When the distance is within the defined range of a sensor its servo will move according to an approximate sine wave.
Each sensor forms a pair with a servo.


# ultrasonic sensors
* uses NewPing library
* this library uses asynchronous measurement as opposed to busy waiting
* it only uses 1 timer, but it can also only do one measurement at the same time
* to prevent interference between sensors simultanious measurements are not recommended anyway
* the algorithm is based on the "15 Sensors Example Sketch":  
	https://bitbucket.org/teckel12/arduino-new-ping/wiki/Help%20with%2015%20Sensors%20Example%20Sketch
* The library doesn't implement timeouts, so you need to implement your own timeouts
* reading sensor values and check for distance overflow can only be done in the callback function

# servos
* uses standard servo library
* the sine wave approximation uses a second power approximation
* the wave always starts and stops at the middle position, only complete periods after being triggered


# misc
* The main loop has no delays (except optionally at startup) and simply polls events and updates setpoints continuously
* Trigger distance, servo amplitude, servo speed and others can all be changed by the user with defines
* hardware: uses extra pull-up resistors for sensors to reduce interference
