/* Adapted from Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;
int pos = 0;

void setup() {
	myservo.attach(6, 470, 2620);
}

void loop() {
	for (pos = 0; pos <= 180; pos ++) { // goes from 0 degrees to 180 degrees
		// in steps of 1 degree
		myservo.write(pos);               // tell servo to go to position in variable 'pos'
		delay(15);                        // waits 15ms for the servo to reach the position
	}
	for (pos = 180; pos >= 0; pos --) { // goes from 180 degrees to 0 degrees
		myservo.write(pos);               // tell servo to go to position in variable 'pos'
		delay(15);                        // waits 15ms for the servo to reach the position
	}
}