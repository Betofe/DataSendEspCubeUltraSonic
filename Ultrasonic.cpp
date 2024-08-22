#include "Ultrasonic.h"

void Ultrasonic::init() {

	Echo = 12;
	trig = 13;
	

	//  Serial.begin(9600);
	pinMode(Echo, INPUT);
	pinMode(trig, OUTPUT);
}



void Ultrasonic::processing() {

	digitalWrite(trig, LOW);
	delayMicroseconds(20);
	digitalWrite(trig, HIGH);
	delayMicroseconds(100);
	digitalWrite(trig, LOW);

	pulse = pulseIn(Echo, HIGH);
	Ceni = pulse / 29.387 / 2;
	//Serial.println(Ceni);
	
	//delay(200);

}
