// Ultrasonic.h

#pragma once

#include<Arduino.h>

class Ultrasonic {
private:
	int Echo;
	int trig;
	
	int pulse;
	
public:
	void init();
	void processing();
	float Ceni;
	
	


};
