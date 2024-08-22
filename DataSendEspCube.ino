/*
 Name:		DataSendEspCube.ino
 Created:	11/8/2022 10:26:24 AM
 Author:	Imami Joel Betofe
*/

#include "MavlinkSettings.h"


MavlinkConnection Mavlink;


// the setup function runs once when you press reset or power the board

void setup() {

	Mavlink.init();
	

}

// the loop function runs over and over again until power down or reset
void loop() {
	
	Mavlink.run();
	Mavlink.processing();
}
