// MavlinkSettings.h
#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include "Libraries/fastmavlink/c_library/common/common.h"
#include "Libraries/fastmavlink/c_library/common/common_msg_entries.h"

// GLOBAL
#define SYS_ID 1
#define COMP_ID MAV_COMP_ID_AUTOPILOT1
#define STATUS_COMP_ID MAV_COMP_ID_PERIPHERAL



class MavlinkConnection {
private:
	// Required variables
	fmav_status_t status;
	uint8_t rx_buf[296];
	uint8_t tx_buf[296];
	fmav_message_t msg;
	
	
	float Ceni;
	bool debug_flag;
	int Echo;
	int trig;

	int pulse;
	/*         METHODS             */

	uint16_t serialAvailable();

	void readSerial(uint8_t* c);

	uint8_t availSerialBuff(uint16_t counter);

	void writeToSerial(uint8_t* t, uint16_t len);

	uint8_t checkMessage(fmav_message_t* msg);

	void handleMessage(fmav_message_t* msg);

	uint8_t groundControlDebug();

	void decodeMessage();

public:

	void init();

	void run();
	
	void processing();
	
};