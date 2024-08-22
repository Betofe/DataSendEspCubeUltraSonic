#include "MavlinkSettings.h"
#include "Ultrasonic.h"

/*          DO NOT FORGET TO SET SERIAL2 PROTOCOL TO MAVLINK 2 AND BAUDRATE TO 57600	   */

HardwareSerial CUBE_SERIAL(1);



void MavlinkConnection::init() {

	Serial.begin(115200);
	CUBE_SERIAL.begin(57600, SERIAL_8N1, 18, 19);
	fmav_init();
	debug_flag = false;
	Echo = 12;
	trig = 13;


	//  Serial.begin(9600);
	pinMode(Echo, INPUT);
	pinMode(trig, OUTPUT);
}

void MavlinkConnection::processing() {

	digitalWrite(trig, LOW);
	delayMicroseconds(20);
	digitalWrite(trig, HIGH);
	delayMicroseconds(100);
	digitalWrite(trig, LOW);

	pulse = pulseIn(Echo, HIGH);
	Ceni = pulse / 29.387 / 2;
	Serial.println(Ceni);

	delay(200);
	

}

uint16_t MavlinkConnection::serialAvailable() {
	uint16_t available = CUBE_SERIAL.available();
	return (available > 0) ? available : 0;
}

void MavlinkConnection::readSerial(uint8_t* c) {
	*c = CUBE_SERIAL.read();
}

uint8_t MavlinkConnection::availSerialBuff(uint16_t counter) {
	return (CUBE_SERIAL.availableForWrite() >= counter) ? 1 : 0;
}

void MavlinkConnection::writeToSerial(uint8_t* buf, uint16_t len) {
	for (uint16_t i = 0; i < len; i++) {
		CUBE_SERIAL.write(buf[i]);
	}
}

uint8_t MavlinkConnection::checkMessage(fmav_message_t* msg) {
	// Message either has no target_sysid or is broadcast, accept
	if (msg->target_sysid == 0) return 1;

	// Message has a target_sysid but it is not ours, reject
	if (msg->target_sysid != SYS_ID) return 0;

	// Message either has no target_compid or is broadcast, accept
	if (msg->target_compid == 0) return 1;

	// Message has a target_compid and it is ours, accept
	if (msg->target_compid == SYS_ID) return 1;

	// Message has a target_compid but it is not ours, so reject
	return 0;
}

void MavlinkConnection::handleMessage(fmav_message_t* msg) {
	switch (msg->msgid) {

	case FASTMAVLINK_MSG_ID_HEARTBEAT: {
		fmav_heartbeat_t data;
		fmav_msg_heartbeat_decode(&data, msg);
		if (data.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA && data.type == MAV_TYPE_QUADROTOR) {
			if (Ceni <= 5) {
				debug_flag = true;
			}
		}
	}break;

	default:
		break;
	}
}

uint8_t MavlinkConnection::groundControlDebug() {
	fmav_statustext_t data;
	data.severity = MAV_SEVERITY_INFO;
	memset(&data.text, 0, FASTMAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
	memcpy(&data.text, "FEEDJOEL", 8);
	data.id = 0;
	data.chunk_seq = 0;
	uint16_t count = fmav_msg_statustext_encode_to_frame_buf(
		tx_buf,
		SYS_ID,
		STATUS_COMP_ID,
		&data,
		&status);
	if (availSerialBuff(count)) {
		writeToSerial(tx_buf, count);
		return 1;
	}
	return 0;
}

void MavlinkConnection::decodeMessage() {
	uint16_t available = serialAvailable();
	for (uint16_t i = 0; i < available; i++) {
		uint8_t c;
		readSerial(&c);
		fmav_result_t result;
		uint8_t res = fmav_parse_to_frame_buf(&result, rx_buf, &status, c);

		if (res == FASTMAVLINK_PARSE_RESULT_OK) {

			res = fmav_check_frame_buf(&result, rx_buf);

			if (res == FASTMAVLINK_PARSE_RESULT_OK) {
				fmav_frame_buf_to_msg(&msg, &result, rx_buf);
				if (checkMessage(&msg))
				{
					handleMessage(&msg);
				}
			}
		}
	}
}

void MavlinkConnection::run() {
	//requestParameters();
	decodeMessage();
	if (debug_flag) {
		if (groundControlDebug()) {
			debug_flag = false;
		}
	}

	
}

