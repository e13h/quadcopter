#include "radio.h"
#define PIN_YAW		        A0

void setup()
{
	Serial.begin(9600);  // Start up serial
	rfBegin(11);  // Initialize ATmega128RFA1 radio on channel 11 (can be 11-26)
	
	// Send a message to other RF boards on this channel
	rfPrint("ATmega128RFA1 Dev Board Online!\r\n");
	
}

void loop()
{
	uint8_t b[256];
	int len;
	
	if (len = rfAvailable())  // If serial comes in...
	{
		rfRead(b, len);
		b[len] = 0;
		Serial.print("Received :");
		Serial.write((char*)b);
	}
	
}
