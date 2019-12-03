#include <arduino.h>
#include <OneLimb.h>

OneLimb oneLimb;

void LED(bool on){
	digitalWrite(13,on);
}

void setup()
{
    Serial.begin(250000);
    pinMode(13,OUTPUT);
	LED(OFF);
}


void loop()
{ 
	// Wait for CHRONO timer
	LED(OFF);
	delay(2000);
	LED(ON); // LED on during calculations.
	// Restart CHRONO timer
	
	
	int Tau = oneLimb.getTorque();
	Serial.print("Torque: ");
	Serial.print(Tau);
	Serial.println();

	// Check CHRONO timer to see time spent?

}

