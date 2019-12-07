#include <arduino.h>
#include <OneLimb.h>
#include <Chrono.h>

OneLimb oneLimb;
Chrono chrono(Chrono::MICROS);

void LED(bool on){
	digitalWrite(13,on);
}

void setup()
{
    Serial.begin(250000);
    pinMode(13,OUTPUT);
	LED(OFF);
	
	oneLimb.setZref(-0.5);
}

void loop()
{ 
	Serial.print("Squareroot of 9: ");
	Serial.println(sqrt(9.0));
	
	double AbsMotorPosition = -1200;
	int Tau = 0;
	unsigned long endTime = 0;
	unsigned long startTime = 0;
	// Wait for CHRONO timer
	LED(OFF);
	delay(2000);
	LED(ON); // LED on during calculations.
	// Restart CHRONO timer
	
	startTime = millis();
	for (size_t i = 0; i < 1000; ++i)
	{
		Tau += oneLimb.getTorque(AbsMotorPosition);
		AbsMotorPosition += 100;
	}

	endTime = millis();
	LED(OFF);

	
	Serial.print("Time: ");
	Serial.print(endTime/1000.0);
	Serial.println();
	Serial.println(startTime);
	Serial.println(endTime);
	Serial.println(Tau);

	// Check CHRONO timer to see time spent?

}

