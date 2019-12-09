#include <OneLimb.h>
#include <Arduino.h>
#include <Chrono.h>

//** sd card libraries, used for storing data in the sd card. 
#include <SD.h>
#include <SPI.h>
#ifndef __MK66FX1M0__
#error "Teensy 3.6 with dual CAN bus is required to run this example"
#endif
#define pi 3.141508


Chrono wait_feedback;
Chrono control_time(Chrono::MICROS);
Chrono samplingvar(Chrono::MICROS);
uint8_t nodeid = 0x02;
String input_string = "";
uint8_t nodesid[4] = {0x2, 0x2 ,0x3, 0x4};

double torqueRef = 0;
int32_t motor_position = 0;
uint32_t anotherPos = 0;

unsigned long feedinterval = 100; 
int32_t statusT = 0;

unsigned long passed_reading = 0;
unsigned long samplingvarelapsed = 0;
unsigned long control_us = 0;

OneLimb oneLimb;

//////**************************//////////////////////
void setup() {
    // Initiate Serial Communication
    Serial.begin(250000);
    delay(5000);
    Serial.println("Starting");
    // Activate the CAN bus device
    pinMode(28, OUTPUT);
    pinMode(35, OUTPUT);
    digitalWrite(28, LOW);
    digitalWrite(35, LOW);

    delay(500);

    LED(OFF);
    oneLimb.setZref(-0.5);
    wait_feedback.restart();
}
///////*************** MAIN PROGRAM ******//////////////////
void loop() {
    
    if (wait_feedback.hasPassed(feedinterval)) {
        wait_feedback.restart();
        Serial.print("loop | Motor position ticks: "); 
        Serial.print(motor_position);
        Serial.print(", "); 
        Serial.print(oneLimb.motorPosToDeg(motor_position));
        Serial.print(" degrees, ");
        Serial.print(oneLimb.motorPosToRad(motor_position));
        Serial.print(" radians, CAN delay:");
        Serial.print(passed_reading ); 
        Serial.print(", control_us: ");
        Serial.print(control_us);
        Serial.print(", Torque ref: ");
        Serial.print(torqueRef);
        Serial.print(", elapsed_comm: ");
        Serial.println(samplingvarelapsed);
        
        Serial.send_now();
    }

	if (control_time.hasPassed(CONTROLDELAY)) {
        
		control_us = control_time.elapsed();
		control_time.restart(); 

		// do some control 
		torqueRef = oneLimb.getTorque(motor_position);
      
    }
  
}