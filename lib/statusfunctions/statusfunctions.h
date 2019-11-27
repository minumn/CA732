#ifndef statusfunctions_h
#define statusfunctions_h
#include <Arduino.h>

// uint8 for mode of the robot 
// 0b 0 0 0 0 0 0 0 0 - Passive Mode 
// 0b 0 0 0 0 0 0 0 1 - Position Mode 
// 0b 0 0 0 0 0 0 1 1 - Position Mode Impedance Control 
// 0b 0 0 0 0 0 1 0 0 - Velocity Mode 

#define VELOCITY_MODE  0b00000100
#define POSITION_MODE  0b00000001 
#define PASSIVE_MODE   0b00000000
#define IMPEDANCE_MODE 0b00000011

class statusrobotword {
	public:
	    statusrobotword(); 
        bool isPassive();
        bool isPosition();
        bool isVelocity();
        bool isImpedance();
        uint8_t status(); 
	private: 
        uint8_t _statusword; 
};

#endif