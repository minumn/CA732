#include "statusfunctions.h"

statusrobotword::statusrobotword() {
    _statusword = PASSIVE_MODE;
}

bool statusrobotword::isPassive() {
    _statusword = PASSIVE_MODE;
    return true; 
}
bool statusrobotword::isPosition() {
    _statusword = POSITION_MODE;
    return true; 
}
bool statusrobotword::isVelocity() {
    _statusword = VELOCITY_MODE;
    return true; 
}
bool statusrobotword::isImpedance() {
    _statusword = IMPEDANCE_MODE;
    return true; 
}

uint8_t statusrobotword::status() {
    return _statusword; 
}