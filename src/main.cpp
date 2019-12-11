#include <arduino.h>
#include <SMC66Registers.h>
#include <CANsmc.h>
#include <Messenger.h>
#include <OneLimb.h>
#include <Chrono.h>
#include <FlexCAN.h>

Messenger myMessenger;
FlexCAN CANbus0(1000000, 0, 1, 1);
FlexCAN CANbus1(1000000, 1, 1, 1);


int kpp = 1; // Proportional torque gain.

int feedbackInterval_ms = 300;
Chrono feedbackTimer(Chrono::MILLIS);

int controlInterval_us = 20000;
Chrono ControlTimer(Chrono::MICROS);

//////************ SETUP **************//////////////////////
void setup() {
  // Initiate Serial Communication
  Serial.begin(250000);
  // Activate the CAN bus device
  pinMode(28, OUTPUT);
  pinMode(35, OUTPUT);
  digitalWrite(28, LOW);
  digitalWrite(35, LOW);

  myMessenger.attach(OnReceived);
  CANbus1.begin();

  delay(500);

  Serial.println("Starting");
  oneLimb.setZref(-0.5);
  
  setTorqueSettings();
  
  SMCCAN.writeToRegisterS(sync_cobid);
}

///////*************** MAIN PROGRAM ******//////////////////
void loop() {
  
  ReadSerial(); // -- check the serial port, always at the end of line send a \r\n (arduino serial monitor does automatically if ticked the option) 
  
  if (feedbackTimer.hasPassed(feedbackInterval_ms)) {
    feedbackTimer.restart();
    SendFeedback();
  }

  if (ControlTimer.hasPassed(controlInterval_us)) {
    control_us = ControlTimer.elapsed();
    ControlTimer.restart();

    GetNewMotorPosition();

    // do some control 
    torque = -kpp*oneLimb.getTorque(motor_position);

    if(abs(torque) > 1000)
        torque = sign(torque)*1000;
   
    if (stopnow) 
      torque = 0;

    SMCCAN.writeToRegister(nodeid, 0x00, (int16_t)torque, targettorqueindex); //  1000 is 100% 
    SMCCAN.waitForReply(nodeid, 0x00, false);
  }
}