// This code is to drive the nanotec motor updated version and working 

#include <FlexCAN.h> // Pawelsky flex can library, used with Jens 
#include <SMC66Registers.h> // is a dictionary with some of the registers of the motor 
#include <Messenger.h>  // is a library for communication with arduino, not mine, but someone else 
#include <CANsmc.h> // My library for communication with the motors 
#include <Chrono.h> // a timing library, useful for timing control loops 
#include <OneLimb.h>

//** sd card libraries, used for storing data in the sd card. 
#include <SD.h>
#include <SPI.h>
#ifndef __MK66FX1M0__
#error "Teensy 3.6 with dual CAN bus is required to run this example"
#endif
#define pi 3.141508

// Instantiate Messenger object with the message function and the default 
// separator (the space character)
Messenger myMessenger = Messenger();
OneLimb oneLimb;
FlexCAN CANbus0(1000000, 0, 1, 1);
FlexCAN CANbus1(1000000, 1, 1, 1);

#define MOTORINUSE 0x03
int kpp = 1; // Proportional torque gain.

int feedbackInterval_ms = 300;
Chrono feedbackTimer_ms(Chrono::MILLIS);

int controlInterval_us = 20000;
Chrono ControlTimer_us(Chrono::MICROS);
Chrono canDelayTimer_us(Chrono::MICROS);

uint8_t nodeid = MOTORINUSE;
uint8_t nodesid[4] = {MOTORINUSE, 0x2 ,0x3, 0x4};

CANsmc SMCCAN(&CANbus1, &Serial, nodesid);
// Generic can Message container 
CAN_message_t inMsg;
// Nanotec motor manufacturer identifiers, CIA 402 compliant 
uint16_t T_PDO1_id = 0x180; // ID
uint16_t T_PDO2_id = 0x280; // ID
uint16_t T_PDO3_id = 0x380;
uint16_t T_PDO4_id = 0x480;

uint16_t R_PDO1_id = 0x200;
uint16_t R_PDO2_id = 0x300;
uint16_t R_PDO4_id = 0x500;
uint16_t sync_m = 0x1005;
uint16_t tpdo2par = 0x1801;
uint16_t tpdo1par = 0x1800;
uint16_t tpdo3par = 0x1802;
uint16_t tpdo4par = 0x1803; 
uint16_t rpdo1par = 0x1400;
uint16_t rpdo2par = 0x1401;
uint16_t rpdo2map = 0x1601; 
uint16_t rpdo4par = 0x1403;
uint16_t tpdo1map = 0x1A00;
uint16_t tpdo2map = 0X1A01;
uint16_t tpdo3map = 0x1A02;
uint32_t sync_cobid = 0x80;

String fileBasis = "LOG";
String fileName;
const char* fileNamePointer;
uint16_t counter = 0;

int32_t motor_position = 0;
unsigned long feedinterval = 300; 

uint16_t statusmotor = 0x00;
int16_t actual_torque = 0x00;
int32_t statusT = 0x00;

unsigned long canDelay_us = 0;
unsigned long control_us = 0;

double torque = 0;
bool stopnow = true; 
bool loggingActive = false;
//////************ FUNCTIONS ************//////////////////

void OnReceived();
void ReadSerial();

void SetTorque() {

}

void configurePDOS() { 
  // this need to execute when the motors is in stop or preoperational state
  Serial.println("Set a new sync cobid");
  Serial.send_now();
  SMCCAN.setSyncCOBID(nodeid, sync_cobid, true); // variable sync_cobid is chosen. It is the synchronize message
  // set tpdo1 and tpdo2 
  uint8_t trans_type = 0x02; // <- every two sync messages it will send data
  uint16_t inhibit_t = 0x0; // wait 0 miliseconds to react
  
  Serial.println("Writing transmission type tpdo1");
  SMCCAN.setPDOTransmissionType(nodeid, tpdo1par, trans_type, true);
  Serial.println("Writing inhibit time tpdo1");
  SMCCAN.setPDOInhibitTime(nodeid, tpdo1par, inhibit_t, true);
  Serial.println("Writing transmission type tpdo2");
  // transmit process data object 2 
  SMCCAN.setPDOTransmissionType(nodeid, tpdo2par, trans_type, true);
  Serial.println("Writindg inhibit time tpdo2");
  // transmit process data object 2 parameters
  SMCCAN.setPDOInhibitTime(nodeid, tpdo2par, inhibit_t, true);
  // Remap TPDO1 and TPDO2 
  // TPDO_1 mapping: Status word 16 bits 
  //                 Modes of operation display 8 bits 
  //                 Position actual value 32 bits 
  Serial.println("Deactivate tpdo1 number of entries to 0");
  SMCCAN.setPDOnumberentries(nodeid, tpdo1map, 0, true);
  // 0x0677 is torque actual value
  Serial.println("Remap to statusword");
  // remapPDO(nodeid, map par, submap, sub, map to, bytes) 
  SMCCAN.remapPDO(nodeid, tpdo1map, 0x01, 0x00, status_index, 16, true);
  Serial.println("Remap to modes of operation display");
  // remapPDO(nodeid, map par, submap, sub, map to, bytes) 
  SMCCAN.remapPDO(nodeid, tpdo1map, 0x02, 0x00, 0x6061, 8, true);
  Serial.println("Remap to position actual value");
  // remapPDO(nodeid, map par, submap, sub, map to, bytes) 
  SMCCAN.remapPDO(nodeid, tpdo1map, 0x03, 0x00, 0x6064, 32, true);
  Serial.println("Reactivate number of entries tpd1");
  SMCCAN.setPDOnumberentries(nodeid, tpdo1map, 3, true);
  // TPDO_2 mapping: Actual torque       
  Serial.println("Deactivate tpdo2 number of entries to 0");
  SMCCAN.setPDOnumberentries(nodeid, tpdo2map, 0, true);
  Serial.println("Remap to torque actual value");
  // remapPDO(nodeid, map par, submap, sub, map to, bytes) 
  SMCCAN.remapPDO(nodeid, tpdo2map, 0x01, 0x00, 0x6077, 16, true);
  Serial.println("Reactivate number of entries tpd2");
  SMCCAN.setPDOnumberentries(nodeid, tpdo2map, 1, true);      

  // Remap transmit PDOS 1 and 2 
  trans_type = 0x01; // read more on internet 
  Serial.println("Writing transmission type rpdo1");
  SMCCAN.setPDOTransmissionType(nodeid, rpdo1par, trans_type, true);
  Serial.println("Writing transmission type rpdo2");
  SMCCAN.setPDOTransmissionType(nodeid, rpdo2par, trans_type, true);
  // RPDO1 mapping: control word 16 bits 
  //                modes of operation 8 bits 
  //                motor drive submde select 32 bits 
  // Deafult values are kept as they are 
  // RPDO2 mapping: target torque 16 bits 
  //                 
  Serial.println("Deactivate rpdo2map number of entries to 0");
  SMCCAN.setPDOnumberentries(nodeid, rpdo2map, 0, true);
  Serial.println("Remap to target torque");
  // remapPDO(nodeid, map par, submap, sub, map to, bytes) 
  SMCCAN.remapPDO(nodeid, rpdo2map, 0x01, 0x00, 0x6071, 16, true);
  Serial.println("Remap to limiting velocity on torque mode");
  // remapPDO(nodeid, map par, submap, sub, map to, bytes) 
  Serial.println("Activate rpdo2map number of entries to 1");
  SMCCAN.setPDOnumberentries(nodeid, rpdo2map, 1, true);
  Serial.println("Activate all PDOS");
  SMCCAN.activatePDO(nodeid, rpdo1par, R_PDO1_id, true);
  SMCCAN.activatePDO(nodeid, rpdo2par, R_PDO2_id, true);
  SMCCAN.activatePDO(nodeid, tpdo1par, T_PDO1_id, true);
  SMCCAN.activatePDO(nodeid, tpdo2par, T_PDO2_id, true);
}

void SendFeedback() {
    Serial.print("MotorPos: "); 
    Serial.print(motor_position);
    Serial.print(" tick, "); 
    Serial.print(oneLimb.motorPosToDeg(motor_position));
    Serial.print(" deg, ");
    Serial.print(oneLimb.motorPosToRad(motor_position));
    Serial.print(" rad, z: ");
    Serial.print(oneLimb.CalculateZ(motor_position));
    Serial.println(" m.");

    Serial.print("Torque: ");
    Serial.print(torque);
    Serial.print(", kp: ");
    Serial.print(kpp);
    Serial.println(".");

    Serial.print("Control loop us: ");
    Serial.print(control_us);
    Serial.print(", stopnow: ");
    Serial.print(stopnow);    
    Serial.print(", CAN delay us: ");
    Serial.print(canDelay_us); 
    Serial.println(".\n");
    Serial.print("Current file name: ");
    Serial.println(fileNamePointer);
     
    Serial.send_now();
}

void deactivatepdos() {
  SMCCAN.deactivatePDO(nodeid, tpdo1par, T_PDO1_id, true);
  SMCCAN.deactivatePDO(nodeid, tpdo2par, T_PDO2_id, true);
  SMCCAN.deactivatePDO(nodeid, tpdo3par, T_PDO3_id, true);
  SMCCAN.deactivatePDO(nodeid, tpdo4par, T_PDO4_id, true);
  SMCCAN.deactivatePDO(nodeid, rpdo2par, R_PDO2_id, true);
}

void setEncoderUnits() { 
  SMCCAN.writeToRegister(nodesid[0], 0x01, (uint32_t)0xFA0, 0x608F);
  SMCCAN.waitForReply(nodeid, 0x01, true);
  delay(10);
  SMCCAN.readRequestFromRegister(nodesid[0], 0x01, 0x608F);
  SMCCAN.waitForReply(nodeid, 0x01, true);
  delay(10);

  // SMCCAN.writeToRegisterS(sync_cobid);
  SMCCAN.writeToRegister(nodesid[0], 0x01, (uint32_t) (10000), 0x608F);
  SMCCAN.waitForReply(nodesid[0], 0x01, true);
  delay(10);
  SMCCAN.writeToRegister(nodesid[0], 0x02, (uint32_t) (1), 0x608F);
  SMCCAN.waitForReply(nodesid[0], 0x02, true);
  delay(10);
  SMCCAN.writeToRegister(nodesid[0], 0x01, (uint32_t) (60), 0x6091);
  SMCCAN.waitForReply(nodesid[0], 0x01, true);
  delay(10);
  SMCCAN.writeToRegister(nodesid[0], 0x02, (uint32_t) (14), 0x6091);
  SMCCAN.waitForReply(nodesid[0], 0x02, true);
  delay(10);
}

void saveApplicationData(){
	uint32_t answer = 0;
	SMCCAN.writeToRegister(nodeid, 0x03, (uint32_t)0x65766173, 0x1010);
	SMCCAN.waitForReplyuInt32(nodeid, 0x03, &answer, false);
			
	while(!answer){
		delay(500);
		SMCCAN.readRequestFromRegister(nodeid, 0x03, 0x1010);
		SMCCAN.waitForReply(nodeid,0x03, true);
		Serial.print("Answer received: ");
		Serial.println(answer);
	}
	Serial.println("Final answer received: " + answer);
	Serial.print(answer);
	delay(5000);
}

void setTorqueSettings() {
  // take the first node id in the nodesid array, this can be run in a for loop
  nodeid = nodesid[0];
  Serial.println("Activate remote node");
  // you can open the library and see what this does
  SMCCAN.start_node(nodeid);
  // this is a message object to hold incoming messages 
  CAN_message_t inMsg1;
  delay(29);
  delay(1000);
  while (CANbus1.available()) { //Clear out the canbus
    CANbus1.read(inMsg1);
    hexDumpAll(inMsg1, &Serial);
  }
  
  // PDOs can only be set on preoperational mode .- read state machine of CiA 402 motor definition on manual 
  Serial.println("setting preoperational the node");
  SMCCAN.preoperational_node(nodesid[0]);
  Serial.println("Deactivate PDOs");
  // PDOs need to be deactivated to configure them 
  deactivatepdos();

  Serial.println("Configure PDos");
  // PDOs are configured below 
  configurePDOS();
  // Start the node when pdos configured 
  Serial.println("Start the node");
  SMCCAN.start_node(nodesid[0]);

  delay(500);
  // The encoder units are set with a premultiplier, see nanotec manual
  // this premultiplier can match the reduction on the arm 
  // ragnar timing belt reduction #define REDUCTION 4.285714286 // reduction in the robot
  Serial.println("Set encoder units");
  setEncoderUnits(); 
  Serial.println("Setting to torque mode");
  // Change the control word to profile torque mode 
  // control word object 0x6060
  SMCCAN.writeToRegister(nodeid, 0x00, (int8_t)0x04, 0x6060); // (node, subindex, pos mode, index)
  SMCCAN.waitForReply(nodeid, 0x00, true);
  // read top see it was set 
  SMCCAN.readRequestFromRegister(nodeid, 0x0, 0x6060);
  SMCCAN.waitForReply(nodeid, 0x00, true);
  
  Serial.println("setting Max torque"); // maximum torqe 1000
  SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)1000, 0x6072);
  SMCCAN.waitForReply(nodeid, 0x00, true);
  
  Serial.println("reading rated current");
  SMCCAN.readRequestFromRegister(nodeid, 0x01, 0x203B); //-> here is the nominal current setting 
  SMCCAN.waitForReply(nodeid, 0x01, true);

}

void testTorque() {
  nodeid = nodesid[0];
  Serial.println("Activate remote node");
  SMCCAN.start_node(nodeid);
  CAN_message_t inMsg1;
  delay(29);
  while (CANbus1.available()) { //Clear out the canbus
    CANbus1.read(inMsg1);
    hexDumpAll(inMsg1, &Serial);
  }
  Serial.println("Setting to torque mode");
  SMCCAN.writeToRegister(nodeid, 0x00, (int8_t)0x04, 0x6060); // (node, subindex, pos mode, index)
  SMCCAN.waitForReply(nodeid, 0x00, true);
  SMCCAN.readRequestFromRegister(nodeid, 0x0, 0x6060);
  SMCCAN.waitForReply(nodeid, 0x00, true);
  
  Serial.println("setting Max torque");
  SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)1000, 0x6072);
  SMCCAN.waitForReply(nodeid, 0x00, true);
  
  Serial.println("reading rated current");
  SMCCAN.readRequestFromRegister(nodeid, 0x01, 0x203B); //-> here is the nominal current setting 
  SMCCAN.waitForReply(nodeid, 0x01, true);
  Serial.println("writing rated current");
  SMCCAN.writeToRegister(nodeid, 0x01, (uint32_t)0x41A, 0x203B); //-> here is the nominal current setting 
  SMCCAN.waitForReply(nodeid, 0x01, true);
  delay(50);
  Serial.println("reading rated current");
  SMCCAN.readRequestFromRegister(nodeid, 0x01, 0x203B); //-> here is the nominal current setting 
  SMCCAN.waitForReply(nodeid, 0x01, true);
  Serial.println("writing duration peak current");
  SMCCAN.writeToRegister(nodeid, 0x02, (uint32_t)0x64, 0x203B); //-> here is the nominal current setting 
  SMCCAN.waitForReply(nodeid, 0x02, true);
  delay(50);
  Serial.println("reading duration peak current");
  SMCCAN.readRequestFromRegister(nodeid, 0x02, 0x203B); //-> here is the nominal current setting 
  SMCCAN.waitForReply(nodeid, 0x02, true);
  

  Serial.println("Setting torque ramp to 1000%/sec");
  // SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)0x3E8, (uint16_t)0x6087); 2710
  // SMCCAN.writeToRegister(nodeid, 0x00, (uint32_t)0x4E20, (uint16_t)0x6087);
  SMCCAN.writeToRegister(nodeid, 0x00, (uint32_t)0x186A0, (uint16_t)0x6087);
  
  SMCCAN.waitForReply(nodeid, 0x00, true);
  // Serial.println("Setting max torque speed to 120000 usteps/sec");
  // SMCCAN.writeToRegister(nodeid, 0x00, (uint8_t)0x62, (uint16_t)0x2704);
  // SMCCAN.waitForReply(nodeid, 0x00, true);

  Serial.println("Setting torque to zero");
  SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)0, targettorque_index);
  SMCCAN.waitForReply(nodeid, 0x00, true);

  Serial.println("Ready to switch on");
  SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)0x06, control_index); // 0x6040 is the state machine control word
  SMCCAN.waitForReply(nodeid, 0x00, true);
  SMCCAN.readRequestFromRegister(nodeid, 0x0, status_index);
  SMCCAN.waitForReply(nodeid, 0x00, true);
  
  Serial.println("Switch on");
  SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)0x07, control_index);
  SMCCAN.waitForReply(nodeid, 0x00, true);
  SMCCAN.readRequestFromRegister(nodeid, 0x0, status_index);
  SMCCAN.waitForReply(nodeid, 0x00, true);
  
  Serial.println("Operation enable");
  SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)0xF, control_index);
  SMCCAN.waitForReply(nodeid, 0x00, true);
  delay(100);
  SMCCAN.readRequestFromRegister(nodeid, 0x0, status_index);
  SMCCAN.waitForReply(nodeid, 0x00, true);
  

  Serial.println("Setting torque to 10p");
  SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)100, targettorque_index); //  1000 is 100% 
  SMCCAN.waitForReply(nodeid, 0x00, true);    
  Serial.send_now();
  Serial.println("Max torque setting");
  SMCCAN.readRequestFromRegister(nodeid, 0x0, 0x6072);
  SMCCAN.waitForReply(nodeid, 0x00, true);
    
  // 6074 -> torque reference controller 
  // 6077 -> actual torque 

  for(int i = 0; i < 50; i++) {
    SMCCAN.readRequestFromRegister(nodeid, 0x0, 0x6074);
    SMCCAN.waitForReply(nodeid, 0x00, true);
    SMCCAN.readRequestFromRegister(nodeid, 0x0, 0x6077);
    SMCCAN.waitForReply(nodeid, 0x00, true);
    delay(100);
  }
  
  
  Serial.println("Setting torque to zero");
  SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)0, targettorque_index);
  SMCCAN.waitForReply(nodeid, 0x00, true);    
  Serial.println("Done");
}

void GetNewMotorPosition(){
    canDelayTimer_us.restart();
    SMCCAN.getInt32FromRegister(nodesid[0], 0x6064, 0x0, &motor_position); 
    canDelay_us = canDelayTimer_us.elapsed(); 
}

//////************ SETUP **************//////////////////////
void setup() {
  // Initiate Serial Communication
  Serial.begin(250000);
  // Activate the CAN bus device
  pinMode(28, OUTPUT);
  pinMode(35, OUTPUT);
  digitalWrite(28, LOW);
  digitalWrite(35, LOW);

  fileName.reserve(200);
  fileName = fileBasis + ".txt";
  fileNamePointer = fileName.c_str();

  myMessenger.attach(OnReceived);
  CANbus1.begin();

  delay(500);

  Serial.println("Starting");
  oneLimb.setZref(-0.5);
  
  setTorqueSettings();
}

///////*************** MAIN PROGRAM ******//////////////////
void loop() {
  
  ReadSerial(); // -- check the serial port, always at the end of line send a \r\n (arduino serial monitor does automatically if ticked the option) 
  
  if (feedbackTimer_ms.hasPassed(feedbackInterval_ms)) {
    feedbackTimer_ms.restart();
    SendFeedback();
  }

  if (ControlTimer_us.hasPassed(controlInterval_us)) {
    control_us = ControlTimer_us.elapsed();
    ControlTimer_us.restart();

    GetNewMotorPosition();

    // do some control 
    torque = kpp*oneLimb.getTorque(motor_position);

    if(abs(torque) > 1000)
        torque = sign(torque)*1000;
   
    if (stopnow) 
      torque = 0;

    SMCCAN.writeToRegister(nodeid, 0x00, (int16_t)torque, targettorque_index); //  1000 is 100% 
    SMCCAN.waitForReply(nodeid, 0x00, false);

    if(loggingActive){
      oneLimb.writeToFile(fileNamePointer);
    }
  }
}

//Read the serial for the Messenger
void ReadSerial() {
  while (Serial.available()) {
    myMessenger.process(Serial.read());
  }
}

//Method to check the received message
void OnReceived() {
  if (myMessenger.checkString("setenc")) {
  	setEncoderUnits(); 
    Serial.println("OK enc");
  } 
  else if (myMessenger.checkString("offset")) {
    Serial.println("OK, offset");
    int offset = myMessenger.readInt();
  	oneLimb.setMotorPositionOffset(offset);
    Serial.println(offset);
  } 
  else if (myMessenger.checkString("stiffness")) {
    Serial.println("OK, stiffness");
    float stiffness = myMessenger.readFloat();
  	oneLimb.setStiffness(stiffness);
    Serial.println(stiffness);
  } 
  else if (myMessenger.checkString("damping")) {
    Serial.println("OK, damping");
    float damping = myMessenger.readFloat();
  	oneLimb.setDamping(damping);
    Serial.println(damping);
  } 
  // else if (myMessenger.checkString("sampletime")) {
  //   Serial.println("OK, sampletime");
  //   float sampletime = myMessenger.readFloat();
  // 	oneLimb.setSampleTime(sampletime);
  //   Serial.println(sampletime);
  // } 
  else if (myMessenger.checkString("log")) {
    Serial.println("OK, log");

    while(SD.exists(fileNamePointer)){
      ++counter;
      fileName = fileBasis + counter + ".txt";
    }
    loggingActive = true;
  } 
  else if (myMessenger.checkString("setz")) {
    Serial.println("OK, z");
    int zpos = myMessenger.readInt();
  	oneLimb.setZref(zpos);
    Serial.println(zpos);
  } 
  else if (myMessenger.checkString("saveappdata")) {
    Serial.println("OK trying to save");
  	saveApplicationData(); 
  }
  else if (myMessenger.checkString("status")) {
    Serial.println("OK, status");
    SMCCAN.getInt32FromRegister(nodesid[0], (uint16_t)status_index, 0x0, &statusT); 
    char str[18];
    __itoa(statusT,str,2);
    Serial.print("0b");
    Serial.println(str);
  }
  else if (myMessenger.checkString("exitfault")) {
    Serial.println("OK, exiting");
    Serial.send_now();
    SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)B10000110, control_index); // 0x6040 is the state machine control word
    SMCCAN.waitForReply(nodeid, 0x00, true);
  
    delay(29);
    CAN_message_t inMsg1;
    while (CANbus1.available()) { //Clear out the canbus
      CANbus1.read(inMsg1);
      hexDumpAll(inMsg1, &Serial);
    }
  }
  else if (myMessenger.checkString("fid")) {
    Serial.println("OK, feed");
    Serial.send_now();
    int feedin = myMessenger.readInt();
    if (feedin< 100) feedin = 100; 
    Serial.println(feedin);
    feedinterval = feedin;
  }
  else if (myMessenger.checkString("s")) {
    Serial.println("OK, stop");
    stopnow = true; 
    loggingActive = false;
  }
  else if (myMessenger.checkString("st")) {
    Serial.println("OK, start");
    stopnow = false; 
  }
  else if (myMessenger.checkString("torque")) {
    Serial.println("OK, torque");
    torque = myMessenger.readFloat(); 
    Serial.println(torque);
  }
  else if (myMessenger.checkString("kp")) {
    Serial.println("OK, kp");
    kpp = myMessenger.readFloat(); 
    Serial.println(kpp);
  }
  else if (myMessenger.checkString("tset")) {
    Serial.println("OK, setup");
    Serial.send_now();
    setTorqueSettings(); 
  }
  // Clean up the serial port
  while (myMessenger.available())
    myMessenger.readInt();
}