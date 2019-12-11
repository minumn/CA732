// This code is to drive the nanotec motor updated version and working 

#include <FlexCAN.h> // Pawelsky flex can library, used with Jens 
#include <SMC66Registers.h> // is a dictionary with some of the registers of the motor 
#include <Messenger.h>  // is a library for communication with arduino, not mine, but someone else 
#include <CANsmc.h> // My library for communication with the motors 
#include <math.h> // this is arduino standard math library
#include <Chrono.h> // a timing library, useful for timing control loops 
#include <statusfunctions.h> // I believe an unuseful library, not finished. 
#include <MatrixMath.h>
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
// file for sd card 
File myFile;
const int chipSelect = BUILTIN_SDCARD;
bool file_open = false; 
// CANbus objects
// FlexCAN CANbus1(1000000, 1, 1, 1);
// FlexCAN CANbus0(1000000, 0, 1, 1);
FlexCAN CANbus0(1000000, 0, 1, 1);
FlexCAN CANbus1(1000000, 1, 1, 1);

Chrono wait_feedback;
Chrono control_time(Chrono::MICROS);
Chrono samplingvar(Chrono::MICROS);
uint8_t nodeid = 0x02;
String input_string = "";
bool string_complete = false;
bool ddither = false; 
bool savepoint = false; 
bool savetofile = false; 
bool goal_evaluate = false;
bool goal_reached = false;  
bool trajectory_done = false; 
bool test_sdo = true; 
bool test_pdo = false; 
bool startsine = false;
uint8_t nodesid[4] = {0x2, 0x2 ,0x3, 0x4};
uint8_t nnodes = 1; 
CANsmc SMCCAN(&CANbus1, &Serial, nodesid);
// Generic can Message container 
CAN_message_t inMsg;
// Nanotec motor manufacturer identifiers, CIA 402 compliant 
uint16_t T_PDO1_id = 0x180; // ID
uint16_t T_PDO2_id = 0x280; // ID
uint16_t T_PDO3_id = 0x380;
uint16_t T_PDO4_id = 0x480;

double torqueRef = 0;

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
uint32_t pdosid[2] = {T_PDO2_id, T_PDO3_id};
uint32_t pdo_cobsid[20]; 
int32_t motor_position = 0;
uint32_t anotherPos = 0;
uint8_t npdos = 2; 
unsigned long feedinterval = 100; 
int32_t statusT = 0;
bool stopnow = false;
// holding value variables 
uint16_t statusmotor = 0x00;
int32_t positionmotor = 0x00;
int16_t target_torque[3] = {0, 0, 0}; // -1000 a 1000 
int16_t actual_torque = 0x00;
int16_t actual_torques[3] = {0, 0, 0};
uint16_t targettorqueindex = 0x6071; 
int32_t offsets[3] = {32768, 29363, 34304};
int32_t of1 = 0; 
int32_t of2 = 0; 
int32_t of3 = 0; 
int32_t pause_time = 10; 
unsigned long passed_reading = 0;
unsigned long samplingvarelapsed = 0;
unsigned long control_us = 0;

OneLimb oneLimb;
//////************ FUNCTIONS ************//////////////////

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
  while (CANbus1.available()) { // Clear out the canbus
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

  Serial.println("writing rated current");
  // this is a second premultiplier for torque 
  // torque is set as a percentage of this rated currrent
  // from motor specs example 3 Amps 4 Nm 
  //                          if set to 1 Amps then max torque 4*1/3 
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
  // this set how fast the torque is changed 
  Serial.println("Setting torque ramp to 1000%/sec");
  SMCCAN.writeToRegister(nodeid, 0x00, (uint32_t)0x2710, (uint16_t)0x6087);
  SMCCAN.waitForReply(nodeid, 0x00, true);
  // no torque 
  Serial.println("Setting torque to zero");
  SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)0, targettorqueindex);
  SMCCAN.waitForReply(nodeid, 0x00, true);
  // state machine of the motor steps to operate 

  
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
  SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)0, targettorqueindex);
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
  SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)100, targettorqueindex); //  1000 is 100% 
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
  SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)0, targettorqueindex);
  SMCCAN.waitForReply(nodeid, 0x00, true);    
  Serial.println("Done");
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

//Method to check the received message
void OnReceived() {
  if (myMessenger.checkString("setenc")) {
  	setEncoderUnits(); 
    Serial.println("OK enc");
  } 
  else if (myMessenger.checkString("saveappdata")) {
    Serial.println("OK trying to save");
  	saveApplicationData(); 
  }
  else if (myMessenger.checkString("st")) {
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
  else if (myMessenger.checkString("s")) {
    Serial.println("OK, stop");
    stopnow = true; 
  }
  else if (myMessenger.checkString("pdoff")) {
    Serial.println("deactivated pdos");
    Serial.println("Ready to switch on");
    SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)0x06, control_index); // 0x6040 is the state machine control word
    SMCCAN.waitForReply(nodeid, 0x00, true);
    SMCCAN.readRequestFromRegister(nodeid, 0x0, status_index);
    SMCCAN.waitForReply(nodeid, 0x00, true);
    SMCCAN.preoperational_node(nodesid[0]);

    deactivatepdos();
  
    SMCCAN.start_node(nodesid[0]);

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
    
  }
  else if (myMessenger.checkString("pdon")) {
    Serial.println("Activated pdos");
    Serial.println("Ready to switch on");
    SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)0x06, control_index); // 0x6040 is the state machine control word
    SMCCAN.waitForReply(nodeid, 0x00, true);
    SMCCAN.readRequestFromRegister(nodeid, 0x0, status_index);
    SMCCAN.waitForReply(nodeid, 0x00, true);
    SMCCAN.preoperational_node(nodesid[0]);
    delay(29);
    CAN_message_t inMsg1;
    while (CANbus1.available()) { //Clear out the canbus
      CANbus1.read(inMsg1);
      hexDumpAll(inMsg1, &Serial);
    }
    configurePDOS();
  
    SMCCAN.start_node(nodesid[0]);

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
    
  }
  else if (myMessenger.checkString("status")) {
    Serial.println("OK, status");
    SMCCAN.getInt32FromRegister(nodesid[0], status_index, 0x0, &statusT); 
    char str[18];
    __itoa(statusT,str,2);
    Serial.print("0b");
    Serial.println(str);
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
//Read the serial for the Messenger
void ReadSerial() {
  while (Serial.available()) {
    myMessenger.process(Serial.read());
  }
}

//////**************************//////////////////////
void setup() {
  // Initiate Serial Communication
  Serial.begin(250000);
  delay(5000);
  Serial.println("ok 1");
  // Activate the CAN bus device
  pinMode(28, OUTPUT);
  pinMode(35, OUTPUT);
  digitalWrite(28, LOW);
  digitalWrite(35, LOW);
  myMessenger.attach(OnReceived);
  // reserve 200 bytes for the input_string:
  input_string.reserve(200);
  CANbus1.begin();
  delay(500);
  Serial.println("Starting");
  Serial.println("ok 2");
  
  setTorqueSettings();
  Serial.println("ok 3");
  
  SMCCAN.writeToRegisterS(sync_cobid);
  LED(OFF);
  oneLimb.setZref(-0.6);
  Serial.println("ok 4");
  samplingvar.hasPassed(1);
  Serial.println("ok 4.1");
  wait_feedback.restart();
  Serial.println("ok 4.2");
}
///////*************** MAIN PROGRAM ******//////////////////
void loop() {  
  // Serial.println("ok 5");
  ReadSerial(); // -- check the serial port, always at the end of line send a \r\n (arduino serial monitor does automatically if ticked the option) 
  // Serial.println("ok 6");
  if (string_complete) {
    input_string = "";
    string_complete = false;
  }
  // SMCCAN.readRequestFromRegister(nodeid, (int8_t)0x00, 0x6041); // (node, subindex, pos mode, index)
  // SMCCAN.waitForReply(nodeid, 0x00, true);
  // Serial.println("ok 7");
  if (wait_feedback.hasPassed(feedinterval)) {
    // Serial.send_now();
    // Serial.println("ok 7.1");
    // Serial.send_now();
    wait_feedback.restart();
    // Serial.println("ok 7.2");
    Serial.print("loop | Motor position ticks: "); 
    Serial.print(motor_position);
    Serial.print(", "); 
    Serial.print(motor_position*0.0357 - 40.76);
    Serial.print(" degrees, ");
    Serial.print(motor_position*0.0006-0.7114);
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
  // Serial.println("ok 8");

	if (control_time.hasPassed(CONTROLDELAY)) {
		// Serial.println("ok 9");

		control_us = control_time.elapsed();
		// Serial.println("ok 11");
		control_time.restart(); 
		// Serial.println("ok 12");
		samplingvar.restart();
		// Serial.println("ok 13");
		SMCCAN.getInt32FromRegister(nodesid[0], 0x6064, 0x0, &motor_position); 
		// Serial.println("ok 14");
		// SMCCAN.getUInt32FromRegister(nodesid[0], 0x205A, 0x0, &anotherPos);
		// Serial.println("ok 15");
		passed_reading = samplingvar.elapsed(); 
		// Serial.println("ok 16");
		// do some control 
		torqueRef = oneLimb.getTorque(motor_position);
		// Serial.print("TorqueRef: ");
		// Serial.println(torqueRef);
		// Serial.print("MotorPosition: ");
		// Serial.println(motor_position);
		
		// SMCCAN.writeToRegister(nodeid, 0x00, (int16_t) torqueRef, targettorqueindex); //  1000 is 100% 
		// Serial.println("ok 18");
		// SMCCAN.waitForReply(nodeid, 0x00, false);
		// Serial.println("ok 19");
      
  }
  // SMCCAN.readRequestFromRegister(nodeid, (int8_t)0x00, 0x6064); // (node, subindex, pos mode, index)
  // SMCCAN.waitForReply(nodeid, 0x00, true);
}