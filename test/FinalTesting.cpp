// This code is to drive the nanotec motor updated version and working 

#include <FlexCAN.h> // Pawelsky flex can library, used with Jens 
#include <SMC66Registers.h> // is a dictionary with some of the registers of the motor 
#include <Messenger.h>  // is a library for communication with arduino, not mine, but someone else 
#include <CANsmc.h> // My library for communication with the motors 
#include <math.h> // this is arduino standard math library
#include <Chrono.h> // a timing library, useful for timing control loops 
#include <statusfunctions.h> // I believe an unuseful library, not finished. 
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
// file for sd card 
File myFile;
const int chipSelect = BUILTIN_SDCARD;
bool file_open = false; 

// CANbus objects
// FlexCAN CANbus1(1000000, 1, 1, 1);
// FlexCAN CANbus0(1000000, 0, 1, 1);
FlexCAN CANbus0(1000000, 0, 1, 1);
FlexCAN CANbus1(1000000, 1, 1, 1);

#define MOTORINUSE 0x03

float guard1[25] = {0.0};

Chrono sampling_time(Chrono::MICROS);
Chrono feedback_samplingtime(Chrono::MICROS);
Chrono cycle_time(Chrono::MICROS);
Chrono feedback_time(Chrono::MICROS); 
Chrono wait_feedback;
Chrono control_time(Chrono::MICROS);
Chrono samplingvar(Chrono::MICROS);
float guard2[25] = {0.0};

#define Ndata 2000
float data[4][Ndata] = {-11.2};
long int counter = 0;
bool save = 0;
bool logvar = 0;


Chrono show_control_var;
Chrono trajectory_time(Chrono::MILLIS);
Chrono testpoint_time;
Chrono calibration_stream;
Chrono req_init; 
Chrono outerloop_time;
Chrono goal_timeout; 
Chrono statemachine_time;
Chrono test_control(Chrono::MICROS);

Chrono chronosine;
uint8_t nodeid = MOTORINUSE;
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
uint8_t nodesid[4] = {MOTORINUSE, 0x2 ,0x3, 0x4};
uint8_t nnodes = 1; 
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
uint16_t control_index = 0x6040;
uint32_t pdosid[2] = {T_PDO2_id, T_PDO3_id};
uint32_t pdo_cobsid[20]; 
int32_t motor_position = 0;
uint8_t npdos = 2; 
unsigned long feedinterval = 300; 
// holding value variables 
uint16_t statusmotor = 0x00;
int32_t positionmotor = 0x00;
int16_t target_torque[3] = {0, 0, 0}; // -1000 a 1000 

int16_t actual_torque = 0x00;
int32_t statusT = 0x00;
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
float offset_matlab[3] = {-0.054265003675517, -0.034080726136466, -0.020180550820493};
// kinematic params length of proximal and distal arms 
float dl[2] = {245 / 1000.0, 173.4 / 1000.0};
// wus = ub wb up wp sp geometric parameters 
float wus[5] = {164.2907962 / 1000.0, 82.1453963 / 1000.0, 20.2073 / 1000.0, 
                10.1036 / 1000.0, 35 / 1000.0};
// float dl[4] = {8.5/100.0, 39.5/1000.0, 18.2/100.0, 17.5/100.0};
float Xk[3];
float Xkw[3];
//float pi = 3.1415F;
float freq = 1.0;
float dXk[3];
float motpos = 0; 
float lmotpos = 0;
float motposd = 0;
float motspeed = 0; 
float motspeedd = 0; 
float error = 0; 
float lerror = 0;
float derror = 0; 
float ierror = 0; 
float kpp = 1.2; 
float kdd = 0.05; 
float kii = 0.005;
float ssamplitude = 0;
float spd = 0;
 
float ooutput = 0;
float fooutput = 0;

bool integralerror = false; 
// Dynamic params 
 // dynparam = [Ib1 Ib2 Ib3 
  //             mL1 mL2 mL3 
  //             ml1 ml2 ml3 
  //                mp mc 
  //             fc1 fv1 
  //             fc2 fv2 
  //             fc3 fv3 
  //          f_offset_1 f_offset_2 f_offset_3             ];
float dynparam[11] = {0.000056466155057, 0.000056369726396, 0.000163723372153,
                      0.019199283969404, 0.019199587573384, 0.019195295654971,
                      0.060719724380676, 0.060719782994896, 0.060719592965882,
                      0.017999774143226, 0.011279874033914};

float frictionparam[9] = {0.018309086074965, 0.041355542465608,
                          0.032511023295248, 0.023982125470682,
                          0.023130491252835, 0.025055927106413,
       0.031649182859597, 0.033864566841839, 0.036878304461692};
// control dynamics 
float mass_matrix[3][3] = {{0.0, 0.0, 0.0},{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}};
float coriolis[3][3] = {{0.0, 0.0, 0.0},{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}};
float grav[3] = {0.0, 0.0, 0.0};

// communication variables 
bool activated = true; 
int control_period = 0; 
float Xc[3] = {0.0, 0.0, -0.3};
float X_home[3] = {0.0, 0.0, -0.25};
float qc[3];
// worskpace variable 
float Xw[3] = {0.0, 0.0, 0.0};
float dXwd[3] = {0.0, 0.0, 0.0};
float dXd[3] = {0.0, 0.0, 0.0};
float ddXwd[3] = {0.0, 0.0, 0.0};
float ddXd[3] = {0.0, 0.0, 0.0};
// internal control variables 
bool passive_mode = true; 
bool toff = false; 
float motorradpos = 0; 
// float reduction[3] = {2.5560155518, 2.5560155518, 2.5560155518};
// float reduction[3] = {2.4970, 2.4970, 2.4970};
// float reduction[3] = {2.5, 2.5, 2.5};
float reduction[3] = {2.5070, 2.513, 2.491};

float q[3] = {0.0, 0.0, 0.0};
float scq[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
int32_t q_raw[3] = {0, 0, 0};
float _lq[3] = {0.0, 0.0, 0.0};
float _llq[3] = {0.0, 0.0, 0.0};
float _lllq[3] = {0.0, 0.0, 0.0};
float _llllq[3] = {0.0, 0.0, 0.0};
float dq[3] = {0.0, 0.0, 0.0};
float _lastpos = 0; 
float _lasterr[3] = {0.0, 0.0, 0.0}; 
float _llasterr[3] = {0.0, 0.0, 0.0}; 
// float kp[3] = {0.25, 0.2, 0.2}; 
// float kp[3] = {0.3 , 0.27 , 0.27 };
float kp_d = 7.5;//6.0;
float kd_d = 0.05;
float ki_d = 8.0;
int control_type; 
float kp_bd = 1.8; //4.0; // 3.5
float kd_bd = 0.05; //0.08; 
float ki_bd = 5.0; //5 

float kp_ld = 1.0; 
float kd_ld = 0.2; 
float ki_ld = 1.0; 

float kp[3] = {kp_d, kp_d , kp_d}; 
float kd[3] = {kd_d, kd_d, kd_d}; //{0.023 , 0.02, 0.0223 }; 
float ki[3] = {ki_d, ki_d, ki_d}; //{0.2, 0.2, 0.2}; 
float velocity_input[3] = {0.0, 0.0, 0.0};
//float torque[3] = {0.0, 0.0, 0.0}; 
double torque = 0;
float torque_gravity[3] = {0.0, 0.0, 0.0};
float torque_coriolis[3] = {0.0, 0.0, 0.0};
float torque_gc[3] = {0.0, 0.0, 0.0};
float torque_friction[3] = {0.0, 0.0, 0.0};

float u_input[3] = {0.0, 0.0, 0.0};
float qd[3] = {0.8, 0.8, 0.8}; 
float erri[3] = {0.0, 0.0, 0.0}; 
float erriv[3] = {0.0, 0.0, 0.0}; 
float telap; 
float kpv[3] = {0.0, 0.0, 0.0};
float kdv[3] = {0.0, 0.0, 0.0};
float kiv[3] = {0.0, 0.0, 0.0};
float kpvv[3] = {0.0, 0.0, 0.0};
float kivv[3] = {0.0, 0.0, 0.0};
float err[3] = {0.0, 0.0, 0.0};
float errv[3] = {0.0, 0.0, 0.0};
float derr[3] = {0.0, 0.0, 0.0};
float errimax = 0.31 / (ki[2]); //limit for integral error 
float Jacobian[3][3];
float iJacobian[3][3];
float iJacobianT[3][3];
float JacobianT[3][3];

float Xei[3]; 

//*/* debugging stuff 
int16_t t3torque = 0; 
bool on_calibration = false; 
bool calstream = false; 
// state machine robot 
int8_t current_state = 0; 
int8_t number_states = 2;
bool _newrollmachine = false; 
bool state_started = false;
bool stopnow = true; 

// trajectory parameters 
bool _newtraj = false; 
int traj_size = 200;
int traj_size_max = 5000;
float traj[3][5000];
float dtraj[3][5000];
float ddtraj[3][5000];

int current = 0; 
int contador = 0;
int traj_rate = 20; 

// variables for identification
float qd_id[3];
float xd_id[3];

// Impedance parameters 
long randNumber; 
float K_x, K_y, K_z; // stiffness impedance axis parameters
float Kd_x, Kd_y, Kd_z; // damping impedance axis parameters 
float Ki_x, Ki_y, Ki_z; // position control integral parameters 

// calibration parameters
int32_t known_pos[3] = {32768 + 1805 + 100 + 500 - 433 + 2359 - 1137 - 1355 + 304 - 1715, 
                        29363 + 5363 + 100  + 500 + 200 - 337 + 2441 - 1027 - 1423 + 169, 
                        34304 + 717 + 500 + 100 + 600 - 1120 + 300 + 1476 - 1146 - 1150 + 557 - 1996};

int32_t known_pos_lower_limit[3] = {-12198, -15859, -15910};

// test points for gravity vector 
int n_test = 20; 
int in_test = 0; 

bool bool_identification = false;

// dither variables 
int argmnt = 0; 
float arg_dither = 0; 
float dther = 0.0;
float signdther = 0; 
float f_dither = 180; 
//////************ FUNCTIONS ************//////////////////

void SetTorque() {
  // Create control word and position
  uint16_t cntrlword = 0x0F; 
  if (passive_mode) { 
    for (int i = 0; i < nnodes; i++) {
      target_torque[i] = 0; 
    }
  }
  for (int i = 0; i < nnodes; i++) {
    nodeid = nodesid[i];
    // Serial.println(target_torque[i]);
    SMCCAN.writeToPDOCustom2(nodeid, R_PDO4_id, cntrlword, target_torque[i]);
  }
}

void ReadVariables() {
  SMCCAN.writeToRegisterS(sync_cobid); // sync message to get data 
  int32_t ofsets[3] = {of1, of2, of3};
  feedback_samplingtime.restart();
  uint16_t statuses[3]; 
  float countsToRad =  0.00012271846; // 2 * pi / 51 200 
  float pre_FK_x[3];
  bool nan_flag = false; 
  
  // Next function is used because the replies will come in different times
  // the function will wait for alll the pdos and fill in the variables  
  SMCCAN.waitForPDOSpalletizer(pdo_cobsid, &statuses, &q_raw, &actual_torques, 
                               nnodes*npdos);  
                               
  for( int i = 0; i < nnodes; i++) {
    nodeid = nodesid[i];
    _llllq[i] = _lllq[i]; 
    _lllq[i] = _llq[i];
    _llq[i] = _lq[i];
    _lq[i] = q[i];
    q[i] = - ((float)(q_raw[i] - offsets[i]) /*- ofsets[i]*/) * countsToRad;  
    q[i] /= reduction[i]; // mechanical gear reduction 

    q[i] += offset_matlab[i]; // something computed from matlab 
    // central difference algorithm dq = q(t+1) - q(t-1) / 2 * delta_t
    dq[i] = (q[i] - _lllq[i]) / ((float)(3.0 * samplingvar.elapsed()) / 1000000.0);
  }  
  
  samplingvar.restart(); 
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
    Serial.print(oneLimb.z);
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
    Serial.print(passed_reading); 
    Serial.println(".\n");
     
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

  Serial.println("writing rated current");
  // this is a second premultiplier for torque 
  // torque is set as a percentage of this rated currrent
  // from motor specs example 3 Amps 4 Nm 
  //                          if set to 1 Amps then max torque 4*1/3 
  SMCCAN.writeToRegister(nodeid, 0x01, (uint32_t)2800, 0x203B); //-> here is the nominal current setting 
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

void OnSetup() {
  
  passive_mode = false; 
  // start the impedance parameters 
  K_x = 170; 
  K_y = 170; 
  K_z = 170; 
  Kd_x = 15; 
  Kd_y = 15; 
  Kd_z = 15; 
  Ki_x = 0;
  Ki_y = 0;
  Ki_z = 0;
  for (int i = 0; i < 1; i++){ 
    erri[i] = 0.0;
    Xei[i] = 0.0; 
  } 
  delay(2000); // a small delay of 3000 ms to let the CANBUS initialize
  CAN_message_t inMsg;
  for (int i=0; i<2; i++) {
    delay(1000);
    while (CANbus1.available()) { //Clear out the canbus
      CANbus1.read(inMsg);
      hexDumpAll(inMsg, &Serial);
    }
  }
  
  // RESET THE NODE (MOTOR) sending the nodeid declared inside 
  // nodesid[0], believe in mine is 0x42 (hexadecimal value)
  Serial.print("Reset node ");
  Serial.println(nodesid[0],HEX);
  SMCCAN.reset_node(nodesid[0]); 
  delay(1500);
  CAN_message_t inMsg1;
  while (CANbus1.available()) { //Clear out the canbus
    CANbus1.read(inMsg1);
    hexDumpAll(inMsg1, &Serial);
  }
  // THIS SETUP IS ONLY DONE FOR THE FIRST MOTOR 
  for (int i = 0; i < 1; i++) {  
    nodeid = nodesid[i]; // i = 0
    Serial.println("Set a new sync cobid");
    Serial.send_now();
    SMCCAN.setSyncCOBID(nodeid, sync_cobid, true); // variable sync_cobid is chosen. It is the synchronize message
    // only TPDO1 and RPDO1 are active  
    uint8_t trans_type = 0x02; // <- every two sync messages it will send data
    Serial.println("Writing transmission type tpdo2");
    Serial.send_now();
    // transmit process data object 2 
    SMCCAN.setPDOTransmissionType(nodeid, tpdo2par, trans_type, true);
    Serial.send_now();
    uint16_t inhibit_t = 0x0; // wait 0 miliseconds to react
    Serial.println("Writindg inhibit time tpdo2");
    Serial.send_now();
    // transmit process data object 2 parameters
    SMCCAN.setPDOInhibitTime(nodeid, tpdo2par, inhibit_t, true);
    
    Serial.println("Deactivating tpdo1");
    Serial.send_now();
    // to configure a transmit data object you need to deactivate first 
    SMCCAN.deactivatePDO(nodeid, tpdo1par, T_PDO1_id, true);
    Serial.println("Writing transmission type tpdo1");
    SMCCAN.setPDOTransmissionType(nodeid, tpdo1par, trans_type, true);
    Serial.println("Writing inhibit time tpdo1");
    SMCCAN.setPDOInhibitTime(nodeid, tpdo1par, inhibit_t, true);
    Serial.println("Deactivating Rpdo1");
    SMCCAN.deactivatePDO(nodeid, rpdo1par, R_PDO1_id, true);
    trans_type = 0x01; // read more on internet 
    Serial.println("Writing transmission type rpdo1");
    SMCCAN.setPDOTransmissionType(nodeid, rpdo1par, trans_type, true);
    Serial.println("Deactivating Rpdo2");
    SMCCAN.deactivatePDO(nodeid, rpdo2par, R_PDO2_id, true);
    Serial.println("Writing transmission type rpdo2");
    SMCCAN.setPDOTransmissionType(nodeid, rpdo2par, trans_type, true);
    Serial.println("Deactivating Rpdo4");
    SMCCAN.deactivatePDO(nodeid, rpdo4par, R_PDO4_id, true);
    Serial.println("Writing transmission type rpdo2");
    SMCCAN.setPDOTransmissionType(nodeid, rpdo4par, trans_type, true);    
    
    trans_type = 0x02; // <- every two sync messages it will send data
    Serial.println("Writing transmission type tpdo3");
    SMCCAN.setPDOTransmissionType(nodeid, tpdo3par, trans_type, true);
    inhibit_t = 0x0;
    Serial.println("Writindg inhibit time tpdo3");
    SMCCAN.setPDOInhibitTime(nodeid, tpdo2par, inhibit_t, true);
    // Next is remap the data of the transmis PDO 
    Serial.println("Deactivate tpdo3 number of entries to 0");
    SMCCAN.setPDOnumberentries(nodeid, tpdo3map, 0, true);
    // 0x0677 is torque actual value
    Serial.println("Remap to torque actual value");
    // remapPDO(nodeid, map par, submap, sub, map to, bytes) 
    SMCCAN.remapPDO(nodeid, tpdo3map, 0x01, 0x00, 0x6077, 16, true);
    Serial.println("Reactivate number of entries tpd3");
    SMCCAN.setPDOnumberentries(nodeid, tpdo3map, 1, true);      

    Serial.println("Activate RTPDO1 RTPDO2 RTPDO4 TPDO1 TPDO2 TPDO3");

    SMCCAN.activatePDO(nodeid, rpdo1par, R_PDO1_id, true);
    SMCCAN.activatePDO(nodeid, rpdo2par, R_PDO2_id, true);
    SMCCAN.activatePDO(nodeid, rpdo4par, R_PDO4_id, true);
    SMCCAN.activatePDO(nodeid, tpdo1par, T_PDO1_id, true);
    SMCCAN.activatePDO(nodeid, tpdo2par, T_PDO2_id, true);
    SMCCAN.activatePDO(nodeid, tpdo3par, T_PDO3_id, true);

    //** Deactivating unused PDOs **//
    SMCCAN.deactivatePDO(nodeid, rpdo1par, R_PDO1_id);
    SMCCAN.deactivatePDO(nodeid, rpdo2par, R_PDO2_id);
    SMCCAN.deactivatePDO(nodeid, tpdo1par, T_PDO1_id);    
    
    Serial.println("Activate remote node");
    SMCCAN.start_node(nodeid);
    Serial.println("Setting to torque mode");
    SMCCAN.writeToRegister(nodeid, 0x00, (int8_t)0x04, 0x6060); // (node, subindex, pos mode, index)
    SMCCAN.waitForReply(nodeid, 0x00, true);
    Serial.println("Ready to switch on");
    SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)0x06, control_index); // 0x6040 is the state machine control word
    SMCCAN.waitForReply(nodeid, 0x00, true);
    Serial.println("Switch on");
    SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)0x07, control_index);
    SMCCAN.waitForReply(nodeid, 0x00, true);
    Serial.println("Operation enable");
    SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)0xF, control_index);
    SMCCAN.waitForReply(nodeid, 0x00, true);
    Serial.println("Setting torque ramp to 1000%/sec");
    // SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)0x3E8, (uint16_t)0x6087); 2710
    SMCCAN.writeToRegister(nodeid, 0x00, (uint32_t)0x4E20, (uint16_t)0x6087);
    SMCCAN.waitForReply(nodeid, 0x00, true);
    Serial.println("Setting max torque speed to 120000 usteps/sec");
    SMCCAN.writeToRegister(nodeid, 0x00, (uint8_t)0x62, (uint16_t)0x2704);
    SMCCAN.waitForReply(nodeid, 0x00, true);
    Serial.println("Setting torque to zero");
    SMCCAN.writeToRegister(nodeid, 0x00, (uint16_t)0, targettorqueindex);
    SMCCAN.waitForReply(nodeid, 0x00, true);    
    Serial.send_now();
    
  }
  int incr = 0; 
  for (int i = 0; i < nnodes; i++)
    for (int j = 0; j < npdos; j++){ 
      pdo_cobsid[incr] = nodesid[i] + pdosid[j];
      incr++;
    }
  Serial.println("Setup complete \r\n\r\n");
  delay(5000);
  // without sending anything send a synchronize message, sync message 1 
  // Send the first sync message 
  SMCCAN.writeToRegisterS(sync_cobid);
  control_time.restart();
  outerloop_time.restart();
  samplingvar.restart();
  ReadVariables(); // this function contains the second sync message to send. Sync message 2, then will send data and receive through readvariables 
  SMCCAN.writeToRegisterS(sync_cobid); // execute several times to fill the velocity variable 
  ReadVariables();
  SMCCAN.writeToRegisterS(sync_cobid);
  ReadVariables();
  SMCCAN.writeToRegisterS(sync_cobid);
  ReadVariables();
  SMCCAN.writeToRegisterS(sync_cobid);
  ReadVariables();
  SMCCAN.writeToRegisterS(sync_cobid);
  // Calibrate();
  //// Calibratev(); ----> This function runs a calibration procedure on velocity control with torque inputs 
  // calibrate2(); 
  ReadVariables();
  SMCCAN.writeToRegisterS(sync_cobid);
  ReadVariables();
  SMCCAN.writeToRegisterS(sync_cobid);
  ReadVariables();
  SMCCAN.writeToRegisterS(sync_cobid);
  ReadVariables();
  SMCCAN.writeToRegisterS(sync_cobid);
  ReadVariables();
  SMCCAN.writeToRegisterS(sync_cobid);
  //// HomePosition();    ------> This function generates a homeposition trajectory for the robot 
  testpoint_time.restart();
}

//Method to check the received message
void OnReceived() {
  if (myMessenger.checkString("activate")) {
    passive_mode = false; 
    activated = true; 
    Serial.println("Active");
    for (int i = 0; i < nnodes; i++) {
      erri[i] = 0.0; 
      erriv[i] = 0.0; 
    }
    Serial.println("OK");
  }
    else if (myMessenger.checkString("setenc")) {
  	setEncoderUnits(); 
    Serial.println("OK enc");
  } 
  else if (myMessenger.checkString("offset")) {
    Serial.println("OK, offset");
    int offset = myMessenger.readInt();
  	oneLimb.setMotorPositionOffset(offset);
    Serial.println(offset);
  } 
  else if (myMessenger.checkString("saveappdata")) {
    Serial.println("OK trying to save");
  	saveApplicationData(); 
  }
  else if (myMessenger.checkString("status")) {
    Serial.println("OK, status");
    SMCCAN.getInt32FromRegister(nodesid[0], status_index, 0x0, &statusT); 
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
  else if (myMessenger.checkString("sync")) {
    Serial.println("OK, sync");
    Serial.send_now();
    SMCCAN.writeToRegisterS(sync_cobid);
    delay(29);
    CAN_message_t inMsg1;
    while (CANbus1.available()) { //Clear out the canbus
      CANbus1.read(inMsg1);
      hexDumpAll(inMsg1, &Serial);
    }
  }
  else if (myMessenger.checkString("lerror")) {
    Serial.println("OK, retrieving last error");
    Serial.send_now();
    Serial.println("Reading the error register");
    // error object 0x1001
    nodeid = 0x2; 
    SMCCAN.readRequestFromRegister(nodeid, 0x0, (uint16_t)0x1001);
    SMCCAN.waitForReply(nodeid, 0x00, true);
    Serial.println("Reading the error stack for last error");
    SMCCAN.readRequestFromRegister(nodeid, 0x1, (uint16_t)0x1003);
    SMCCAN.waitForReply(nodeid, 0x01, true);
    delay(29);
    CAN_message_t inMsg1;
    while (CANbus1.available()) { //Clear out the canbus
      CANbus1.read(inMsg1);
      hexDumpAll(inMsg1, &Serial);
    }
    Serial.println("going out lerror");
  }
  else if (myMessenger.checkString("setz")) {
    Serial.println("OK, z");
    float zpos = -myMessenger.readFloat();
  	oneLimb.setZref(zpos);
    Serial.println(zpos);
  } 
  else if (myMessenger.checkString("testtorque")) {
    Serial.println("OK, torquetest");
    Serial.send_now();
    testTorque(); 
  }
  else if (myMessenger.checkString("fid")) {
    Serial.println("OK, feed");
    Serial.send_now();
    int feedin = myMessenger.readInt();
    if (feedin< 100) feedin = 100; 
    Serial.println(feedin);
    feedinterval = feedin;
  }
  else if (myMessenger.checkString("testsine")) {
    startsine = true; 
    spd = motposd;
    float samplitude = myMessenger.readFloat();
    if (samplitude == 0) {
      samplitude = 0.5; 
    }
    freq = myMessenger.readFloat();
    if (freq == 0) {
      freq = 1.0; 
    }
    ssamplitude = samplitude; 
    Serial.println("Ok startsine");
    Serial.print("AMplitude ");
    Serial.print(samplitude);
    Serial.print(" freq ");
    Serial.println(freq);
  }
  else if (myMessenger.checkString("sines")) {
    startsine = false; 
    
    Serial.println("OK");
  }
  else if (myMessenger.checkString("s")) {
    Serial.println("OK, stop");
    stopnow = true; 
    save = true;
    logvar = false;
  }
  else if (myMessenger.checkString("st")) {
    Serial.println("OK, start");
    stopnow = false;
  }
  else if (myMessenger.checkString("log")) {
    Serial.println("OK, log");
    sampling_time.restart();
    cycle_time.restart();
    feedback_time.restart();
    logvar = true;
    save = false;
  }
  else if (myMessenger.checkString("intc")) {
    Serial.println("OK, integral control");
    integralerror = true; 
    ierror = 0;  
  }
  else if (myMessenger.checkString("ni")) {
    Serial.println("OK, stop integral error");
    integralerror = false; 
    ierror = 0;  
  }
  else if (myMessenger.checkString("ki")) {
    Serial.println("OK, ki");
    kii = myMessenger.readFloat(); 
    Serial.println(kii);
  }
  else if (myMessenger.checkString("torque")) {
    Serial.println("OK, torque");
    torque = myMessenger.readFloat(); 
    Serial.println(torque);
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
  else if (myMessenger.checkString("kp")) {
    Serial.println("OK, kp");
    kpp = myMessenger.readFloat(); 
    Serial.println(kpp);
  }
  else if (myMessenger.checkString("pd")) {
    Serial.println("OK, pos");
    motposd = myMessenger.readFloat(); 
    Serial.println(motposd);
  }
  else if (myMessenger.checkString("vd")) {
    Serial.println("OK, vel");
    motspeedd = myMessenger.readFloat(); 
    Serial.println(motspeedd);
  }
  else if (myMessenger.checkString("kd")) {
    Serial.println("OK, kd");
    kdd = myMessenger.readFloat(); 
    Serial.println(kdd); 
  }

  else if (myMessenger.checkString("setup")) {
    Serial.println("OK, setup");
    Serial.send_now();
    OnSetup(); 
  }
  else if (myMessenger.checkString("tset")) {
    Serial.println("OK, setup");
    Serial.send_now();
    setTorqueSettings(); 
  }
  

  else if (myMessenger.checkString("toff")) {
    toff = true; 
    Serial.println("OK");
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

void GetNewMotorPosition(){
    feedback_samplingtime.restart();
    LED(1);
    SMCCAN.getInt32FromRegister(nodesid[0], 0x6064, 0x0, &motor_position); 
    LED(0);
    passed_reading = feedback_samplingtime.elapsed(); 
}

//////************ SETUP **************//////////////////////
void setup() {
    // Initiate Serial Communication
    Serial.begin(250000);
    // Activate the CAN bus device
    pinMode(28, OUTPUT);
    pinMode(35, OUTPUT);
    pinMode(13, OUTPUT); // LED for timing tests.
    digitalWrite(28, LOW);
    digitalWrite(35, LOW);
    myMessenger.attach(OnReceived);
    // reserve 200 bytes for the input_string:
    input_string.reserve(200);
    CANbus1.begin();
    delay(5000);
    Serial.println("Starting");
    oneLimb.setZref(-0.6);
    oneLimb.setMotorPositionOffset(-350);
    // LED(1);
    
    setTorqueSettings();

    // Init SD card.
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        while (1){
          delay(1000);
          Serial.println("Card failed, or not present");
        }
    }
    Serial.println("card initialized.");
    SD.remove("datalog.txt");
    delay(1000);
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile){
        dataFile.println("time, tau, z, zr");
        Serial.println("File ok");
        dataFile.close();
    } else {
        while(1){
            Serial.println("File error");
            delay(1000);
        }
    }

  
  SMCCAN.writeToRegisterS(sync_cobid);
}

///////*************** MAIN PROGRAM ******//////////////////
void loop() {
  
  ReadSerial(); // -- check the serial port, always at the end of line send a \r\n (arduino serial monitor does automatically if ticked the option) 
  
  if (wait_feedback.hasPassed(feedinterval)) {
    wait_feedback.restart();
    SendFeedback();
  }

  if (test_sdo && test_control.hasPassed(5000)) {
    control_us = test_control.elapsed();
    test_control.restart();

    GetNewMotorPosition();

    // do some control 
    torque = kpp*oneLimb.getTorque(motor_position);

    if(abs(torque) > 1000)
        torque = sign(torque)*1000;
   
    if (stopnow) 
      torque = 0;

    // LED(1);

    SMCCAN.writeToRegister(nodeid, 0x00, (int16_t)torque, targettorqueindex); //  1000 is 100% 
    SMCCAN.waitForReply(nodeid, 0x00, false);

    // LED(0);

    if (logvar && (counter < (Ndata-1))){
      data[0][counter] = sampling_time.elapsed();
      data[1][counter] = torque;
      data[2][counter] = oneLimb.z;
      data[3][counter] = oneLimb.zr;
      ++counter;
    } else if (stopnow && save && (counter > 10)){
      File file = SD.open("datalog.txt", FILE_WRITE);

      if (file){
        Serial.println("File open, trying to save.");
        save = false;
        String str = "";
        for (size_t i = 0; i < counter; i++)
        {
          str = "";
          for (size_t j = 0; j < 4; j++)
          {
            str += data[j][i]*100;
            if (j<3) str += ", ";
          }
          file.println(str);
          Serial.println(str);
        }
        file.close();
      } else {
        Serial.println("Couldnt open file nor save!");
        delay(500);
      }
    }
  }

  // if (!stopnow && (sampling_time.elapsed() > 1000000)){ // 100ms?
  //   oneLimb.setZref(-0.5);
  // }
}