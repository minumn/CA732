#include <Arduino.h>
#include <FlexCAN.h> 
#include "CANsmc.h"
#include <SMC66Registers.h>
#define pi12 1.5707963267949

CANsmc::CANsmc(FlexCAN* CANbus, Stream* port, uint8_t nodesid[4])
{
	// Attach serial port and canbus to class 
	_serialport = port; 
	_canport = CANbus; 
  _timeout = 500; 
  timeoutcomm.restart();
  
  memcpy(nodesids, nodesid, sizeof(nodesids));
  _lt = micros(); 
  for(int i=0;i<4;i++) joints_velocity[i] = 0.0;
  // This are new offsets with respect to ragnar readings 
  //  2.9583    0.0099   -1.3746   -4.3786
  offsets[0] = -pi12  - 5.906863575;// - 1.4854 - 0.0023;
  offsets[1] = -pi12 +2.9169618833333333;+ 1.452607347 +1.37925; // - 0.0156 - 0.0084;
  offsets[2] = pi12 -0.08288235277777783; //  - 0.1067 + 1.469;
  offsets[3] = pi12 + 7.305978455555556; // + 1.4701 + 1.4649;
  tpdosid[0] = TX_PDO21_OBID;
  tpdosid[1] = TX_PDO22_OBID;
  // This was computed from matlab using a self-calibration method
  // to reduce the error in the forward kinematics 
  offsetscalibration[0] = 0.013144336131269;
  offsetscalibration[1] = -0.001705550151094;
  offsetscalibration[2] = 0.096971874003720;
  offsetscalibration[3] = -0.007045195061084;
 //     0.012129672257339   0.013055240486628  -0.007165089547312   0.016217276350355
  offsetscalibration[0] = -0.018013844399125 + 0.012129672257339 ;
  offsetscalibration[1] = -0.007996736054328 + 0.013055240486628 ;
  offsetscalibration[2] = -0.018940303097932 - 0.007165089547312;
  offsetscalibration[3] =  0.006279753670964 + 0.016217276350355;

  //  -3.849  1.525   1.573   4.296
  // for (int i = 0; i < 4; i++) offsetscalibration[i] = 0.0; 
  // for (int i = 0; i < 4; i++) offsets[i] = 0.0; 
  
  int incr = 0; 
  for( int i = 0; i < 4; i++) {
    for( int j = 0; j < 2; j++) {
      ragpdos_tcobids[incr] = nodesid[i] + tpdosid[j]; // for receiving only 
      incr++; 
    }
  }
}

void CANsmc::setTimeOutCAN(int timeout)
{
  // is set in micros 
  _timeout = timeout; 
}

void CANsmc::checkTorque(uint8_t nodeid, bool dump) {
  float pertor = getActualTorquePercent(nodeid);
  //  Serial.println(pertor);
  if (pertor < 2) {
    //    Serial.println("zero torque");
    setPassiveMode(nodeid);
    if(dump)
      Serial.println("torquereset");

    setPositionMode(nodeid);
  }
}

void CANsmc::clearErrors(uint8_t nodeid, bool dump){
    uint32_t data = 0; 
    writeToRegister(nodeid, ERR_BITS, data);
    waitForReply(nodeid, ERR_BITS, dump); 
}

void CANsmc::configurePDOS(uint8_t nodeid) {
  // Specify the COBID of the sync message 
  uint16_t objet_index = COB_ID_SYNC_message_index; 
  uint8_t subindex = 0x00; 
  uint32_t datas = COBID_SYNC_MESSAGE;
  writeToRegister(nodeid, subindex, datas, objet_index); 
  // Activate receive and send as synchronous 
  objet_index = RX_PDO21_COMM_PAR;
  subindex = transmission_type; 
  datas = 0; 
  writeToRegister(nodeid, subindex, datas, objet_index); 
  subindex = pdo_cobid;
  datas = RX_PDO21_OBID;
  writeToRegister(nodeid, subindex, datas, objet_index); 
  // Activate receive and send as synchronous 
  objet_index = RX_PDO22_COMM_PAR;
  subindex = transmission_type; 
  datas = 0; 
  writeToRegister(nodeid, subindex, datas, objet_index); 
  subindex = pdo_cobid;
  datas = RX_PDO22_OBID;
  writeToRegister(nodeid, subindex, datas, objet_index); 
  // Remap PDO21 
  // 1 write 0 in number of entries 
  objet_index = TX_PDO21_MAPPING_PAR;
  subindex = number_of_entries; 
  datas = 0; 
  writeToRegister(nodeid, subindex, datas, objet_index); 
  // 2 remap to something else 
  objet_index = TX_PDO21_MAPPING_PAR;
  subindex = PDO_mapping_entry_1;
  datas = 0x2012; // this is 32 bits, use 2014 for 16 bit 
  datas = (datas << 16) + ENCODER_POS;
  datas = (datas << 16) + 0x20; // use 10 for 16 bits
  writeToRegister(nodeid, subindex, datas, objet_index); 
  // remap the second entry for follow error 
  objet_index = TX_PDO21_MAPPING_PAR;
  subindex = PDO_mapping_entry_2;
  datas = 0x2012; // this is 32 bits, use 2014 for 16 bit 
  datas = (datas << 16) + FLWERR;
  datas = (datas << 16) + 0x20; // use 10 for 16 bits 
  writeToRegister(nodeid, subindex, datas, objet_index); 
  // 3 write 0 in number of entries 
  objet_index = TX_PDO21_MAPPING_PAR;
  subindex = number_of_entries; 
  datas = 2;
  writeToRegister(nodeid, subindex, datas, objet_index); 
  // 4 activate the PDO 
  objet_index = TX_PDO21_COMM_PAR;
  subindex = pdo_cobid;
  datas = TX_PDO22_OBID;
  writeToRegister(nodeid, subindex, datas, objet_index); 
  // Remap PDO22 
  // 1 write 0 in number of entries 
  objet_index = TX_PDO22_MAPPING_PAR;
  subindex = number_of_entries; 
  datas = 0; 
  writeToRegister(nodeid, subindex, datas, objet_index); 
  // 2 remap to something else 
  objet_index = TX_PDO22_MAPPING_PAR;
  subindex = PDO_mapping_entry_1;
  datas = 0x2014; // this is 32 bits, use 2014 for 16 bit 
  datas = (datas << 16) + ACTUAL_TORQUE;
  datas = (datas << 16) + 0x10; // use 10 for 16 bits 
  writeToRegister(nodeid, subindex, datas, objet_index); 
  // remap the second entry for RUN CURRENT 
  objet_index = TX_PDO22_MAPPING_PAR;
  subindex = PDO_mapping_entry_2;
  datas = 0x2014; // this is 32 bits, use 2014 for 16 bit 
  datas = (datas << 16) + RUN_CURRENT;
  datas = (datas << 16) + 0x10; // use 10 for 16 bits 
  writeToRegister(nodeid, subindex, datas, objet_index); 
  // 3 write 0 in number of entries 
  objet_index = TX_PDO22_MAPPING_PAR;
  subindex = number_of_entries; 
  datas = 2;
  writeToRegister(nodeid, subindex, datas, objet_index); 
  // 4 activate the PDO 
  objet_index = TX_PDO22_COMM_PAR;
  subindex = pdo_cobid;
  datas = TX_PDO22_OBID; 
  writeToRegister(nodeid, subindex, datas, objet_index); 
}

void CANsmc::setTorque(uint8_t nodeidn, float current_t) {
  // receives the torque in mili amps
  uint8_t nodeid = nodesids[nodeidn];
  float runncurrent = current_t; // * 1000;
  float percent = 0;
  /////
  if (abs(current_t) > 2.0) 
    checkTorque(nodeid, true);
  //////
  intruncu[nodeidn] = runncurrent;
  if (abs(runncurrent) > 800){
    runncurrent = 800;
    intruncu[nodeidn] = runncurrent;
    if (current_t < 0)
      runncurrent *= -1; 
  }
  setRunCurrent(nodeid, abs(runncurrent));
  int32_t encopos = getEncoderPosition(nodeid);
  int32_t psoll = encopos;
  if (abs(runncurrent) < 5.9) {
    percent = abs(runncurrent) * 348.89; //(100/5.87)*(2048/100)
    //int32_t start_velocity = 0x01; 
    //SMCCAN.writeToRegister(nodeid, V_START, start_velocity);
    //SMCCAN.waitForReply(nodeid, V_START, false); 
  }
  else {
    ////percent = 2048 * abs(runncurrent); // 2048 * 4; //3000
    if (abs(runncurrent)<20) percent = 5 * 2048 * abs(runncurrent); 
    else if (abs(runncurrent)<100) percent = 1.5 * 2048 * abs(runncurrent); 
    // else percent = 716.8 * abs(runncurrent); // 2048/5
    else percent = 50 * abs(runncurrent); // 2048/5
    
    ////percent = 3048 * runncurrent; 
    //int32_t start_velocity = 30000; 
    //SMCCAN.writeToRegister(nodeid, V_START, start_velocity);
    //SMCCAN.waitForReply(nodeid, V_START, false); 
  }

  if (runncurrent < 0)
    psoll -= percent;
  else
    psoll += percent;

  if (runncurrent == 0)
    psoll = encopos;

  writeToRegister(nodeid, P_SOLL, psoll);
  waitForReply(nodeid, P_SOLL, false);
}
 // CAN_message_t inMsg; 
  // inMsg.id = id; 
  // inMsg.len = len; 
  // inMsg.buf[0] = d0; 
  // inMsg.buf[1] = d1; 
  // inMsg.buf[2] = d2; 
  // inMsg.buf[3] = d3; 
  // inMsg.buf[4] = d4; 
  // inMsg.buf[5] = d5; 
  // inMsg.buf[6] = d6; 
  // inMsg.buf[7] = d7; 
  
  // hexDumpAll(inMsg, &Serial);
  // Serial.print("sending: ");
  // Serial.print(psoll);
  // Serial.print(" ");
  // Serial.print(vsol);
  // Serial.print(" ");
  // Serial.print(asol);
  // Serial.print(" ");
  // Serial.print(runcu);
  // Serial.print(" ");
  // Serial.print(modreg);
  // Serial.println(); 
  // CAN_message_t inMsg; 
  // inMsg.id = id; 
  // inMsg.len = len; 
  // inMsg.buf[0] = d0; 
  // inMsg.buf[1] = d1; 
  // inMsg.buf[2] = d2; 
  // inMsg.buf[3] = d3; 
  // inMsg.buf[4] = d4; 
  // inMsg.buf[5] = d5; 
  // inMsg.buf[6] = d6; 
  // inMsg.buf[7] = d7; 
  
  // hexDumpAll(inMsg, &Serial);

void CANsmc::setTorquePDOa(uint8_t nodeidn, float current_t, int32_t vsol, 
                           uint16_t asol, bool signedv) {
  // this receives asol and vsol. 
  // receives the torque in mili amps
  // Serial.println("entered here");
  //** Experimental
  if(!signedv) vsol = abs(vsol); 
  uint8_t nodeid = nodesids[nodeidn];
  float runncurrent = current_t; // * 1000;
  uint16_t runcu; 
  float percent = 0;
  uint16_t modreg = 0x02;
  //** experimental 
  modreg = 0x01; 
  //** experimental 
  int32_t encopos = encoder_pos[nodeidn];
  int32_t psoll = encopos;
  uint16_t pdoid;
  uint32_t id; 
  uint8_t len; 
  uint8_t d0, d1, d2, d3, d4, d5, d6, d7;

  if (abs(runncurrent) > 800/*2000*/){
    runncurrent = 800/*2000*/;
    if (current_t < 0) {
      runncurrent *= -1;
      //** experimental
    }
    //** experimental 
  }
  //** experimental 
  intruncu[nodeidn] = runncurrent;
  // Serial.println(nodeidn);
  // Serial.println(current_t);
  // Serial.println(vsol);
  runcu = abs(runncurrent) * C_current;
  // Serial.println(runcu);
  if (abs(runncurrent) < 5.9) {
    percent = abs(runncurrent) * 348.89; //(100/5.87)*(2048/100)
  }
  else {
    ////percent = 2048 * abs(runncurrent); // 2048 * 4; //3000
    if (abs(runncurrent) < 20) percent = 5 * 2048 * abs(runncurrent); 
    else if (abs(runncurrent) < 100) percent = 1.5 * 2048 * abs(runncurrent); 
    // else percent = 716.8 * abs(runncurrent); // 2048/5
    else percent = 50.0 * abs(runncurrent); // 2048/5
  }
  // percent = 50.0 * 600; 
  if (runncurrent < 0 && !signedv)
    vsol *= (-1);
  
  pdoid = RX_PDO21_OBID;
  id = nodeid + pdoid; 
  len = 4; 
  // if (abs(runncurrent) < 5.87) 
  //   modreg = 0x0; 
    
  d0 = vsol & 0xFF;
  d1 = (vsol >> 8) & 0xFF;
  d2 = (vsol >> 16) & 0xFF;
  d3 = (vsol >> 24) & 0xFF; //MSB , little endian format  
  d4 = 0; 
  d5 = 0; 
  d6 = 0;
  d7 = 0; 
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
  
  delayMicroseconds(300); 
  pdoid = RX_PDO22_OBID;
  id = nodeid + pdoid; 
  len = 6; 
  d0 = lowByte(asol);
  d1 = highByte(asol);
  d2 = lowByte(runcu);
  d3 = highByte(runcu);
  d4 = lowByte(modreg);
  d5 = highByte(modreg);
  d6 = 0;
  d7 = 0; //MSB , little endian format
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}



void CANsmc::setTorquePDO(uint8_t nodeidn, float current_t) {
  // receives the torque in mili amps
  // Serial.println("entered here");
  uint8_t nodeid = nodesids[nodeidn];
  float runncurrent = current_t; // * 1000;
  uint16_t runcu; 
  int32_t vsol = 51428; // 2 rps 
  vsol = 120000;
  vsol *= 2.0;  
  vsol = 31000; 
  vsol = 33000; 
  uint16_t asol = 100000; 
  //** experimental 
  asol = float(5142) * float(1.2);  
  asol = 18142;
  asol *= 2.0; 
  asol = 65500; 
  asol = 60000; 
  // asol = 10000;
  //** experimental 
  uint16_t modreg = 0x01;
  //** experimental 
  modreg = 0x01; 
  //** experimental 
  int32_t encopos = encoder_pos[nodeidn];
  int32_t psoll = encopos;
  uint16_t pdoid;
  uint32_t id; 
  uint8_t len; 
  uint8_t d0, d1, d2, d3, d4, d5, d6, d7;

  if (abs(runncurrent) > 800/*2000*/){
    runncurrent = 800/*2000*/;
    if (current_t < 0) {
      runncurrent *= -1;
      //** experimental
      vsol *= (-1);  
    }
    //** experimental 
  }
  //** experimental 
  else if (current_t < 0)  vsol *= (-1); 
  vsol_com[nodeidn] = vsol; 
  intruncu[nodeidn] = runncurrent;
  runcu = abs(runncurrent) * C_current;
  // if (runcu == 0)  vsol = 0x0; 
  pdoid = RX_PDO21_OBID;
  id = nodeid + pdoid; 
  len = 4; 
  d0 = vsol & 0xFF;
  d1 = (vsol >> 8) & 0xFF;
  d2 = (vsol >> 16) & 0xFF;
  d3 = (vsol >> 24) & 0xFF; //MSB , little endian format  
  d4 = 0; 
  d5 = 0; 
  d6 = 0;
  d7 = 0; 
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
  
  // delayMicroseconds(300); 
  
  delayMicroseconds(100); 
  pdoid = RX_PDO22_OBID;
  id = nodeid + pdoid; 
  len = 6; 
  // runcu = 0x0; 
  d0 = lowByte(asol);
  d1 = highByte(asol);
  d2 = lowByte(runcu);
  d3 = highByte(runcu);
  d4 = lowByte(modreg);
  d5 = highByte(modreg);
  d6 = 0;
  d7 = 0; //MSB , little endian format
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}

void CANsmc::readVariables(){
  // also computes the joint speed of arms 
  _dt = (float(micros()) -float(_lt)) / 1000000.0; //dt in seconds
  // Serial.print(_dt);  
  // Serial.print(" ");
  // Serial.println(micros());
  // Serial.println(_lt);
  _lt = micros();
  
  for(int i=0; i<4; i++) {
    _last_encoder_pos[i] = encoder_pos[i];
    encoder_pos[i] = getEncoderPosition(nodesids[i]);
    follow_err[i] = getFollowError(nodesids[i]);
    actual_torque_percent[i] = getActualTorquePercent(nodesids[i]);
    run_currents[i] = getRunCurrent(nodesids[i]);
    joints_velocity[i] = float(encoder_pos[i] - _last_encoder_pos[i])/_dt; 
    joints_velocity[i] *= COUNTS_TO_RAD;
    joints_velocity[i] /=  REDUCTION;
    joints_position[i] = float(encoder_pos[i]) * COUNTS_TO_RAD / REDUCTION;
    joints_position[i] -=  offsets[i];
    joints_position[i] += offsetscalibration[i];
  }
  encoder_posd0 = encoder_pos[0];
  encoder_posd1 = encoder_pos[1];
  encoder_posd2 = encoder_pos[2];
  encoder_posd3 = encoder_pos[3];
}

float convertActualTorquePercent(uint16_t torque_percent) {
  float percent = (torque_percent * COUNT_TO_PERCENT ) * 100;
  return percent;
}

void CANsmc::updateVar(){
  
  for(int i=0; i<4; i++) {
    actual_torque_percent[i] = convertActualTorquePercent(actual_torque[i]);
    joints_position[i] = float(encoder_pos[i]) * COUNTS_TO_RAD / REDUCTION;
    joints_position[i] -=  offsets[i];
    joints_position[i] += offsetscalibration[i];
    
  }
  encoder_posd0 = encoder_pos[0];
  encoder_posd1 = encoder_pos[1];
  encoder_posd2 = encoder_pos[2];
  encoder_posd3 = encoder_pos[3];
}

void CANsmc::computeVel() {
  // dt comes in ms 
  _dt = (float(micros()) -float(_lt)) / 1000000.0; //dt in seconds
  // Serial.print(_dt);  
  // Serial.print(" ");
  // Serial.println(micros());
  // Serial.println(_lt);
  _lt = micros();
  for (int i = 0; i < 4; i ++){
    joints_velocity[i] = float(encoder_pos[i] - _last_encoder_pos[i])/_dt; 
    joints_velocity[i] *= COUNTS_TO_RAD;
    joints_velocity[i] /=  REDUCTION;
    _last_encoder_pos[i] = encoder_pos[i];
  }
}

void CANsmc::jointsToCounts(float joints[4], float (*counts_vector)[4]) {
  // joints should come directly from the FK
  // offsets are accounted for in here 
  for (int i = 0; i<4; i++) {
    (*counts_vector)[i] = joints[i] + offsets[i];
    (*counts_vector)[i] -= offsetscalibration[i];
    (*counts_vector)[i] /= COUNTS_TO_RAD; 
    (*counts_vector)[i] *= REDUCTION;
  }
}

// Send a actual position read request
int32_t CANsmc::getPIST(uint8_t nodeid) {
  //returns the positions in counts
  int32_t actual_position = 0;
  readRequestFromRegister(nodeid, P_IST);
  waitForReplyInt32(nodeid, P_IST, &actual_position, false);
  return actual_position;
}
// ------------------------------------------------------------

float CANsmc::getActualTorquePercent(uint8_t nodeid) {
  // returns torque in percent of running current, needs to read running current as well
  // returns torque in Nm
  float percent = 0;
  uint32_t torque_percent = 0;
  readRequestFromRegister(nodeid, ACTUAL_TORQUE);
  waitForReplyuInt32(nodeid, ACTUAL_TORQUE, &torque_percent, false);
  percent = (torque_percent * COUNT_TO_PERCENT ) * 100;
  return percent;
}


// ------------------------------------------------------------
float CANsmc::getTorque(uint8_t nodeid) {
  // returns torque in Nm
  float actual_current = 0.0;
  float torque = 0.0;
  torque = getActualTorquePercent(nodeid);
  uint32_t runcu = 0;
  runcu = getRunCurrent(nodeid);
  //  r_current = read_runcurrent();
  actual_current = (torque * runcu * current_C) / 1000; //divide over 1000 cause result is in mA, we want in A
  torque = actual_current * AMPS_TO_TORQUE;
  return torque;
}
// ------------------------------------------------------------
// Send a encoder position read request
int32_t CANsmc::getEncoderPosition(uint8_t nodeid) {
  //returns encoder position in counts
  int32_t enc_position = 0;
  readRequestFromRegister(nodeid, ENCODER_POS);
  // Serial.println("waiting for message");
  // ttiming = micros();
  waitForReplyInt32(nodeid, ENCODER_POS, &enc_position, false);
  return enc_position;
}

// Send a actual velocity read request
int32_t CANsmc::getActualVelocity(uint8_t nodeid) {
  //returns encoder position in counts
  int32_t actual_velocity = 0;
  readRequestFromRegister(nodeid, V_IST);
  // Serial.println("waiting for message");
  // ttiming = micros();
  waitForReplyInt32(nodeid, V_IST, &actual_velocity, false);
  return actual_velocity;
}
// Send a actual velocity read request
int32_t CANsmc::getEncoderVelocity(uint8_t nodeid) {
  //returns encoder position in counts
  int32_t actual_velocity = 0;
  readRequestFromRegister(nodeid, V_ENCODER);
  // Serial.println("waiting for message");
  // ttiming = micros();
  waitForReplyInt32(nodeid, V_ENCODER, &actual_velocity, false);
  return actual_velocity;
}

// ------------------------------------------------------------
float countsToRad(int32_t counts) {
  return counts * COUNTS_TO_RAD;
}

void CANsmc::getUInt32FromRegister(
  uint8_t nodeid, uint16_t object_index, uint8_t subindex, 
  uint32_t * number32/*, bool dump = false*/) {
  readRequestFromRegister(nodeid, subindex, object_index);
  delayMicroseconds(300);
  waitForReplyuInt32(nodeid, subindex, number32, false);
  // waitForReply(nodeid, subindex, &number32);//, dump);
}

void CANsmc::getInt32FromRegister(
  uint8_t nodeid, uint16_t object_index, uint8_t subindex, 
  int32_t * number32/*, bool dump = false*/) {
  readRequestFromRegister(nodeid, subindex, object_index);
  delayMicroseconds(300);
  waitForReplyInt32(nodeid, subindex, number32, false); //, dump);
}

float CANsmc::getVSOLL(uint8_t nodeid) {
  // returns the velocity in rpm
  int32_t current_velocity = 0;
  readRequestFromRegister(nodeid, V_SOLL);
  waitForReplyInt32(nodeid, V_SOLL, &current_velocity, false);
  return current_velocity * VEL_UNITS;
}

//-------------------------------------------------------------
// Send a velocity read request
float CANsmc::getVelEnc(uint8_t nodeid) {
  // returns the velocity in rpm
  int32_t current_velocity = 0;
  readRequestFromRegister(nodeid, V_ENCODER);
  waitForReplyInt32(nodeid, V_ENCODER, &current_velocity, false);
  return (float) (current_velocity * VEL_UNITS);
}
// ------------------------------------------------------------
// Send a velocity read request
float CANsmc::getVIST(uint8_t nodeid) {
  // returns the velocity in rpm
  int32_t current_velocity = 0;
  readRequestFromRegister(nodeid, V_IST);
  waitForReplyInt32(nodeid, V_IST, &current_velocity, false);
  return (float)(current_velocity * VEL_UNITS);
}

void CANsmc::setStandbyCurrent(uint8_t nodeid, float current) {
  // Set the current in miliamperes, use 2000 for exampel
  // current = current * .3;
  uint32_t current_int = 0;
  if (current < 0) {
    current = -current;
  }
  if (current > 2000) {
    current = 2000;
  }
  current_int = (current / current_C);
  writeToRegister(nodeid, STANDBY_CURRENT, current_int);
  waitForReply(nodeid, STANDBY_CURRENT, false);
}
// ------------------------------------------------------------
// Send a standbycurrent read request
float CANsmc::getStandbyCurrent(uint8_t nodeid) {
  // returns the velocity in mA
  uint32_t current = 0;
  readRequestFromRegister(nodeid, STANDBY_CURRENT);
  waitForReplyuInt32(nodeid, STANDBY_CURRENT, &current, false);
  return current * current_C;
}
// Function to set running current in mA
void CANsmc::setRunCurrent(uint8_t nodeid, float current) {
  //  Serial.println(current);
  // Set the current in miliamperes, use 2000 for exampel
  uint32_t current_int = 0;
  CAN_message_t inMsg;
  if (current < 0) {
    current = -current;
  }
  if (current > 2000) {
    current = 2000;
  }
  current_int = current * C_current;
  writeToRegister(nodeid, RUN_CURRENT, current_int);
  waitForReply(nodeid, RUN_CURRENT, false);
}
// ------------------------------------------------------------

uint32_t CANsmc::getRunCurrent(uint8_t nodeid) {
  uint32_t run_current = 0;
  CAN_message_t inMsg;
  readRequestFromRegister(nodeid, RUN_CURRENT);
  waitForReplyuInt32(nodeid, RUN_CURRENT, &run_current, false);
  return run_current;
}

int32_t CANsmc::getFollowError(uint8_t nodeid) {
  //returns encoder position in counts
  int32_t fllerror = 0;
  readRequestFromRegister(nodeid, FLWERR);
  waitForReplyInt32(nodeid, FLWERR, &fllerror, false);
  return fllerror;
}

void CANsmc::setPSOLL(uint8_t nodeid, int32_t psoll) {
  writeToRegister(nodeid, P_SOLL, psoll);
  waitForReply(nodeid, P_SOLL, false);
}

void CANsmc::setVelocityInt32(uint8_t nodeid, int32_t velocity) {
  // each unit is 0.01 rpm
  writeToRegister(nodeid, V_SOLL, velocity);
  waitForReply(nodeid, V_SOLL, false);
}
void CANsmc::setAsoll(uint8_t nodeid, int32_t asoll) {

  writeToRegister(nodeid, A_SOLL, asoll);
  waitForReply(nodeid, A_SOLL, true);
}

void CANsmc::setVelocityMode(uint8_t nodeid) {
  uint32_t veloccity_mode = 0x00000001;
  writeToRegister(nodeid, MODE_REG, veloccity_mode);
  waitForReply(nodeid, MODE_REG, false);
}

void CANsmc::setPositionMode(uint8_t nodeid) {
  uint32_t possition_mode = 0x00000002;
  writeToRegister(nodeid, MODE_REG, possition_mode);
  waitForReply(nodeid, MODE_REG, true);
}

void CANsmc::setPassiveMode(uint8_t nodeid) {
  uint32_t passsive_mode = 0;
  writeToRegister(nodeid, MODE_REG, passsive_mode);
  waitForReply(nodeid, MODE_REG, false);
}

void CANsmc::start_node(uint8_t nodeid) {
  // sent to cobid 0x00 
  wrMsg(0x00, 2, 0x01, nodeid, 0, 0, 0, 0, 0, 0);
}

void CANsmc::preoperational_node(uint8_t nodeid) {
  // sent to cobid 0x00 
  wrMsg(0x00, 2, 0x80, nodeid, 0, 0, 0, 0, 0, 0);
}

void CANsmc::reset_node(uint8_t nodeid) {
  // command 0x81 resets the node
  wrMsg(0x00, 2, 0x81, nodeid, 0, 0, 0, 0, 0, 0);
}

void CANsmc::wrMsg(
  uint32_t  id, uint8_t len, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, 
  uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7) {
  CAN_message_t msg;
  for (int i = 0; i < 7; i++)
    msg.buf[i] = 0;

  msg.id = id;
  msg.len = len;
  msg.ext = 0;

  msg.buf[0] = d0;
  msg.buf[1] = d1;
  msg.buf[2] = d2;
  msg.buf[3] = d3;
  msg.buf[4] = d4;
  msg.buf[5] = d5;
  msg.buf[6] = d6;
  msg.buf[7] = d7;

  _canport->write(msg);
}


bool CANsmc::waitForReply(uint8_t nodeid, uint8_t subindex, bool dump) {
  //   Function to be called after write to register. Will wait for a
  //  reply from the service called, return wether it was success or not 
  bool replied = false;
  uint32_t id = nodeid + WRITE_REQUEST_CAN;
  CAN_message_t inMsg;
  //_timetotimeout = micros();
  while (!replied /*|| !(micros() - _timetotimeout > _timeout)*/) {
    while (_canport->available()) {
      //Clear out the canbus
      _canport->read(inMsg);
      if (dump) {
        hexDumpAll(inMsg, _serialport);
      }
      if (/*inMsg.id == id && */inMsg.buf[3] == subindex) {
        replied = true;
      }
    }
  }
  return replied; 
}

bool CANsmc::waitForReplyInt32(uint8_t nodeid, uint8_t subindex, int32_t* number32, bool dump) {
  //  Function to be called after writeToRegister
  // will return a register value of int32
  bool replied = false;
  //uint32_t id = nodeid + WRITE_REQUEST_CAN;
  int32_t value = 0;
  CAN_message_t inMsg;
  //_timetotimeout = micros(); 
  while (!replied /*|| !(micros() - _timetotimeout > _timeout)*/) {
    while (_canport->available()) {
      //Clear out the canbus
      _canport->read(inMsg);
      if (dump)
        hexDumpAll(inMsg, _serialport);

      if (inMsg.buf[0] != CAN_ERROR_RESPONSE) {
        //int byte_num = numByte(inMsg.buf[0]);
        if (/*inMsg.id == id && */inMsg.buf[3] == subindex) {
          reconstructInt32(&value, inMsg);
          replied = true;
          // Serial.println("Received position");
        }
      }
    }
  }
  *number32 = value; 
  return replied;
}

bool CANsmc::waitForPDOSragnar(bool dump) {
  uint8_t cttr = 0; 
  uint8_t npdo = 8; // only one node 
  bool replied = false; 
  uint16_t value16 = 0; 
  int32_t value32 = 0; 
  union {
    int32_t myInt32 ;            // 4 bytes
    byte myInt32InBytes[4] ;   //  mapped onto the same storage as myInt32
  } myUnion ;
  union {
    int32_t myInt32 ;            // 4 bytes
    byte myInt32InBytes[4] ;   //  mapped onto the same storage as myInt32
  } myUnion2 ;
  union {
    uint32_t myuInt32 ;            // 4 bytes
    byte myuInt32InBytes[4] ;   //  mapped onto the same storage as myInt32
  } myUnion3 ;
  
  union {
    uint16_t myuInt16 ;            // 4 bytes
    byte myuInt16InBytes[2] ;   //  mapped onto the same storage as myuInt16
  } myUnion16 ;
  //_timetotimeout = micros(); 
  timeoutcomm.restart();
  // Serial.println("enter here");
  bool notreply = true;
  timeoutcomm.restart(); 
  // Serial.println("waiting pdos");
  CAN_message_t inMsgl; 
  inMsgl.id = 0x0; 
  while (!replied /*|| !timeoutcomm.hasPassed(2)*/) {
    while (_canport->available() || !timeoutcomm.hasPassed(2)) {
      //Clear out the canbus
      CAN_message_t inMsg;
      _canport->read(inMsg);
      if (dump)
        hexDumpAll(inMsg, _serialport);
      if (inMsg.id != inMsgl.id) {
        for (int i = 0; i < npdo; i++) {
          if(inMsg.id == ragpdos_tcobids[i]) { 
            // 0 := 0x06 + PDO21 
          // 1 := 0x06 + PDO22 
          // 2 := 0x07 + PDO21
          // 3 := 0x07 + PDO22 
          // Serial.println(millis());
          /*
            Serial.print("pdo ");
            Serial.println(inMsg.id, HEX);
            
            Serial.print(" ");
            Serial.print(inMsg.len);
            Serial.print(" ");
            Serial.print(inMsg.buf[0], HEX);
            Serial.print(inMsg.buf[1], HEX);
            Serial.print(inMsg.buf[2], HEX);
            Serial.print(inMsg.buf[3], HEX);
            Serial.print(inMsg.buf[4], HEX);
            Serial.print(inMsg.buf[5], HEX);
            Serial.print(inMsg.buf[6], HEX);
            Serial.println(inMsg.buf[7], HEX);
          */
            switch(i) {
              case 0:
                for (int i = 0; i < 4; i++) myUnion.myInt32InBytes[i] = inMsg.buf[i];
                for (int i = 0; i < 4; i++) myUnion3.myuInt32InBytes[i] = inMsg.buf[i+4]; 
                cttr++;
                //_last_encoder_pos[0] = encoder_pos[0];
                encoder_pos[0] = myUnion.myInt32; 
                status_bits[0] = myUnion3.myuInt32; 
                break; 
              case 1:
                value16 = (inMsg.buf[1] << 8) + inMsg.buf[0];
                actual_torque[0] = value16; 
                value16 = (inMsg.buf[3] << 8) + inMsg.buf[2];
                run_currents[0] = value16;
                value32 = (inMsg.buf[7] << 24) + (inMsg.buf[6] << 16) + (inMsg.buf[5] << 8) + inMsg.buf[4];
                encoder_velocity[0] = value32; 
                cttr++;              
                break;
              case 2:
                for (int i = 0; i < 4; i++) myUnion.myInt32InBytes[i] = inMsg.buf[i];
                for (int i = 0; i < 4; i++) myUnion3.myuInt32InBytes[i] = inMsg.buf[i+4]; 
                cttr++;
                //_last_encoder_pos[1] = encoder_pos[1];
                encoder_pos[1] = myUnion.myInt32; 
                status_bits[1] = myUnion3.myuInt32; 
                break; 
              case 3:
                value16 = (inMsg.buf[1] << 8) + inMsg.buf[0];
                actual_torque[1] = value16; 
                value16 = (inMsg.buf[3] << 8) + inMsg.buf[2];
                run_currents[1] = value16;
                value32 = (inMsg.buf[7] << 24) + (inMsg.buf[6] << 16) + (inMsg.buf[5] << 8) + inMsg.buf[4];
                encoder_velocity[1] = value32; 
                cttr++;              
                break;
              case 4:
                for (int i = 0; i < 4; i++) myUnion.myInt32InBytes[i] = inMsg.buf[i];
                for (int i = 0; i < 4; i++) myUnion3.myuInt32InBytes[i] = inMsg.buf[i+4]; 
                cttr++;
                //_last_encoder_pos[2] = encoder_pos[2];
                encoder_pos[2] = myUnion.myInt32; 
                status_bits[2] = myUnion3.myuInt32; 
                break; 
              case 5:
                value16 = (inMsg.buf[1] << 8) + inMsg.buf[0];
                actual_torque[2] = value16; 
                value16 = (inMsg.buf[3] << 8) + inMsg.buf[2];
                run_currents[2] = value16;
                value32 = (inMsg.buf[7] << 24) + (inMsg.buf[6] << 16) + (inMsg.buf[5] << 8) + inMsg.buf[4];
                encoder_velocity[2] = value32;  
                cttr++;              
                break;
              case 6:
                for (int i = 0; i < 4; i++) myUnion.myInt32InBytes[i] = inMsg.buf[i];
                for (int i = 0; i < 4; i++) myUnion3.myuInt32InBytes[i] = inMsg.buf[i+4]; 
                cttr++;
                //_last_encoder_pos[3] = encoder_pos[3];
                encoder_pos[3] = myUnion.myInt32; 
                status_bits[3] = myUnion3.myuInt32; 
                break; 
              case 7:
                value16 = (inMsg.buf[1] << 8) + inMsg.buf[0];
                actual_torque[3] = value16; 
                value16 = (inMsg.buf[3] << 8) + inMsg.buf[2];
                run_currents[3] = value16;
                value32 = (inMsg.buf[7] << 24) + (inMsg.buf[6] << 16) + (inMsg.buf[5] << 8) + inMsg.buf[4];
                encoder_velocity[3] = value32; 
                cttr++;              
                break;
            }
          }
        }
      }
      inMsgl.id = inMsg.id; 
      if(cttr == npdo) replied = true;
    }
    if (timeoutcomm.hasPassed(1)){
      notreply = !replied;
      replied = true; 
    }
  }
  for (int i = 0; i < 4; i++) {
    actual_torque_percent[i] = actual_torque[i] * COUNT_TO_PERCENT * 100; 
  }
  if (notreply) {
    Serial.print("Not replied received: ");
    Serial.println(cttr);
  } 
  // Serial.print("Number pdos ");
  // Serial.println(cttr);         
}

bool CANsmc::waitForPDOSpalletizer(uint32_t pdocobid[20], uint16_t (*status)[3],
                          int32_t (*q)[3], int16_t (*act_torque)[3], int npdo, 
                          bool dump) {
  uint8_t cttr = 0;
  CAN_message_t inMsg;
  bool replied = false; 
  uint16_t value16 = 0; 
  union {
    int32_t myInt32 ;            // 4 bytes
    byte myInt32InBytes[4] ;   //  mapped onto the same storage as myFloat
  } myUnion ;
  union {
    int16_t myInt16 ;            // 4 bytes
    byte myInt16InBytes[2] ;   //  mapped onto the same storage as myFloat
  } myUnion16 ;
  //_timetotimeout = micros(); 
  while (!replied /*|| !(micros() - _timetotimeout > _timeout)*/) {
    while (_canport->available()) {
      //Clear out the canbus
      _canport->read(inMsg);
      if (dump)
        hexDumpAll(inMsg, _serialport);
      for (int i = 0; i < npdo; i++) {
        if(inMsg.id == pdocobid[i]) { 
          // 0 := 0x42 + PDO2 
          // 1 := 0x42 + PDO3 
          // 2 := 0x43 + PDO2
          // 3 := 0x43 + PDO3 
          // Serial.print("pdoid: ");
          // Serial.println(inMsg.id, HEX);
          switch(i) {
            case 0: 
              value16 = (inMsg.buf[1] << 8) + inMsg.buf[0];
              for (int i= 0; i<4; i++) myUnion.myInt32InBytes[i] = inMsg.buf[i+2];
              cttr++;
              (*status)[0] = value16; 
              (*q)[0] = myUnion.myInt32; 
              break; 
            case 1:
              for (int i= 0; i<2; i++) myUnion16.myInt16InBytes[i] = inMsg.buf[i];
              cttr++;
              (*act_torque)[0] = myUnion16.myInt16; 
              break;
            case 2: 
              value16 = (inMsg.buf[1] << 8) + inMsg.buf[0];
              for (int i= 0; i<4; i++) myUnion.myInt32InBytes[i] = inMsg.buf[i+2];
              cttr++;
              (*status)[1] = value16; 
              (*q)[1] = myUnion.myInt32; 
              break; 
            case 3:
              for (int i= 0; i<2; i++) myUnion16.myInt16InBytes[i] = inMsg.buf[i];
              cttr++;
              (*act_torque)[1] = myUnion16.myInt16; 
              break; 
            case 4: 
              value16 = (inMsg.buf[1] << 8) + inMsg.buf[0];
              for (int i= 0; i<4; i++) myUnion.myInt32InBytes[i] = inMsg.buf[i+2];
              cttr++;
              (*status)[2] = value16; 
              (*q)[2] = myUnion.myInt32; 
              break; 
            case 5:
              for (int i= 0; i<2; i++) myUnion16.myInt16InBytes[i] = inMsg.buf[i];
              cttr++;
              (*act_torque)[2] = myUnion16.myInt16; 
              break; 
          }
        }
      }
      if(cttr == npdo) replied = true;
    }
  }           
  // Serial.print("number pdos: ");
  // Serial.println(cttr);
}

bool CANsmc::waitForReplyCustomPDO1(uint8_t nodeid, uint32_t pdoid, 
		                       uint16_t* number16, bool dump) {
  //  Function to be called after writeToRegister
  // will return a register value of int32
  bool replied = false;
  //uint32_t id = nodeid + WRITE_REQUEST_CAN;
  uint16_t value16 = 0; 
  CAN_message_t inMsg;
  uint32_t coboid = nodeid + pdoid; 
  //_timetotimeout = micros(); 
  while (!replied /*|| !(micros() - _timetotimeout > _timeout)*/) {
    while (_canport->available()) {
      //Clear out the canbus
      _canport->read(inMsg);
      if (dump)
        hexDumpAll(inMsg, _serialport);
      if(inMsg.id == coboid) {
        value16 = (inMsg.buf[1] << 8) + inMsg.buf[0];
        replied = true;
      }
    }
  }
  *number16 = value16; 
  return replied;
}

bool CANsmc::waitForReplyCustomPDO2(uint8_t nodeid, uint32_t pdoid, 
		                       uint16_t* number16, int32_t* number32, 
								           bool dump) {
  //  Function to be called after writeToRegister
  // will return a register value of int32
  bool replied = false;
  //uint32_t id = nodeid + WRITE_REQUEST_CAN;
  int32_t value = 0;
  uint16_t value16 = 0; 
  CAN_message_t inMsg;
  uint32_t coboid = nodeid + pdoid; 
  union {
    int32_t myInt32 ;            // 4 bytes
    byte myInt32InBytes[4] ;   //  mapped onto the same storage as myFloat
  } myUnion ;
  //_timetotimeout = micros(); 
  while (!replied /*|| !(micros() - _timetotimeout > _timeout)*/) {
    while (_canport->available()) {
      //Clear out the canbus
      _canport->read(inMsg);
      if (dump)
        hexDumpAll(inMsg, _serialport);
      if(inMsg.id == coboid) {
        value16 = (inMsg.buf[1] << 8) + inMsg.buf[0];
        for (int i= 0; i<4; i++) myUnion.myInt32InBytes[i] = inMsg.buf[i+2];
        replied = true;  
      }
    }
  }
  *number32 = myUnion.myInt32; 
  *number16 = value16; 
  return replied;
}

bool CANsmc::waitForReplyCustomPDO3(uint8_t nodeid, uint32_t pdoid, 
		                                int16_t* number16, bool dump) {
  //  Function to be called after writeToRegister
  // will return a register value of int32
  bool replied = false;
  //uint32_t id = nodeid + WRITE_REQUEST_CAN;
  int32_t value = 0;
  CAN_message_t inMsg;
  uint32_t coboid = nodeid + pdoid; 
  union {
    int16_t myInt16 ;            // 4 bytes
    byte myInt16InBytes[2] ;   //  mapped onto the same storage as myFloat
  } myUnion ;
  //_timetotimeout = micros(); 
  while (!replied /*|| !(micros() - _timetotimeout > _timeout)*/) {
    while (_canport->available()) {
      //Clear out the canbus
      _canport->read(inMsg);
      if (dump)
        hexDumpAll(inMsg, _serialport);
      if(inMsg.id == coboid) {        
        for (int i= 0; i<2; i++) myUnion.myInt16InBytes[i] = inMsg.buf[i];
        replied = true;  
      }
    }
  }
  *number16 = myUnion.myInt16; 
  return replied;
}
		

bool CANsmc::waitForReplyuInt32(uint8_t nodeid, uint8_t subindex, uint32_t* numberu32, bool dump) {
  //  Function to be called after writeToRegister
  // will return a register value of uint32
  bool replied = false;
  uint32_t value = 0;
  //uint32_t id = nodeid + WRITE_REQUEST_CAN;
  CAN_message_t inMsg;
  //_timetotimeout = micros(); 
  while (!replied /*|| !(micros() - _timetotimeout > _timeout)*/) {
    while (_canport->available()) {
      //Clear out the canbus
      _canport->read(inMsg);
      if (dump)
        hexDumpAll(inMsg, _serialport);

      if (inMsg.buf[0] != CAN_ERROR_RESPONSE) {
        //int byte_num = numByte(inMsg.buf[0]);
        if (/*inMsg.id == id &&*/ inMsg.buf[3] == subindex) {
          reconstructUint32(&value, inMsg);
          replied = true;
          // Serial.println("Received position");
        }
      }
    }
  }
  *numberu32 = value; 
  return replied;
}

// General function to write to register format uint32_t
void  CANsmc::writeToRegister(uint8_t nodeid, uint8_t subindex, uint32_t datas) {
  uint32_t id = nodeid + WRITE_REQUEST_CAN;
  uint8_t len = 8;
  uint8_t d0 = CANWRITE_4BYTE;
  uint8_t d1 = lowByte(object_index_32); //register number for 4 bytes
  uint8_t d2 = highByte(object_index_32);
  uint8_t d3 = subindex;

  uint8_t d4 = datas & 0xFF;
  uint8_t d5 = (datas >> 8) & 0xFF;
  uint8_t d6 = (datas >> 16) & 0xFF;
  uint8_t d7 = (datas >> 24) & 0xFF; //MSB , little endian format
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}

//----------------------------------------------------
// General function to write to register format int32_t
void CANsmc::writeToRegister(uint8_t nodeid, uint8_t subindex, int32_t datas) {
  uint32_t id = nodeid + WRITE_REQUEST_CAN;
  uint8_t len = 8;
  uint8_t d0 = CANWRITE_4BYTE;
  uint8_t d1 = lowByte(object_index_32); //register number for 4 bytes
  uint8_t d2 = highByte(object_index_32);
  uint8_t d3 = subindex;
  uint8_t d4 = datas & 0xFF;
  uint8_t d5 = (datas >> 8) & 0xFF;
  uint8_t d6 = (datas >> 16) & 0xFF;
  uint8_t d7 = (datas >> 24) & 0xFF; //MSB , little endian format
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}
void  CANsmc::writeToRegister(uint8_t nodeid, uint8_t subindex, uint32_t datas,
                                uint16_t objet_index) {
  uint32_t id = nodeid + WRITE_REQUEST_CAN;
  uint8_t len = 8;
  uint8_t d0 = CANWRITE_4BYTE;
  uint8_t d1 = lowByte(objet_index); //register number for 4 bytes
  uint8_t d2 = highByte(objet_index);
  uint8_t d3 = subindex;

  uint8_t d4 = datas & 0xFF;
  uint8_t d5 = (datas >> 8) & 0xFF;
  uint8_t d6 = (datas >> 16) & 0xFF;
  uint8_t d7 = (datas >> 24) & 0xFF; //MSB , little endian format
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}
void  CANsmc::writeToRegister(uint8_t nodeid, uint8_t subindex, uint16_t datas,
                                uint16_t objet_index) {
  uint32_t id = nodeid + WRITE_REQUEST_CAN;
  uint8_t len = 8;
  uint8_t d0 = CANWRITE_2BYTE;
  uint8_t d1 = lowByte(objet_index); //register number for 4 bytes
  uint8_t d2 = highByte(objet_index);
  uint8_t d3 = subindex;

  uint8_t d4 = datas & 0xFF;
  uint8_t d5 = (datas >> 8) & 0xFF;
  uint8_t d6 = 0x00;
  uint8_t d7 = 0x00; //MSB , little endian format
  // CAN_message_t inMsg; 
  // inMsg.id = id; 
  // inMsg.len = len; 
  // inMsg.buf[0] = d0; 
  // inMsg.buf[1] = d1; 
  // inMsg.buf[2] = d2; 
  // inMsg.buf[3] = d3; 
  // inMsg.buf[4] = d4; 
  // inMsg.buf[5] = d5; 
  // inMsg.buf[6] = d6; 
  // inMsg.buf[7] = d7; 
  
  // hexDumpAll(inMsg, &Serial);
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}
void  CANsmc::writeToRegister(uint8_t nodeid, uint8_t subindex, int16_t datas,
                                uint16_t objet_index) {
  uint32_t id = nodeid + WRITE_REQUEST_CAN;
  uint8_t len = 8;
  uint8_t d0 = CANWRITE_2BYTE;
  uint8_t d1 = lowByte(objet_index); //register number for 4 bytes
  uint8_t d2 = highByte(objet_index);
  uint8_t d3 = subindex;

  uint8_t d4 = datas & 0xFF;
  uint8_t d5 = (datas >> 8) & 0xFF;
  uint8_t d6 = 0x00;
  uint8_t d7 = 0x00; //MSB , little endian format
  // CAN_message_t inMsg; 
  // inMsg.id = id; 
  // inMsg.len = len; 
  // inMsg.buf[0] = d0; 
  // inMsg.buf[1] = d1; 
  // inMsg.buf[2] = d2; 
  // inMsg.buf[3] = d3; 
  // inMsg.buf[4] = d4; 
  // inMsg.buf[5] = d5; 
  // inMsg.buf[6] = d6; 
  // inMsg.buf[7] = d7; 
  
  // hexDumpAll(inMsg, &Serial);
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}
void  CANsmc::writeToRegister(uint8_t nodeid, uint8_t subindex, int8_t datas,
                                uint16_t objet_index) {
  uint32_t id = nodeid + WRITE_REQUEST_CAN;
  uint8_t len = 8;
  uint8_t d0 = CANWRITE_1BYTE;
  uint8_t d1 = lowByte(objet_index); //register number for 4 bytes
  uint8_t d2 = highByte(objet_index);
  uint8_t d3 = subindex;

  uint8_t d4 = datas & 0xFF;
  uint8_t d5 = 0x00;
  uint8_t d6 = 0x00;
  uint8_t d7 = 0x00; //MSB , little endian format
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}
void  CANsmc::writeToRegister(uint8_t nodeid, uint8_t subindex, uint8_t datas,
                                uint16_t objet_index) { 
  
  uint32_t id = nodeid + WRITE_REQUEST_CAN;
  uint8_t len = 8;
  uint8_t d0 = CANWRITE_1BYTE;
  uint8_t d1 = lowByte(objet_index); //register number for 4 bytes
  uint8_t d2 = highByte(objet_index);
  uint8_t d3 = subindex;

  uint8_t d4 = datas & 0xFF;
  uint8_t d5 = 0x00;
  uint8_t d6 = 0x00;
  uint8_t d7 = 0x00; //MSB , little endian format
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}
//----------------------------------------------------
// General function to write to register format int32_t
 void CANsmc::writeToRegister(uint8_t nodeid, uint8_t subindex, int32_t datas,
                              uint16_t objet_index) {
  uint32_t id = nodeid + WRITE_REQUEST_CAN;
  uint8_t len = 8;
  uint8_t d0 = CANWRITE_4BYTE;
  uint8_t d1 = lowByte(objet_index); //register number for 4 bytes
  uint8_t d2 = highByte(objet_index);
  uint8_t d3 = subindex;
  uint8_t d4 = datas & 0xFF;
  uint8_t d5 = (datas >> 8) & 0xFF;
  uint8_t d6 = (datas >> 16) & 0xFF;
  uint8_t d7 = (datas >> 24) & 0xFF; //MSB , little endian format
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}
void CANsmc::writeToPDOCustom1(uint8_t nodeid, uint32_t pdoid, 
		                   uint16_t controlword){
  uint32_t id = nodeid + pdoid; 
  uint8_t len = 2; 
  uint8_t d0 = lowByte(controlword); //register number for 4 bytes
  uint8_t d1 = highByte(controlword);
  uint8_t d2 = 0;
  uint8_t d3 = 0;
  uint8_t d4 = 0;
  uint8_t d5 = 0;
  uint8_t d6 = 0;
  uint8_t d7 = 0; //MSB , little endian format
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}
                           
void CANsmc::writeToPDOCustom2(uint8_t nodeid, uint32_t pdoid, 
		                           uint16_t controlword, int32_t datas){
  uint32_t id = nodeid + pdoid;
  uint8_t len = 8;
  uint8_t d0 = lowByte(controlword); //register number for 4 bytes
  uint8_t d1 = highByte(controlword);
  uint8_t d2 = datas & 0xFF;
  uint8_t d3 = (datas >> 8) & 0xFF;
  uint8_t d4 = (datas >> 16) & 0xFF;
  uint8_t d5 = (datas >> 24) & 0xFF; //MSB , little endian format
  uint8_t d6 = 0;
  uint8_t d7 = 0; //MSB , little endian format
  
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}	
// send a read request
void CANsmc::readRequestFromRegister(uint8_t nodeid, uint8_t subindex) {

  uint32_t comobject_id = WRITE_REQUEST_CAN; // This is the R_SDO object id
  uint32_t id = nodeid + comobject_id;
  uint8_t len = 8;
  uint8_t d0 = READ_REQUEST_CAN; //read request
  uint8_t d1 = lowByte(object_index_32);
  uint8_t d2 = highByte(object_index_32); //reguister 2012h in little endian
  uint8_t d3 = subindex; //subindex
  uint8_t d4 = 0; //filling message
  uint8_t d5 = 0;
  uint8_t d6 = 0;
  uint8_t d7 = 0;
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}

void CANsmc::writeToRegisterS(uint32_t syncobid){

  uint32_t comobject_id = syncobid; // This is the R_SDO object id
  uint32_t id = syncobid;
  uint8_t len = 1;
  uint8_t d0 = 0x00; //read request
  uint8_t d1 = 0x00;
  uint8_t d2 = 0x00; //reguister 2012h in little endian
  uint8_t d3 = 0x00; //subindex
  uint8_t d4 = 0; //filling message
  uint8_t d5 = 0;
  uint8_t d6 = 0;
  uint8_t d7 = 0;
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}

void CANsmc::setSyncCOBID(uint8_t nodeid, uint32_t sync_cobid, 
                             bool dump = false) {
  writeToRegister(nodeid, 0x00, sync_cobid, COB_ID_SYNC_message_index);
  waitForReply(nodeid, 0x00, dump);
}

void CANsmc::setPDOTransmissionType(uint8_t nodeid, uint16_t pdo_com_par, 
                            uint8_t type, bool dump){
  uint8_t sub = transmission_type; 
  writeToRegister(nodeid, sub, type, pdo_com_par);
  waitForReply(nodeid, sub, dump);
} 

void CANsmc::setPDOInhibitTime(uint8_t nodeid, uint16_t pdo_com_par, 
                               uint16_t time_, bool dump){
  uint8_t sub = inhibit_time; 
  writeToRegister(nodeid, sub, time_, pdo_com_par);
  waitForReply(nodeid, sub, dump);
} 

void CANsmc::setPDOnumberentries(uint8_t nodeid, uint16_t pdo_map, 
                                 uint8_t number_entries, bool dump){
  uint8_t sub = number_of_entries;
  writeToRegister(nodeid, sub, number_entries, pdo_map);
  waitForReply(nodeid, sub, dump);
 }

void CANsmc::remapPDO(uint8_t nodeid, uint16_t pdo_map, 
                      uint8_t sub_map, uint8_t sub, uint16_t map_to, 
                      int numb_bytes, bool dump){
  uint32_t data; 
  data = (map_to << 16) + (sub << 8); 
  if (numb_bytes == 8) data += 0x8;
  else if (numb_bytes == 16) data += 0x10; 
  else if (numb_bytes == 32) data += 0x20;
  writeToRegister(nodeid, sub_map, data, pdo_map);
  waitForReply(nodeid, sub_map, dump);
}

void CANsmc::deactivatePDO(uint8_t nodeid, uint16_t pdo_com_par, 
		                   uint16_t pdoid, bool dump){
  uint8_t sub = pdo_cobid; 
  uint32_t data = 0x01 << 31; //set bit 31 
  data += pdoid + nodeid; 
  writeToRegister(nodeid, sub, data, pdo_com_par);
  waitForReply(nodeid, sub, dump);
}

void CANsmc::activatePDO(uint8_t nodeid, uint16_t pdo_com_par, 
		                     uint16_t pdoid, bool dump){
  uint8_t sub = pdo_cobid; 
  uint32_t data = 0x00;  //set '0' bit 31 
  data += pdoid + nodeid; 
  writeToRegister(nodeid, sub, data, pdo_com_par);
  waitForReply(nodeid, sub, dump);
}

void CANsmc::readRequestFromRegister(uint8_t nodeid, uint8_t subindex, uint16_t index) {

  uint32_t comobject_id = WRITE_REQUEST_CAN; // This is the R_SDO object id
  uint32_t id = nodeid + comobject_id;
  uint8_t len = 8;
  uint8_t d0 = READ_REQUEST_CAN; //read request
  uint8_t d1 = lowByte(index);
  uint8_t d2 = highByte(index); //reguister 2012h in little endian
  uint8_t d3 = subindex; //subindex
  uint8_t d4 = 0; //filling message
  uint8_t d5 = 0;
  uint8_t d6 = 0;
  uint8_t d7 = 0;
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}

int numByte(uint8_t code) {
  int number = 0;
  switch (code) {
    case CANREAD_1BYTE:
      number = 1;
      break;
    case CANREAD_2BYTE:
      number = 2;
      break;
    case CANREAD_3BYTE:
      number = 3;
      break;
    case CANREAD_4BYTE:
      number = 4;
      break;
  }
  return number;
}


void hexDump(uint8_t dumpLen, uint8_t *bytePtr, Stream* port) {
  uint8_t working;
  uint8_t hex[17] = "0123456789abcdef";
  while ( dumpLen-- ) {
    working = *bytePtr++;
    port->write( hex[ working >> 4 ] );
    port->write( hex[ working & 15 ] );
  }
  port->write('\r');
  port->write('\n');
}

// -------------------------------------------------------------
void hexDumpAll(CAN_message_t msg, Stream* port) {
  port->print(" ID: ");  writeID(msg.id, port);
  port->print(" data: "); hexDump(8, msg.buf, port);
  port->print("  len: "); port->print(msg.len); port->print(" ");
  port->print(" ext: "); port->println(msg.ext);
}
// -------------------------------------------------------------
void writeID(uint32_t id, Stream* port) {
  port->print("0x");
  if (id <= 0xfff)
    port->print("0");
  if (id <= 0xff)
    port->print("0");
  if (id <= 0xf)
    port->print("0");
  port->print(id, HEX);
}

void wrMsg(FlexCAN* CANbus, uint32_t  id, uint8_t len, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7) {
  CAN_message_t msg;
  for (int i = 0; i < 7; i++)
    msg.buf[i] = 0;

  msg.id = id;
  msg.len = len;
  msg.ext = 0;

  msg.buf[0] = d0;
  msg.buf[1] = d1;
  msg.buf[2] = d2;
  msg.buf[3] = d3;
  msg.buf[4] = d4;
  msg.buf[5] = d5;
  msg.buf[6] = d6;
  msg.buf[7] = d7;

  CANbus->write(msg);
}

// General function to write to register format uint32_t
void  writeToRegister(FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex, uint32_t datas) {
  uint32_t id = nodeid + WRITE_REQUEST_CAN;
  uint8_t len = 8;
  uint8_t d0 = CANWRITE_4BYTE;
  uint8_t d1 = lowByte(object_index_32); //register number for 4 bytes
  uint8_t d2 = highByte(object_index_32);
  uint8_t d3 = subindex;

  uint8_t d4 = datas & 0xFF;
  uint8_t d5 = (datas >> 8) & 0xFF;
  uint8_t d6 = (datas >> 16) & 0xFF;
  uint8_t d7 = (datas >> 24) & 0xFF; //MSB , little endian format
  wrMsg(CANbus, id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}
//----------------------------------------------------
// General function to write to register format int32_t
void writeToRegister(FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex, int32_t datas) {
  uint32_t id = nodeid + WRITE_REQUEST_CAN;
  uint8_t len = 8;
  uint8_t d0 = CANWRITE_4BYTE;
  uint8_t d1 = lowByte(object_index_32); //register number for 4 bytes
  uint8_t d2 = highByte(object_index_32);
  uint8_t d3 = subindex;
  uint8_t d4 = datas & 0xFF;
  uint8_t d5 = (datas >> 8) & 0xFF;
  uint8_t d6 = (datas >> 16) & 0xFF;
  uint8_t d7 = (datas >> 24) & 0xFF; //MSB , little endian format
  wrMsg(CANbus, id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}

// send a read request
void readRequestFromRegister(FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex) {

  uint32_t comobject_id = WRITE_REQUEST_CAN; // This is the R_SDO object id
  uint32_t id = nodeid + comobject_id;
  uint8_t len = 8;
  uint8_t d0 = READ_REQUEST_CAN; //read request
  uint8_t d1 = lowByte(object_index_32);
  uint8_t d2 = highByte(object_index_32); //reguister 2012h in little endian
  uint8_t d3 = subindex; //subindex
  uint8_t d4 = 0; //filling message
  uint8_t d5 = 0;
  uint8_t d6 = 0;
  uint8_t d7 = 0;
  wrMsg(CANbus, id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}

void reconstructInt32(int32_t* number32, CAN_message_t inMsg) {
  // Reconstruct from the last 4 bytes of canmessage, little endian format
  // to a signed integer of 32 bits
  for (int i = 0; i < 8; i++ ) {
    if (bitRead(inMsg.buf[4], i)) {
      bitSet(*number32, i);
    }
    else {
      bitClear(*number32, i);
    }
  }
  for (int i = 0; i < 8; i++ ) {
    if (bitRead(inMsg.buf[5], i)) {
      bitSet(*number32, i + 8);
    }
    else {
      bitClear(*number32, i + 8);
    }
  }
  for (int i = 0; i < 8; i++ ) {
    if (bitRead(inMsg.buf[6], i)) {
      bitSet(*number32, i + 16);
    }
    else {
      bitClear(*number32, i + 16);
    }
  }
  for (int i = 0; i < 8; i++ ) {
    if (bitRead(inMsg.buf[7], i)) {
      bitSet(*number32, i + 24);
    }
    else {
      bitClear(*number32, i + 24);
    }
  }
}
// ------------------------------------------------------------
void reconstructUint32(uint32_t* number32, CAN_message_t inMsg) {
  // Reconstruct from the last 4 bytes of canmessage, little endian format
  // to a signed integer of 32 bits
  for (int i = 0; i < 8; i++ ) {
    if (bitRead(inMsg.buf[4], i)) {
      bitSet(*number32, i);
    }
    else {
      bitClear(*number32, i);
    }
  }
  for (int i = 0; i < 8; i++ ) {
    if (bitRead(inMsg.buf[5], i)) {
      bitSet(*number32, i + 8);
    }
    else {
      bitClear(*number32, i + 8);
    }
  }
  for (int i = 0; i < 8; i++ ) {
    if (bitRead(inMsg.buf[6], i)) {
      bitSet(*number32, i + 16);
    }
    else {
      bitClear(*number32, i + 16);
    }
  }
  for (int i = 0; i < 8; i++ ) {
    if (bitRead(inMsg.buf[7], i)) {
      bitSet(*number32, i + 24);
    }
    else {
      bitClear(*number32, i + 24);
    }
  }
}

