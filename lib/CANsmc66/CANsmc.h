#ifndef CANsmc_h
#define CANsmc_h
#include <SMC66Registers.h>
#include <FlexCAN.h>
#include <Arduino.h>
#include <Chrono.h>

void wrMsg(
	FlexCAN* CANbus, uint32_t  id, uint8_t len, uint8_t d0, uint8_t d1, 
	uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
void writeToRegister(
	FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex, uint32_t datas);
void writeToRegister(
	FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex, int32_t datas);
void readRequestFromRegister(
	FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex);
void reconstructInt32(int32_t* number32, CAN_message_t inMsg);
void reconstructUint32(uint32_t* number32, CAN_message_t inMsg);
void hexDump(uint8_t dumpLen, uint8_t *bytePtr, Stream* port);
void hexDumpAll(CAN_message_t msg, Stream* port);
void writeID(uint32_t id, Stream* port);
int numByte(uint8_t code);
float convertActualTorquePercent(uint16_t torque);
/* void waitForReply(FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex, bool dump);
int32_t waitForReplyInt32(FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex, bool dump);
 uint32_t waitForReplyuInt32(FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex, bool dump);*/
extern "C" {
// callback function
    typedef void (*externalFunction)(void);
}

class CANsmc
{
	public:
	    CANsmc(FlexCAN* CANbus, Stream* port, uint8_t nodesid[4]); 
		void wrMsg(
			uint32_t  id, uint8_t len, uint8_t d0, uint8_t d1, uint8_t d2, 
			uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
		void configurePDOS(uint8_t nodeid);
        void writeToRegister(uint8_t nodeid, uint8_t subindex, uint32_t datas);
        void writeToRegister(uint8_t nodeid, uint8_t subindex, int32_t datas);
		void writeToRegister(uint8_t nodeid, uint8_t subindex, uint32_t datas,
                             uint16_t objet_index);
        void writeToRegister(uint8_t nodeid, uint8_t subindex, int32_t datas,
                             uint16_t objet_index);
		void writeToRegister(uint8_t nodeid, uint8_t subindex, int8_t datas,
                             uint16_t objet_index);
		void writeToRegister(uint8_t nodeid, uint8_t subindex, uint8_t datas,
                             uint16_t objet_index); 
		void writeToRegister(uint8_t nodeid, uint8_t subindex, uint16_t datas,
                             uint16_t objet_index);
		void writeToRegister(uint8_t nodeid, uint8_t subindex, int16_t datas,
                             uint16_t objet_index);
		void writeToPDOCustom1(uint8_t nodeid, uint32_t pdoid, 
		                       uint16_t controlword);
		void writeToPDOCustom2(uint8_t nodeid, uint32_t pdoid, 
		                       uint16_t controlword, int32_t position);
	    
		void writeToRegisterS(uint32_t syncobid);
		void start_node(uint8_t nodeid);
		void reset_node(uint8_t nodeid);
		void preoperational_node(uint8_t nodeid);

        void readRequestFromRegister(uint8_t nodeid, uint8_t subindex);
		void readRequestFromRegister(uint8_t nodeid, uint8_t subindex,
		                                     uint16_t index);
		void getUInt32FromRegister(
  				uint8_t nodeid, uint16_t object_index, uint8_t subindex, 
  				uint32_t * number32);//, bool dump);
		void getInt32FromRegister(
  				uint8_t nodeid, uint16_t object_index, uint8_t subindex, 
  				int32_t * number32); //, bool dump);
		// PDO related functions 
		void setSyncCOBID(uint8_t nodeid, uint32_t sync_cobid , bool dump);
		void setPDOTransmissionType(uint8_t nodeid, uint16_t pdo_com_par, 
		                            uint8_t type, bool dump = false); 
		void setPDOnumberentries(uint8_t nodeid, uint16_t pdo_map, 
                                 uint8_t number_entries, bool dump = false);
	    void setPDOInhibitTime(uint8_t nodeid, uint16_t pdo_com_par, 
		                       uint16_t time_, bool dump = false);
		void deactivatePDO(uint8_t nodeid, uint16_t pdo_com_par, 
		                   uint16_t pdoid, bool dump = false);
		void activatePDO(uint8_t nodeid, uint16_t pdo_com_par, 
		                 uint16_t pdoid, bool dump = false);
        void remapPDO(uint8_t nodeid, uint16_t pdo_map, 
                      uint8_t sub_map, uint8_t sub, uint16_t map_to, 
                      int numb_bytes, bool dump = false);

        bool waitForReply(uint8_t nodeid, uint8_t subindex, bool dump = false);

		
		bool waitForReplyCustomPDO1(uint8_t nodeid, uint32_t pdoid, 
		                            uint16_t* number16, bool dump = false); 
		bool waitForReplyCustomPDO2(uint8_t nodeid, uint32_t pdoid, 
		                            uint16_t* number16, int32_t* number32, 
								    bool dump = false);
	    bool waitForReplyCustomPDO3(uint8_t nodeid, uint32_t pdoid, 
		                            int16_t* number16, bool dump = false);
		bool waitForPDOSpalletizer(uint32_t pdocobid[20], uint16_t (*status)[3],
                          int32_t (*q)[3], int16_t (*act_torque)[3], int npdo, 
                          bool dump = false);
	    bool waitForPDOSragnar(bool dump = false);
		void updateVar(); // always to be called after waitforpdosragnar
		void computeVel(); // can in desired vel freq
		void setTorquePDO(uint8_t nodeidn, float current_t);
        void setTorquePDOa(uint8_t nodeidn, float current_t, int32_t vsol, uint16_t asol, bool signedv = false);

		bool waitForReplyInt32(
			uint8_t nodeid, uint8_t subindex, int32_t* number32, bool dump = false);
		bool waitForReplyuInt32(
			uint8_t nodeid, uint8_t subindex, uint32_t* numberu32, bool dump = false);
		void setTimeOutCAN(int timeout); 
		void setPSOLL(uint8_t nodeid, int32_t psoll);
		void setVelocityInt32(uint8_t nodeid, int32_t velocity);
		void setAsoll(uint8_t nodeid, int32_t asoll);
		void setVelocityMode(uint8_t nodeid);
		void setPositionMode(uint8_t nodeid);
		void setPassiveMode(uint8_t nodeid);
		void setTorque(uint8_t nodeid, float current_t);
		void checkTorque(uint8_t nodeid, bool dump = false);  
		void clearErrors(uint8_t nodeid, bool dump = false); 
		int32_t getActualVelocity(uint8_t nodeid);
		int32_t getEncoderVelocity(uint8_t nodeid);
		int32_t getFollowError(uint8_t nodeid);
		void setRunCurrent(uint8_t nodeid, float current);
		uint32_t getRunCurrent(uint8_t nodeid);
		void setStandbyCurrent(uint8_t nodeid, float current);
		float getStandbyCurrent(uint8_t nodeid);
		float getVSOLL(uint8_t nodeid);
		float getVelEnc(uint8_t nodeid);
		float getVIST(uint8_t nodeid);
		int32_t getPIST(uint8_t nodeid);
		float getActualTorquePercent(uint8_t nodeid);
		float getTorque(uint8_t nodeid);
		int32_t getEncoderPosition(uint8_t nodeid);
		void jointsToCounts(float joints[4], float (*counts_vector)[4]);
		uint8_t nodesids[4];
		int32_t encoder_pos[4]; 
		int32_t follow_err[4];
		uint32_t status_bits[4];
		int32_t actual_velocity[4];
		int32_t encoder_velocity[4];
		int32_t _last_encoder_pos[4];
		float joints_velocity[4];
		float joints_position[4];
		float offsets[4];
		float actual_torque_percent[4];
		uint16_t actual_torque[4];
		uint32_t run_currents[4]; 
		void readVariables(); 
		float encoder_posd0;
		float encoder_posd1;
		float encoder_posd2;
		float encoder_posd3;
		float offsetscalibration[4];	
		uint32_t ragpdos_tcobids[8]; // this are only transmitting pdos 	
		uint16_t tpdosid[2]; // transmit pdos id 
		int16_t intruncu[4] = {0, 0, 0, 0}; 
		int32_t vsol_com[4] = {0, 0, 0, 0};
		Chrono timeoutcomm;
	private: 
	    Stream* _serialport;  
		FlexCAN* _canport; 
		int _timeout;
		unsigned long _timetotimeout; 
		float _dt; 
		uint32_t _lt; 
};

#endif