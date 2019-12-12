#ifndef SMC66Registers_h
#define SMC66Registers_h
#define MESSENGERBUFFERSIZE 64

#define object_index_32 0x2012
#define object_index_16 0x2014

#define status_index 0x6041
#define position_actual_value_index 0x6064
#define control_index 0x6040
#define targettorque_index 0x6071



#define MODE_REG 2
#define P_SOLL 3
#define V_SOLL 5
#define A_SOLL 6
#define RUN_CURRENT 7
#define STANDBY_CURRENT 9
#define P_IST 10
#define V_IST 12
#define ENCODER_POS 16
#define V_START 13
#define FLWERR 20
#define STATUSBITS 25
#define MIN_P_IST 28
#define MAX_P_IST 30
#define ERR_BITS 35
#define IN_POSITION_WINDOW 33
#define SETUP_BITS 124
#define P_NEW 144
#define CUR_SCALE_MAX 212
#define CUR_SCALE_MIN 213
#define CUR_SCALE_FACTOR 215
#define ACTUAL_TORQUE 217
#define CUR_SCALE_INC 218
#define CUR_SCALE_DEC 219
#define V_ENCODER 253

#define AMPS_TO_TORQUE 0.88571428 //specific to my motor units Nm/A rms
#define COUNTS_PER_REVOLUTION 409600 
#define COUNTS_TO_RAD 0.00001533980787 //dimensions are rad/counts
#define RPM_TO_RADS 0.1047197551 //dimensions are rads / rpm
#define RADS_TO_RPM 9.5492965855 //dimensions are rpm / rads
#define TORQUE_TO_AMPS 1.1290322 // dimensions are A rms/ Nm
#define REDUCTION 4.285714286 // reduction in the robot

#define READ_REQUEST_CAN 0x40
#define WRITE_REQUEST_CAN 0x600

#define CANWRITE_1BYTE 0x2F
#define CANWRITE_2BYTE 0x2B
#define CANWRITE_3BYTE 0x27
#define CANWRITE_4BYTE 0x23

#define CAN_WRITE_RESPONSE 0x60
#define CAN_ERROR_RESPONSE 0x80

#define CANREAD_1BYTE 0x4F
#define CANREAD_2BYTE 0x4B
#define CANREAD_3BYTE 0x47
#define CANREAD_4BYTE 0x43	

#define current_C 5.87 // miliamps
#define C_current 0.17035775 // 1/miliamps
#define VEL_UNITS 0.01 // convertion units to rpm

#define EXT_ENCODER_VEL 172
#define V_ENCODER 253

#define COUNT_TO_PERCENT 0.00048852 //CONVERSION FOR TORQUE 

#define RAD2DEG 57.29577951

#define RX_PDO21_COMM_PAR 0x1414
#define RX_PDO21_MAPPING_PAR 0x1614
#define disable_RX_PDO21_OBID 0x80000200 //object id of PDO 
#define RX_PDO21_OBID 0x00000200 //object id of PDO 

#define RX_PDO22_COMM_PAR 0x1415
#define RX_PDO22_MAPPING_PAR 0x1615
#define disable_RX_PDO22_OBID 0x80000300
#define RX_PDO22_OBID 0x00000300

#define TX_PDO21_COMM_PAR 0x1814
#define TX_PDO21_MAPPING_PAR 0x1A14
#define disable_TX_PDO21_OBID 0x80000180
#define TX_PDO21_OBID 0x00000180

#define TX_PDO22_COMM_PAR 0x1815
#define TX_PDO22_MAPPING_PAR 0x1A15
#define disable_TX_PDO22_OBID 0x80000280
#define TX_PDO22_OBID 0x00000280

//PDO specific sub index 
#define number_of_entries 0x00
#define pdo_cobid 0x01
#define transmission_type 0x02
#define inhibit_time 0x03
#define event_timer 0x05

#define PDO_mapping_entry_1 0x01 
#define PDO_mapping_entry_2 0x02
#define PDO_mapping_entry_3 0x03

// COB ID SYNC MESSAGE
#define COB_ID_SYNC_message_index 0x1005
#define SYNC_RATE 0x1006

// my SYNC COB ID 
#define COBID_SYNC_MESSAGE 0xF9302

#define STORE_PARAMETERS 0x1010
// save signature "e v a s"
// hex 65h 76h 61h 73h 
// #define SAVE_SIGNATURE 0x65766173
#define SAVE_SIGNATURE 0x65766173

#endif

#ifndef PI
#define PI 3.14159265359
#endif