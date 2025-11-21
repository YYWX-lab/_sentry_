#ifndef RV_PROTOCOL_H
#define RV_PROTOCOL_H

#include "main.h"

#define RV_MCU_TO_PC_HEAD 0x5A
#define RV_PC_TO_MCU_HEAD 0xA5

#define RV_RX_PACK_SIZE sizeof(struct RV_RX_STR) 
#define RV_TX_PACK_SIZE sizeof(struct RV_TX_STR)


typedef struct RV_RX_STR//PC TO MCU
{
  uint8_t header;
  uint8_t tracking : 1;
  uint8_t id : 3;          // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;
	
  float x;
  float y;
  float z;
	
  float yaw;
	
  float vx;
  float vy;
  float vz;
	
  float v_yaw;
	
  float r1;
  float r2;
  float dz;
	
  uint16_t checksum;
}RV_RX_STR;



typedef struct RV_TX_STR//MCU TO PC
{
  uint8_t header;
  uint8_t detect_color : 1;  // 0-red 1-blue
  uint8_t reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum;
}RV_TX_STR;



typedef struct RV_GB_MOVE_STR
{
	
	float yaw_e;
	float pitch_e;
	float s;
	float t;
	
	float last_x;
	float last_y;
	float last_z;
	
}RV_GB_MOVE_STR;




void RV_MCU_TO_PC_PackMake(uint8_t* tx_pack,RV_TX_STR* tx_str);
uint8_t RV_PC_PackAnalysis(uint8_t* rx_pack,RV_RX_STR* rx_str);












#endif
