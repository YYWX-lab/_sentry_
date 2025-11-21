#include "RV_protocol.h"
#include "string.h"
#include "crc.h"

void RV_MCU_TO_PC_PackMake(uint8_t* tx_pack,RV_TX_STR* tx_str)
{
//	tx_str->header = RV_MCU_TO_PC_HEAD;
//	
//	tx_str->checksum = get_crc16_check_sum((uint8_t*)tx_str, sizeof(struct RV_TX_STR) - sizeof(tx_str->checksum), 0xFFFF);
//   memcpy(tx_pack, &tx_str, RV_TX_PACK_SIZE);
	
	
	 uint8_t len = 0;
    uint8_t *p = (uint8_t *)(&tx_str->roll);

    tx_pack[0] = RV_MCU_TO_PC_HEAD;
    tx_pack[1] = tx_str->detect_color | (tx_str->reset_tracker << 1) | (tx_str->reserved << 2);
    len += 2;

    tx_pack[len]   = p[0] & 0xff;
    tx_pack[len+1] = p[1] & 0xff;
    tx_pack[len+2] = p[2] & 0xff;
    tx_pack[len+3] = p[3] & 0xff;
    len += 4;

    p = (uint8_t *)(&tx_str->pitch);

    tx_pack[len]   = p[0] & 0xff;
    tx_pack[len+1] = p[1] & 0xff;
    tx_pack[len+2] = p[2] & 0xff;
    tx_pack[len+3] = p[3] & 0xff;
    len += 4;

    p = (uint8_t *)(&tx_str->yaw);

    tx_pack[len]   = p[0] & 0xff;
    tx_pack[len+1] = p[1] & 0xff;
    tx_pack[len+2] = p[2] & 0xff;
    tx_pack[len+3] = p[3] & 0xff;
    len += 4;
	 
	  p = (uint8_t *)(&tx_str->aim_x);

    tx_pack[len]   = p[0] & 0xff;
    tx_pack[len+1] = p[1] & 0xff;
    tx_pack[len+2] = p[2] & 0xff;
    tx_pack[len+3] = p[3] & 0xff;
    len += 4;
	 
	  p = (uint8_t *)(&tx_str->aim_y);

    tx_pack[len]   = p[0] & 0xff;
    tx_pack[len+1] = p[1] & 0xff;
    tx_pack[len+2] = p[2] & 0xff;
    tx_pack[len+3] = p[3] & 0xff;
    len += 4;
	 
	  p = (uint8_t *)(&tx_str->aim_z);

    tx_pack[len]   = p[0] & 0xff;
    tx_pack[len+1] = p[1] & 0xff;
    tx_pack[len+2] = p[2] & 0xff;
    tx_pack[len+3] = p[3] & 0xff;
    len += 4;
	
	 append_crc16_check_sum(tx_pack,len + 2);
    len += 2;
	
	
}



//  1  HEAD ERROR
//  2  CRC  ERROR
//  0  OK
uint8_t RV_PC_PackAnalysis(uint8_t* rx_pack,RV_RX_STR* rx_str)
{
	
	if(rx_pack[0] == RV_PC_TO_MCU_HEAD)
	{
//		
//	 uint16_t calculatedChecksum = get_crc16_check_sum(rx_pack, RV_RX_PACK_SIZE - sizeof(uint16_t), 0xFFFF);
//    uint16_t receivedChecksum = *(uint16_t*)&rx_pack[RV_RX_PACK_SIZE - sizeof(uint16_t)];

//    if (calculatedChecksum != receivedChecksum)
//    {
//        return 2;  // ??????
//    }
	
	
        if (!verify_crc16_check_sum(rx_pack,48))
        {
            return 2;  // ??????
        }
	
		
//	  memcpy(rx_str, rx_pack, RV_RX_PACK_SIZE);
	 
	 
        rx_str->header   = rx_pack[0];
        rx_str->tracking = rx_pack[1] & 0x80;
        rx_str->id			= rx_pack[1] & 0x70;
        rx_str->armors_num = rx_pack[1] & 0x0E;
        rx_str->reserved = rx_pack[1] & 0x01;
        
        
        uint8_t len = 2;
        
        
        uint8_t *fp = (uint8_t *)&rx_str->x;
        fp[0] = rx_pack[len];
        fp[1] = rx_pack[len+1];
        fp[2] = rx_pack[len+2];
        fp[3] = rx_pack[len+3];
        len += 4;
        
        fp = (uint8_t *)&rx_str->y;
        fp[0] = rx_pack[len];
        fp[1] = rx_pack[len+1];
        fp[2] = rx_pack[len+2];
        fp[3] = rx_pack[len+3];
        len += 4;
        
        fp = (uint8_t *)&rx_str->z;
        fp[0] = rx_pack[len];
        fp[1] = rx_pack[len+1];
        fp[2] = rx_pack[len+2];
        fp[3] = rx_pack[len+3];
        len += 4;
        
        fp = (uint8_t *)&rx_str->yaw;
        fp[0] = rx_pack[len];
        fp[1] = rx_pack[len+1];
        fp[2] = rx_pack[len+2];
        fp[3] = rx_pack[len+3];
        len += 4;
        
        
        fp = (uint8_t *)&rx_str->vx;
        fp[0] = rx_pack[len];
        fp[1] = rx_pack[len+1];
        fp[2] = rx_pack[len+2];
        fp[3] = rx_pack[len+3];
        len += 4;
        
        
        
        fp = (uint8_t *)&rx_str->vy;
        fp[0] = rx_pack[len];
        fp[1] = rx_pack[len+1];
        fp[2] = rx_pack[len+2];
        fp[3] = rx_pack[len+3];
        len += 4;
        
        
        
        
        fp = (uint8_t *)&rx_str->vz;
        fp[0] = rx_pack[len];
        fp[1] = rx_pack[len+1];
        fp[2] = rx_pack[len+2];
        fp[3] = rx_pack[len+3];
        len += 4;
        
        
        fp = (uint8_t *)&rx_str->r1;
        fp[0] = rx_pack[len];
        fp[1] = rx_pack[len+1];
        fp[2] = rx_pack[len+2];
        fp[3] = rx_pack[len+3];
        len += 4;
        
        
        fp = (uint8_t *)&rx_str->r2;
        fp[0] = rx_pack[len];
        fp[1] = rx_pack[len+1];
        fp[2] = rx_pack[len+2];
        fp[3] = rx_pack[len+3];
        len += 4;
        
        
        
        
        fp = (uint8_t *)&rx_str->dz;
        fp[0] = rx_pack[len];
        fp[1] = rx_pack[len+1];
        fp[2] = rx_pack[len+2];
        fp[3] = rx_pack[len+3];
        len += 4;
        
	 
	 
	 
		return 0;
	}
	
	return 1;
}

