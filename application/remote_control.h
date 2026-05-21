#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "main.h"
#define MAX_FRAME_SIZE 36 

#define CRSF_FRAMETYPE_GPS 0x02                      
#define CRSF_FRAMETYPE_BATTERY_SENSOR 0x08           
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14        
#define CRSF_FRAMETYPE_OPENTX_SYNC 0x10        
#define CRSF_FRAMETYPE_RADIO_ID 0x3A             
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16    
#define CRSF_FRAMETYPE_ATTITUDE 0x1E             
#define CRSF_FRAMETYPE_FLIGHT_MODE 0x21            
#define CRSF_FRAMETYPE_DEVICE_PING 0x28             
#define CRSF_FRAMETYPE_DEVICE_INFO 0x29            
#define CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY 0x2B 
#define CRSF_FRAMETYPE_PARAMETER_READ 0x2C        
#define CRSF_FRAMETYPE_PARAMETER_WRITE 0x2D        
#define CRSF_FRAMETYPE_COMMAND 0x32              
#define CRSF_FRAMETYPE_MSP_REQ 0x7A        
#define CRSF_FRAMETYPE_MSP_RESP 0x7B          
#define CRSF_FRAMETYPE_MSP_WRITE 0x7C         
#define CRSF_FRAMETYPE_HEARTBEAT 0x0B              
// ַ
#define CRSF_ADDRESS_BROADCAST 0x00       
#define CRSF_ADDRESS_USB 0x10               
#define CRSF_ADDRESS_TBS_CORE_PNP_PRO 0x80 
#define CRSF_ADDRESS_RESERVED1 0x8A    
#define CRSF_ADDRESS_CURRENT_SENSOR 0xC0   
#define CRSF_ADDRESS_GPS 0xC2              
#define CRSF_ADDRESS_TBS_BLACKBOX 0xC4     
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDRESS_RESERVED2 0xCA        
#define CRSF_ADDRESS_RACE_TAG 0xCC         
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEA 
#define CRSF_ADDRESS_CRSF_RECEIVER 0xEC     
#define CRSF_ADDRESS_CRSF_TRANSMITTER 0xEE 

#define CHANNELS_Frame_Length 0x18 
#define LINK_Frame_Length 0x0C    
typedef struct
{
    // CRSF_FRAMETYPE_RC_CHANNELS_PACKED 
    uint16_t channels[16]; 
    float Left_X;         
    float Left_Y;        
    float Right_X;        
    float Right_Y;         
    float S1;              
    float S2;           
    uint8_t A;            
    uint8_t B;            
    uint8_t C;           
    uint8_t D;           
    uint8_t E;            
    uint8_t F;           

    // CRSF_FRAMETYPE_LINK_STATISTICS 
    uint8_t uplink_RSSI_1;        
    uint8_t uplink_RSSI_2;         
    uint8_t uplink_Link_quality; 
    int8_t uplink_SNR;           
    uint8_t active_antenna;       
    uint8_t rf_Mode;               
    uint8_t uplink_TX_Power;  
    uint8_t downlink_RSSI;    
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;        

    // CRSF_FRAMETYPE_HEARTBEAT 
    uint16_t heartbeat_counter; 

} ELRS_Data;
const ELRS_Data *get_remote_control_point(void);
void ELRS_Init(void);
void ELRS_UARTE_RxCallback(uint16_t Size);
#endif