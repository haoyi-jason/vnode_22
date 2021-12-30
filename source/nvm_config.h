#ifndef _NVM_CONFIG_
#define _NVM_CONFIG_

#define SZ_NVM_BOARD    32
#define SZ_NVM_CONFIG   1024 // board config
#define SZ_NVM_USER     256
#define SZ_NVM_DIGCFG   128

#define OFFSET_NVM_BOARD        0
#define OFFSET_NVM_CONFIG       OFFSET_NVM_BOARD + SZ_NVM_BOARD
#define OFFSET_NVM_USER         OFFSET_NVM_CONFIG + SZ_NVM_CONFIG
#define OFFSET_NVM_DIGCFG       OFFSET_NVM_USER + SZ_NVM_USER

#define OFFSET_NVM_WIRELESS     1024*2
/* NVM Structures */

#define OFFSET_MODULE   OFFSET_NVM_CONFIG + 128
typedef struct{
  uint32_t flag;
  uint8_t name[32]; // name of module
  uint32_t verNum;  // firmware version, should copy from application header
  uint32_t serialNum; // vender defined 
  uint8_t vender[32]; // vender defined string
  uint8_t vender2[32];// vender defined string
}module_setting_t;



#endif