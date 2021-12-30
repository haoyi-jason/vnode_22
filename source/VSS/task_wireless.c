#include "ch.h"
#include "hal.h"
#include "task_wireless.h"
#include "nvm_config.h"
#include "rsi_common_apis.h"

//#ifdef RSI_BT_ENABLE
#include <rsi_bt_apis.h>
#include <rsi_bt_common_apis.h>
#include <rsi_bt_common.h>
#include <rsi_bt_config.h>
#include <rsi_bt.h>
//#endif

//#ifdef RSI_WLAN_ENABLE
#include "rsi_app_wlan.h"
//#endif
#include "rsi_app_bt_spp_slave.h"

uint8_t global_buf[GLOBAL_BUFF_LEN] = {0};

#define NVM_FLAG        0xCD
enum _state{
  STA_NOT_INIT,
  STA_INITIALIZING,
  STA_IDLE,
  STA_CONNECTED,
  STA_DISCONNECTED,
};

static void inotify(io_queue_t *qp){};
static void onotify(io_queue_t *qp){};

struct _runTime{
  uint8_t state;
  thread_t *self;
  thread_reference_t trp;
  uint8_t commType;
  //input_queue_t iq;
  //output_queue_t oq;
  //uint8_t ob[RSI_APP_BUF_SIZE];
  //uint8_t ib[RSI_APP_BUF_SIZE];
  struct{
    uint8_t buffer[512];
    uint16_t size;
    uint8_t buff_in_use;
  }buffer;
  struct{
    uint32_t rsi_app_async_event_map;
    rsi_bt_resp_get_local_name_t local_name;
    uint8_t local_dev_addr[RSI_DEV_ADDR_LEN];
    uint8_t str_conn_bd_addr[18];
  }rsi_bt;
  struct{
    thread_t *rsi_bt;
    thread_t *rsi_wlan;
    thread_t *rsi_driver;
  }rsi_handle;
  struct{
    uint8_t state;
    uint8_t buffer[512];
    uint16_t buflen;
    uint8_t buf_in_use;
    struct{
      uint32_t module_ip_addr;
      uint16_t source_port;
      uint32_t sender_ip_addr;
      uint16_t remote_port;
      uint8_t module_mac_addr[6];
      uint8_t sender_mac_addr[6];
    }device_params;
  }wlan;
  int32_t clientSocket;
};

struct _nvmParam{
  uint8_t flag;
  struct{
    uint8_t wlan_mode;
    char prefix1[16];
    char passwd1[16];
    char prefix2[16];
    char passwd2[16];
    uint16_t connectTimeout;
    uint8_t secType;
    uint8_t pairedInfo[32];
  }wlan;
  struct{
    uint8_t ip[4];
    uint8_t mask[4];
    uint8_t gateway[4];
  }lan;
};

static struct _nvmParam nvmParam,*w_nvmParam=&nvmParam;
static struct _runTime runTime, *w_runTime = &runTime;

static void load_settings()
{
  uint16_t nvmSz = sizeof(nvmParam);
  if(nvmSz > SZ_NVM_CONFIG){
    // error
    while(1);
  }
//  nvm_flash_read(OFFSET_NVM_CONFIG,(uint8_t*)&nvmParam,nvmSz);
  eepromRead(OFFSET_NVM_WIRELESS,nvmSz,(uint8_t*)&nvmParam);
  if(nvmParam.flag != NVM_FLAG){
    nvmParam.flag = NVM_FLAG;
    nvmParam.wlan.connectTimeout = 10;
    memcpy(nvmParam.wlan.prefix1,"VSS-III\0",8);
    memcpy(nvmParam.wlan.prefix2,"VSS-III\0",8);
    memcpy(nvmParam.wlan.passwd1,"53290921\0",9);
    memcpy(nvmParam.wlan.passwd2,"53290921\0",9);
    nvmParam.wlan.secType = 1;
//    nvmParam.nodeParam.activeSensor = SENSOR_ADXL355;
//    nvmParam.nodeParam.commType = COM_IF_BT;
//    nvmParam.nodeParam.opMode = OP_STREAM;
//    
//    nvmParam.adxlParam.fs = 0x1;
//    nvmParam.adxlParam.odr = ADXL355_ODR_500;
//    nvmParam.adxlParam.hpf = 0;
    
//    memcpy((uint8_t*)&nvmParam.moduleParam,(uint8_t*)&module_default, sizeof(module_setting_t));
    //nvm_flash_write(OFFSET_NVM_CONFIG,(uint8_t*)&nvmParam,nvmSz);
    eepromWrite(OFFSET_NVM_WIRELESS,nvmSz,(uint8_t*)&nvmParam);
  }
}

static void save_settings(uint8_t option)
{
  //chSysLock();
//  chSysDisable();
    //nvm_flash_write(OFFSET_NVM_CONFIG,(uint8_t*)&nvmParam,sizeof(nvmParam));
    eepromWrite(OFFSET_NVM_WIRELESS,sizeof(nvmParam),(uint8_t*)&nvmParam);
    //chSysUnlock();
  //  chSysEnable();
}
//! callback functions

//! stations connect notify call back handler in AP mode
void rsi_stations_connect_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
{

  //! increment connected stations count
  //rsi_stations_count++;

}

//! stations disconnect notify call back handler in AP mode
void rsi_stations_disconnect_notify_handler(uint16_t status,  uint8_t *buffer, const uint32_t length)
{
  //! decrement connected stations count
  //rsi_stations_count--;
}

//! packet receive notify call back handler in AP mode
void rsi_packet_receive_notify_handler(uint16_t status, uint8_t *buffer, uint32_t length)
{

  //! check destination mac address of the received packet 
  status = rsi_wlan_check_mac_address(buffer, length);

  if(status != RSI_SUCCESS)
  {
    return;
  }
  else
  {
    if(!runTime.wlan.buf_in_use)
    {
      //! if not in use

      //! copy the buffer to wlan app cb wlan buffer
      //memcpy(runTime.wlan.buffer, buffer, length); 
      memcpy(runTime.wlan.buffer,buffer,length);
      runTime.wlan.buflen = length;
      runTime.wlan.buf_in_use = 1;
      chEvtSignal(runTime.self,EV_WLAN_RECEIVED);
      //! hold length information
//      runTime.wlan.buflen = length;

      //! make buffer in use
//      runTime.wlan.buf_in_use = 1;

      //! raise event to wlan app task
//      runTime.wlan.event_map |= RSI_WLAN_EVENT;


    }
    else
      //!if buffer is in use
    {
      return;
    }


  }
}

void rsi_wlan_app_callbacks_init(void)
{

  //! Initialze station connect notify  call back
  rsi_wlan_register_callbacks(RSI_STATIONS_CONNECT_NOTIFY_CB, rsi_stations_connect_notify_handler);

  //! Initialze station disconnect notify call back
  rsi_wlan_register_callbacks(RSI_STATIONS_DISCONNECT_NOTIFY_CB, rsi_stations_disconnect_notify_handler);

  //! Initialize packet receive notify call back
  rsi_wlan_register_callbacks(RSI_WLAN_DATA_RECEIVE_NOTIFY_CB, rsi_packet_receive_notify_handler);

  //rsi_wlan_register_callbacks(RSI_REMOTE_SOCKET_TERMINATE_CB, remote_socket_terminated);
}
void socket_async_recive(uint32_t sock_no, uint8_t *buffer, uint32_t length)
{
  memcpy(runTime.wlan.buffer,buffer,length);
  runTime.wlan.buflen = length;
  runTime.wlan.buf_in_use = 1;
  chEvtSignal(runTime.self,SOCKET_RX);
}
void  rsi_wlan_sta_app_task(void *p)
{
  msg_t         exec = *(msg_t*)p;
  int32_t       tatus = RSI_SUCCESS;
  int32_t       pkt_type = 0; 
  int32_t       server_socket,new_socket,udp_socket;
  struct        rsi_sockaddr_in server_addr, client_addr,udp_server; 
  int8_t        recv_buffer[128];
  uint32_t      recv_size;
  int32_t       addr_size;
  uint16_t      tcp_keep_alive_time = 1000 ; 
  uint8_t       buffer[128];
  int32_t status;
  //lan_setting_t *lan = (lan_setting_t*)buffer;
  //param_read_ptr(NVM_LAN,buffer,128);

  uint32_t ip=0,mask=0,gw=0;
  
  for(uint8_t i=0;i<4;i++){
    ip <<= 8;
    mask <<=8;
    gw <<= 8;
    ip |= nvmParam.lan.ip[i];
    mask |= nvmParam.lan.mask[i];
    gw |= nvmParam.lan.gateway[i];
  }

//  wireless_param_t *wlan = (wireless_param_t*)buffer;
//  node_param_t *node = (node_param_t*)buffer;
//  param_read_ptr(NVM_NODE,buffer,128);
//  uint8_t commType = node->commType;
//  param_read_ptr(NVM_WLAN,buffer,128);
  
  runTime.self = chThdGetSelfX();
  
  bool bRun = true;
  uint8_t stage = 0;
  uint8_t wlan_role = nvmParam.wlan.wlan_mode;
  if(exec == EXEC_AP){ // AP mode
    stage = 99;
  }
  uint8_t chNo = 0x01; // <<-
  msg_t errCode = MSG_OK;
  rsi_rsp_scan_t *scan = (rsi_rsp_scan_t*)recv_buffer;
  thread_t *udpThread = NULL;
  bool udpActive = false;
  rsi_wlan_app_callbacks_init();
  while(bRun)
  {
    switch(stage){
    case 0:
      status = rsi_wlan_scan((int8_t*)nvmParam.wlan.prefix2,(uint8_t)chNo,scan,128);
      if(status < 0){
        errCode = 0x1;
        chThdResume(&runTime.trp,errCode);
      }else{
        stage = 1;
      }
      break;
    case 1:
      memcpy(recv_buffer,nvmParam.wlan.passwd2,8);
      recv_buffer[8] = 0x0;
      status = rsi_wlan_connect((int8_t*)nvmParam.wlan.prefix2,nvmParam.wlan.secType&0xf,recv_buffer);
      if(status != RSI_SUCCESS){
        errCode = 0x2;
        chThdResume(&runTime.trp,errCode);
      }else{
        stage = 2;
      }
      break;
    case 2:
      if(nvmParam.wlan.wlan_mode == WLAN_STA_DHCP){
        status = rsi_config_ipaddress(RSI_IP_VERSION_4,RSI_DHCP  ,0,0,0,NULL,0,0);
      }else{
        status = rsi_config_ipaddress(RSI_IP_VERSION_4,RSI_STATIC,(int8_t*)&ip,(int8_t*)&mask,(int8_t*)&gw,NULL,0,0);        
      }
      
      if(status != RSI_SUCCESS){
        errCode = 0x2;
        chThdResume(&runTime.trp,errCode);
      }else{
        rsi_rsp_wireless_info_t info;
        rsi_wlan_get(RSI_WLAN_INFO,(uint8_t*)&info,sizeof(info));
        if(nvmParam.wlan.wlan_mode == WLAN_STA_DHCP)
          stage = 3;
        else
          stage = 4;
        chThdResume(&runTime.trp,MSG_OK);
      }
      break;
    case 3: // create udp socket to broadcast message
      if(udpActive){
        eventmask_t evt = chEvtWaitAnyTimeout(SOCKET_RX,TIME_MS2I(1000));
        if(evt & SOCKET_RX){
          // valid connection string
          bool valid = false;
          if(strncmp(runTime.wlan.buffer,"REQUESTLINK",runTime.wlan.buflen) == 0){
            valid = true;
          }
          if(strncmp(runTime.wlan.buffer,nvmParam.wlan.pairedInfo,runTime.wlan.buflen) == 0){
            valid = true;
          }
          if(valid){
            rsi_shutdown(server_socket,0);
            rsi_shutdown(udp_socket,0);
            stage = 4;
          }          
        }else{
          status = rsi_sendto(udp_socket, (int8_t *)recv_buffer, (strlen(recv_buffer)), 0, (struct rsi_sockaddr *)&udp_server, sizeof(udp_server));
          if(status < 0){
            status = rsi_wlan_get_status();
            rsi_shutdown(udp_socket,0);
            errCode = (msg_t)status;
            bRun = false; 
          }
        }        
      }
      else{
        server_socket = rsi_socket_async(AF_INET,SOCK_DGRAM,0,socket_async_recive);
        if(server_socket < 0){
          errCode = stage;
          bRun = false;
          break;
        }
        memset(&server_addr,0,sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(5001);
        status = rsi_bind(server_socket,(struct rsi_sockaddr*)&server_addr,sizeof(server_addr));
        if(status != RSI_SUCCESS){
          status = rsi_wlan_get_status();
          rsi_shutdown(server_socket,0);
          errCode = stage;
          bRun = false;
          break;
        }        

        udp_socket = rsi_socket(AF_INET,SOCK_DGRAM,0);
        if(udp_socket < 0){
          errCode = stage;
          break;
        }
        memset(&udp_server,0,sizeof(udp_server));
        udp_server.sin_family = AF_INET;
        udp_server.sin_port = 5001;
        udp_server.sin_addr.s_addr = 0xFFFFFFFF;//255.255.255.255
        udpActive = true;
      }
      break;
    case 4:
      server_socket = rsi_socket_async(AF_INET,SOCK_STREAM,0,socket_async_recive);
      
      if(server_socket < 0){
        errCode = 0x3;
        break;
      }
      memset(&server_addr,0,sizeof(server_addr));
      server_addr.sin_family = AF_INET;
      server_addr.sin_port = htons(DEVICE_PORT);
      status = rsi_bind(server_socket,(struct rsi_sockaddr*)&server_addr,sizeof(server_addr));
      if(status != RSI_SUCCESS){
        status = rsi_wlan_get_status();
        rsi_shutdown(server_socket,0);
        errCode = 0x3;
        break;
      }        

      status = rsi_listen(server_socket,1);
      if(status != RSI_SUCCESS){
        status = rsi_wlan_get_status();
        rsi_shutdown(server_socket,0);
        errCode = 0x3;
        break;
      }        
      stage = 5;
      break;
    case 5:
      addr_size = sizeof(server_socket);
          //! Socket accept
      new_socket = rsi_accept(server_socket, (struct rsi_sockaddr *)&client_addr, &addr_size);
          
      if(new_socket < 0)
      {
        status = rsi_wlan_get_status();
        if(status != 0xff62){
          rsi_shutdown(server_socket, 0);
          errCode = 0x4;
        }
      }else{
        runTime.clientSocket = new_socket;
//        if(connected_cb){
//          connected_cb(&new_socket);
//        }
        stage = 6;    
      }
      
      break;
    case 6:
      {
        eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
        if(evt & SOCKET_CONNECT){
         // runTime.wlan.socket_evt_map &= ~SOCKET_CONNECT;
        }
        if(evt & SOCKET_DISCONNECTED){
          //runTime.wlan.socket_evt_map &= ~SOCKET_DISCONNECTED;
          //runTime.wlan.state = RSI_WLAN_SOCKET_SERVER_LISTEN_STATE; 
          //chEvtSignal(runTime.mainThread,EV_CLIENT_DISCONNECT);
          stage = 5;
        }
        if(evt & SOCKET_TX){
          //runTime.wlan.socket_evt_map &= ~SOCKET_TX;
          status = rsi_send(new_socket,runTime.wlan.buffer, runTime.wlan.buflen, 0);
          runTime.wlan.buf_in_use = 0;
          // check if fail
          if(status < 0){
           // chEvtSignal(runTime.mainThread,EV_CLIENT_DISCONNECT);
            stage = 5;
          }
        }
        if(evt & SOCKET_RX){
          //runTime.wlan.socket_evt_map &= ~SOCKET_RX;
          //if(dataArrived)
           // dataArrived(&new_socket);
          //else
           // chEvtSignal(runTime.mainThread,EV_CMD_RX);
        }
        if(evt & SOCKET_READ){
//          runTime.wlan.socket_evt_map &= ~SOCKET_RX;
//          if(dataArrived)
//            dataArrived(&new_socket);
//          else
//            chEvtSignal(runTime.mainThread,EV_CMD_RX);
        }
      }
      break;
    case 99: // ap mode
        //! Configure IP 
        status = rsi_config_ipaddress(RSI_IP_VERSION_4, RSI_STATIC, (uint8_t *)&ip, (uint8_t *)&mask, (uint8_t *)&gw, NULL, 0,0);
        if(status != RSI_SUCCESS){
          
        }
        else{
          
        }
        //! Get MAC address of the Access point
        status = rsi_wlan_get(RSI_MAC_ADDRESS, runTime.wlan.device_params.module_mac_addr, MAC_ADDRESS_SIZE);

        if(status != RSI_SUCCESS)
        {
          break;
        }
        else
        {
          //! update wlan application state
          runTime.wlan.state = RSI_WLAN_AP_UP_STATE; 
        }

        //! Start Access point
        int8_t str_bd_addr[18],str[32],psk[32];
        rsi_6byte_dev_address_to_ascii ((int8_t *)str_bd_addr, runTime.wlan.device_params.module_mac_addr);
        str_bd_addr[17] = 0x0;
        uint8_t lenSz;
        uint8_t *wptr = str;
        uint8_t *q = runTime.wlan.device_params.module_mac_addr;
        wptr += sprintf(wptr,"%s",nvmParam.wlan.prefix1);
        uint8_t digits = runTime.commType & 0xf;
        if(digits){
          wptr += sprintf(wptr,"-");
          for(uint8_t i=0;i<digits;i++){
            wptr += sprintf(wptr,"%02X",*q++);
          }
        }
        lenSz = wptr - str;
        if(lenSz > 32)
          str[31] = 0x0;
        else
          str[lenSz] = 0x0;
        memcpy(psk,nvmParam.wlan.passwd1,strlen(nvmParam.wlan.passwd1));
        psk[strlen(nvmParam.wlan.passwd1)] = 0x0;
        if(psk[0] == 0x0){
          sprintf(psk,"53290921\0");
        }
//        status =  rsi_wlan_ap_start((int8_t *)SSID, CHANNEL_NO, SECURITY_TYPE, ENCRYPTION_TYPE, PSK, BEACON_INTERVAL, DTIM_COUNT);
        status =  rsi_wlan_ap_start((int8_t *)str, CHANNEL_NO, SECURITY_TYPE, ENCRYPTION_TYPE, psk, BEACON_INTERVAL, DTIM_COUNT);
        if(status != RSI_SUCCESS)
        {
          break;
        }
        stage = 4;

      break;
    default:
      break;
    }
    if(chThdShouldTerminateX() || (errCode != 0x0)){
      bRun = false;
    }
    
    chThdSleepMilliseconds(5);
  }
  
  chThdExit(errCode);
}





//! Function to check destination mac address of the recieved packet
int32_t rsi_wlan_check_mac_address(uint8_t *buffer, uint32_t length)
{
  rsi_ether_header_t *ether_header;

  ether_header = (rsi_ether_header_t *)buffer;

  if ((memcmp(ether_header->ether_dest, runTime.wlan.device_params.module_mac_addr,MAC_ADDRESS_SIZE) == 0) ||
      (ether_header->ether_dest[0] == 0xFF))
  {
    return RSI_SUCCESS;
  }
  return RSI_FAILURE;

}

int32_t rsi_wlan_set_ipaddress(uint32_t ip_addr, uint32_t network_mask,uint32_t gateway)
{
  //! Configure IP 
  int32_t status = rsi_config_ipaddress(RSI_IP_VERSION_4, RSI_STATIC, (uint8_t *)&ip_addr, (uint8_t *)&network_mask, (uint8_t *)&gateway, NULL, 0,0);
  if(status != RSI_SUCCESS)
  {
    return status;
  }
  //! hold ip address of the Access point in a variable
  runTime.wlan.device_params.module_ip_addr = ip_addr;
  return RSI_SUCCESS;
}


int32_t rsi_wlan_check_packet_type(uint8_t *buffer, uint32_t length)
{
  rsi_arp_packet_t *arp_packet;
  rsi_ip_header_ping_t rsi_ip_header_ping;
  rsi_ether_header_t *ether_header;
  rsi_ip_header_t   *ip_header;
  rsi_udp_header_t   *udp_header;

  ether_header = (rsi_ether_header_t *)buffer;

  if (ether_header->ether_type == RSI_ARP_PACKET)
  {
    /* Setup a pointer to the ARP message.  */
    arp_packet = (rsi_arp_packet_t *)(buffer  + ETHERNET_HEADER_OFFSET);

    /* Determine if the ARP message type is valid.  */
    if (arp_packet->opcode == ARP_OPTION_REQUEST)
    {
      return ARP_OPTION_REQUEST; 
    }
    else if (arp_packet->opcode == ARP_OPTION_RESPONSE)
    {
      return ARP_OPTION_RESPONSE; 
    }
    else
    {
      return RSI_FAILURE; 
    }
  }
  else if (ether_header->ether_type == RSI_IP_PACKET) 
  {
    memcpy((uint8_t *)&rsi_ip_header_ping ,&buffer[14], sizeof(rsi_ip_header_ping_t));
    /* Setup a pointer to the IP message.  */
    ip_header = (rsi_ip_header_t *)(buffer  + ETHERNET_HEADER_OFFSET);

    /* Determine what protocol the current IP datagram is.  */

    if((rsi_bytes4R_to_uint32(ip_header->dstaddr) == runTime.wlan.device_params.module_ip_addr)&&(ip_header->proto == UDP_PROTOCOL))
    {

      udp_header = (rsi_udp_header_t *)(buffer  + IP_HEADER_OFFSET);

//      if(runTime.wlan.device_params.source_port == convert_le_be(udp_header->dest_port))
//      {
//        //! copy sender ip address in a global variable
//        runTime.wlan.device_params.sender_ip_addr = rsi_bytes4R_to_uint32(ip_header->srcaddr); 
//        
//        //! copy remote port in a global variable
//        runTime.wlan.device_params.remote_port = convert_le_be(udp_header->src_port); 
//      }
      return RSI_IP_PACKET; 

    }
    //! Check for IP Packet
    //! Check for ICMP Protocol
    else if (rsi_ip_header_ping.ip_header_word_2[1] == PROTO_ICMP) 
    {
      //rsi_wlan_send_ping_response(runTime.wlan.buffer, runTime.wlan.buflen);
    }
  }
  return RSI_FAILURE; 
}

//! Function to get mac address from arp response
void rsi_wlan_get_mac_address(uint8_t *buffer, uint32_t length)
{
  rsi_arp_packet_t *arp_packet;
  rsi_ether_header_t *ether_header;


  ether_header = (rsi_ether_header_t *)buffer;

  //! Setup a pointer to the ARP message.  
  arp_packet = (rsi_arp_packet_t *)(buffer  + ETHERNET_HEADER_OFFSET);

  //! copy Access point mac address to source address of ARP response packet
  if(memcmp(ether_header->ether_dest, runTime.wlan.device_params.module_mac_addr, MAC_ADDRESS_SIZE)== 0)
  { 
    //! Pick up the sender's physical address from the message.  
    memcpy(runTime.wlan.device_params.sender_mac_addr,(arp_packet->sender_mac_addr),MAC_ADDRESS_SIZE); 
  }

}

void rsi_app_wlan_cb_data_rx(void(*cb)(void *p))
{
  //dataArrived = cb;
}

void rsi_app_wlan_cb_client_connect(void (*cb)(void *p))
{
  //connected_cb = cb;
}

void rsi_app_wlan_cb_client_disconnect(void (*cb)(void *p))
{
  //disconnect_cb = cb;
}

void start_driver_task(void)
{
    rsi_task_create(rsi_wireless_driver_task, "driver_task",256, NULL, NORMALPRIO, &runTime.rsi_handle.rsi_driver);
}
int8_t rsi_wlan_init(void)
{
  int32_t status;
  uint8_t buffer[128];
  //wireless_param_t *wlan = (wireless_param_t*)buffer;
  //param_read_ptr(NVM_WLAN,buffer,128);

  msg_t exec;
  if(nvmParam.wlan.wlan_mode & WLAN_STA){
    status = rsi_wireless_init(RSI_WLAN_CLIENT_MODE, RSI_OPERMODE_WLAN_ONLY);
    if(status != RSI_SUCCESS)
    {
      return -1;
    }
    exec = EXEC_STA;
    rsi_task_create(rsi_wlan_sta_app_task, "wlan_task", RSI_WLAN_TASK_STACK_SIZE, &exec, RSI_WLAN_TASK_PRIORITY, &runTime.rsi_handle.rsi_wlan);
    // lock until thread finishing initial task
    chSysLock();
    msg_t ret = chThdSuspendS(&runTime.trp);
    chSysUnlock();
    if(ret != MSG_OK){
      chThdSleepMilliseconds(100);
      //chThdTerminate(runTime.rsi_handle.rsi_wlan);
      chThdWait(runTime.rsi_handle.rsi_wlan);
//      chThdRelease(runTime.rsi_handle.rsi_wlan);
      runTime.rsi_handle.rsi_wlan = NULL;
      status = rsi_wireless_deinit();
      chThdTerminate(runTime.rsi_handle.rsi_driver);
      chThdWait(runTime.rsi_handle.rsi_driver);
      //chThdRelease(appParam.rsi_handle.rsi_driver);
      runTime.rsi_handle.rsi_driver = NULL;
      chThdSleepMilliseconds(100);
      status = rsi_device_init(RSI_LOAD_IMAGE_I_FW);
      if(status != RSI_SUCCESS)
      {
        return -1;
      }

      //! Task created for Driver task
      start_driver_task();
      status = rsi_wireless_init(RSI_WLAN_AP_MODE, RSI_OPERMODE_WLAN_ONLY);
      if(status != RSI_SUCCESS)
      {
        return -1;
      }
      exec = EXEC_AP;
      rsi_task_create(rsi_wlan_sta_app_task, "wlan_task", RSI_WLAN_TASK_STACK_SIZE, &exec, RSI_WLAN_TASK_PRIORITY, &runTime.rsi_handle.rsi_wlan);
    }
  }
  else{
    exec = EXEC_AP;
    status = rsi_wireless_init(RSI_WLAN_AP_MODE, RSI_OPERMODE_WLAN_ONLY);
    if(status != RSI_SUCCESS)
    {
      return -1;
    }
    rsi_task_create(rsi_wlan_sta_app_task, "wlan_task", RSI_WLAN_TASK_STACK_SIZE, &exec, RSI_WLAN_TASK_PRIORITY, &runTime.rsi_handle.rsi_wlan);
  }
  
}
//* End Wifi Section */

/** BT Section **/
void rsi_bt_app_on_conn (uint16_t resp_status, rsi_bt_event_bond_t *conn_event)
{
  rsi_6byte_dev_address_to_ascii((int8_t*)runTime.rsi_bt.str_conn_bd_addr,conn_event->dev_addr);
  chEvtSignal(runTime.self,EVT_RSI_BT_ON_CONN);
}


void rsi_bt_app_on_pincode_req(uint16_t resp_status, rsi_bt_event_user_pincode_request_t *user_pincode_request)
{
  chEvtSignal(runTime.self,RSI_APP_EVENT_PINCODE_REQ);
}

void rsi_ble_app_on_linkkey_req (uint16_t status, rsi_bt_event_user_linkkey_request_t  *user_linkkey_req)
{
  chEvtSignal(runTime.self,RSI_APP_EVENT_LINKKEY_REQ);
}

int32_t rsi_bt_app_get_rssi(uint8_t *rssi)
{
  return rsi_bt_get_rssi(runTime.rsi_bt.str_conn_bd_addr,rssi);
}

void rsi_ble_app_on_linkkey_save (uint16_t status, rsi_bt_event_user_linkkey_save_t *user_linkkey_save)
{
  memcpy(runTime.buffer.buffer,user_linkkey_save->linkKey, 16);
  runTime.buffer.size = 16;
  chEvtSignal(runTime.self,RSI_APP_EVENT_LINKKEY_SAVE);
}

void rsi_bt_app_on_auth_complete (uint16_t resp_status, rsi_bt_event_auth_complete_t *auth_complete)
{
  chEvtSignal(runTime.self,RSI_APP_EVENT_AUTH_COMPLT);
}

void rsi_bt_app_on_disconn (uint16_t resp_status, rsi_bt_event_disconnect_t *bt_disconnected)
{
  chEvtSignal(runTime.self,RSI_APP_EVENT_DISCONNECTED);
}

void rsi_bt_app_on_spp_connect (uint16_t resp_status, rsi_bt_event_spp_connect_t *spp_connect)
{
  chEvtSignal(runTime.self,RSI_APP_EVENT_SPP_CONN);
}

void rsi_bt_app_on_spp_disconnect (uint16_t resp_status, rsi_bt_event_spp_disconnect_t *spp_disconn)
{
  chEvtSignal(runTime.self,RSI_APP_EVENT_SPP_DISCONN);
}

void rsi_bt_app_on_spp_data_rx (uint16_t resp_status, rsi_bt_event_spp_receive_t *spp_receive)
{
  // copy to SDW1
  chSysLock();
  uint8_t *buf = ibqGetEmptyBufferI(&SDW1.iqueue);
  if(buf != NULL){
    memcpy (buf, spp_receive->data, spp_receive->data_len);
    /* Signaling that data is available in the input queue.*/
    chnAddFlagsI(&SDW1, CHN_INPUT_AVAILABLE);

    /* Posting the filled buffer in the queue.*/
    ibqPostFullBufferI(&SDW1.iqueue, spp_receive->data_len);
  }
  chSysUnlock();
  chEvtSignal(runTime.self,RSI_APP_EVENT_SPP_RX);
}

void rsi_bt_on_passkey_display (uint16_t resp_status, rsi_bt_event_user_passkey_display_t *bt_event_user_passkey_display)
{
  chEvtSignal(runTime.self,RSI_APP_EVENT_SPP_DISCONN);
}

void rsi_bt_on_passkey_request (uint16_t resp_status, rsi_bt_event_user_passkey_request_t *user_passkey_request)
{
  chEvtSignal(runTime.self,RSI_APP_EVENT_SPP_DISCONN);
}

void rsi_bt_on_ssp_complete (uint16_t resp_status, rsi_bt_event_ssp_complete_t *ssp_complete)
{
  chEvtSignal(runTime.self,RSI_APP_EVENT_SPP_DISCONN);
}

void rsi_bt_on_confirm_request (uint16_t resp_status, rsi_bt_event_user_confirmation_request_t *user_confirmation_request)
{
  chEvtSignal(runTime.self,RSI_APP_SSP_CONFIRM_REQ);
}

void rsi_bt_spp_slave (void *p)
{
  (void)p;
  int32_t status = 0;
  int32_t temp_event_map = 0;
  uint8_t str_bd_addr[18] = {0};
  uint8_t eir_data[64] = {2,1,0};

  runTime.self = chThdGetSelfX();
  //! BT register GAP callbacks:
  rsi_bt_gap_register_callbacks(
      NULL, //role_change
      rsi_bt_app_on_conn, 
      NULL, //
      rsi_bt_app_on_disconn,
      NULL,//scan_resp
      NULL,//remote_name_req
      rsi_bt_on_passkey_display,//passkey_display
      NULL,//remote_name_req+cancel
      rsi_bt_on_confirm_request,//confirm req
      rsi_bt_app_on_pincode_req,
      rsi_bt_on_passkey_request,//passkey request
      NULL,//inquiry complete
      rsi_bt_app_on_auth_complete,
      rsi_ble_app_on_linkkey_req,//linkkey request
      rsi_bt_on_ssp_complete,//ssp coplete
      rsi_ble_app_on_linkkey_save,
      NULL, //get services
      NULL,
      NULL,
      NULL); //search service

  //! get the local device address(MAC address).
  status = rsi_bt_get_local_device_address(runTime.rsi_bt.local_dev_addr);
  if(status != RSI_SUCCESS)
  {
    return ;
  }
  rsi_6byte_dev_address_to_ascii ((int8_t *)str_bd_addr, runTime.rsi_bt.local_dev_addr);

  //! set the local device name
  char str[64];
  uint8_t lenSz;
  uint8_t *wptr = str;
  uint8_t *q = runTime.rsi_bt.local_dev_addr;

  wptr += sprintf(wptr,"%s",nvmParam.wlan.prefix1);

  uint8_t digits = runTime.commType & 0xf;
  //digits = 8;
  if(digits){
    wptr += sprintf(wptr,"-");
    for(uint8_t i=0;i<digits;i++){
      wptr += sprintf(wptr,"%02X",*q++);
    }
  }
  lenSz = (uint8_t)wptr - (uint8_t)str;
  str[lenSz] = 0x0;
  
  status = rsi_bt_set_local_name(str);
  if(status != RSI_SUCCESS)
  {
    return ;
  }

  //! get the local device name
  status = rsi_bt_get_local_name(&runTime.rsi_bt.local_name);
  if(status != RSI_SUCCESS)
  {
    return ;
  }
  
  //! prepare Extended Response Data 
  eir_data[3] = lenSz + 1;//strlen (RSI_BT_LOCAL_NAME) + 1;
  eir_data[4] = 9;
  strcpy ((char *)&eir_data[5], str);
  //! set eir data
  rsi_bt_set_eir_data (eir_data, lenSz + 5);
  //! start the discover mode
  status = rsi_bt_start_discoverable();
  if(status != RSI_SUCCESS)
  {
    return ;
  }

  //! start the connectability mode
  status = rsi_bt_set_connectable();
  if(status != RSI_SUCCESS)
  {
    return ;
  }

    // master
//  status = rsi_bt_connect(REMOTE_BD_ADDR);
//  if(status != RSI_SUCCESS)
//    {
//      return status;
//    }

  //***//
  status = rsi_bt_set_ssp_mode(1,0);
  if(status != RSI_SUCCESS)
  {
    return ;
  }
  
  
//  status = rsi_bt_disable_authentication();
//  if(status != RSI_SUCCESS)
//  {
//    return ;
//  }
  
//  rsi_bt_set_local_class_of_device(0);
//  rsi_bt_set_local_class_of_device(0x240404);

  //! initilize the SPP profile
  status = rsi_bt_spp_init ();
  if(status != RSI_SUCCESS)
  {
    return ;
  }

  //! register the SPP profile callback's
  rsi_bt_spp_register_callbacks (rsi_bt_app_on_spp_connect,
                                 rsi_bt_app_on_spp_disconnect,
                                 rsi_bt_app_on_spp_data_rx);

  static uint8_t connected = 0;
  runTime.state = STA_IDLE;
  while(1)
  {
    //! Application main loop
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
    
    //! if any event is received, it will be served.
    if(evt & EVT_RSI_BT_ON_CONN)
    {
      
    }
        
    if(evt & RSI_APP_EVENT_PINCODE_REQ)
    {
      //! pincode request event
      uint8_t *pin_code = (uint8_t *)PIN_CODE;

      //! sending the pincode requet reply
      status = rsi_bt_pincode_request_reply((int8_t *)runTime.rsi_bt.str_conn_bd_addr, pin_code, 1);
      if(status != RSI_SUCCESS)
      {
        return ;
      }
    }
    if(evt & RSI_APP_EVENT_LINKKEY_REQ)
    {
      //! linkkey request event
      if(nvmParam.wlan.pairedInfo[0]==0)
        rsi_bt_linkkey_request_reply ((int8_t *)runTime.rsi_bt.str_conn_bd_addr, NULL, 0);
      else
        rsi_bt_linkkey_request_reply ((int8_t *)runTime.rsi_bt.str_conn_bd_addr, &nvmParam.wlan.pairedInfo[1], 1);
    }
    if(evt & RSI_APP_EVENT_LINKKEY_SAVE)
    {
      //! linkkey save event
      memcpy(&nvmParam.wlan.pairedInfo[1],runTime.buffer.buffer,runTime.buffer.size);
      nvmParam.wlan.pairedInfo[0] = 0x1;
      save_settings(0);
    }
      if(evt & RSI_APP_EVENT_AUTH_COMPLT)
      {
        //! authentication complete event

      }
      if(evt & RSI_APP_EVENT_DISCONNECTED)
      {
        runTime.state = STA_DISCONNECTED;
      }

      if(evt & RSI_APP_EVENT_SPP_CONN)
      {
        runTime.buffer.size = 0;
        runTime.state = STA_CONNECTED;
      }
      if(evt & RSI_APP_EVENT_SPP_DISCONN)
      {
        //! spp disconnected event
        runTime.state = STA_DISCONNECTED;
        memset(runTime.rsi_bt.str_conn_bd_addr,0,18);
      }
      if(evt & RSI_APP_EVENT_SPP_RX)
      {
        // call binshell to parse command
      }
      if(evt & RSI_APP_EVENT_SPP_TX)
      {
          if(runTime.state == STA_CONNECTED){
            size_t n;
            chSysLock();
            uint8_t *buf = obqGetFullBufferI(&SDW1.oqueue,&n);
            status = rsi_bt_spp_transfer (runTime.rsi_bt.str_conn_bd_addr, buf,n);
            chSysUnlock();
          }
      }
      if(evt & RSI_APP_EVENT_PASSKEY_REQUEST)
      {
        rsi_bt_accept_ssp_confirm((int8_t *)runTime.rsi_bt.str_conn_bd_addr);
      }
      if(evt & RSI_APP_EVENT_SSP_COMPLETE)
      {
      }
      if(evt & RSI_APP_EVENT_CONFIRM_REQUEST)
      {
        rsi_bt_accept_ssp_confirm((int8_t *)runTime.rsi_bt.str_conn_bd_addr);

      }
      if(evt & RSI_APP_SSP_CONFIRM_REQ)
      {
        rsi_bt_accept_ssp_confirm((int8_t *)runTime.rsi_bt.str_conn_bd_addr);
      }    
  }

  return ;
}


int8_t rsi_bts_init()
{
  int32_t status;
    status = rsi_wireless_init(RSI_WLAN_AP_MODE, RSI_OPERMODE_WLAN_BT_CLASSIC);
    rsi_task_create(rsi_bt_spp_slave, "bt_task", RSI_BT_TASK_STACK_SIZE, NULL, RSI_BT_TASK_PRIORITY, &runTime.rsi_handle.rsi_bt);
    if(status != RSI_SUCCESS)
    {
      return 0;
    }
}

int32_t task_wireless_init(uint8_t commType)
{
  int32_t status;
  w_nvmParam=&nvmParam;
  w_runTime = &runTime;  
  runTime.commType = commType;
  
  load_settings();
  
  palSetPadMode(GPIOA,2,PAL_MODE_INPUT); // INT
  palSetPadMode(GPIOA,3,PAL_MODE_OUTPUT_PUSHPULL); // RST
  palSetPadMode(GPIOA,0,PAL_MODE_INPUT); // 
  palSetPadMode(GPIOA,1,PAL_MODE_INPUT); // 
  palClearPad(GPIOA,0);
  palClearPad(GPIOA,1);
  
  // toggle reset
  palClearPad(GPIOA,3);
  chThdSleepMilliseconds(100);
  palSetPad(GPIOA,3);
  
  sdwObjectInit(&SDW1);
  
  status = rsi_driver_init(global_buf, GLOBAL_BUFF_LEN);
  
  if((status < 0) || (status > GLOBAL_BUFF_LEN))
  {
    return status;
  }
  
  status = rsi_device_init(RSI_LOAD_IMAGE_I_FW);
  if(status != RSI_SUCCESS)
  {
    return status;
  }

  //! Task created for Driver task
  start_driver_task();
  //rsi_task_create(rsi_wireless_driver_task, "driver_task",RSI_DRIVER_TASK_STACK_SIZE, NULL, RSI_DRIVER_TASK_PRIORITY, &runTime.rsi_handle.rsi_driver);
    
  //iqObjectInit(&runTime.iq,runTime.ib, RSI_APP_BUF_SIZE,inotify,NULL);
  //iqObjectInit(&runTime.oq,runTime.ob, RSI_APP_BUF_SIZE,onotify,NULL);
  if(commType == COMM_USE_WIFI){
    rsi_wlan_init();
  }
  else if(commType == COMM_USE_BT){
    rsi_bts_init();
  }
  return status;
}


/** Interface implementation **/

SerialWLANDriver SDW1;

static SerialWLANConfig wconfig = {
  0x10,
  {0x00,0x00,0x00,0x00},
};

static size_t _write(void *ip, const uint8_t *bp, size_t n)
{
  SerialWLANDriver *sdwp = (SerialWLANDriver*)ip;
  size_t sz = obqWriteTimeout(&((SerialWLANDriver*)ip)->oqueue,bp,n,TIME_INFINITE);
  obqPostFullBuffer(&sdwp->oqueue, sz);
  return sz;
}

static size_t _read(void *ip, uint8_t *bp, size_t n)
{
  return ibqReadTimeout(&((SerialWLANDriver*)ip)->iqueue,bp,n,TIME_INFINITE);
}

static msg_t _put(void *ip, uint8_t b)
{
  return obqPutTimeout(&((SerialWLANDriver*)ip)->oqueue,b,TIME_INFINITE);
}

static msg_t _get(void *ip)
{
  return ibqGetTimeout(&((SerialWLANDriver*)ip)->iqueue,TIME_INFINITE);
}

static msg_t _putt(void *ip, uint8_t b, sysinterval_t timeout)
{
  return obqPutTimeout(&((SerialWLANDriver*)ip)->oqueue,b,timeout);
}

static msg_t _gett(void *ip, sysinterval_t timeout)
{
  return ibqGetTimeout(&((SerialWLANDriver*)ip)->iqueue,timeout);
}

static size_t _writet(void *ip, const uint8_t *bp, size_t n,sysinterval_t timeout)
{
  return obqWriteTimeout(&((SerialWLANDriver*)ip)->oqueue,bp,n,timeout);
}

static size_t _readt(void *ip, uint8_t *bp, size_t n,sysinterval_t timeout)
{
  return ibqReadTimeout(&((SerialWLANDriver*)ip)->iqueue,bp,n,timeout);
}

static msg_t _ctl(void *ip, unsigned int operation, void *arg)
{
  
  return MSG_OK;
}

static const struct SerialWLANDriverVMT vmt = {
  (size_t)0,
  _write,_read,_put,_get,
  _putt,_gett,_writet,_readt,
  _ctl
};

static void ibnotify(io_buffers_queue_t *bqp)
{
  
}

static void obnotify(io_buffers_queue_t *bqp)
{
  SerialWLANDriver *sdwp = bqGetLinkX(bqp);
  
  chEvtSignal(runTime.self,RSI_APP_EVENT_SPP_TX);
  
}


void sdwInit(void){}

void sdwObjectInit(SerialWLANDriver *sdwp)
{
  sdwp->vmt = &vmt;
  osalEventObjectInit(&sdwp->event);
  ibqObjectInit(&sdwp->iqueue, true, sdwp->ib, RSI_APP_BUF_SIZE, 1, ibnotify, sdwp);
  obqObjectInit(&sdwp->oqueue, true, sdwp->ob, RSI_APP_BUF_SIZE, 1, obnotify, sdwp);
}
void sdwStart(SerialWLANDriver *sdwp, const SerialWLANConfig *config)
{
  
}
void sdwStop(void)
{
  
}