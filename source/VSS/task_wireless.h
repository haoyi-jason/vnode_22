#ifndef _TASK_WIRELESS_
#define _TASK_WIRELESS_

#define GLOBAL_BUFF_LEN 8000
#define RSI_APP_BUF_SIZE 600
#define RSI_BT_TASK_STACK_SIZE 1024
#define RSI_BT_TASK_PRIORITY   NORMALPRIO
#define WLAN_AP         0x0
#define WLAN_STA        0x10

#define GLOBAL_BUFF_LEN       8000

#define WLAN_STA_STATIC 0x10
#define WLAN_STA_DHCP   0x11

#define EXEC_STA        0x1
#define EXEC_AP         0x2

#define RSI_WLAN_AP_MODE    6
//#define RSI_WLAN_CLIENT_MODE    0
#define RSI_WLAN_TASK_STACK_SIZE  1024
#define RSI_WLAN_TASK_PRIORITY   64
#define RSI_DRIVER_TASK_PRIORITY   NORMALPRIO
#define RSI_DRIVER_TASK_STACK_SIZE  256

// WLAN Events
#define  SOCKET_CONNECT         EVENT_MASK(0)
#define  SOCKET_DISCONNECTED    EVENT_MASK(1)
#define  SOCKET_TX              EVENT_MASK(2)
#define  SOCKET_RX              EVENT_MASK(3)
#define  SOCKET_READ            EVENT_MASK(4)
#define  EV_WLAN_RECEIVED       EVENT_MASK(5)
#define  EV_STA_CONNECTED       EVENT_MASK(6)
#define  EV_STA_DISCONNECTED    EVENT_MASK(7)
// BT Events
//#define EVT_RSI_BT_ON_CONN      EVENT_MASK(10)
//#define EVT_RSI_BT_PINCODE_REQ  EVENT_MASK(11)
//#define EVT_RSI_BT_LINKKEY_REQ  EVENT_MASK(12)
//#define EVT_RSI_BT_LINKKEY_SAVE EVENT_MASK(13)
//#define EVT_RSI_BT_AUTH_CMPT    EVENT_MASK(14)
//#define EVT_RSI_BT_DISCONN      EVENT_MASK(15)
//#define EVT_RSI_BT_CONN         EVENT_MASK(16)
//#define EVT_RSI_BT_SPP_DISCONN  EVENT_MASK(17)
//#define RSI_APP_EVENT_SPP_RX    EVENT_MASK(18)
//#define RSI_APP_EVENT_SPP_TX    EVENT_MASK(19)
//#define RSI_APP_SSP_CONFIRM_REQ EVENT_MASK(20)

#define RSI_APP_EVENT_CONNECTED       EVENT_MASK(0)
#define RSI_APP_EVENT_PINCODE_REQ     EVENT_MASK(1)
#define RSI_APP_EVENT_LINKKEY_SAVE    EVENT_MASK(2)
#define RSI_APP_EVENT_AUTH_COMPLT     EVENT_MASK(3)
#define RSI_APP_EVENT_DISCONNECTED    EVENT_MASK(4)
#define RSI_APP_EVENT_LINKKEY_REQ     EVENT_MASK(5)
#define RSI_APP_EVENT_SPP_CONN        EVENT_MASK(6)
#define RSI_APP_EVENT_SPP_DISCONN     EVENT_MASK(7)
#define RSI_APP_EVENT_SPP_RX          EVENT_MASK(8)
#define RSI_APP_EVENT_SPP_TX          EVENT_MASK(9)
/** ssp related defines********/
#define RSI_APP_EVENT_PASSKEY_DISPLAY EVENT_MASK(10)
#define RSI_APP_EVENT_PASSKEY_REQUEST EVENT_MASK(11)
#define RSI_APP_EVENT_SSP_COMPLETE    EVENT_MASK(12)
#define RSI_APP_EVENT_CONFIRM_REQUEST EVENT_MASK(13)
#define RSI_APP_EVENT_SPP_RX_REMAIN   EVENT_MASK(14)
#define RSI_APP_EVENT_SPP_TXPTR         EVENT_MASK(15)
#define RSI_APP_EVENT_SPP_TXCMD         EVENT_MASK(16)

//! Channel Number
#define CHANNEL_NO             11 

//! Security Type
#define SECURITY_TYPE        RSI_WPA2

//! Encryption Type
#define ENCRYPTION_TYPE      RSI_CCMP

//! Beacon Interval
#define BEACON_INTERVAL        100
//! ARP PACKET 
#define RSI_ARP_PACKET          0x0608

//! IP PACKET
#define RSI_IP_PACKET           0x0008

//! BT event: set when a packet from BT received 
#define RSI_BT_EVENT            BIT(0)

//! WLAN event: set when a  packet from WLAN device received
#define RSI_WLAN_EVENT           BIT(1)

#define RSI_SOCK_EVENT           BIT(2)

//! Application buffers size
//#define RSI_APP_BUF_SIZE        320

#define  ICMP_PING_REPLY    0

#define  ICMP_PING_REQUEST  8

#define  CHECKSUM_LENGTH    2

#define  IP_ADDRESS_SIZE    4

//! Standard Defines

//!   ethernet header offset
#define ETHERNET_HEADER_OFFSET    14

//! IP header offset
#define IP_HEADER_OFFSET         (14 + 20)

//! UDP header offset
#define UDP_HEADER_OFFSET        (14 + 20 + 8)

//!ARP REQUEST CODE
#define ARP_OPTION_REQUEST       0x0100

//! ARP RESPONSE CODE
#define ARP_OPTION_RESPONSE      0x0200

//! ARP Message size
#define ARP_MESSAGE_SIZE         28

//! Protocol type of UDP
#define UDP_PROTOCOL             17

//!Time to live
#define TIME_TO_LIVE             64


#define  MAC_ADDRESS_SIZE   6

#define  PROTO_ICMP         0x1


//! DTIM Count
#define DTIM_COUNT             4  

//! Device port number
#define DEVICE_PORT        5001

//! Remote port number
#define REMOTE_PORT        5001
#define MAC_ADDRESS_SIZE         6

typedef enum{
  COMM_USE_BT = 0x40,
  COMM_USE_WIFI = 0x80,
  COMM_USE_SERIAL = 0x20
}comm_use_t;

typedef struct rsi_ether_header_s 
{
  //! destination mac address
  uint8_t ether_dest[6];

  //! source mac address
  uint8_t ether_source[6];

  //! ethernet type
  uint16_t ether_type;

} rsi_ether_header_t;
//! IP header structure
typedef struct rsi_ip_header_s 
{
  //! version 
  uint8_t ver_ihl;

  //! type of service
  uint8_t tos;

  //! length higher byte
  uint8_t len_hb;

  //! length lower byte
  uint8_t len_lb;

  //! id higher byte
  uint8_t id_hb;

  //! id lower byte
  uint8_t id_lb;

  //! fragmentation higher byte
  uint8_t frag_hb;

  //! fragmentation lower byte
  uint8_t frag_lb;

  //! time to live
  uint8_t ttl;

  //! protocol
  uint8_t proto;

  //! check sum higher byte
  uint8_t csum_hb;

  //! check sum lower byte
  uint8_t csum_lb;

  //! source ip address
  uint8_t srcaddr[4];

  //! destination ip address
  uint8_t dstaddr[4];

} rsi_ip_header_t;

typedef struct rsi_ip_header_ping_s
{
  uint8_t       ip_header_word_0[4];
  uint8_t       ip_header_word_1[4];
  uint8_t       ip_header_word_2[4];
  /* Define the source IP address.  */
  uint8_t       ip_header_source_ip[4];
  /* Define the destination IP address.  */
  uint8_t       ip_header_destination_ip[4];
}rsi_ip_header_ping_t;


typedef struct rsi_eth_header_ping_s
 {
   uint8_t  src_mac_addr[6];
   uint8_t  dst_mac_addr[6];
   uint8_t  pkt_type[2];
 }rsi_eth_header_ping_t;


//! Enumeration for states in applcation 
typedef enum rsi_wlan_app_state_e
{
  RSI_WLAN_INITIAL_STATE              = 0,
  RSI_WLAN_AP_UP_STATE                = 1,
  RSI_WLAN_SOCKET_SERVER_LISTEN_STATE = 2,
  RSI_WLAN_SOCKET_CONNECTED_STATE     = 3

}rsi_wlan_app_state_t;

//! UDP header structure
typedef struct rsi_udp_header_s 
{
  //! source port
  uint16_t src_port;

  //! destination port
  uint16_t dest_port;

  //! payload length
  uint16_t length;

  //! check sum
  uint16_t check_sum;

}rsi_udp_header_t;

//! UDP pkt total header, excluding payload 
typedef struct rsi_udp_pkt_header_s
{
  //! ethernet header
  rsi_ether_header_t ether_header;

  //! ip header
  rsi_ip_header_t  ip_header;

  //! udp header
  rsi_udp_header_t udp_header;

}rsi_udp_pkt_header_t;

//! APR packet structure
typedef struct rsi_arp_packet_s
{
  //! hardware type
  uint16_t hardware_type;

  //! protocol type
  uint16_t protocol_type;

  //! hardware size
  uint8_t  hardware_size;

  //! protocol size
  uint8_t  protocol_size;

  //! protocol opcode
  uint16_t opcode;

  //!sender physical address
  uint8_t  sender_mac_addr[6];

  //! sender ip address
  uint8_t  sender_ip_addr[4];

  //! target physical address
  uint8_t  target_mac_addr[6];

  //! target ip address
  uint8_t  target_ip_addr[4];

}rsi_arp_packet_t;

//! wlan application control block

typedef struct rsi_wlan_device_params_s
{
  //! device static ip address
  uint32_t    module_ip_addr;

  //! udp source port no
  uint16_t source_port;

  //! remote device ip address
  uint32_t    sender_ip_addr;

  //! udp remote port no
  uint16_t remote_port;

  //! module physical address
  uint8_t module_mac_addr[6];

  //! target physical address
  uint8_t sender_mac_addr[6];

}rsi_wlan_device_params_t;

//! wlan application control block
typedef struct rsi_wlan_app_cb_s
{
  //! wlan application state 
  rsi_wlan_app_state_t state;

  //! length of buffer to copy
  //uint32_t bt_pkt_length;

  //! application buffer
  //uint8_t bt_buffer[RSI_APP_BUF_SIZE];

  //! to check application buffer availability
  //uint8_t bt_buf_in_use;

  //! length of buffer to copy
  uint32_t wlan_pkt_length;

  //! application buffer
  uint8_t wlan_buffer[RSI_APP_BUF_SIZE];

  //! to check application buffer availability
  uint8_t wlan_buf_in_use;

  //! application events bit map 
  uint32_t event_map;

  //! wlan device details
  rsi_wlan_device_params_t device_params;
  
  uint32_t socket_evt_map;

}rsi_wlan_app_cb_t;

#define WIRELESS_BUFFER_SIZE    512
//-- try to implement base on baseSequentialChannel class */

typedef struct SerialWLANDriver SerialWLANDriver;

#define _wireless_driver_data   \
  _base_asynchronous_channel_data \
  input_queue_t iqueue; \
  output_queue_t oqueue; \
  uint8_t ib[WIRELESS_BUFFER_SIZE]; \
  uint8_t ob[WIRELESS_BUFFER_SIZE];
  


typedef struct{
  uint8_t commType;
  struct{
    uint8_t mode; // dhcp/static, ap/station
    uint32_t address;
    uint32_t gateway;
    uint32_t netmask;
  }wlan_config;
  struct{
    uint8_t mode; // master/slave
    uint8_t bd_info[32]; // bounding information
  }bt_config;
}SerialWLANConfig;

#define _serial_wlan_driver_data \
  _base_asynchronous_channel_data \
  input_buffers_queue_t iqueue; \
  output_buffers_queue_t oqueue; \
  uint8_t ib[RSI_APP_BUF_SIZE]; \
  uint8_t ob[RSI_APP_BUF_SIZE]; \
  const SerialWLANConfig *config; \
  event_source_t es;

#define _serial_wlan_driver_methods     \
  _base_asynchronous_channel_methods

struct SerialWLANDriverVMT{
  _serial_wlan_driver_methods
};

struct SerialWLANDriver{
  const struct SerialWLANDriverVMT *vmt;
  _serial_wlan_driver_data
};
      
//* External declarations **/
extern SerialWLANDriver SDW1;
void sdwInit(void);
void sdwObjectInit(SerialWLANDriver *sdwp);
void sdwStart(SerialWLANDriver *sdwp, const SerialWLANConfig *config);
void sdwStop(void);
void set_wlan_config(uint8_t *d, uint16_t sz);
uint16_t get_wlan_config(uint8_t *d, uint16_t sz);

void wireless_read_lan_param(uint8_t *d, uint16_t *szRead, uint16_t maxSz);
void wireless_write_lan_param(uint8_t *d, uint16_t szWrite);
void wireless_read_wlan_param(uint8_t *d, uint16_t *szRead, uint16_t maxSz);
void wireless_write_wlan_param(uint8_t *d, uint16_t szWrite);
void wireless_param_load_default();

#endif
