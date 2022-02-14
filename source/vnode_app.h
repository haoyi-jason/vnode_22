#ifndef _VNODE_APP_
#define _VNODE_APP_

#define SENSOR_ADXL355  0x01
#define SENSOR_BMI160   0x02
#define SENSOR_ISM330   0x04

#define COM_IF_USB      0x10
#define COM_IF_SERIAL   0x20
#define COM_IF_BT       0x40
#define COM_IF_WIFI     0x80

enum {
  OP_STREAM,
  OP_VNODE,
  OP_FNODE,
  OP_OLED,
  OP_LOGSD,
  NOF_OP
};

typedef struct {
  uint8_t opMode;
  uint8_t commType;
  uint8_t activeSensor;
  char log_file_prefix[16];
  uint32_t logFileSize;
}node_param_t;

typedef struct{
  uint8_t power;
  uint8_t odr;
  uint8_t range;
  uint8_t lpf;
}_triaxis_sensor_t;

typedef struct{
  _triaxis_sensor_t accel,gyro;
}imu_config_t;

typedef struct{
  uint8_t fs;
  uint8_t odr;
  uint8_t hpf;
  uint8_t intmask;
}adxl355_cfg_t;

typedef struct{
  uint16_t sampleNumber;
  uint16_t samplePeriodMs;
}time_domain_param_t;

typedef struct{
  uint8_t sla;
  uint8_t dataBits;
  uint8_t parity;
  uint8_t stopBits;
  uint32_t baudrate;
}mbRTUOpts_t;



#endif