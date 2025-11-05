
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Defines -------------------------------------------------------------------*/
// Disabled on 2022.0415    #define T58_T68_T78
#define T130                                                                    // Enabled on 2022.04.15
  
#define DIAGNOSTIC_COMMAND_ID           0x19FFA010                              // Receive   for diagnostic program
#define DIAGNOSTIC_RESPONSE_ID          0x19FFA011                              // Send
#define BOOTLOADER_COMMAND_ID           0x19F00000                              // Receive

#define TRANSMISSION_PACKET_ID          0x19FFB060
#define TRANSMISSION_PACKET1_ID         0x19FFB061
  
#define CAN_RX_PACKET_ID                0x19FFB010  
#define CAN_RX_ANGLE_ID                 0x0CF02980                              // CAN ID: 0CF02980, PGN: F029, SA = 80
#define CAN_RX_GYRO_ID                  0x0CF02A80                              // CAN ID: 0CF02A80, PGN: F02A, SA = 80
  
#define TRANSMISSION_PACKET_TIME        100  
  
  
typedef union {
  uint8_t data[9];
  struct {
    uint8_t  pitchAngleLSB              : 8;                    // Unit = degree, Scale = 1/32768 Degree/bit, Range: -250 to 252, offset = -250
    uint8_t  pitchAngleMID              : 8;
    uint8_t  pitchAngleMSB              : 8;
    
    uint8_t  rollAngleLSB               : 8;                    // Unit = degree, Scale = 1/32768 Degree/bit, Range: -250 to 252, offset = -250
    uint8_t  rollAngleMID               : 8;
    uint8_t  rollAngleMSB               : 8;
    
    uint8_t  pitchAngleCompensation     : 2;                    // 00 = ON, 01 = OFF, 10 = ERROR, 11 = N/A
    uint8_t  pitchAngleFOM              : 2;                    // 00 = Fully functional, 01 = Degraded, 10 = ERROR, 11 = N/A
    uint8_t  rollAngleCompensation      : 2;                    // 00 = ON, 01 = OFF, 10 = ERROR, 11 = N/A
    uint8_t  rollAngleFOM               : 2;                    // 00 = Fully functional, 01 = Degraded, 10 = ERROR, 11 = N/A
    
    uint8_t  latency                    : 8;                    // Unit = ms, Scale: 0.5ms/Bit, Range: 0 to 125, Offset = 0
    
    uint8_t  flag                       : 8;
  };
} canRxAngle_t;

typedef union {
  uint8_t data[9];
  struct {
    // Byte 0
    uint8_t  pitchAngularRateLSB        : 8;                    // Unit = degree/s, Scale = 1/128 degree/s/Bit, Range = -250 to 250.99, Offset = -250
    uint8_t  pitchAngularRateMSB        : 8;
    
    uint8_t  rollAngularRateLSB         : 8;                    // Unit = degree/s, Scale = 1/128 degree/s/Bit, Range = -250 to 250.99, Offset = -250
    uint8_t  rollAngularRateMSB         : 8;

    uint8_t  yawAngularRateLSB          : 8;                    // Unit = degree/s, Scale = 1/128 degree/s/Bit, Range = -250 to 250.99, Offset = -250
    uint8_t  yawAngularRateMSB          : 8;
    
    uint8_t  pitchRateFOM               : 2;                    // 00 = Fully functional, 01 = Degraded, 10 = ERROR, 11 = N/A
    uint8_t  rollRateFOM                : 2;                    // 00 = Fully functional, 01 = Degraded, 10 = ERROR, 11 = N/A
    uint8_t  yawRateFOM                 : 2;                    // 00 = Fully functional, 01 = Degraded, 10 = ERROR, 11 = N/A
    uint8_t  res0                       : 2;
    
    uint8_t  latency                    : 8;                    // Unit = ms, Scale: 0.5ms/Bit, Range: 0 to 125, Offset = 0
    
    uint8_t  flag                       : 8;
  };
} canRxGyro_t;

extern canRxAngle_t                     canRxAngle;
extern canRxAngle_t                     canRxAnglePrevious;
extern canRxGyro_t                      canRxGyro;
extern canRxGyro_t                      canRxGyroPrevious;

void MX_CAN_Init(void);

void can_variable_init();
void can_receive_process();
uint8_t can_transmit_process();


#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */
