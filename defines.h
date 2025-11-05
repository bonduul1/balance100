
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEFINES_H
#define __DEFINES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#define   BIT_0    0x0001
#define   BIT_1    0x0002
#define   BIT_2    0x0004
#define   BIT_3    0x0008
#define   BIT_4    0x0010
#define   BIT_5    0x0020
#define   BIT_6    0x0040
#define   BIT_7    0x0080

#define   BIT_8    0x0100
#define   BIT_9    0x0200
#define   BIT_A    0x0400
#define   BIT_B    0x0800
#define   BIT_C    0x1000
#define   BIT_D    0x2000
#define   BIT_E    0x4000
#define   BIT_F    0x8000
  
#ifndef OFF
#define OFF     0
#endif
  
#ifndef ON
#define ON      1
#endif

#ifndef FALSE
#define FALSE   0
#endif
  
#ifndef TRUE
#define TRUE    1
#endif
  
#ifdef __cplusplus
}
#endif

#endif /* __DEFINES_H */