#ifndef __CAN_DEVICE_H
#define __CAN_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define CAN_QUEUE_SIZE  32

/* CAN 消息结构体 */
typedef struct {
    uint32_t id;        /* CAN ID */
    uint8_t  ide;       /* 0=标准帧 1=扩展帧 */
    uint8_t  rtr;       /* 0=数据帧 1=远程帧 */
    uint8_t  dlc;       /* 数据长度 0~8 */
    uint8_t  data[8];   /* 数据 */
} CanMsg_t;

struct CAN_Device {
    char *name;
    int (*Init)(struct CAN_Device *pDev);
    int (*Send)(struct CAN_Device *pDev, CanMsg_t *pMsg, int timeout);
    int (*Recv)(struct CAN_Device *pDev, CanMsg_t *pMsg, int timeout);
};

struct CAN_Device *GetCANDevice(char *name);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_DEVICE_H */
