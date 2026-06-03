#ifndef __BT_PROTO_H__
#define __BT_PROTO_H__

#include <stdint.h>

/* Frame constants */
#define BT_HEAD     0xA5
#define BT_TAIL     0x5A
#define BT_RX_LEN   19    /* Host→MCU frame size */
#define BT_TX_LEN   7     /* MCU→Host frame size */

/* FreeRTOS task — replaces old SerialCmdTask */
void BtHandlerTask(void *argument);

#endif /* __BT_PROTO_H__ */
