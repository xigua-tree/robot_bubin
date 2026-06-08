#ifndef __BT_PROTO_H
#define __BT_PROTO_H

#include <stdint.h>
#include <stdbool.h>

#define BT_FRAME_HEAD  0xA5
#define BT_FRAME_TAIL  0x5A

/* RX 包: A5 + speed(float) + Kp(float) + Ki(float) + Kd(float) + checksum + 5A */
#define BT_RX_DATA_LEN  28   /* 4个float */
#define BT_RX_PACKET_LEN 31  /* 1 + 16 + 1 + 1 */

/* TX 包: A5 + count(i32) + encL(i32) + encR(i32) + speed(f32) + error(f32) + roll(f32) + pitch(f32) + yaw(f32) + checksum + 5A */
#define BT_TX_DATA_LEN  44   /* 4+4+4+4+4 + 4+4+4 = 32 */
#define BT_TX_PACKET_LEN 47  /* 1 + 32 + 1 + 1 = 35 */

/* 接收解析结果 */
typedef struct {
    float target_speed;
    float Kp;
    float Ki;
    float Kd;
    int Vx;
    int Vy;
    float yaw;
} BT_RxPacket_t;

/* 发送数据 */
typedef struct {
    int32_t count;      /* 编码器累计计数 */
    int32_t Encoder_l;  /* 左编码器值 */
    int32_t Encoder_r;  /* 右编码器值 */
    float   speed;      /* 滤波后速度 RPM */
    float   error;      /* 速度误差 */
    float   roll;       /* 横滚角 (度) */
    float   pitch;      /* 俯仰角 (度) */
    float   yaw;        /* 偏航角 (度) */
    int Vx;
    int Vy;
    float wheel_rpm[4];
} BT_TxPacket_t;

/* 帧解析状态机 */
typedef enum {
    BT_STATE_HEAD = 0,
    BT_STATE_DATA,
    BT_STATE_CHECKSUM,
    BT_STATE_TAIL
} BT_ParseState_t;

/* 解析一个字节，返回 true 表示收到完整有效帧 */
bool BT_Parse_Byte(uint8_t byte, BT_RxPacket_t *result);

/* 打包发送帧到 buffer，返回帧长度 */
uint8_t BT_Pack_Tx(const BT_TxPacket_t *pkt, uint8_t *buf);
void blue_setparam_task();

#endif
