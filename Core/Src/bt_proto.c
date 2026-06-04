#include "bt_proto.h"
#include <string.h>

/* 状态机静态变量 */
static BT_ParseState_t s_state = BT_STATE_HEAD;
static uint8_t  s_data_buf[BT_RX_DATA_LEN];
static uint8_t  s_data_idx;
static uint8_t  s_checksum;

bool BT_Parse_Byte(uint8_t byte, BT_RxPacket_t *result)
{
    switch (s_state) {
    case BT_STATE_HEAD:
        if (byte == BT_FRAME_HEAD) {
            s_data_idx = 0;
            s_checksum = 0;
            s_state = BT_STATE_DATA;
        }
        break;

    case BT_STATE_DATA:
        s_data_buf[s_data_idx++] = byte;
        if (s_data_idx >= BT_RX_DATA_LEN) {
            s_state = BT_STATE_CHECKSUM;
        }
        break;

    case BT_STATE_CHECKSUM:
        /* 计算数据校验和 */
        for (uint8_t i = 0; i < BT_RX_DATA_LEN; i++) {
            s_checksum += s_data_buf[i];
        }
        if (byte == s_checksum) {
            s_state = BT_STATE_TAIL;
        } else {
            s_state = BT_STATE_HEAD;  /* 校验失败，丢弃 */
        }
        break;

    case BT_STATE_TAIL:
        if (byte == BT_FRAME_TAIL) {
            /* 完整帧接收成功，解析数据 */
            memcpy(&result->target_speed, &s_data_buf[0],  4);
            memcpy(&result->Kp,           &s_data_buf[4],  4);
            memcpy(&result->Ki,           &s_data_buf[8],  4);
            memcpy(&result->Kd,           &s_data_buf[12], 4);
            s_state = BT_STATE_HEAD;
            return true;
        }
        s_state = BT_STATE_HEAD;  /* 包尾不匹配，丢弃 */
        break;
    }
    return false;
}

uint8_t BT_Pack_Tx(const BT_TxPacket_t *pkt, uint8_t *buf)
{
    buf[0] = BT_FRAME_HEAD;

    /* 速度 (4 bytes, little-endian) */
    memcpy(&buf[1], &pkt->speed, 4);

    /* 校验和（数据部分4字节之和的低8位） */
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 4; i++) {
        checksum += buf[1 + i];
    }
    buf[5] = checksum;

    buf[6] = BT_FRAME_TAIL;
    return BT_TX_PACKET_LEN;
}
