#include "protocol.h"
#include <string.h>

/* ============ Internal helpers ============ */

static void write_header(uint8_t *buf, uint8_t cmd, uint8_t len)
{
    buf[0] = PROTO_HEAD1;
    buf[1] = PROTO_HEAD2;
    buf[2] = cmd;
    buf[3] = len;
}

static void write_tail(uint8_t *buf, uint8_t data_len)
{
    buf[4 + data_len] = PROTO_TAIL;
}

static void write_int32_le(uint8_t *buf, int32_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

/* ============ Pack functions ============ */

int proto_pack_set_duty(uint8_t *buf, uint8_t motor_id, int8_t duty)
{
    write_header(buf, CMD_SET_DUTY, 2);
    buf[4] = motor_id;
    buf[5] = (uint8_t)duty;
    write_tail(buf, 2);
    return 7;
}

int proto_pack_duty_ack(uint8_t *buf)
{
    write_header(buf, CMD_SET_DUTY, 0);
    write_tail(buf, 0);
    return 5;
}

int proto_pack_get_encoder_req(uint8_t *buf, uint8_t motor_id)
{
    write_header(buf, CMD_GET_ENCODER, 1);
    buf[4] = motor_id;
    write_tail(buf, 1);
    return 6;
}

int proto_pack_get_encoder_resp(uint8_t *buf, int32_t enc_val)
{
    write_header(buf, CMD_GET_ENCODER, 4);
    write_int32_le(&buf[4], enc_val);
    write_tail(buf, 4);
    return 9;
}

int proto_pack_estop_req(uint8_t *buf)
{
    write_header(buf, CMD_ESTOP, 0);
    write_tail(buf, 0);
    return 5;
}

int proto_pack_estop_ack(uint8_t *buf)
{
    write_header(buf, CMD_ESTOP, 0);
    write_tail(buf, 0);
    return 5;
}

int proto_pack_get_enc_all_req(uint8_t *buf)
{
    write_header(buf, CMD_GET_ENC_ALL, 0);
    write_tail(buf, 0);
    return 5;
}

int proto_pack_get_enc_all_resp(uint8_t *buf, const int32_t enc_vals[4])
{
    write_header(buf, CMD_GET_ENC_ALL, 16);
    for (int i = 0; i < 4; i++) {
        write_int32_le(&buf[4 + i * 4], enc_vals[i]);
    }
    write_tail(buf, 16);
    return 21;
}

int proto_pack_error(uint8_t *buf, uint8_t errcode)
{
    write_header(buf, CMD_ERROR, 1);
    buf[4] = errcode;
    write_tail(buf, 1);
    return 6;
}

/* ============ Getters ============ */

uint8_t proto_get_cmd(const uint8_t *frame)
{
    return frame[0];
}

uint8_t proto_get_len(const uint8_t *frame)
{
    return frame[1];
}

const uint8_t *proto_get_data(const uint8_t *frame)
{
    return &frame[2];
}

/* ============ Validation ============ */

int proto_validate_frame(const uint8_t *buf, int buf_len)
{
    if (buf_len < 5) return 0;
    if (buf[0] != PROTO_HEAD1) return 0;
    if (buf[1] != PROTO_HEAD2) return 0;
    uint8_t len = buf[3];
    if (len > MAX_DATA_LEN) return 0;
    if (buf_len < (int)(5 + len)) return 0;
    if (buf[4 + len] != PROTO_TAIL) return 0;
    return 1;
}
