#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>

/* Frame constants */
#define PROTO_HEAD1       0xCC
#define PROTO_HEAD2       0x33
#define PROTO_TAIL        0xDD
#define MAX_DATA_LEN      32
#define FRAME_BUF_SIZE    (5 + MAX_DATA_LEN)

/* Command codes */
#define CMD_SET_DUTY      0x01
#define CMD_GET_ENCODER   0x02
#define CMD_ESTOP         0x03
#define CMD_GET_ENC_ALL   0x04
#define CMD_ERROR         0xFF

/* Error codes */
#define ERR_UNKNOWN_CMD   0x01
#define ERR_LEN_MISMATCH  0x02
#define ERR_INVALID_ID    0x03

/* Pack functions: return total frame length */
int proto_pack_set_duty(uint8_t *buf, uint8_t motor_id, int8_t duty);
int proto_pack_duty_ack(uint8_t *buf);
int proto_pack_get_encoder_req(uint8_t *buf, uint8_t motor_id);
int proto_pack_get_encoder_resp(uint8_t *buf, int32_t enc_val);
int proto_pack_estop_req(uint8_t *buf);
int proto_pack_estop_ack(uint8_t *buf);
int proto_pack_get_enc_all_req(uint8_t *buf);
int proto_pack_get_enc_all_resp(uint8_t *buf, const int32_t enc_vals[4]);
int proto_pack_error(uint8_t *buf, uint8_t errcode);

/* Getters: extract fields from a validated frame (frame points to CMD at offset 2) */
uint8_t proto_get_cmd(const uint8_t *frame);
uint8_t proto_get_len(const uint8_t *frame);
const uint8_t *proto_get_data(const uint8_t *frame);

/* Validation: checks HEAD1/HEAD2 at [0]/[1] and TAIL at [4+LEN] */
int proto_validate_frame(const uint8_t *buf, int buf_len);

#endif /* __PROTOCOL_H__ */
