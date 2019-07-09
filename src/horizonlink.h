#ifndef _HORIZONLINK_H
#define _HORIZONLINK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// publish-subscribe TLVs
typedef struct {
    float component[4];
} horizonlink_quat_t;

typedef struct {
    uint16_t channel[16];
    uint8_t flags;
} horizonlink_sbus_t;

// point-to-point TLVs
typedef struct {
    uint8_t seq;
    uint8_t type; // 0 with data, 1 request/ACK, 2 NACK
} horizonlink_cmd_t;

typedef struct {
    horizonlink_cmd_t cmd;
    float pid[9]; // p,i,d in yaw,pitch,roll
} horizonlink_pid_t;

typedef struct {
    uint8_t msg_type;
    union {
        horizonlink_pid_t pid;
    } p2ptlv_union;
} horizonlink_p2ptlv_union_t;

// STLVs
typedef struct {
    horizonlink_sbus_t *sbus;
    uint8_t rssi;
} horizonlink_fport_ctrl_t;

// TLV set
typedef struct {
    horizonlink_fport_ctrl_t *fport_ctrl;
    horizonlink_quat_t *att_quat;
    horizonlink_sbus_t *sbus;
    horizonlink_pid_t *att_pid;
} horizonlink_tlv_set_t;

// packet processing
int horizonlink_pack(horizonlink_tlv_set_t*);
int horizonlink_unpack(horizonlink_tlv_set_t*);
// raw buffer processing
size_t horizonlink_disperse(uint8_t*, size_t);
bool horizonlink_assemble(uint8_t);

#ifdef __cplusplus
}
#endif // extern "C" {

#endif // _HORIZONLINK_H
