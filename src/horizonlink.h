#ifndef _HORIZONLINK_H
#define _HORIZONLINK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Definitions ----------------------------------------------------------------*/
// frame size
#define _HORIZONLINK_TL0_LEN       (2)
#define _HORIZONLINK_MIN_FRAME_LEN (_HORIZONLINK_TL0_LEN + 1) // T L CRC
#define _HORIZONLINK_MAX_FRAME_LEN (128)
// special characters
#define _HORIZONLINK_FRAME_MARKER  (0x7E)
#define _HORIZONLINK_ESCAPE_CHAR   (0x7D)
#define _HORIZONLINK_ESCAPE_MASK   (0x20)
#define _HORIZONLINK_CRC_VALUE     (0xFF)
// STLVs
#define _HORIZONLINK_STLV_FPORT_CTRL_TYPE (0x19)
#define _HORIZONLINK_STLV_FPORT_CTRL_LEN  (24)
// publish-subscribe TLVs
#define _HORIZONLINK_TLV_ATT_QUAT_TYPE    (0x42)
#define _HORIZONLINK_TLV_ATT_QUAT_LEN     (16)
#define _HORIZONLINK_TLV_SBUS_TYPE        (0x43)
#define _HORIZONLINK_TLV_SBUS_LEN         (23)
// point-to-point TLVs
#define _HORIZONLINK_TLV_ATT_PID_TYPE     (0xA1)
#define _HORIZONLINK_TLV_ATT_PID_T0_LEN   (37)
#define _HORIZONLINK_TLV_ATT_PID_Tx_LEN   (2)

typedef struct {
    uint8_t tx_buf[_HORIZONLINK_MAX_FRAME_LEN];
    int len;
} horizonlink_tx_handle_t;

typedef struct {
    uint8_t rx_buf[_HORIZONLINK_MAX_FRAME_LEN];
    int len;
    int pos;
    bool esc;
} horizonlink_rx_handle_t;

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
} horizonlink_subcmd_t;

typedef struct {
    horizonlink_subcmd_t subcmd;
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
int horizonlink_pack(horizonlink_tx_handle_t*, horizonlink_tlv_set_t*);
int horizonlink_unpack(horizonlink_rx_handle_t*, horizonlink_tlv_set_t*);
// raw buffer processing
bool horizonlink_disperse(horizonlink_tx_handle_t*, uint8_t*, int*);
bool horizonlink_assemble(horizonlink_rx_handle_t*, uint8_t);

#ifdef __cplusplus
}
#endif // extern "C" {

#endif // _HORIZONLINK_H
