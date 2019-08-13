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
// STLVs
#define _HORIZONLINK_STLV_FPORT_CTRL_TYPE (0x19)
#define _HORIZONLINK_STLV_FPORT_CTRL_LEN  (24)
// publish-subscribe TLVs
#define _HORIZONLINK_TLV_SBUS_TYPE        (0x21)
#define _HORIZONLINK_TLV_SBUS_LEN         (23)
#define _HORIZONLINK_TLV_STATUS_TYPE      (0x40)
#define _HORIZONLINK_TLV_STATUS_LEN       (3)
#define _HORIZONLINK_TLV_ATT_QUAT_TYPE    (0x41)
#define _HORIZONLINK_TLV_ATT_QUAT_LEN     (16)
#define _HORIZONLINK_TLV_ATT_IMU_TYPE     (0x42)
#define _HORIZONLINK_TLV_ATT_IMU_LEN      (24)
// point-to-point TLVs
#define _HORIZONLINK_TLV_ATT_PID_TYPE     (0xA1)
#define _HORIZONLINK_TLV_ATT_PID_T0_LEN   (37)

typedef struct {
    uint8_t tx_buf[_HORIZONLINK_MAX_FRAME_LEN];
    int len;
} horizonlink_tx_handler_t;

typedef struct {
    uint8_t rx_buf[_HORIZONLINK_MAX_FRAME_LEN];
    int len;
    int pos;
    bool esc;
} horizonlink_rx_handler_t;

// publish-subscribe TLVs
typedef struct {
    uint16_t channel[16];
    uint8_t flags;
} horizonlink_tlv_sbus_t;

typedef struct {
    uint16_t genid;
    uint8_t apseq;
} horizonlink_tlv_status_t;

typedef struct {
    float component[4];
} horizonlink_tlv_quat_t;

typedef struct {
    float gyro[3];
    float accel[3];
} horizonlink_tlv_imu_t;

// point-to-point TLVs
typedef struct {
    uint8_t seq;
    uint8_t type; // 0 with data, 1 request/ACK, 2 NACK
} horizonlink_cmd_t;

typedef struct {
    horizonlink_cmd_t cmd;
    float pid[9]; // p,i,d in yaw,pitch,roll
} horizonlink_p2p_pid_t;

typedef struct {
    uint8_t type;
    union {
        horizonlink_p2p_pid_t pid;
    } u;
} horizonlink_p2ptlv_union_t;

// STLVs
typedef struct {
    horizonlink_tlv_sbus_t *sbus;
    uint8_t rssi;
} horizonlink_fport_ctrl_t;

// TLV set
typedef struct {
    horizonlink_fport_ctrl_t *fport_ctrl;
    horizonlink_tlv_sbus_t *tlv_sbus;
    horizonlink_tlv_status_t *tlv_status;
    horizonlink_tlv_quat_t *tlv_att_quat;
    horizonlink_tlv_imu_t *tlv_att_imu;
    horizonlink_p2p_pid_t *p2p_att_pid;
} horizonlink_tlv_set_t;

// packet processing
int horizonlink_pack(horizonlink_tx_handler_t*, horizonlink_tlv_set_t*);
int horizonlink_pack_p2ptlv(horizonlink_tx_handler_t*, horizonlink_p2ptlv_union_t*);
int horizonlink_unpack(horizonlink_rx_handler_t*, horizonlink_tlv_set_t*);
// raw buffer processing
bool horizonlink_scatter(horizonlink_tx_handler_t*, uint8_t*, int*);
bool horizonlink_gather(horizonlink_rx_handler_t*, uint8_t);

#ifdef __cplusplus
}
#endif // extern "C" {

#endif // _HORIZONLINK_H
