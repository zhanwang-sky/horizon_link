#ifndef __HORIZON_LINK_H
#define __HORIZON_LINK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t channel[16];
    uint8_t flags;
} hlink_sbus_t;

typedef struct {
    float component[4];
} hlink_quat_t;

typedef struct {
    hlink_sbus_t *sbus;
    uint8_t rssi;
} hlink_fport_ctrl_t;

typedef struct {
    hlink_fport_ctrl_t *fport_ctrl;
    hlink_sbus_t *sbus;
    hlink_quat_t *quat;
} hlink_tlv_set_t;

// sending
int hlink_make_frame(hlink_tlv_set_t*);
size_t hlink_prepare_tx_buf(uint8_t*, size_t);
// receiving
bool hlink_receive_data(uint8_t);
int hlink_process_frame(hlink_tlv_set_t*);

#ifdef __cplusplus
}
#endif // extern "C" {

#endif // __HORIZON_LINK_H
