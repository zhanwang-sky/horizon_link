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
    hlink_sbus_t *sbus;
    uint8_t rssi;
} hlink_fport_ctrl_t;

typedef struct {
    hlink_fport_ctrl_t *fport_ctrl;
    hlink_sbus_t *sbus;
} hlink_tlv_set_t;

bool hlink_receive_data(uint8_t data);
int hlink_process_frame(hlink_tlv_set_t *tlv_set);

#ifdef __cplusplus
}
#endif // extern "C" {

#endif // __HORIZON_LINK_H
