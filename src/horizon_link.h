#ifndef __HORIZON_LINK_H
#define __HORIZON_LINK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool receive_data(uint8_t data);
void process_frame(void);

#ifdef __cplusplus
}
#endif // extern "C" {

#endif // __HORIZON_LINK_H
