#include <stdint.h>
#include "horizon_link.h"

// test
#include <stdio.h>

#define MAX_FRAME_LEN (65535)

uint8_t frame[MAX_FRAME_LEN];

int extract_frame(uint8_t data) {
    putchar(data);
    return 0;
}

int process_frame(void) {
    return 0;
}
