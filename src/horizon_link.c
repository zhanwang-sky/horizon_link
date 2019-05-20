#include <stdint.h>
#include <stdio.h>
#include "horizon_link.h"

#define NR_RX_BUFS (2)
#define MAX_FRAME_LEN (65535)
#define FRAME_MARKER (0x7E)
#define ESCAPE_CHAR  (0x7D)
#define ESCAPE_MASK  (0x20)

typedef uint8_t frame_buf[MAX_FRAME_LEN]; // typedef vector<uint8_t> frame_buf;
frame_buf rx_buf[NR_RX_BUFS]; // vector<frame_buf> rx_buf;
volatile size_t rx_buf_size[NR_RX_BUFS]; // rx_buf[i].size();
const size_t frame_buf_max_size = MAX_FRAME_LEN; // /* frame_buf.max_size(); */

volatile int isr_buf_id = 0; // decltype(rx_buf[isr_buf_id]) = frame_buf
volatile int task_buf_id = 0;

int isr_frame_position = 0; // decltype(frame_buf[isr_frame_position]) = uint8_t

// task
void process_frame(void) {
    task_buf_id = (!task_buf_id) ? 1 : 0;
    return;
}

// ISR
int receive_data(uint8_t data) {
    int notify_new_frame = 0;

    putchar(data);

    if (data == '\r') {
        // ignore CR
        return 0;
    }

    if (data == '\n') {
        notify_new_frame = 1;
    }

    if (notify_new_frame) {
        if (isr_buf_id == task_buf_id) {
            isr_buf_id = (!isr_buf_id) ? 1 : 0;
        } else {
            // processing task is not ready
            notify_new_frame = 0;
        }
    }

    return notify_new_frame;
}
