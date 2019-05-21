#include <stddef.h>
#include "horizon_link.h"

// test
#include <stdio.h>

#define NR_RX_BUFS (2)
#define MIN_FRAME_LEN (6)
#define MAX_FRAME_LEN (259)
#define FRAME_MARKER (0x7E)
#define ESCAPE_CHAR  (0x7D)
#define ESCAPE_MASK  (0x20)
#define CRC_VALUE    (0xFF)

typedef uint8_t frame_buf[MAX_FRAME_LEN]; // typedef vector<uint8_t> frame_buf;

frame_buf rx_buf[NR_RX_BUFS]; // vector<frame_buf> rx_buf;
volatile size_t rx_buf_size[NR_RX_BUFS]; // rx_buf[i].size();
const size_t frame_buf_max_size = MAX_FRAME_LEN; // /* frame_buf::max_size(); */

volatile int isr_buf_id = 0; // decltype(rx_buf[isr_buf_id]) => frame_buf
volatile int task_buf_id = 0;

// task
bool validate_checksum(uint8_t *buf, uint8_t len) {
    uint16_t checksum = 0;
    for (uint8_t i = 0; i < len; i++) {
        checksum += buf[i];
    }
    checksum = (checksum & 0xFF) + (checksum >> 8);
    return checksum == CRC_VALUE;
}

void process_frame(void) {
    // test
    for (size_t i = 0; i < rx_buf_size[task_buf_id]; i++) {
        if ((i % 0x10) == 0) {
            printf("\n%08lX: ", i);
        }
        printf("%02X ", rx_buf[task_buf_id][i]);
    }
    printf("\n");

    // validate checksum
    if (!validate_checksum(rx_buf[task_buf_id], rx_buf_size[task_buf_id])) {
        // test
        printf("bad crc\n");
    } else {
        // test
        printf("checksum ok\n");
    }

    // parse TLVs

    task_buf_id = (task_buf_id != 0) ? 0 : 1;

    return;
}

// ISR
bool receive_data(uint8_t data) {
    static int frame_position = 0;
    static bool escaped_character = false;
    bool new_frame_received = false;

    if (FRAME_MARKER == data) {
        if (frame_position < (MIN_FRAME_LEN - 1)) {
            // Head
            // or under size frame, treat as a new frame
            frame_position = 1;
            // TODO: record time
        } else {
            // TODO: check time
            // Tail
            rx_buf_size[isr_buf_id] = frame_position - 1;
            frame_position = 0;
            if (isr_buf_id == task_buf_id) {
                isr_buf_id = (isr_buf_id != 0) ? 0 : 1;
                new_frame_received = true;
            }
        }
        escaped_character = false;
    } else if (frame_position > 0) {
        if (frame_position >= (MAX_FRAME_LEN - 1)) {
            // oversize frame
            frame_position = 0;
        } else {
            if (data == ESCAPE_CHAR) {
                // XXX 7D+7E?
                escaped_character = true;
            } else {
                if (escaped_character) {
                    data ^= ESCAPE_MASK;
                    escaped_character = false;
                }
                rx_buf[isr_buf_id][frame_position - 1] = data;
                frame_position++;
            }
        }
    } else {
        // frame header not recognized, discard
    }

    return new_frame_received;
}
