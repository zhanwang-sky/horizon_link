/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <string.h>
#include "horizon_link.h"
// test
#include <stdio.h>

/* Private definitions -------------------------------------------------------*/
// common
#define _HLINK_NR_RX_BUFS (2)
// hlink frame
#define _HLINK_MIN_TLV_LEN (2)
#define _HLINK_MIN_FRAME_LEN (_HLINK_MIN_TLV_LEN + 3) // 7e T L CRC 7e
#define _HLINK_MAX_FRAME_LEN (256)
#define _HLINK_FRAME_MARKER (0x7E)
#define _HLINK_ESCAPE_CHAR  (0x7D)
#define _HLINK_ESCAPE_MASK  (0x20)
#define _HLINK_CRC_VALUE    (0xFF)
// STLVs
#define _HLINK_STLV_FPORT_CTRL_TYPE (0x19)
#define _HLINK_STLV_FPORT_CTRL_LEN  (26)

/* Private typedef -----------------------------------------------------------*/
// typedef vector<uint8_t> _hlink_frame_buf_t;
typedef uint8_t _hlink_frame_buf_t[_HLINK_MAX_FRAME_LEN];

/* Private variables ---------------------------------------------------------*/
// vector<_hlink_frame_buf_t> _hlink_rx_buf;
static _hlink_frame_buf_t _hlink_rx_buf[_HLINK_NR_RX_BUFS];
// _hlink_rx_buf[i].size();
static volatile size_t _hlink_rx_buf_size[_HLINK_NR_RX_BUFS];
// /* _hlink_frame_buf_t::max_size(); */
static const size_t _hlink_frame_buf_t_max_size = _HLINK_MAX_FRAME_LEN;

// _hlink_rx_buf[_hlink_isr_buf_id]
static volatile size_t _hlink_isr_buf_id = 0;
// _hlink_rx_buf[_hlink_task_buf_id]
static volatile size_t _hlink_task_buf_id = 0;

// for convenience
static hlink_tlv_set_t _hlink_tlv_set_mask;
static int _hlink_nr_tlvs = 0;

/* Functions -----------------------------------------------------------------*/
// task
static void _hlink_decode_sbus(uint8_t *buf, hlink_sbus_t *sbus) {
    printf("decode sbus\n");

    sbus->channel[0] = ((buf[1] << 8) | buf[0]) & 0x7FF;
    sbus->channel[1] = ((buf[2] << 5) | (buf[1] >> 3)) & 0x7FF;
    sbus->channel[2] = ((buf[4] << 10) | (buf[3] << 2) | (buf[2] >> 6)) & 0x7FF;
    sbus->channel[3] = ((buf[5] << 7) | (buf[4] >> 1)) & 0x7FF;
    sbus->channel[4] = ((buf[6] << 4) | (buf[5] >> 4)) & 0x7FF;
    sbus->channel[5] = ((buf[8] << 9) | (buf[7] << 1) | (buf[6] >> 7)) & 0x7FF;
    sbus->channel[6] = ((buf[9] << 6) | (buf[8] >> 2)) & 0x7FF;
    sbus->channel[7] = ((buf[10] << 3) | (buf[9] >> 5)) & 0x7FF;

    sbus->channel[8] = ((buf[12] << 8) | buf[11]) & 0x7FF;
    sbus->channel[9] = ((buf[13] << 5) | (buf[12] >> 3)) & 0x7FF;
    sbus->channel[10] = ((buf[15] << 10) | (buf[14] << 2) | (buf[13] >> 6)) & 0x7FF;
    sbus->channel[11] = ((buf[16] << 7) | (buf[15] >> 1)) & 0x7FF;
    sbus->channel[12] = ((buf[17] << 4) | (buf[16] >> 4)) & 0x7FF;
    sbus->channel[13] = ((buf[19] << 9) | (buf[18] << 1) | (buf[17] >> 7)) & 0x7FF;
    sbus->channel[14] = ((buf[20] << 6) | (buf[19] >> 2)) & 0x7FF;
    sbus->channel[15] = ((buf[21] << 3) | (buf[20] >> 5)) & 0x7FF;

    sbus->flags = buf[22];
}

static size_t _hlink_parse_stlv(uint8_t *buf, size_t frame_len, hlink_tlv_set_t *tlv_set) {
    uint8_t type = buf[0];
    size_t retval = 0;

    printf("parse stlv\n");

    switch (type) {
    case _HLINK_STLV_FPORT_CTRL_TYPE:
        if (buf[1] != 0 || frame_len < _HLINK_STLV_FPORT_CTRL_LEN) {
            break;
        }
        retval = _HLINK_STLV_FPORT_CTRL_LEN;
        if (tlv_set->fport_ctrl != NULL) {
            // record tlv
            _hlink_tlv_set_mask.fport_ctrl = tlv_set->fport_ctrl;
            _hlink_nr_tlvs++;

            _hlink_decode_sbus(buf + 2, tlv_set->fport_ctrl->sbus);
            tlv_set->fport_ctrl->rssi = buf[25];
        }
    default:
        break;
    }

    if (!retval) {
        printf("no fport ctrl\n");
    }

    return retval;
}

static size_t _hlink_parse_tlv(uint8_t *buf, size_t frame_len, hlink_tlv_set_t *tlv_set) {
    printf("parse tlv\n");
    // if (frame_len < _HLINK_MIN_TLV_LEN) { return frame_len; }
    return frame_len;
}

static bool _hlink_validate_checksum(uint8_t *buf, size_t frame_len) {
    uint16_t checksum = 0;
    for (size_t i = 0; i < frame_len; i++) {
        checksum += buf[i];
    }
    checksum = (checksum & 0xFF) + (checksum >> 8);
    return checksum == _HLINK_CRC_VALUE;
}

int hlink_process_frame(hlink_tlv_set_t *tlv_set) {
    uint8_t *buf = _hlink_rx_buf[_hlink_task_buf_id];
    size_t frame_position = 0,
           frame_len = _hlink_rx_buf_size[_hlink_task_buf_id];

    memset(&_hlink_tlv_set_mask, 0, sizeof(_hlink_tlv_set_mask));
    _hlink_nr_tlvs = 0;

    // test
    for (size_t i = 0; i < frame_len; i++) {
        if ((i % 0x10) == 0) {
            if (i != 0) {
                printf("\n");
            }
            printf("%08lX: ", i);
        }
        printf("%02X ", buf[i]);
    }
    printf("\n");

    // validate checksum
    if (_hlink_validate_checksum(buf, frame_len)) {
        // test
        printf("crc ok\n");
        frame_len--; // exclude crc
        frame_position = _hlink_parse_stlv(buf, frame_len, tlv_set);
        while (frame_position < frame_len) {
            frame_position += _hlink_parse_tlv(buf + frame_position, frame_len - frame_position, tlv_set);
        }
    } else {
        // test
        printf("bad crc\n");
    }

    _hlink_task_buf_id = (_hlink_task_buf_id != 0) ? 0 : 1;

    memcpy(tlv_set, &_hlink_tlv_set_mask, sizeof(_hlink_tlv_set_mask));
    return _hlink_nr_tlvs;
}

// ISR
bool hlink_receive_data(uint8_t data) {
    static size_t frame_position = 0;
    static bool escaped_character = false;
    bool new_frame_received = false;

    if (_HLINK_FRAME_MARKER == data) {
        if (frame_position < (_HLINK_MIN_FRAME_LEN - 1)) {
            // Head
            // or under size frame, treat as a new frame
            frame_position = 1;
            // TODO: record time
        } else {
            // TODO: check time
            // Tail
            _hlink_rx_buf_size[_hlink_isr_buf_id] = frame_position - 1;
            frame_position = 0;
            if (_hlink_isr_buf_id == _hlink_task_buf_id) {
                _hlink_isr_buf_id = (_hlink_isr_buf_id != 0) ? 0 : 1;
                new_frame_received = true;
            }
        }
        escaped_character = false;
    } else if (frame_position > 0) {
        if (frame_position >= (_HLINK_MAX_FRAME_LEN - 1)) {
            // oversize frame
            frame_position = 0;
        } else {
            if (data == _HLINK_ESCAPE_CHAR) {
                // XXX 7D+7E?
                escaped_character = true;
            } else {
                if (escaped_character) {
                    data ^= _HLINK_ESCAPE_MASK;
                    escaped_character = false;
                }
                _hlink_rx_buf[_hlink_isr_buf_id][frame_position - 1] = data;
                frame_position++;
            }
        }
    } else {
        // frame header not recognized, discard
    }

    return new_frame_received;
}
