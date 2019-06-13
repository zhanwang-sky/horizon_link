/* Includes -------------------------------------------------------------------*/
#include <string.h>
#include "horizon_link.h"
// test
#include <stdio.h>

/* Private definitions --------------------------------------------------------*/
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
#define _HLINK_STLV_FPORT_CTRL_LEN  (24)
// TLVs
#define _HLINK_TLV_SBUS_TYPE (0x41)
#define _HLINK_TLV_SBUS_LEN  (23)
#define _HLINK_TLV_QUAT_TYPE (0x21)
#define _HLINK_TLV_QUAT_LEN  (8)

/* Private typedef ------------------------------------------------------------*/
typedef uint8_t _hlink_frame_buf_t[_HLINK_MAX_FRAME_LEN];

/* Private variables ----------------------------------------------------------*/
static _hlink_frame_buf_t _hlink_rx_buf[_HLINK_NR_RX_BUFS];
static volatile size_t _hlink_rx_buf_size[_HLINK_NR_RX_BUFS];
static _hlink_frame_buf_t _hlink_tx_frame_buf;
static size_t _hlink_tx_frame_buf_size;

static volatile int _hlink_isr_buf_id = 0;
static volatile int _hlink_task_buf_id = 0;

static hlink_tlv_set_t _hlink_tlv_set_mask;
static int _hlink_nr_tlvs = 0;

/* Private functions ----------------------------------------------------------*/
void _hlink_encode_sbus(hlink_sbus_t *sbus, uint8_t *buf) {
    int buf_base, ch_base;

    for (int i = 0; i < 2; i++) {
        buf_base = 11 * i;
        ch_base = 8 * i;
        buf[buf_base + 0] = sbus->channel[ch_base + 0] & 0xFF;
        buf[buf_base + 1] = ((sbus->channel[ch_base + 0] >> 8) & 0x07) | ((sbus->channel[ch_base + 1] << 3) & 0xF8);
        buf[buf_base + 2] = ((sbus->channel[ch_base + 1] >> 5) & 0x3F) | ((sbus->channel[ch_base + 2] << 6) & 0xC0);
        buf[buf_base + 3] = (sbus->channel[ch_base + 2] >> 2) & 0xFF;
        buf[buf_base + 4] = ((sbus->channel[ch_base + 2] >> 10) & 0x01) | ((sbus->channel[ch_base + 3] << 1) & 0xFE);
        buf[buf_base + 5] = ((sbus->channel[ch_base + 3] >> 7) & 0x0F) | ((sbus->channel[ch_base + 4] << 4) & 0xF0);
        buf[buf_base + 6] = ((sbus->channel[ch_base + 4] >> 4) & 0x7F) | ((sbus->channel[ch_base + 5] << 7) & 0x80);
        buf[buf_base + 7] = (sbus->channel[ch_base + 5] >> 1) & 0xFF;
        buf[buf_base + 8] = ((sbus->channel[ch_base + 5] >> 9) & 0x03) | ((sbus->channel[ch_base + 6] << 2) & 0xFC);
        buf[buf_base + 9] = ((sbus->channel[ch_base + 6] >> 6) & 0x1F) | ((sbus->channel[ch_base + 7] << 5) & 0xE0);
        buf[buf_base + 10] = (sbus->channel[ch_base + 7] >> 3) & 0xFF;
    }

    buf[22] = sbus->flags;
}

void _hlink_decode_sbus(uint8_t *buf, hlink_sbus_t *sbus) {
    int ch_base, buf_base;

    printf("decode sbus\n");

    for (int i = 0; i < 2; i++) {
        ch_base = i * 8;
        buf_base = i * 11;
        sbus->channel[ch_base + 0] = ((buf[buf_base + 1] << 8) | buf[buf_base + 0]) & 0x7FF;
        sbus->channel[ch_base + 1] = ((buf[buf_base + 2] << 5) | (buf[buf_base + 1] >> 3)) & 0x7FF;
        sbus->channel[ch_base + 2] = ((buf[buf_base + 4] << 10) | (buf[buf_base + 3] << 2) | (buf[buf_base + 2] >> 6)) & 0x7FF;
        sbus->channel[ch_base + 3] = ((buf[buf_base + 5] << 7) | (buf[buf_base + 4] >> 1)) & 0x7FF;
        sbus->channel[ch_base + 4] = ((buf[buf_base + 6] << 4) | (buf[buf_base + 5] >> 4)) & 0x7FF;
        sbus->channel[ch_base + 5] = ((buf[buf_base + 8] << 9) | (buf[buf_base + 7] << 1) | (buf[buf_base + 6] >> 7)) & 0x7FF;
        sbus->channel[ch_base + 6] = ((buf[buf_base + 9] << 6) | (buf[buf_base + 8] >> 2)) & 0x7FF;
        sbus->channel[ch_base + 7] = ((buf[buf_base + 10] << 3) | (buf[buf_base + 9] >> 5)) & 0x7FF;
    }

    sbus->flags = buf[22];
}

void _hlink_encode_quat(hlink_quat_t *quat, uint8_t *buf) {
    uint16_t *p = (uint16_t*) &quat->component[0];
    for (int i = 0; i < 4; i++) {
        buf[2 * i] = *p & 0xFF;
        buf[2 * i + 1] = *p >> 8;
        p++;
    }
}

void _hlink_decode_quat(uint8_t *buf, hlink_quat_t *quat) {
    uint16_t *p = (uint16_t*) &quat->component[0];
    for (int i = 0; i < 4; i++) {
        *p = buf[2 * i];
        *p |= buf[2 * i + 1] << 8;
        p++;
    }
}

size_t _hlink_parse_stlv(uint8_t *buf, size_t frame_len, hlink_tlv_set_t *tlv_set) {
    uint8_t type = buf[0]; // it is guaranteed that frame_len is at least 2 bytes
    size_t next_tlv = 0; // default value, assume that the frame doesn't contain a STLV

    printf("parse stlv\n");

    switch (type) {
    case _HLINK_STLV_FPORT_CTRL_TYPE:
        if ((buf[1] == 0) && (frame_len >= _HLINK_STLV_FPORT_CTRL_LEN + 2)) {
            next_tlv = frame_len; // mark next tlv position (stop parsing remaining data)
            if (tlv_set->fport_ctrl != NULL) {
                // record this STLV
                _hlink_tlv_set_mask.fport_ctrl = tlv_set->fport_ctrl;
                _hlink_nr_tlvs++;
                // decode
                _hlink_decode_sbus(buf + 2, tlv_set->fport_ctrl->sbus);
                tlv_set->fport_ctrl->rssi = buf[25];
            }
        }
        break;
    default:
        break;
    }

    return next_tlv;
}

size_t _hlink_parse_tlv(uint8_t *buf, size_t frame_len, hlink_tlv_set_t *tlv_set) {
    uint8_t type;
    uint8_t len;
    size_t next_tlv = frame_len;

    printf("parse tlv\n");

    if (frame_len < _HLINK_MIN_TLV_LEN) {
        return frame_len;
    }

    type = buf[0];
    len = buf[1];
    if (frame_len < ((size_t) len + 2)) {
        // under size
        printf("under size\n");
        return frame_len;
    }

    switch (type) {
    case _HLINK_TLV_SBUS_TYPE:
        // len is fixed
        if (len == _HLINK_TLV_SBUS_LEN) {
            next_tlv = len + 2; // mark next tlv position
            if ((tlv_set->sbus != NULL)
                // in case of duplicated TLV
                && (_hlink_tlv_set_mask.sbus == NULL)) {
                // record this tlv
                _hlink_tlv_set_mask.sbus = tlv_set->sbus;
                _hlink_nr_tlvs++;
                // decode
                _hlink_decode_sbus(buf + 2, tlv_set->sbus);
            }
        }
        break;
    case _HLINK_TLV_QUAT_TYPE:
        if (len == _HLINK_TLV_QUAT_LEN) {
            next_tlv = len + 2;
            if ((tlv_set->quat != NULL)
                && (_hlink_tlv_set_mask.quat == NULL)) {
                // recode
                _hlink_tlv_set_mask.quat = tlv_set->quat;
                _hlink_nr_tlvs++;
                // decode
                _hlink_decode_quat(buf + 2, tlv_set->quat);
            }
        }
        break;
    default:
        break;
    }

    return next_tlv;
}

uint8_t _hlink_compute_checksum(uint8_t *buf, size_t len) {
    uint16_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum += buf[i];
    }
    checksum = (checksum & 0xFF) + (checksum >> 8);
    return (0xFF - checksum);
}

bool _hlink_validate_checksum(uint8_t *buf, size_t frame_len) {
    uint16_t checksum = 0;
    for (size_t i = 0; i < frame_len; i++) {
        checksum += buf[i];
    }
    checksum = (checksum & 0xFF) + (checksum >> 8);
    return (checksum == _HLINK_CRC_VALUE);
}

/* Functions -----------------------------------------------------------------*/
int hlink_make_frame(hlink_tlv_set_t *tlv_set) {
    hlink_tlv_set_t tlv_set_mask;
    int nr_tlvs = 0;
    size_t frame_position = 0;

    memset(&tlv_set_mask, 0, sizeof(tlv_set_mask));

    if ((tlv_set->sbus != NULL)
        && ((_HLINK_MAX_FRAME_LEN - 1) >= (frame_position + 2 + _HLINK_TLV_SBUS_LEN))) {
        tlv_set_mask.sbus = tlv_set->sbus;
        nr_tlvs++;
        // encode sbus TLV
        _hlink_tx_frame_buf[frame_position++] = _HLINK_TLV_SBUS_TYPE;
        _hlink_tx_frame_buf[frame_position++] = _HLINK_TLV_SBUS_LEN;
        _hlink_encode_sbus(tlv_set->sbus, _hlink_tx_frame_buf + frame_position);
        frame_position += _HLINK_TLV_SBUS_LEN;
    }

    if ((tlv_set->quat != NULL)
        && ((_HLINK_MAX_FRAME_LEN - 1) >= (frame_position + 2 + _HLINK_TLV_QUAT_LEN))) {
        tlv_set_mask.quat = tlv_set->quat;
        nr_tlvs++;
        // encode quat TLV
        _hlink_tx_frame_buf[frame_position++] = _HLINK_TLV_QUAT_TYPE;
        _hlink_tx_frame_buf[frame_position++] = _HLINK_TLV_QUAT_LEN;
        _hlink_encode_quat(tlv_set->quat, _hlink_tx_frame_buf + frame_position);
        frame_position += _HLINK_TLV_QUAT_LEN;
    }

    if (nr_tlvs > 0) {
        _hlink_tx_frame_buf[frame_position] = _hlink_compute_checksum(_hlink_tx_frame_buf, frame_position);
        frame_position++;
        _hlink_tx_frame_buf_size = frame_position;
    }

    memcpy(tlv_set, &tlv_set_mask, sizeof(tlv_set_mask));
    return nr_tlvs;
}

size_t hlink_prepare_tx_buf(uint8_t *buf, size_t buf_len) {
    size_t si = 0, di = 0;

    if (buf_len < (_hlink_tx_frame_buf_size + 2)) {
        return 0;
    }

    buf[di++] = _HLINK_FRAME_MARKER;

    for (; si < _hlink_tx_frame_buf_size; si++) {
        if ((_hlink_tx_frame_buf[si] == _HLINK_FRAME_MARKER)
            || (_hlink_tx_frame_buf[si] == _HLINK_ESCAPE_CHAR)) {
            if ((di + 2) >= buf_len) {
                return 0;
            }
            buf[di++] = _HLINK_ESCAPE_CHAR;
            buf[di++] = _hlink_tx_frame_buf[si] ^ _HLINK_ESCAPE_MASK;
        } else {
            if ((di + 1) >= buf_len) {
                return 0;
            }
            buf[di++] = _hlink_tx_frame_buf[si];
        }
    }

    buf[di++] = _HLINK_FRAME_MARKER;

    return di;
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
        frame_len--; // exclude crc
        frame_position = _hlink_parse_stlv(buf, frame_len, tlv_set);
        while (frame_position < frame_len) {
            frame_position += _hlink_parse_tlv(buf + frame_position, frame_len - frame_position, tlv_set);
        }
    } else {
        // test
        printf("bad crc\n");
    }

    // pseudo memory barrier: make a dependency on frame_position
    if (frame_position <= frame_len) {
        _hlink_task_buf_id = (_hlink_task_buf_id != 0) ? 0 : 1;
    }

    memcpy(tlv_set, &_hlink_tlv_set_mask, sizeof(_hlink_tlv_set_mask));
    return _hlink_nr_tlvs;
}

/* ISR ------------------------------------------------------------------------*/
bool hlink_receive_data(uint8_t data) {
    static size_t frame_position = 0;
    static bool escaped_character = false;
    bool new_frame_received = false;

    if (_HLINK_FRAME_MARKER == data) {
        if (frame_position < (_HLINK_MIN_FRAME_LEN - 1)) {
            // Head or under size frame, treat as a new frame
            frame_position = 1;
        } else {
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
