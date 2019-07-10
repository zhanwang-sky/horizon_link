/* Includes -------------------------------------------------------------------*/
#include <string.h>
#include "horizonlink.h"
// test
#include <stdio.h>

/* Private functions ----------------------------------------------------------*/
void _horizonlink_encode_quat(const horizonlink_quat_t *quat, uint8_t *buf) {
    uint32_t *p = (uint32_t*) &quat->component[0];
    for (int i = 0; i < 4; i++) {
        buf[4 * i] = *p & 0xFF;
        buf[4 * i + 1] = (*p >> 8) & 0xFF;
        buf[4 * i + 2] = (*p >> 16) & 0xFF;
        buf[4 * i + 3] = *p >> 24;
        p++;
    }
}

void _horizonlink_decode_quat(const uint8_t *buf, horizonlink_quat_t *quat) {
    uint32_t *p = (uint32_t*) &quat->component[0];
    for (int i = 0; i < 4; i++) {
        *p = buf[4 * i];
        *p |= buf[4 * i + 1] << 8;
        *p |= buf[4 * i + 2] << 16;
        *p |= buf[4 * i + 3] << 24;
        p++;
    }
}

void _horizonlink_encode_sbus(const horizonlink_sbus_t *sbus, uint8_t *buf) {
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

void _horizonlink_decode_sbus(const uint8_t *buf, horizonlink_sbus_t *sbus) {
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

void _horizonlink_encode_pid(const horizonlink_pid_t *pid, uint8_t *buf) {
    uint32_t *p = (uint32_t*) pid->pid;
    for (int i = 0; i < 9; i++) {
        buf[4*i] = *p & 0xFF;
        buf[4*i + 1] = (*p >> 8) & 0xFF;
        buf[4*i + 2] = (*p >> 16) & 0xFF;
        buf[4*i + 3] = *p >> 24;
        p++;
    }
}

void _horizonlink_decode_pid(const uint8_t *buf, horizonlink_pid_t *pid) {
    uint32_t *p = (uint32_t*) pid->pid;
    for (int i = 0; i < 9; i++) {
        *p = buf[4*i];
        *p |= buf[4*i + 1] << 8;
        *p |= buf[4*i + 2] << 16;
        *p |= buf[4*i + 3] << 24;
        p++;
    }
}

int _horizonlink_parse_stlv(uint8_t *buf, int frame_len, horizonlink_tlv_set_t *tlv_set,
        horizonlink_tlv_set_t *tlv_set_mask, int *nr_tlvs) {
    uint8_t type = buf[0]; // it is guaranteed that frame_len is at least 2 bytes
    int next_tlv = 0; // default value, assume that the frame doesn't contain a STLV

    printf("parse stlv\n");

    switch (type) {
    case _HORIZONLINK_STLV_FPORT_CTRL_TYPE:
        if ((buf[1] == 0) && (frame_len >= _HORIZONLINK_STLV_FPORT_CTRL_LEN + 2)) {
            next_tlv = frame_len; // mark next tlv position (stop parsing remaining data)
            if (tlv_set->fport_ctrl != NULL) {
                // record this STLV
                tlv_set_mask->fport_ctrl = tlv_set->fport_ctrl;
                (*nr_tlvs)++;
                // decode
                _horizonlink_decode_sbus(buf + 2, tlv_set->fport_ctrl->sbus);
                tlv_set->fport_ctrl->rssi = buf[25];
            }
        }
        break;
    default:
        break;
    }

    return next_tlv;
}

int _horizonlink_parse_tlv(uint8_t *buf, int frame_len, horizonlink_tlv_set_t *tlv_set,
        horizonlink_tlv_set_t *tlv_set_mask, int *nr_tlvs) {
    uint8_t type;
    uint8_t len;
    int next_tlv = frame_len;

    printf("parse tlv\n");

    if (frame_len < _HORIZONLINK_TL0_LEN) {
        return frame_len;
    }

    type = buf[0];
    len = buf[1];
    if (frame_len < (len + 2)) {
        // under size
        printf("under size\n");
        return frame_len;
    }

    switch (type) {
    case _HORIZONLINK_TLV_SBUS_TYPE:
        // len is fixed
        if (len == _HORIZONLINK_TLV_SBUS_LEN) {
            next_tlv = len + 2; // mark next tlv position
            if ((tlv_set->sbus != NULL)
                // in case of duplicated TLV
                && (tlv_set_mask->sbus == NULL)) {
                // decode
                _horizonlink_decode_sbus(buf + 2, tlv_set->sbus);
                // record this tlv
                tlv_set_mask->sbus = tlv_set->sbus;
                (*nr_tlvs)++;
            }
        }
        break;
    case _HORIZONLINK_TLV_ATT_QUAT_TYPE:
        if (len == _HORIZONLINK_TLV_ATT_QUAT_LEN) {
            next_tlv = len + 2;
            if ((tlv_set->att_quat != NULL)
                && (tlv_set_mask->att_quat == NULL)) {
                // decode
                _horizonlink_decode_quat(buf + 2, tlv_set->att_quat);
                // record
                tlv_set_mask->att_quat = tlv_set->att_quat;
                (*nr_tlvs)++;
            }
        }
        break;
    case _HORIZONLINK_TLV_ATT_PID_TYPE:
        if ((len == _HORIZONLINK_TLV_ATT_PID_T0_LEN)
            || (len == _HORIZONLINK_TLV_ATT_PID_Tx_LEN)) {
            next_tlv = len + 2;
            if ((tlv_set->att_pid != NULL)
                && (tlv_set_mask->att_pid == NULL)) {
                tlv_set->att_pid->subcmd.seq = buf[2];
                if (len == _HORIZONLINK_TLV_ATT_PID_T0_LEN) {
                    tlv_set->att_pid->subcmd.type = 0;
                    _horizonlink_decode_pid(buf + 3, tlv_set->att_pid);
                } else {
                    tlv_set->att_pid->subcmd.type = buf[3];
                }
                // record
                tlv_set_mask->att_pid = tlv_set->att_pid;
                (*nr_tlvs)++;
            }
        }
        break;
    default:
        break;
    }

    return next_tlv;
}

uint8_t _horizonlink_compute_checksum(uint8_t *buf, int len) {
    uint16_t checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum += buf[i];
    }
    checksum = (checksum & 0xFF) + (checksum >> 8);
    return (0xFF - checksum);
}

bool _horizonlink_validate_checksum(uint8_t *buf, int frame_len) {
    uint16_t checksum = 0;
    for (int i = 0; i < frame_len; i++) {
        checksum += buf[i];
    }
    checksum = (checksum & 0xFF) + (checksum >> 8);
    return (checksum == _HORIZONLINK_CRC_VALUE);
}

/* Functions ------------------------------------------------------------------*/
int horizonlink_pack(horizonlink_tx_handle_t *tx_hdl, horizonlink_tlv_set_t *tlv_set) {
    int frame_pos = 0;
    horizonlink_tlv_set_t tlv_set_mask;
    int nr_tlvs = 0;

    // sanity check
    if (!tx_hdl || !tlv_set) {
        return 0;
    }

    memset(&tlv_set_mask, 0, sizeof(tlv_set_mask));

    // publish-subscribe TLVs
    if ((tlv_set->att_quat != NULL)
        && ((_HORIZONLINK_MAX_FRAME_LEN - 1) >= (frame_pos + 2 + _HORIZONLINK_TLV_ATT_QUAT_LEN))) {
        tlv_set_mask.att_quat = tlv_set->att_quat;
        nr_tlvs++;
        // encode attitude quaternion TLV
        tx_hdl->tx_buf[frame_pos++] = _HORIZONLINK_TLV_ATT_QUAT_TYPE;
        tx_hdl->tx_buf[frame_pos++] = _HORIZONLINK_TLV_ATT_QUAT_LEN;
        _horizonlink_encode_quat(tlv_set->att_quat, tx_hdl->tx_buf + frame_pos);
        frame_pos += _HORIZONLINK_TLV_ATT_QUAT_LEN;
    }

    if ((tlv_set->sbus != NULL)
        && ((_HORIZONLINK_MAX_FRAME_LEN - 1) >= (frame_pos + 2 + _HORIZONLINK_TLV_SBUS_LEN))) {
        tlv_set_mask.sbus = tlv_set->sbus;
        nr_tlvs++;
        // encode sbus TLV
        tx_hdl->tx_buf[frame_pos++] = _HORIZONLINK_TLV_SBUS_TYPE;
        tx_hdl->tx_buf[frame_pos++] = _HORIZONLINK_TLV_SBUS_LEN;
        _horizonlink_encode_sbus(tlv_set->sbus, tx_hdl->tx_buf + frame_pos);
        frame_pos += _HORIZONLINK_TLV_SBUS_LEN;
    }

    // point-to-point TLVs
    if (tlv_set->att_pid != NULL) {
        int actual_len = 0;
        if (tlv_set->att_pid->subcmd.type == 0) {
            actual_len = _HORIZONLINK_TLV_ATT_PID_T0_LEN;
        } else {
            actual_len = _HORIZONLINK_TLV_ATT_PID_Tx_LEN;
        }
        if ((_HORIZONLINK_MAX_FRAME_LEN - 1) >= (frame_pos + 2 + actual_len)) {
            tlv_set_mask.att_pid = tlv_set->att_pid;
            nr_tlvs++;
            // encode attitude pid TLV
            tx_hdl->tx_buf[frame_pos++] = _HORIZONLINK_TLV_ATT_PID_TYPE;
            tx_hdl->tx_buf[frame_pos++] = actual_len;
            tx_hdl->tx_buf[frame_pos] = tlv_set->att_pid->subcmd.seq;
            if (tlv_set->att_pid->subcmd.type == 0) {
                _horizonlink_encode_pid(tlv_set->att_pid, tx_hdl->tx_buf + frame_pos + 1);
            } else {
                tx_hdl->tx_buf[frame_pos + 1] = tlv_set->att_pid->subcmd.type;
            }
            frame_pos += actual_len;
        }
    }

    if (nr_tlvs > 0) {
        tx_hdl->tx_buf[frame_pos] = _horizonlink_compute_checksum(tx_hdl->tx_buf, frame_pos);
        frame_pos++;
        tx_hdl->len = frame_pos;
    }

    memcpy(tlv_set, &tlv_set_mask, sizeof(tlv_set_mask));
    return nr_tlvs;
}

int horizonlink_unpack(horizonlink_rx_handle_t *rx_hdl, horizonlink_tlv_set_t *tlv_set) {
    uint8_t *buf = rx_hdl->rx_buf;
    int frame_pos = 0;
    int frame_len = rx_hdl->len;
    horizonlink_tlv_set_t tlv_set_mask;
    int nr_tlvs = 0;

    // sanity check
    if (!rx_hdl || !tlv_set) {
        return 0;
    }

    memset(&tlv_set_mask, 0, sizeof(tlv_set_mask));

    // test
    for (int i = 0; i < frame_len; i++) {
        if ((i % 0x10) == 0) {
            if (i != 0) {
                printf("\n");
            }
            printf("%08X: ", i);
        }
        printf("%02X ", buf[i]);
    }
    printf("\n");

    // validate checksum
    if (_horizonlink_validate_checksum(buf, frame_len)) {
        frame_len--; // exclude crc
        frame_pos = _horizonlink_parse_stlv(buf, frame_len, tlv_set,
                &tlv_set_mask, &nr_tlvs);
        while (frame_pos < frame_len) {
            frame_pos += _horizonlink_parse_tlv(buf + frame_pos, frame_len - frame_pos, tlv_set,
                    &tlv_set_mask, &nr_tlvs);
        }
    } else {
        // test
        printf("bad crc\n");
    }

    memcpy(tlv_set, &tlv_set_mask, sizeof(tlv_set_mask));
    return nr_tlvs;
}

bool horizonlink_disperse(horizonlink_tx_handle_t *tx_hdl, uint8_t *buf, int *buf_len) {
    int si = 0, di = 0;

    // sanity check
    if (!tx_hdl || !buf || !buf_len) {
        return false;
    }

    if (*buf_len < (tx_hdl->len + 2)) {
        return false;
    }

    buf[di++] = _HORIZONLINK_FRAME_MARKER;

    for (; si < tx_hdl->len; si++) {
        if ((tx_hdl->tx_buf[si] == _HORIZONLINK_FRAME_MARKER)
            || (tx_hdl->tx_buf[si] == _HORIZONLINK_ESCAPE_CHAR)) {
            if ((di + 2) >= *buf_len) {
                return false;
            }
            buf[di++] = _HORIZONLINK_ESCAPE_CHAR;
            buf[di++] = tx_hdl->tx_buf[si] ^ _HORIZONLINK_ESCAPE_MASK;
        } else {
            if ((di + 1) >= *buf_len) {
                return false;
            }
            buf[di++] = tx_hdl->tx_buf[si];
        }
    }

    buf[di++] = _HORIZONLINK_FRAME_MARKER;
    *buf_len = di;

    return true;
}

/* ISR ------------------------------------------------------------------------*/
bool horizonlink_assemble(horizonlink_rx_handle_t *rx_hdl, uint8_t data) {
    bool new_frame_received = false;

    // sanity check
    if (!rx_hdl) {
        return false;
    }

    if (_HORIZONLINK_FRAME_MARKER == data) {
        if (rx_hdl->pos <= _HORIZONLINK_MIN_FRAME_LEN) {
            // Head or under size frame, treat as a new frame
            rx_hdl->pos = 1;
        } else {
            // Tail
            rx_hdl->len = rx_hdl->pos - 1;
            rx_hdl->pos = 0;
            new_frame_received = true;
        }
        rx_hdl->esc = false;
    } else if (rx_hdl->pos > 0) {
        if (rx_hdl->pos > _HORIZONLINK_MAX_FRAME_LEN) {
            // oversize frame
            rx_hdl->pos = 0;
        } else {
            if (data == _HORIZONLINK_ESCAPE_CHAR) {
                // XXX 7D+7E?
                rx_hdl->esc = true;
            } else {
                if (rx_hdl->esc) {
                    data ^= _HORIZONLINK_ESCAPE_MASK;
                    rx_hdl->esc = false;
                }
                rx_hdl->rx_buf[rx_hdl->pos - 1] = data;
                rx_hdl->pos++;
            }
        }
    } else {
        // frame header not recognized, discard
    }

    return new_frame_received;
}
