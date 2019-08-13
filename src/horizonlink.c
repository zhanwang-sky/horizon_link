/* Includes -------------------------------------------------------------------*/
#ifdef __DEBUG__
#include <stdio.h>
#endif
#include <string.h>
#include "horizonlink.h"

/* Private functions ----------------------------------------------------------*/
void _horizonlink_encode_sbus(const horizonlink_tlv_sbus_t *sbus, uint8_t *buf) {
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

void _horizonlink_decode_sbus(const uint8_t *buf, horizonlink_tlv_sbus_t *sbus) {
    int ch_base, buf_base;

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

void _horizonlink_encode_status(const horizonlink_tlv_status_t *status, uint8_t *buf) {
    buf[0] = status->genid & 0xFF;
    buf[1] = status->genid >> 8;
    buf[2] = status->apseq;
}

void _horizonlink_decode_status(const uint8_t *buf, horizonlink_tlv_status_t *status) {
    status->genid = buf[0];
    status->genid |= buf[1] << 8;
    status->apseq = buf[2];
}

void _horizonlink_encode_quat(const horizonlink_tlv_quat_t *quat, uint8_t *buf) {
    uint32_t *p = (uint32_t*) quat->component;
    for (int i = 0; i < 4; i++) {
        buf[4 * i] = *p & 0xFF;
        buf[4 * i + 1] = (*p >> 8) & 0xFF;
        buf[4 * i + 2] = (*p >> 16) & 0xFF;
        buf[4 * i + 3] = *p >> 24;
        p++;
    }
}

void _horizonlink_decode_quat(const uint8_t *buf, horizonlink_tlv_quat_t *quat) {
    uint32_t *p = (uint32_t*) quat->component;
    for (int i = 0; i < 4; i++) {
        *p = buf[4 * i];
        *p |= buf[4 * i + 1] << 8;
        *p |= buf[4 * i + 2] << 16;
        *p |= buf[4 * i + 3] << 24;
        p++;
    }
}

void _horizonlink_encode_imu(const horizonlink_tlv_imu_t *imu, uint8_t *buf) {
    uint32_t *p = (uint32_t*) imu->gyro;
    for (int i = 0; i < 6; i++) {
        buf[4*i] = *p & 0xFF;
        buf[4*i + 1] = (*p >> 8) & 0xFF;
        buf[4*i + 2] = (*p >> 16) & 0xFF;
        buf[4*i + 3] = *p >> 24;
        p++;
    }
}

void _horizonlink_decode_imu(const uint8_t *buf, horizonlink_tlv_imu_t *imu) {
    uint32_t *p = (uint32_t*) imu->gyro;
    for (int i = 0; i < 6; i++) {
        *p = buf[4*i];
        *p |= buf[4*i + 1] << 8;
        *p |= buf[4*i + 2] << 16;
        *p |= buf[4*i + 3] << 24;
        p++;
    }
}

void _horizonlink_encode_pid(const horizonlink_p2p_pid_t *pid, uint8_t *buf) {
    uint32_t *p = (uint32_t*) pid->pid;
    for (int i = 0; i < 9; i++) {
        buf[4*i] = *p & 0xFF;
        buf[4*i + 1] = (*p >> 8) & 0xFF;
        buf[4*i + 2] = (*p >> 16) & 0xFF;
        buf[4*i + 3] = *p >> 24;
        p++;
    }
}

void _horizonlink_decode_pid(const uint8_t *buf, horizonlink_p2p_pid_t *pid) {
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
    int next_tlv = 0; // default value, assume that it doesn't contain an STLV

#ifdef __DEBUG__
    printf(">>>%s(%d) ENTER\n", __FUNCTION__, __LINE__);
    printf("parse stlv: %02hhX\n", buf[0]);
#endif

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

#ifdef __DEBUG__
    printf("return %d\n", next_tlv);
    printf("<<<%s(%d) EXIT\n", __FUNCTION__, __LINE__);
#endif
    return next_tlv;
}

int _horizonlink_parse_tlv(uint8_t *buf, int frame_len, horizonlink_tlv_set_t *tlv_set,
        horizonlink_tlv_set_t *tlv_set_mask, int *nr_tlvs) {
    uint8_t type;
    uint8_t len;
    int next_tlv = frame_len;

#ifdef __DEBUG__
    printf(">>>%s(%d) ENTER\n", __FUNCTION__, __LINE__);
#endif

    if (frame_len < _HORIZONLINK_TL0_LEN) {
#ifdef __DEBUG__
        printf("frame is less than TL0\n");
#endif
        goto EXIT;
    }

    type = buf[0];
    len = buf[1];
    if (frame_len < (len + 2)) {
        // under size
#ifdef __DEBUG__
        printf("under size TLV\n");
#endif
        goto EXIT;
    }

#ifdef __DEBUG__
    printf("parse tlv: %02hhX\n", type);
#endif

    switch (type) {
    case _HORIZONLINK_TLV_SBUS_TYPE:
        // len is fixed
        if (len == _HORIZONLINK_TLV_SBUS_LEN) {
            next_tlv = len + 2; // mark next tlv position
            if ((tlv_set->tlv_sbus != NULL)
                // in case of duplicated TLV
                && (tlv_set_mask->tlv_sbus == NULL)) {
                // decode
                _horizonlink_decode_sbus(buf + 2, tlv_set->tlv_sbus);
                // record this tlv
                tlv_set_mask->tlv_sbus = tlv_set->tlv_sbus;
                (*nr_tlvs)++;
            }
        }
        break;
    case _HORIZONLINK_TLV_STATUS_TYPE:
        if (len == _HORIZONLINK_TLV_STATUS_LEN) {
            next_tlv = len + 2;
            if ((tlv_set->tlv_status != NULL)
                && (tlv_set_mask->tlv_status == NULL)) {
                // decode
                _horizonlink_decode_status(buf + 2, tlv_set->tlv_status);
                // record
                tlv_set_mask->tlv_status = tlv_set->tlv_status;
                (*nr_tlvs)++;
            }
        }
        break;
    case _HORIZONLINK_TLV_ATT_QUAT_TYPE:
        if (len == _HORIZONLINK_TLV_ATT_QUAT_LEN) {
            next_tlv = len + 2;
            if ((tlv_set->tlv_att_quat != NULL)
                && (tlv_set_mask->tlv_att_quat == NULL)) {
                // decode
                _horizonlink_decode_quat(buf + 2, tlv_set->tlv_att_quat);
                // record
                tlv_set_mask->tlv_att_quat = tlv_set->tlv_att_quat;
                (*nr_tlvs)++;
            }
        }
        break;
    case _HORIZONLINK_TLV_ATT_IMU_TYPE:
        if (len == _HORIZONLINK_TLV_ATT_IMU_LEN) {
            next_tlv = len + 2;
            if ((tlv_set->tlv_att_imu != NULL)
                && (tlv_set_mask->tlv_att_imu == NULL)) {
                // decode
                _horizonlink_decode_imu(buf + 2, tlv_set->tlv_att_imu);
                // record
                tlv_set_mask->tlv_att_imu = tlv_set->tlv_att_imu;
                (*nr_tlvs)++;
            }
        }
        break;
    case _HORIZONLINK_TLV_ATT_PID_TYPE:
        if ((len == _HORIZONLINK_TLV_ATT_PID_T0_LEN) || (len == 2)) {
            next_tlv = len + 2;
            if ((tlv_set->p2p_att_pid != NULL)
                && (tlv_set_mask->p2p_att_pid == NULL)) {
                tlv_set->p2p_att_pid->cmd.seq = buf[2];
                if (len == _HORIZONLINK_TLV_ATT_PID_T0_LEN) {
                    tlv_set->p2p_att_pid->cmd.type = 0;
                    _horizonlink_decode_pid(buf + 3, tlv_set->p2p_att_pid);
                } else {
                    tlv_set->p2p_att_pid->cmd.type = buf[3]; // XXX buf[3] should not be 0
                }
                // record
                tlv_set_mask->p2p_att_pid = tlv_set->p2p_att_pid;
                (*nr_tlvs)++;
            }
        }
        break;
    default:
        break;
    }

EXIT:
#ifdef __DEBUG__
    printf("return %d\n", next_tlv);
    printf("<<<%s(%d) EXIT\n", __FUNCTION__, __LINE__);
#endif
    return next_tlv;
}

uint8_t _horizonlink_compute_checksum(const uint8_t *buf, int len) {
    uint16_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += buf[i];
    }
    uint8_t checksum = (uint8_t) (sum & 0xFF) + (uint8_t) (sum >> 8);
    checksum = ~checksum;
#ifdef __DEBUG__
    printf("checksum = %02hhX\n", checksum);
#endif
    return checksum;
}

bool _horizonlink_validate_checksum(const uint8_t *buf, int frame_len) {
#ifdef __DEBUG__
    printf("actual checksum is %02hhX\n", buf[frame_len-1]);
#endif
    return (_horizonlink_compute_checksum(buf, frame_len-1) == buf[frame_len-1]);
}

/* Functions ------------------------------------------------------------------*/
int horizonlink_pack(horizonlink_tx_handler_t *tx_hdl, horizonlink_tlv_set_t *tlv_set) {
    int frame_pos = 0;
    horizonlink_tlv_set_t tlv_set_mask;
    int nr_tlvs = 0;

    // sanity check
    if (!tx_hdl || !tlv_set) {
        return 0;
    }

    memset(&tlv_set_mask, 0, sizeof(tlv_set_mask));

    // publish-subscribe TLVs
    if ((tlv_set->tlv_sbus != NULL)
        && ((_HORIZONLINK_MAX_FRAME_LEN - 1) >= (frame_pos + 2 + _HORIZONLINK_TLV_SBUS_LEN))) {
        tlv_set_mask.tlv_sbus = tlv_set->tlv_sbus;
        nr_tlvs++;
        // encode sbus TLV
        tx_hdl->tx_buf[frame_pos++] = _HORIZONLINK_TLV_SBUS_TYPE;
        tx_hdl->tx_buf[frame_pos++] = _HORIZONLINK_TLV_SBUS_LEN;
        _horizonlink_encode_sbus(tlv_set->tlv_sbus, tx_hdl->tx_buf + frame_pos);
        frame_pos += _HORIZONLINK_TLV_SBUS_LEN;
    }

    if ((tlv_set->tlv_status != NULL)
        && ((_HORIZONLINK_MAX_FRAME_LEN - 1) >= (frame_pos + 2 + _HORIZONLINK_TLV_STATUS_LEN))) {
        tlv_set_mask.tlv_status = tlv_set->tlv_status;
        nr_tlvs++;
        // encode status TLV
        tx_hdl->tx_buf[frame_pos++] = _HORIZONLINK_TLV_STATUS_TYPE;
        tx_hdl->tx_buf[frame_pos++] = _HORIZONLINK_TLV_STATUS_LEN;
        _horizonlink_encode_status(tlv_set->tlv_status, tx_hdl->tx_buf + frame_pos);
        frame_pos += _HORIZONLINK_TLV_STATUS_LEN;
    }

    if ((tlv_set->tlv_att_quat != NULL)
        && ((_HORIZONLINK_MAX_FRAME_LEN - 1) >= (frame_pos + 2 + _HORIZONLINK_TLV_ATT_QUAT_LEN))) {
        tlv_set_mask.tlv_att_quat = tlv_set->tlv_att_quat;
        nr_tlvs++;
        // encode attitude quaternion TLV
        tx_hdl->tx_buf[frame_pos++] = _HORIZONLINK_TLV_ATT_QUAT_TYPE;
        tx_hdl->tx_buf[frame_pos++] = _HORIZONLINK_TLV_ATT_QUAT_LEN;
        _horizonlink_encode_quat(tlv_set->tlv_att_quat, tx_hdl->tx_buf + frame_pos);
        frame_pos += _HORIZONLINK_TLV_ATT_QUAT_LEN;
    }

    if ((tlv_set->tlv_att_imu != NULL)
        && ((_HORIZONLINK_MAX_FRAME_LEN - 1) >= (frame_pos + 2 + _HORIZONLINK_TLV_ATT_IMU_LEN))) {
        tlv_set_mask.tlv_att_imu = tlv_set->tlv_att_imu;
        nr_tlvs++;
        // encode attitude IMU TLV
        tx_hdl->tx_buf[frame_pos++] = _HORIZONLINK_TLV_ATT_IMU_TYPE;
        tx_hdl->tx_buf[frame_pos++] = _HORIZONLINK_TLV_ATT_IMU_LEN;
        _horizonlink_encode_imu(tlv_set->tlv_att_imu, tx_hdl->tx_buf + frame_pos);
        frame_pos += _HORIZONLINK_TLV_ATT_IMU_LEN;
    }

    // point-to-point TLVs
    if (tlv_set->p2p_att_pid != NULL) {
        int actual_len;
        if (tlv_set->p2p_att_pid->cmd.type == 0) {
            actual_len = _HORIZONLINK_TLV_ATT_PID_T0_LEN;
        } else {
            actual_len = 2;
        }
        if ((_HORIZONLINK_MAX_FRAME_LEN - 1) >= (frame_pos + 2 + actual_len)) {
            tlv_set_mask.p2p_att_pid = tlv_set->p2p_att_pid;
            nr_tlvs++;
            // encode attitude pid TLV
            tx_hdl->tx_buf[frame_pos++] = _HORIZONLINK_TLV_ATT_PID_TYPE;
            tx_hdl->tx_buf[frame_pos++] = actual_len;
            tx_hdl->tx_buf[frame_pos] = tlv_set->p2p_att_pid->cmd.seq;
            if (tlv_set->p2p_att_pid->cmd.type == 0) {
                _horizonlink_encode_pid(tlv_set->p2p_att_pid, tx_hdl->tx_buf + frame_pos + 1);
            } else {
                tx_hdl->tx_buf[frame_pos + 1] = tlv_set->p2p_att_pid->cmd.type;
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

int horizonlink_pack_p2ptlv(horizonlink_tx_handler_t *tx_hdl, horizonlink_p2ptlv_union_t *tlv_union) {
    int frame_pos = 0;
    int nr_tlvs = 0;
    int actual_len;

    // sanity check
    if (!tx_hdl || !tlv_union) {
        return 0;
    }

    switch (tlv_union->type) {
    case _HORIZONLINK_TLV_ATT_PID_TYPE:
        if (tlv_union->u.pid.cmd.type == 0) {
            actual_len = _HORIZONLINK_TLV_ATT_PID_T0_LEN;
        } else {
            actual_len = 2;
        }
        if ((_HORIZONLINK_MAX_FRAME_LEN - 1) >= (actual_len + 2)) {
            nr_tlvs = 1;
            // encode attitude pid TLV
            tx_hdl->tx_buf[frame_pos++] = _HORIZONLINK_TLV_ATT_PID_TYPE;
            tx_hdl->tx_buf[frame_pos++] = actual_len;
            tx_hdl->tx_buf[frame_pos] = tlv_union->u.pid.cmd.seq;
            if (tlv_union->u.pid.cmd.type == 0) {
                _horizonlink_encode_pid(&tlv_union->u.pid, tx_hdl->tx_buf + frame_pos + 1);
            } else {
                tx_hdl->tx_buf[frame_pos + 1] = tlv_union->u.pid.cmd.type;
            }
            frame_pos += actual_len;
        }
        break;
    default:
        break;
    }

    if (nr_tlvs > 0) {
        tx_hdl->tx_buf[frame_pos] = _horizonlink_compute_checksum(tx_hdl->tx_buf, frame_pos);
        frame_pos++;
        tx_hdl->len = frame_pos;
    }

    return nr_tlvs;
}

int horizonlink_unpack(horizonlink_rx_handler_t *rx_hdl, horizonlink_tlv_set_t *tlv_set) {
    uint8_t *buf = rx_hdl->rx_buf;
    int frame_pos = 0;
    int frame_len = rx_hdl->len;
    horizonlink_tlv_set_t tlv_set_mask;
    int nr_tlvs = 0;

#ifdef __DEBUG__
    printf(">>>%s(%d) ENTER\n", __FUNCTION__, __LINE__);
#endif

    // sanity check
    if (!rx_hdl || !tlv_set) {
#ifdef __DEBUG__
        printf("invalid arguments\n");
#endif
        goto EXIT;
    }

    memset(&tlv_set_mask, 0, sizeof(tlv_set_mask));

#ifdef __DEBUG__
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
#endif

    // validate checksum
    if (_horizonlink_validate_checksum(buf, frame_len)) {
        frame_len--; // exclude crc
        frame_pos = _horizonlink_parse_stlv(buf, frame_len, tlv_set,
                &tlv_set_mask, &nr_tlvs);
        while (frame_pos < frame_len) {
            frame_pos += _horizonlink_parse_tlv(buf + frame_pos, frame_len - frame_pos, tlv_set,
                    &tlv_set_mask, &nr_tlvs);
        }
#ifdef __DEBUG__
    } else {
        printf("bad crc\n");
    }
#else
    }
#endif

    memcpy(tlv_set, &tlv_set_mask, sizeof(tlv_set_mask));

EXIT:
#ifdef __DEBUG__
    printf("<<<%s(%d) EXIT\n", __FUNCTION__, __LINE__);
#endif
    return nr_tlvs;
}

bool horizonlink_scatter(horizonlink_tx_handler_t *tx_hdl, uint8_t *buf, int *buf_len) {
    int si = 0, di = 0;

    // sanity check
    if (!tx_hdl || !buf || !buf_len) {
#ifdef __DEBUG__
        printf("check 1: invalid arguments\n");
#endif
        return false;
    }

    if (*buf_len < (tx_hdl->len + 2)) {
#ifdef __DEBUG__
        printf("check 2.1: insufficient buffer space\n");
#endif
        return false;
    }

    buf[di++] = _HORIZONLINK_FRAME_MARKER;

    for (; si < tx_hdl->len; si++) {
        if ((tx_hdl->tx_buf[si] == _HORIZONLINK_FRAME_MARKER)
            || (tx_hdl->tx_buf[si] == _HORIZONLINK_ESCAPE_CHAR)) {
            if ((di + 2) >= *buf_len) {
#ifdef __DEBUG__
                printf("check 2.2.1: insufficient buffer space\n");
#endif
                return false;
            }
            buf[di++] = _HORIZONLINK_ESCAPE_CHAR;
            buf[di++] = tx_hdl->tx_buf[si] ^ _HORIZONLINK_ESCAPE_MASK;
        } else {
            if ((di + 1) >= *buf_len) {
#ifdef __DEBUG__
                printf("check 2.2.2: insufficient buffer space\n");
#endif
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
bool horizonlink_gather(horizonlink_rx_handler_t *rx_hdl, uint8_t data) {
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
