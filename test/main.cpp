#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <horizonlink.h>

horizonlink_tx_handler_t tx_hdl;
horizonlink_rx_handler_t rx_hdl;
uint8_t raw_buf[2 * _HORIZONLINK_MAX_FRAME_LEN] = { 0 };

horizonlink_tlv_sbus_t sbus_tx;
horizonlink_tlv_sbus_t sbus_rx;
horizonlink_tlv_status_t status_tx;
horizonlink_tlv_status_t status_rx;
horizonlink_tlv_quat_t att_quat_tx;
horizonlink_tlv_quat_t att_quat_rx;
horizonlink_tlv_imu_t att_imu_tx;
horizonlink_tlv_imu_t att_imu_rx;
horizonlink_p2p_pid_t att_pid_tx;
horizonlink_p2p_pid_t att_pid_rx;
horizonlink_tlv_set_t tlv_set_tx;
horizonlink_tlv_set_t tlv_set_rx;

std::vector<uint8_t> p2ptlvs = { _HORIZONLINK_TLV_ATT_PID_TYPE };

void stuff_sbus(horizonlink_tlv_sbus_t *sbus) {
    for (int i = 0; i < 16; i++) {
        sbus->channel[i] = _HORIZONLINK_FRAME_MARKER * (i+1);
    }
    sbus->flags = _HORIZONLINK_FRAME_MARKER;
}

bool compare_sbus(horizonlink_tlv_sbus_t *lhs, horizonlink_tlv_sbus_t *rhs) {
    for (int i = 0; i < 16; i++) {
        printf("lhs->channel[%d] = %hu, rhs->channel[%d] = %hu\n",
                i, lhs->channel[i], i, rhs->channel[i]);
        if (lhs->channel[i] != rhs->channel[i]) {
            printf("%s(%d) i = %d\n", __FUNCTION__, __LINE__, i);
            return true;
        }
    }
    printf("lhs->flags = %02hhX, rhs->flags = %02hhX\n",
            lhs->flags, rhs->flags);
    if (lhs->flags != rhs->flags) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    return false;
}

void stuff_status(horizonlink_tlv_status_t *status) {
    status->genid = 0x7E7D;
    status->apseq = 0x5D;
}

bool compare_status(horizonlink_tlv_status_t *lhs, horizonlink_tlv_status_t *rhs) {
    printf("lhs->genid = %04hX, rhs->genid = %04hX\n",
            lhs->genid, rhs->genid);
    if (lhs->genid != rhs->genid) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    printf("lhs->apseq = %02hhX, rhs->apseq = %02hhX\n",
            lhs->apseq, rhs->apseq);
    if (lhs->apseq != rhs->apseq) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    return false;
}

void stuff_quat(horizonlink_tlv_quat_t *quat) {
    quat->component[0] = 3.14159;
    quat->component[1] = 2.71828;
    quat->component[2] = 0.61803;
    quat->component[3] = 6.62607;
}

bool compare_quat(horizonlink_tlv_quat_t *lhs, horizonlink_tlv_quat_t *rhs) {
    for (int i = 0; i < 4; i++) {
        printf("lhs->component[%d] = %f, rhs->component[%d] = %f\n",
                i, lhs->component[i], i, rhs->component[i]);
        if (lhs->component[i] != rhs->component[i]) {
            printf("%s(%d) i = %d\n", __FUNCTION__, __LINE__, i);
            return true;
        }
    }
    return false;
}

void stuff_imu(horizonlink_tlv_imu_t *imu) {
    imu->gyro[0] = 0.123456;
    imu->gyro[1] = 1.23456;
    imu->gyro[2] = 12.3456;
    imu->accel[0] = 0.654321;
    imu->accel[1] = 6.54321;
    imu->accel[2] = 65.4321;
    /*
    imu->accel[0] = 0.654321;
    imu->accel[1] = 6.54321;
    imu->accel[2] = 65.4321;
    */
}

bool compare_imu(horizonlink_tlv_imu_t *lhs, horizonlink_tlv_imu_t *rhs) {
    for (int i = 0; i < 3; i++) {
        printf("lhs->gyro[%d] = %f, rhs->gyro[%d] = %f\n",
                i, lhs->gyro[i], i, rhs->gyro[i]);
        if (lhs->gyro[i] != rhs->gyro[i]) {
            printf("%s(%d) i = %d\n", __FUNCTION__, __LINE__, i);
            return true;
        }
    }
    for (int i = 0; i < 3; i++) {
        printf("lhs->accel[%d] = %f, rhs->accel[%d] = %f\n",
                i, lhs->accel[i], i, rhs->accel[i]);
        if (lhs->accel[i] != rhs->accel[i]) {
            printf("%s(%d) i = %d\n", __FUNCTION__, __LINE__, i);
            return true;
        }
    }
    return false;
}

void stuff_pid(horizonlink_p2p_pid_t *pid) {
    pid->cmd.seq = _HORIZONLINK_ESCAPE_CHAR;
    for (int i = 0; i < 9; i++) {
        pid->pid[i] = -3.5 + i * 12.5;
    }
}

bool compare_pid(horizonlink_p2p_pid_t *lhs, horizonlink_p2p_pid_t *rhs) {
    printf("lhs->cmd.seq = %02hhX, rhs->cmd.seq = %02hhX\n",
            lhs->cmd.seq, rhs->cmd.seq);
    if (lhs->cmd.seq != rhs->cmd.seq) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    printf("lhs->cmd.type = %02hhX, rhs->cmd.type = %02hhX\n",
            lhs->cmd.type, rhs->cmd.type);
    if (lhs->cmd.type != rhs->cmd.type) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (!lhs->cmd.type) {
        for (int i = 0; i < 9; i++) {
            printf("lhs->pid[%d] = %f, rhs->pid[%d] = %f\n",
                    i, lhs->pid[i], i, rhs->pid[i]);
            if (lhs->pid[i] != rhs->pid[i]) {
                printf("%s(%d) i = %d\n", __FUNCTION__, __LINE__, i);
                return true;
            }
        }
    }
    return false;
}

void reset_tlvs(horizonlink_tlv_set_t *tlv_set) {
    if (tlv_set->tlv_sbus) {
        memset(tlv_set->tlv_sbus, 0, sizeof(decltype(*(tlv_set->tlv_sbus))));
    }
    if (tlv_set->tlv_status) {
        memset(tlv_set->tlv_status, 0, sizeof(decltype(*(tlv_set->tlv_status))));
    }
    if (tlv_set->tlv_att_quat) {
        memset(tlv_set->tlv_att_quat, 0, sizeof(decltype(*(tlv_set->tlv_att_quat))));
    }
    if (tlv_set->tlv_att_imu) {
        memset(tlv_set->tlv_att_imu, 0, sizeof(decltype(*(tlv_set->tlv_att_imu))));
    }
    if (tlv_set->p2p_att_pid) {
        memset(tlv_set->p2p_att_pid, 0, sizeof(decltype(*(tlv_set->p2p_att_pid))));
    }
}

void stuff_tlvs(horizonlink_tlv_set_t *tlv_set) {
    if (tlv_set->tlv_sbus) {
        stuff_sbus(tlv_set->tlv_sbus);
    }
    if (tlv_set->tlv_status) {
        stuff_status(tlv_set->tlv_status);
    }
    if (tlv_set->tlv_att_quat) {
        stuff_quat(tlv_set->tlv_att_quat);
    }
    if (tlv_set->tlv_att_imu) {
        stuff_imu(tlv_set->tlv_att_imu);
    }
    if (tlv_set->p2p_att_pid) {
        stuff_pid(tlv_set->p2p_att_pid);
    }
}

bool compare_tlvs(horizonlink_tlv_set_t *lhs, horizonlink_tlv_set_t *rhs) {
    if (!lhs->tlv_sbus || !rhs->tlv_sbus) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (compare_sbus(lhs->tlv_sbus, rhs->tlv_sbus)) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (!lhs->tlv_status || !rhs->tlv_status) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (compare_status(lhs->tlv_status, rhs->tlv_status)) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (!lhs->tlv_att_quat || !rhs->tlv_att_quat) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (compare_quat(lhs->tlv_att_quat, rhs->tlv_att_quat)) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (!lhs->tlv_att_imu || !rhs->tlv_att_imu) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (compare_imu(lhs->tlv_att_imu, rhs->tlv_att_imu)) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (!lhs->p2p_att_pid || !rhs->p2p_att_pid) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (compare_pid(lhs->p2p_att_pid, rhs->p2p_att_pid)) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    return false;
}

int p2ptlv_test(uint8_t type) {
    horizonlink_p2ptlv_union_t p2ptlv;

    printf("tlv type: %02hhX\n", type);

    for (int i = 0; i < 3; i++) {
        printf("////////////////////\n");
        printf("round %d\n", i+1);

        // reset
        memset(&tx_hdl, 0, sizeof(tx_hdl));
        memset(&rx_hdl, 0, sizeof(rx_hdl));
        memset(&p2ptlv, 0, sizeof(p2ptlv));
        reset_tlvs(&tlv_set_rx);

        // stuff
        p2ptlv.type = type;
        switch (type) {
        case _HORIZONLINK_TLV_ATT_PID_TYPE:
            stuff_pid(&p2ptlv.u.pid);
            p2ptlv.u.pid.cmd.type = i;
            break;
        default:
            printf("(%d) unknown tlv type\n", __LINE__);
            goto ERR_EXIT;
        }

        // pack
        int nr_tlvs = horizonlink_pack_p2ptlv(&tx_hdl, &p2ptlv);
        printf("(%d) horizonlink_pack returns %d\n", __LINE__, nr_tlvs);
        printf("frame length = %d\n", tx_hdl.len);
        if (nr_tlvs != 1) {
            goto ERR_EXIT;
        }

        // send
        int bytes_to_send = sizeof(raw_buf);
        bool ready = horizonlink_scatter(&tx_hdl, raw_buf, &bytes_to_send);
        printf("(%d) horizonlink_scatter returns %s\n", __LINE__, ready ? "true" : "false");
        printf("(%d) %d bytes to send\n", __LINE__, bytes_to_send);
        if (!ready) {
            goto ERR_EXIT;
        }

        /* --------- */

        // receive
        int cnt;
        for (cnt = 0; cnt < bytes_to_send; cnt++) {
            if ((ready = horizonlink_gather(&rx_hdl, raw_buf[cnt]))) {
                break;
            }
        }
        printf("(%d) horizonlink_gather returns %s\n", __LINE__, ready ? "true" : "false");
        printf("(%d) cnt = %d\n", __LINE__, cnt);
        if (!ready) {
            goto ERR_EXIT;
        }
        if (cnt != (bytes_to_send - 1)) {
            goto ERR_EXIT;
        }

        // unpack
        nr_tlvs = horizonlink_unpack(&rx_hdl, &tlv_set_rx);
        printf("(%d) horizonlink_unpack returns %d\n", __LINE__, nr_tlvs);
        if (nr_tlvs != 1) {
            goto ERR_EXIT;
        }

        // compare
        switch (type) {
        case _HORIZONLINK_TLV_ATT_PID_TYPE:
            if (compare_pid(&p2ptlv.u.pid, tlv_set_rx.p2p_att_pid)) {
                printf("(%d) doesn't match", __LINE__);
            }
            break;
        default:
            // should not get here
            printf("(%d) unknown tlv type\n", __LINE__);
            goto ERR_EXIT;
        }

        printf("round %d pass\n", i+1);
    }

    return 0;

ERR_EXIT:
    return -1;
}

int main() {
    // prepare
    tlv_set_tx.tlv_sbus = &sbus_tx;
    tlv_set_tx.tlv_status = &status_tx;
    tlv_set_tx.tlv_att_quat = &att_quat_tx;
    tlv_set_tx.tlv_att_imu = &att_imu_tx;
    tlv_set_tx.p2p_att_pid = &att_pid_tx;

    tlv_set_rx.tlv_sbus = &sbus_rx;
    tlv_set_rx.tlv_status = &status_rx;
    tlv_set_rx.tlv_att_quat = &att_quat_rx;
    tlv_set_rx.tlv_att_imu = &att_imu_rx;
    tlv_set_rx.p2p_att_pid = &att_pid_rx;

    printf("tlv_set test:\n");

    ////////////////////
    // round x
    for (int x = 0; x < 3; x++) {
        printf("////////////////////\n");
        printf("round %d\n", x+1);

        // reset
        memset(&tx_hdl, 0, sizeof(tx_hdl));
        memset(&rx_hdl, 0, sizeof(rx_hdl));
        reset_tlvs(&tlv_set_tx);
        reset_tlvs(&tlv_set_rx);

        // stuff
        stuff_tlvs(&tlv_set_tx);
        tlv_set_tx.p2p_att_pid->cmd.type = x;

        // pack
        int nr_tlvs;
        nr_tlvs = horizonlink_pack(&tx_hdl, &tlv_set_tx);
        printf("(%d) horizonlink_pack returns %d\n", __LINE__, nr_tlvs);
        printf("frame length = %d\n", tx_hdl.len);
        if (nr_tlvs != 5) {
            goto ERR_EXIT;
        }

        // send
        int bytes_to_send = sizeof(raw_buf);
        bool ready = horizonlink_scatter(&tx_hdl, raw_buf, &bytes_to_send);
        printf("(%d) horizonlink_scatter returns %s\n", __LINE__, ready ? "true" : "false");
        printf("(%d) %d bytes to send\n", __LINE__, bytes_to_send);
        if (!ready) {
            goto ERR_EXIT;
        }

        /* --------- */

        // receive
        int i;
        for (i = 0; i < bytes_to_send; i++) {
            if ((ready = horizonlink_gather(&rx_hdl, raw_buf[i]))) {
                break;
            }
        }
        printf("(%d) horizonlink_gather returns %s\n", __LINE__, ready ? "true" : "false");
        printf("(%d) i = %d\n", __LINE__, i);
        if (!ready) {
            goto ERR_EXIT;
        }
        if (i != (bytes_to_send - 1)) {
            goto ERR_EXIT;
        }

        // unpack
        nr_tlvs = horizonlink_unpack(&rx_hdl, &tlv_set_rx);
        printf("(%d) horizonlink_unpack returns %d\n", __LINE__, nr_tlvs);
        if (nr_tlvs != 5) {
            goto ERR_EXIT;
        }

        // compare
        if (compare_tlvs(&tlv_set_tx, &tlv_set_rx)) {
            goto ERR_EXIT;
        }

        printf("round %d pass\n", x+1);
    }

    printf("========================================\n");
    printf("p2ptlv test:\n");

    for (auto type: p2ptlvs) {
        if (p2ptlv_test(type) < 0) {
            printf("(%d) type %02hhX fail!\n", __LINE__, type);
            goto ERR_EXIT;
        }
    }

    printf("\nall passed!\n");

    return 0;

ERR_EXIT:
    printf("failed!\n");
    exit(1);
}
