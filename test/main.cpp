#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <horizonlink.h>

void horizonlink_send(uint8_t);
void horizonlink_recv();

horizonlink_tlv_set_t tlv_set_tx;
horizonlink_tlv_set_t tlv_set_rx;
horizonlink_quat_t att_quat_tx;
horizonlink_quat_t att_quat_rx;
horizonlink_sbus_t sbus_tx;
horizonlink_sbus_t sbus_rx;
horizonlink_pid_t att_pid_tx;
horizonlink_pid_t att_pid_rx;

uint8_t raw_buf[500] = { 0 };

void stuff_quat(horizonlink_quat_t *quat) {
    quat->component[0] = 3.14159;
    quat->component[1] = 2.71828;
    quat->component[2] = 0.61803;
    quat->component[3] = 6.62607;
}

bool compare_quat(horizonlink_quat_t *lhs, horizonlink_quat_t *rhs) {
    for (int i = 0; i < 4; i++) {
        if (lhs->component[i] != rhs->component[i]) {
            printf("%s(%d) i = %d\n", __FUNCTION__, __LINE__, i);
            return true;
        }
    }
    return false;
}

void stuff_sbus(horizonlink_sbus_t *sbus) {
    for (int i = 0; i < 16; i++) {
        sbus->channel[i] = 126 + i * 62.5;
    }
    sbus->flags = 0x7D;
}

bool compare_sbus(horizonlink_sbus_t *lhs, horizonlink_sbus_t *rhs) {
    for (int i = 0; i < 16; i++) {
        if (lhs->channel[i] != rhs->channel[i]) {
            printf("%s(%d) i = %d\n", __FUNCTION__, __LINE__, i);
            return true;
        }
    }
    if (lhs->flags != rhs->flags) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    return false;
}

void stuff_pid(horizonlink_pid_t *pid) {
    pid->cmd.seq = 0x5D;
    for (int i = 0; i < 9; i++) {
        pid->pid[i] = -3.5 + i * 12.5;
    }
}

bool compare_pid(horizonlink_pid_t *lhs, horizonlink_pid_t *rhs) {
    if (lhs->cmd.seq != rhs->cmd.seq) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (lhs->cmd.type != rhs->cmd.type) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (lhs->cmd.type == 0) {
        for (int i = 0; i < 9; i++) {
            if (lhs->pid[i] != rhs->pid[i]) {
                printf("%s(%d) i = %d\n", __FUNCTION__, __LINE__, i);
                return true;
            }
        }
    }
    return false;
}

void reset_tlvs(horizonlink_tlv_set_t *tlv_set) {
    if (tlv_set->att_quat) {
        memset(tlv_set->att_quat, 0, sizeof(decltype(*(tlv_set->att_quat))));
    }
    if (tlv_set->sbus) {
        memset(tlv_set->sbus, 0, sizeof(decltype(*(tlv_set->sbus))));
    }
    if (tlv_set->att_pid) {
        memset(tlv_set->att_pid, 0, sizeof(decltype(*(tlv_set->att_pid))));
    }
}

void stuff_tlvs(horizonlink_tlv_set_t *tlv_set) {
    if (tlv_set->att_quat) {
        stuff_quat(tlv_set->att_quat);
    }
    if (tlv_set->sbus) {
        stuff_sbus(tlv_set->sbus);
    }
    if (tlv_set->att_pid) {
        stuff_pid(tlv_set->att_pid);
    }
}

bool compare_tlvs(horizonlink_tlv_set_t *lhs, horizonlink_tlv_set_t *rhs) {
    if (lhs->att_quat == NULL || rhs->att_quat == NULL) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (compare_quat(lhs->att_quat, rhs->att_quat)) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (lhs->sbus == NULL || rhs->sbus == NULL) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (compare_sbus(lhs->sbus, rhs->sbus)) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (lhs->att_pid == NULL || rhs->att_pid == NULL) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    if (compare_pid(lhs->att_pid, rhs->att_pid)) {
        printf("%s(%d)\n", __FUNCTION__, __LINE__);
        return true;
    }
    return false;
}

int main() {
    int i;
    int rc;

    // prepare
    tlv_set_tx.att_quat = &att_quat_tx;
    tlv_set_tx.sbus = &sbus_tx;
    tlv_set_tx.att_pid = &att_pid_tx;

    tlv_set_rx.att_quat = &att_quat_rx;
    tlv_set_rx.sbus = &sbus_rx;
    tlv_set_rx.att_pid = &att_pid_rx;

    ////////////////////
    // round x
    for (int x = 0; x < 3; x++) {
    printf("////////////////////\n");
    printf("round %d\n", x+1);

    // reset
    reset_tlvs(&tlv_set_tx);
    reset_tlvs(&tlv_set_rx);
    // stuff tx
    stuff_tlvs(&tlv_set_tx);
    // XXX
    tlv_set_tx.att_pid->cmd.type = x;
    // XXX
    // pack
    rc = horizonlink_pack(&tlv_set_tx);
    printf("(%d) horizonlink_pack returns %d\n", __LINE__, rc);
    if (rc != 3) {
        goto ERR_EXIT;
    }
    // prepare buffer
    rc = (int) horizonlink_disperse(raw_buf, sizeof(raw_buf));
    printf("(%d) horizonlink_disperse returns %d\n", __LINE__, rc);
    if (rc == 0) {
        goto ERR_EXIT;
    }
    // assemble frame
    for (i = 0; i < rc; i++) {
        if (horizonlink_assemble(raw_buf[i])) {
            break;
        }
    }
    printf("(%d) %d bytes processed\n", __LINE__, i);
    if (i != (rc - 1)) {
        goto ERR_EXIT;
    }
    // unpack
    rc = (int) horizonlink_unpack(&tlv_set_rx);
    printf("(%d) horizonlink_unpack returns %d\n", __LINE__, rc);
    if (rc != 3) {
        goto ERR_EXIT;
    }
    // compare
    if (compare_tlvs(&tlv_set_tx, &tlv_set_rx)) {
        goto ERR_EXIT;
    }

    printf("round %d pass\n", x+1);
    }

    printf("\nall tests pass!\n");

    return 0;

ERR_EXIT:
    printf("failed!\n");
    exit(1);
}
