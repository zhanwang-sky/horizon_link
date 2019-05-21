#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include "apue.h"
#include "horizon_link.h"

#define RX_BUF_LEN (200)

extern int optind, opterr, optopt;
//extern char *optarg;
uint8_t rx_buf[RX_BUF_LEN];

int main(int argc, char *argv[]) {
    int opt = -1;
    char *path = NULL;
    int oflag = O_RDONLY | O_NOCTTY | O_NONBLOCK;
    int rx_fd = -1;
    int rx_bytes = -1;

    printf("argc = %d\n", argc);
    if (argc != 2 && argc != 3) {
        apue_err_quit("usage: ./a.out [-p] path");
    }

    opterr = 0;
    while ((opt = getopt(argc, argv, "p")) != EOF) {
        switch (opt) {
        case 'p':
            oflag &= ~O_NONBLOCK;
            break;
        case '?':
            apue_err_quit("unrecognized option: -%c", optopt);
            break;
        }
    }
    path = argv[optind];

    if (argc < 2) {
        apue_err_quit("usage: ./a.out [dev_path]");
    }

    if ((rx_fd = open(path, oflag)) < 0) {
        apue_err_sys("failed to open %s", path);
    }

    printf("%s opened, fd = %d\n", path, rx_fd);

    if (fcntl(rx_fd, F_SETFL, 0) < 0) {
        apue_err_sys("failed to reset file status flags");
    }

    while ((rx_bytes = read(rx_fd, rx_buf, sizeof(rx_buf))) > 0) {
        printf("%d bytes received\n", rx_bytes);
        for (int i = 0; i < rx_bytes; i++) {
            if (receive_data(rx_buf[i])) {
                process_frame();
            }
        }
    }

    printf("bye-bye\n");

    return 0;
}
