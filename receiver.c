#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include "horizon_link.h"

#define RX_BUF_LEN (200)

uint8_t rx_buf[RX_BUF_LEN];

int main(int argc, char *argv[]) {
    int rx_fd;
    int rx_bytes;

    if (argc < 2) {
        fputs("usage: ./a.out xxx.p\n", stderr);
        exit(1);
    }

    if ((rx_fd = open(argv[1], O_RDONLY | O_NOCTTY | O_NONBLOCK)) < 0) {
        perror("can not open rx dev");
        exit(1);
    }

    printf("%s opened, fd %d\n", argv[1], rx_fd);

    if (fcntl(rx_fd, F_SETFL, 0) < 0) {
        perror("failed to reset file status flags");
    }

    while ((rx_bytes = read(rx_fd, rx_buf, sizeof(rx_buf))) > 0) {
        //printf("%d bytes received\n", rx_bytes);
        for (int i = 0; i < rx_bytes; i++) {
            if (extract_frame(rx_buf[i])) {
                process_frame();
            }
        }
    }

    return 0;
}
