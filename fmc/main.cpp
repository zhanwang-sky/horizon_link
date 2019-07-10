// cpp
#include <thread>
#include "uart.h"
// Unix
#include <unistd.h> // write(), read(), close()

using namespace std;

void thr_read(int fd) {
    ssize_t bytes_read;
    char buf[6] = { '\0' };

    bytes_read = read(fd, buf, 5);
    printf("%zd bytes read\n", bytes_read);
    if (bytes_read < 0) {
        printf("read serial port error\n");
    } else {
        printf("%s\n", buf);
    }
}

void thr_write(int fd) {
    ssize_t bytes_written;

    bytes_written = write(fd, "12345", 5);
    printf("%zd bytes written\n", bytes_written);
    if (bytes_written < 0) {
        printf("write serial port error\n");
    }
}

void test_rw(int fd) {
    thread t_r(thr_read, fd);
    sleep(1);
    thread t_w(thr_write, fd);

    t_r.join();
    t_w.join();
}

int main(int argc, char *argv[]) {
    int fd;

    if (argc != 2) {
        printf("usage: ./fmc.out serial_device.\n");
        exit(1);
    }

    fd = uart_open(argv[1]);
    if (fd < 0) {
        printf("open serial device failed.\n");
        exit(1);
    }

    test_rw(fd);

    uart_close(fd);
    return 0;
}
