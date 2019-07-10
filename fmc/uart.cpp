// legacy
#include <cstring>
// Unix
#include <fcntl.h> // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
// others
#include <apue.h>

// reference:
// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/#basic-setup-in-c
// https://blog.csdn.net/yanhuan136675/article/details/82766466

int uart_open(char *name) {
    // Phase 1
    int fd;
    fd = open(name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        apue_err_ret("failed to open '%s'", name);
        goto ERR_EXIT;
    }

    if (!isatty(fd)) {
        apue_err_msg("'%s' is not a tty", name);
        goto ERR_EXIT;
    }

    if (fcntl(fd, F_SETFL, 0) < 0) {
        apue_err_ret("can't reset to blocking mode");
        goto ERR_EXIT;
    }

    // Phase 2
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    // Read in existing settings, and handle any error
    if(tcgetattr(fd, &tty) < 0) {
        apue_err_ret("error from tcgetattr");
        goto ERR_EXIT;
    }

    // Control Modes (c_cflags)
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
    //tty.c_cflag |= PARENB; // Set parity bit, enabling parity
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
    //tty.c_cflag |= CSTOPB; // Set stop field, two stop bits used in communication
    tty.c_cflag |= CS8; // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    //tty.c_cflag |= CRTSCTS; // Enable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    // Local Modes (c_lflag)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

    // Input Modes (c_iflag)
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
        // Disable any special handling of received bytes

    // Output Modes (c_oflag)
    tty.c_oflag &= ~OPOST;
        // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;
        // Prevent conversion of newline to carriage return/line feed
    tty.c_oflag &= ~OXTABS;
        // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
    tty.c_oflag &= ~ONOEOT;
        // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

    // VMIN and VTIME (c_cc)
    tty.c_cc[VTIME] = 2; // 200ms
    tty.c_cc[VMIN] = 128; // 128 bytes @9600

    // Baud Rate
    cfsetspeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(fd, TCSANOW, &tty) < 0) {
        apue_err_ret("error from tcsetattr");
        goto ERR_EXIT;
    }

    return fd;

ERR_EXIT:
    if (fd >= 0) { close(fd); }
    return -1;
}

void uart_close(int fd) {
    close(fd);
}
