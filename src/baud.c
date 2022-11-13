//
// Created by wsy on 11/12/22.
//

#include "baud.h"
#include <sys/ioctl.h>
#include <asm-generic/termbits.h>

int setbaud(int fd, int speed) {
    struct termios2 tio;
    ioctl(fd, TCGETS2, &tio);
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = speed;
    tio.c_ospeed = speed;
    int r = ioctl(fd, TCSETS2, &tio);
    return r;
}
