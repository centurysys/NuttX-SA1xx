/****************************************************************************
 * XBee - Serial bridge
 *
 *  Copyright (C) 2013 Century Systems
 *  Author: Takeyoshi Kikuchi <kikuchi@centurysys.co.jp>
 *
 *  Last Modified: 2013/01/21 17:54:50 kikuchi
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <errno.h>

#include "common.h"

#define BUFSIZE 512

static int uart_fds[2] = {-1, -1}; /* 0: RS232C / 1: XBee */

char xbee_buf[BUFSIZE];

static void *xbee_bridge_thread(void *pvarg);

static void usage(void)
{
}

int xbee_bridge_main(int argc, char **argv)
{
    int i, fd, pid;
    char *ports[2] = {"/dev/ttyS232", "/dev/ttySXBee"};
    struct termios tty_old, tty_new;

    for (i = 0; i < 2; i++) {
        if ((fd = open(ports[i], O_RDWR)) < 0) {
            info("open(%s) failed, '%s'\n", ports[i], strerror(errno));
            goto ret1;
        }

        info(" open(%s) --> fd: %d\n", ports[i], fd);
        uart_fds[i] = fd;

        if (tcgetattr(fd, &tty_old) != 0) {
            info("tcgetattr() failed, '%s'\n", strerror(errno));
            goto ret1;
        }

        memcpy(&tty_new, &tty_old, sizeof(struct termios));

        if (cfsetspeed(&tty_new, 115200) != 0) {
            info("cfsetspeed() failed, '%s'\n", strerror(errno));
            goto ret1;
        }

        if (tcsetattr(fd, TCSANOW, &tty_new) != 0) {
            info("tcsetattr() failed, '%s'\n", strerror(errno));
            goto ret1;
        }
    }

    if (pthread_create(&pid, NULL, xbee_bridge_thread, (void *) uart_fds) == 0)
        pthread_setname_np(pid, "XBeeBridge");

    return 0;

ret1:
    for (i = 0; i < 2; i++) {
        if (uart_fds[i] != -1)
            close(uart_fds[i]);
    }

    return -1;
}

static void uart_proxy(int fd0, int fd1, char *buf, int bufsize)
{
    char *uartbuf = buf;
    struct pollfd fds[2];
    int i, res, readlen, writelen, len;

    memset(uartbuf, 0, bufsize);
    while (1) {
        memset(fds, 0, ARRAY_SIZE(fds));

        fds[0].fd = fd0;
        fds[0].events = POLLIN;
        fds[1].fd = fd1;
        fds[1].events = POLLIN;

        res = poll(fds, 2, 1000);
        if (res < 0) {
            info("%s: poll() failed with %d\n", __FUNCTION__, res);
            break;
        } else if (res == 0)
            /* timeout */
            continue;

        for (i = 0; i < 2; i++) {
            if (fds[i].revents & POLLIN) {
                memset(uartbuf, 0, bufsize);

                readlen = read(fds[i].fd, uartbuf, bufsize);

                if (readlen <= 0)
                    continue;

                //info("- %d - : read %d bytes.\n", i, readlen);

                writelen = 0;
                while (writelen < readlen) {
                    len = write(fds[1 - i].fd, uartbuf + writelen, readlen - writelen);
                    if (len <= 0)
                        break;

                    //info(" -> %d\n", len);
                    writelen += len;
                }
            }
        }
    }
}

static void *xbee_bridge_thread(void *pvarg)
{
    uart_proxy(uart_fds[0], uart_fds[1], xbee_buf, BUFSIZE);

    return NULL;
}
