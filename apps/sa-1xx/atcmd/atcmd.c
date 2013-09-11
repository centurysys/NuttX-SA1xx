/****************************************************************************
 * AT command test
 *
 *  Copyright (C) 2012 Century Systems
 *  Author: Takeyoshi Kikuchi <kikuchi@centurysys.co.jp>
 *
 *  Last Modified: 2013/01/18 13:55:50 kikuchi
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

static void usage(void)
{
    info("usage: atcmd [OPTION] send_string\n");
    info("  example: atcmd -a -d /dev/ttyUSB2 -t 100 -s 115200 ATI\n\n");
    info("  -a  AT command mode\n");
    info("  -d  select device\n");
    info("  -s  set UART speed (default: 9600)\n");
    info("  -t  repeat times\n");
}

int atcmd_main(int argc, char **argv)
{
    int i, serfd, writelen, option, recvlen, res = -1;
    int speed = 9600, times = 1, mode_at = 0;
    char *path, *cmd, *buf, *ptr;
    struct pollfd fds[1];
    struct termios tty_old, tty_new;

    while ((option = getopt(argc, argv, "d:s:t:a")) != ERROR) {
        switch (option) {
        case 'd':
            path = optarg;
            break;

        case 's':
            speed = atoi(optarg);
            break;

        case 't':
            times = atoi(optarg);
            break;

        case 'a':
            mode_at = 1;
            break;

        default:
            usage();
            return -1;
        }
    }

    argc -= optind;
    argv += optind;

    if (argc != 1) {
        usage();
        return -1;
    }

    cmd = argv[0];

    if (!(buf = zalloc(2048))) {
        info("zalloc() failed, '%s'\n", strerror(errno));
        return -1;
    }

    if ((serfd = open(path, O_RDWR)) < 0) {
        info("open(%s) failed, '%s'\n", path, strerror(errno));
        goto ret1;
    }

    if (tcgetattr(serfd, &tty_old) != 0) {
        info("tcgetattr() failed, '%s'\n", strerror(errno));
        goto ret2;
    }

    memcpy(&tty_new, &tty_old, sizeof(struct termios));

    if (cfsetspeed(&tty_new, speed) != 0) {
        info("cfsetspeed() failed, '%s'\n", strerror(errno));
        goto ret3;
    }

    if (tcsetattr(serfd, TCSANOW, &tty_new) != 0) {
        info("tcsetattr() failed, '%s'\n", strerror(errno));
        goto ret3;
    }

    for (i = 0; i < times + mode_at; i++) {
        if (mode_at == 1 && i == 0) {
            writelen = write(serfd, "+++", 3);
            usleep(1000);
        } else {
            info("=== AT command cycle: '%d'\n", i);

            writelen = write(serfd, cmd, strlen(cmd));
            if (writelen <= 0) {
                info("!!! write '%s' failed (%d), %d\n", cmd, writelen, errno);
                break;
            }

            writelen = write(serfd, "\r\n", 2);
            if (writelen <= 0) {
                info("!!! write '\\r\\n' failed (%d), %d\n", writelen, errno);
                break;
            }

            info("# write '%s' succeeded.\n", cmd);
        }

        ptr = buf;
        memset(buf, 0, 2048);

        while (1) {
            memset(fds, 0, sizeof(fds));

            fds[0].fd = serfd;
            fds[0].events = POLLIN;

            res = poll(fds, 1, 1000);
            if (res < 0) {
                /* timeout or error */
                info("!!! poll() failed (%d), %d\n", res, errno);
                break;
            } else if (res == 0) {
                info("# poll() timeouted...\n", res, errno);
                break;
            }

            if (fds[0].revents & POLLIN) {
                recvlen = read(serfd, ptr, 2048 - (int) (ptr - buf));
                if (recvlen <= 0)
                    break;

                //info("%d bytes received.\n", recvlen);
                ptr += recvlen;
            }
        }

        if (ptr != buf) {
            info("\n--- received buf (%d bytes) ---\n", (int) (ptr - buf));
            info("%s\n", buf);
            info("--------------------\n\n");
        }
    }

    info("\n");

ret3:
    tcsetattr(serfd, TCSANOW, &tty_old);
ret2:
    close(serfd);
ret1:
    free(buf);

    return res;
}
