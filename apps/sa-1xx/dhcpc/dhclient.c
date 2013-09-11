/****************************************************************************
 * DHCP Client utility
 *
 *  Copyright (C) 2012 Century Systems
 *  Author: Takeyoshi Kikuchi <kikuchi@centurysys.co.jp>
 *
 *  Last Modified: 2012/11/21 15:57:31 kikuchi
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <net/if.h>

#include <apps/netutils/uiplib.h>
#include <apps/netutils/resolv.h>
#include <apps/netutils/dhcpc.h>

#include "common.h"

int dhcpc_main(int argc, char **argv)
{
    void *handle;
    uint8_t mac[IFHWADDRLEN];

    uip_getmacaddr("eth0", mac);

    /* Set up the DHCPC modules */
    handle = dhcpc_open(&mac, IFHWADDRLEN);

    /* Get an IP address.  Note that there is no logic for renewing the IP address in this
     * example.  The address should be renewed in ds.lease_time/2 seconds.
     */

    if (handle) {
        struct dhcpc_state ds;

        (void) dhcpc_request(handle, &ds);
        uip_sethostaddr("eth0", &ds.ipaddr);

        if (ds.netmask.s_addr != 0)
            uip_setnetmask("eth0", &ds.netmask);

        if (ds.default_router.s_addr != 0)
            uip_setdraddr("eth0", &ds.default_router);

        if (ds.dnsaddr.s_addr != 0)
            resolv_conf(&ds.dnsaddr);

        dhcpc_close(handle);
    }

    return OK;
}
