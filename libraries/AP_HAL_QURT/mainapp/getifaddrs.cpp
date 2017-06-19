/* 
   get system network addresses

   based on code from Samba

   Copyright (C) Andrew Tridgell 1998
   Copyright (C) Jeremy Allison 2007
   Copyright (C) Jelmer Vernooij <jelmer@samba.org> 2007

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/time.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>

void freeifaddrs(struct ifaddrs *ifp)
{
	if (ifp != nullptr) {
		free(ifp->ifa_name);
		free(ifp->ifa_addr);
		free(ifp->ifa_netmask);
		free(ifp->ifa_dstaddr);
		freeifaddrs(ifp->ifa_next);
		free(ifp);
	}
}

static struct sockaddr *sockaddr_dup(struct sockaddr *sa)
{
	struct sockaddr *ret;
	socklen_t socklen;
	socklen = sizeof(struct sockaddr_storage);
	ret = (struct sockaddr *)calloc(1, socklen);
	if (ret == nullptr)
		return nullptr;
	memcpy(ret, sa, socklen);
	return ret;
}

/* this works for Linux 2.2, Solaris 2.5, SunOS4, HPUX 10.20, OSF1
   V4.0, Ultrix 4.4, SCO Unix 3.2, IRIX 6.4 and FreeBSD 3.2.

   It probably also works on any BSD style system.  */

int getifaddrs(struct ifaddrs **ifap)
{
	struct ifconf ifc;
	char buff[8192];
	int fd, i, n;
	struct ifreq *ifr=nullptr;
	struct ifaddrs *curif;
	struct ifaddrs *lastif = nullptr;

	*ifap = nullptr;

	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		return -1;
	}
  
	ifc.ifc_len = sizeof(buff);
	ifc.ifc_buf = buff;

	if (ioctl(fd, SIOCGIFCONF, &ifc) != 0) {
		close(fd);
		return -1;
	} 

	ifr = ifc.ifc_req;
  
	n = ifc.ifc_len / sizeof(struct ifreq);

	/* Loop through interfaces, looking for given IP address */
	for (i=n-1; i>=0; i--) {
		if (ioctl(fd, SIOCGIFFLAGS, &ifr[i]) == -1) {
			freeifaddrs(*ifap);
			close(fd);
			return -1;
		}

		curif = (struct ifaddrs *)calloc(1, sizeof(struct ifaddrs));
		if (curif == nullptr) {
			freeifaddrs(*ifap);
			close(fd);
			return -1;
		}
		curif->ifa_name = strdup(ifr[i].ifr_name);
		if (curif->ifa_name == nullptr) {
			free(curif);
			freeifaddrs(*ifap);
			close(fd);
			return -1;
		}
		curif->ifa_flags = ifr[i].ifr_flags;
		curif->ifa_dstaddr = nullptr;
		curif->ifa_data = nullptr;
		curif->ifa_next = nullptr;

		curif->ifa_addr = nullptr;
		if (ioctl(fd, SIOCGIFADDR, &ifr[i]) != -1) {
			curif->ifa_addr = sockaddr_dup(&ifr[i].ifr_addr);
			if (curif->ifa_addr == nullptr) {
				free(curif->ifa_name);
				free(curif);
				freeifaddrs(*ifap);
				close(fd);
				return -1;
			}
		}

		curif->ifa_netmask = nullptr;
		if (ioctl(fd, SIOCGIFNETMASK, &ifr[i]) != -1) {
			curif->ifa_netmask = sockaddr_dup(&ifr[i].ifr_addr);
			if (curif->ifa_netmask == nullptr) {
				if (curif->ifa_addr != nullptr) {
					free(curif->ifa_addr);
				}
				free(curif->ifa_name);
				free(curif);
				freeifaddrs(*ifap);
				close(fd);
				return -1;
			}
		}

		if (lastif == nullptr) {
			*ifap = curif;
		} else {
			lastif->ifa_next = curif;
		}
		lastif = curif;
	}

	close(fd);

	return 0;
}  

const char *get_ipv4_broadcast(void)
{
    struct ifaddrs *ifap = nullptr;
    if (getifaddrs(&ifap) != 0) {
        return nullptr;
    }
    struct ifaddrs *ia;
    for (ia=ifap; ia; ia=ia->ifa_next) {
        struct sockaddr_in *sin = (struct sockaddr_in *)ia->ifa_addr;
        struct sockaddr_in *nmask = (struct sockaddr_in *)ia->ifa_netmask;
        struct in_addr bcast;
        if (strcmp(ia->ifa_name, "lo") == 0) {
            continue;
        }
        bcast.s_addr = sin->sin_addr.s_addr | ~nmask->sin_addr.s_addr;
        const char *ret = inet_ntoa(bcast);
        freeifaddrs(ifap);
        return ret;
    }
    freeifaddrs(ifap);
    return nullptr;
}

#ifdef MAIN_PROGRAM
int main(void)
{
    printf("%s\n", get_ipv4_broadcast());
    return 0;
}
#endif
