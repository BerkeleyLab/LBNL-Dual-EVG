/*
 * Copyright 2020, Lawrence Berkeley National Laboratory
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS
 * AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "gpio.h"
#include "bwudp.h"

#define ETHERNET_PAYLOAD_CAPACITY   1500

#ifndef BWUDP_ENDPOINT_CAPACITY
#define BWUDP_ENDPOINT_CAPACITY 4
#endif

/*****************************************************************************
 * IPv4 header
 */
#define IPV4_TYPE_UDP   17
struct ipv4Header {
    uint8_t         type_len;
    uint8_t         dscp_ecn;
    uint16_t        length;
    uint16_t        identification;
    uint16_t        flags_offset;
    uint8_t         ttl;
    uint8_t         protocol;
    uint16_t        checksum;
    ipv4Address     source;
    ipv4Address     destination;
};

/*****************************************************************************
 * UDP header
 */
struct udpHeader {
    uint16_t    sourcePort;
    uint16_t    destinationPort;
    uint16_t    length;
    uint16_t    checksum;
};

/*****************************************************************************
 * Ethernet frame offset to place IPv4 header on 32-bit boundary
 */
struct offsetUDPframe {
    char              pad[2];
    uint8_t           destinationMAC[6];
    uint8_t           sourceMAC[6];
    uint8_t           protocol[2];
    struct ipv4Header ipv4;
    struct udpHeader  udp;
    char              payload[ETHERNET_PAYLOAD_CAPACITY -
                            sizeof(struct ipv4Header)-sizeof(struct udpHeader)];
};

/*****************************************************************************
 * ARP
 */
struct arpPacket {
    uint8_t htype[2];
    uint8_t ptype[2];
    uint8_t hlen;
    uint8_t plen;
    uint8_t oper[2];
    uint8_t sha[6];
    uint8_t spa[4];
    uint8_t tha[6];
    uint8_t tpa[4];
};
struct offsetARPframe {
    char      pad[2];
    uint8_t   destinationMAC[6];
    uint8_t   sourceMAC[6];
    uint8_t   protocol[2];
    struct arpPacket arp;
};

/*****************************************************************************
 * Per-client/server workspace
 */
struct bwudpEndpoint {
    struct bwudpEndpoint  *next;
    struct bwudpInterface *interface;
    ethernetMAC            farMAC;
    ipv4Address            farAddress;
    uint16_t               farPort;
    uint16_t               nearPort;
    bwudpCallback          callback;
#ifdef BWUDP_ENABLE_CLIENT_SUPPORT
    ipv4Address            arpAddresss;
#endif
};
static struct bwudpEndpoint endpoints[BWUDP_ENDPOINT_CAPACITY];
static int nextFreeEndpoint = 0;

/*****************************************************************************
 * Per-interface workspace
 */
struct bwudpInterface {
#if BWUDP_INTERFACE_CAPACITY > 1
    int                        index,
#endif
    ethernetMAC                myEthernetMAC;
    ipv4Address                myAddress;
    struct bwudpEndpoint      *endpoints;
    struct offsetUDPframe      rxFrame;
    struct offsetUDPframe      txFrame;
    struct bwudpStatistics     statistics;
#ifdef BWUDP_ENABLE_CLIENT_SUPPORT
    ipv4Address             myNetmask;
    ipv4Address             myGateway;
    struct offsetARPframe   arpFrame;
    struct bwudpEndpoint   *arpEndpoint;
    int                     deferredTxFrameLength;
#endif
};
static struct bwudpInterface interfaces[BWUDP_INTERFACE_CAPACITY];
#if BWUDP_INTERFACE_CAPACITY > 1
# define INTERFACE_INDEX ip->index,
#else
# define INTERFACE_INDEX
#endif

#ifdef BWUDP_ENABLE_CLIENT_SUPPORT
static struct bwudpInterface *defaultRouteInterface;
static int
isOnNetwork(struct bwudpInterface *ip, ipv4Address *addr)
{
    unsigned int i;
    for (i = 0 ; i < sizeof addr->a ; i++) {
        if ((addr->a[i] & ip->myNetmask.a[i]) !=
                                    (ip->myAddress.a[i] & ip->myNetmask.a[i])) {
            return 0;
        }
    }
    return 1;
}
#endif

static uint16_t
headerChecksum(struct ipv4Header *ip)
{
    unsigned int i;
    uint32_t sum = 0;
    uint16_t *p16 = (uint16_t *)ip, c;
    ip->checksum = 0;
    for (i = 0 ; i < sizeof(*ip)/sizeof(*p16) ; i++, p16++) {
        sum += ntohs(*p16);
    }
    sum = (sum & 0xFFFF) + (sum >> 16);
    c = ~((sum & 0xFFFF) + (sum >> 16));
    return htons(c);
}

int
bwudpRegisterInterface(BWUDP_INTERFACE_INDEX
                       ethernetMAC *eMAC,
                       ipv4Address *address,
                       ipv4Address *netmask,
                       ipv4Address *gateway)
{
#if BWUDP_INTERFACE_CAPACITY == 1
    int interfaceIndex = 0;
#endif
    struct bwudpInterface *ip = &interfaces[interfaceIndex];
    if (ip->txFrame.protocol[0] != 0) {
        return -1;
    }
    ip->myEthernetMAC = *eMAC;
    ip->myAddress = *address;
#if BWUDP_INTERFACE_CAPACITY > 1
    ip->index = interfaceIndex;
#endif
#ifdef BWUDP_ENABLE_CLIENT_SUPPORT
    if (netmask) {
        ip->myNetmask = *netmask;
        if (gateway) {
            ip->myGateway = *gateway;
            defaultRouteInterface = ip;
        }
    }
    else {
        memset(&ip->myNetmask, 0xFF, sizeof(ipv4Address));
    }
#endif
    memcpy(&ip->txFrame.sourceMAC, &ip->myEthernetMAC, sizeof(struct ethernetMAC));
    ip->txFrame.protocol[0] = 0x08;
    ip->txFrame.protocol[1] = 0x00;
    ip->txFrame.ipv4.type_len = 0x45;
    ip->txFrame.ipv4.ttl = 5;
    ip->txFrame.ipv4.protocol = IPV4_TYPE_UDP;
    ip->txFrame.ipv4.source = ip->myAddress;
    bwudpInitializeInterface(INTERFACE_INDEX ip->myEthernetMAC.a,
                                             ip->myAddress.a);
    return 0;
}

void
bwudpCrank(void)
{
    struct bwudpInterface *ip = interfaces;
#if BWUDP_INTERFACE_CAPACITY > 1
    for ( ; ip < &interfaces[nextFreeInterface]  ; ip++) {
#endif
    int length = bwudpFetchFrame(INTERFACE_INDEX &ip->rxFrame.destinationMAC);
    if (length > 0) {
        int protocol = (ip->rxFrame.protocol[0] << 8) | ip->rxFrame.protocol[1];
        if (protocol == 0x0800) {
            struct bwudpEndpoint *ep;
            for (ep = ip->endpoints ; ep != NULL ; ep = ep->next) {
                if (ip->rxFrame.udp.destinationPort == ep->nearPort) {
                    int payloadLength = ntohs(ip->rxFrame.udp.length) - 8;
                    int framePayloadCapacity = length - 14 -
                                                    sizeof(struct ipv4Header) -
                                                    sizeof(struct udpHeader);
                    if ((payloadLength < 0)
                     || (payloadLength > framePayloadCapacity)) {
                        ip->statistics.mangled++;
                    }
                    else {
                        memcpy(&ep->farMAC, &ip->rxFrame.sourceMAC,
                                                           sizeof(ethernetMAC));
                        memcpy(&ep->farAddress, &ip->rxFrame.ipv4.source,
                                                           sizeof(ipv4Address));
                        ep->farPort = ip->rxFrame.udp.sourcePort;
                        ip->statistics.accepted++;
                        ep->callback(ep, ip->rxFrame.payload, payloadLength);
                    }
                    break;
                }
            }
            if (ep == NULL) {
                ip->statistics.rejected++;
            }
        }
#ifdef BWUDP_ENABLE_CLIENT_SUPPORT
        else if (protocol == 0x0806) {
            struct bwudpEndpoint *ep = ip->arpEndpoint;
            struct arpPacket *ap = (struct arpPacket *)&ip->rxFrame.ipv4;
            if ((length + sizeof(ip->arpFrame).pad >= (int) sizeof(ip->arpFrame))
             && (ep != NULL)
             && (ep->next == ep)
             && !memcmp(ap->spa, &ep->arpAddresss, sizeof(ipv4Address))) {
                ip->arpEndpoint = NULL;
                ep->next = ip->endpoints;
                ip->endpoints = ep;
                memcpy(&ep->farMAC, ap->sha, sizeof(ethernetMAC));
                memcpy(&ip->txFrame.destinationMAC, &ep->farMAC,
                                                           sizeof(ethernetMAC));
                bwudpSendFrame(INTERFACE_INDEX &ip->txFrame.destinationMAC,
                                                     ip->deferredTxFrameLength);
            }
            ip->statistics.arp++;
        }
#endif
        else {
            ip->statistics.badProtocol++;
        }
    }
#if BWUDP_INTERFACE_CAPACITY > 1
    }
#endif
}

const struct bwudpStatistics *
#if BWUDP_INTERFACE_CAPACITY > 1
bwudpStatistics(int idx) { return &interfaces[idx].statistics; }
#else
bwudpStatistics(void) { return &interfaces[0].statistics; }
#endif

/*
 * Send a packet
 */
void
bwudpSend(bwudpHandle handle, const char *payload, int length)
{
    struct bwudpEndpoint *ep = handle;
    struct bwudpInterface *ip = ep->interface;;
    int l = sizeof(struct ipv4Header) + sizeof(struct udpHeader) + length;
    if (l > ETHERNET_PAYLOAD_CAPACITY) {
        return;
    }
    memcpy(&ip->txFrame.destinationMAC, &ep->farMAC, sizeof(ethernetMAC));
    ip->txFrame.ipv4.destination =  ep->farAddress;
    ip->txFrame.ipv4.length = htons(l);
    ip->txFrame.ipv4.checksum = headerChecksum(&ip->txFrame.ipv4);
    ip->txFrame.udp.sourcePort =  ep->nearPort;
    ip->txFrame.udp.destinationPort =  ep->farPort;
    length += sizeof(struct udpHeader);
    ip->txFrame.udp.length = htons(length);
    memcpy(ip->txFrame.payload, payload, length);
    l += sizeof(ip->txFrame.destinationMAC) + sizeof(ip->txFrame.sourceMAC) +
                                                   sizeof(ip->txFrame.protocol);
    if (l < 60) l = 60;
#ifdef BWUDP_ENABLE_CLIENT_SUPPORT
    if (ep->next == ep) {
        /* Fill in and send ARP request then return */
        if ((ip->arpEndpoint != NULL) && (ip->arpEndpoint != ep)) {
            return;
        }
        if (isOnNetwork(ip, &ep->farAddress)) {
            memcpy(&ep->arpAddresss, &ep->farAddress, sizeof(ipv4Address));
        }
        else {
            if (!defaultRouteInterface) return;
            memcpy(&ep->arpAddresss, &defaultRouteInterface->myGateway,
                                                           sizeof(ipv4Address));
        }
        ip->deferredTxFrameLength = l;
        ip->arpEndpoint = ep;
        memset(&ip->arpFrame.destinationMAC, 0xFF, sizeof(ethernetMAC));
        memcpy(&ip->arpFrame.sourceMAC, &ip->myEthernetMAC,sizeof(ethernetMAC));
        ip->arpFrame.protocol[0] = 0x08;    /* 0806 -- ARP */
        ip->arpFrame.protocol[1] = 0x06;
        ip->arpFrame.arp.htype[0] = 0x000;  /* 0001 -- Ethernet */
        ip->arpFrame.arp.htype[1] = 0x01;
        ip->arpFrame.arp.ptype[0] = 0x08;   /* 0800 -- IPv4 */
        ip->arpFrame.arp.ptype[1] = 0x00;
        ip->arpFrame.arp.hlen = sizeof(ethernetMAC);
        ip->arpFrame.arp.plen = sizeof(ipv4Address);
        ip->arpFrame.arp.oper[0] = 0x00;    /* 0001 -- Request */
        ip->arpFrame.arp.oper[1] = 0x01;
        memcpy(ip->arpFrame.arp.sha, &ip->myEthernetMAC, sizeof(ethernetMAC));
        memcpy(ip->arpFrame.arp.spa, &ip->myAddress, sizeof(ipv4Address));
        memset(ip->arpFrame.arp.tha, 0, sizeof(ethernetMAC));
        memcpy(ip->arpFrame.arp.tpa, &ep->arpAddresss, sizeof(ipv4Address));
        bwudpSendFrame(INTERFACE_INDEX &ip->arpFrame.destinationMAC, 60);
        return;
    }
#endif
    bwudpSendFrame(INTERFACE_INDEX &ip->txFrame.destinationMAC, l);
}

int
bwudpRegisterServer(BWUDP_INTERFACE_INDEX int port, bwudpCallback callback)
{
#if BWUDP_INTERFACE_CAPACITY == 1
    int interfaceIndex = 0;
#endif
    struct bwudpInterface *ip = &interfaces[interfaceIndex];
    struct bwudpEndpoint *ep;
    if (nextFreeEndpoint >= BWUDP_ENDPOINT_CAPACITY) {
        return -1;
    }
    ep = &endpoints[nextFreeEndpoint++];;
    ep->interface = ip;
    ep->nearPort = port;
    ep->callback = callback;
    ep->next = ip->endpoints;
    ip->endpoints = ep;
    return 0;
}

#ifdef BWUDP_ENABLE_CLIENT_SUPPORT
bwudpHandle
bwudpCreateClient(ipv4Address *serverAddress, int serverPort,
                                          int localPort, bwudpCallback callback)
{
    struct bwudpInterface *ip = defaultRouteInterface;
    struct bwudpEndpoint *ep;
    int i;
    for (i = 0 ; i < nextFreeEndpoint ; i++) {
        if (isOnNetwork(&interfaces[i], serverAddress)) {
            ip = &interfaces[i];
            break;
        }
    }
    if ((ip == NULL) || (nextFreeEndpoint >= BWUDP_ENDPOINT_CAPACITY)) {
        return NULL;
    }
    ep = &endpoints[nextFreeEndpoint++];;
    ep->interface = ip;
    memcpy(&ep->farAddress, serverAddress, sizeof(ipv4Address));
    ep->nearPort = localPort;
    ep->farPort = serverPort;
    ep->callback = callback;
    ep->next = ep;
    return ep;
}
#endif
