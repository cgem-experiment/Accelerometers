/*
 * net_config.h  — app-level network settings
 *
 *  Created on: Sep 4, 2025
 *      Author: marq1234
 */

#pragma once

// STM32 (this device)
#define NET_IPADDR0   192
#define NET_IPADDR1   168
#define NET_IPADDR2   1
#define NET_IPADDR3   30

#define NET_NETMASK0  255
#define NET_NETMASK1  255
#define NET_NETMASK2  255
#define NET_NETMASK3  0

#define NET_GW0       192
#define NET_GW1       168
#define NET_GW2       1
#define NET_GW3       1

// PC/Destination (where you’re sending UDP)
#define NET_PCIP0     192
#define NET_PCIP1     168
#define NET_PCIP2     1
#define NET_PCIP3     10

// UDP ports
#define NET_ACCEL_SRC_PORT  40001       // Accelerometer data (high rate) source (MCU) port
#define NET_HK_SRC_PORT (NET_ACCEL_SRC_PORT + 1) //Housekeeping data source (MCU) port

#define NET_ACCEL_DST_PORT  12345   // Accelerometer data destination (PC) port
#define NET_HK_DST_PORT NET_ACCEL_DST_PORT // Housekeeping data destination (PC) port, use same port for now and filter by SYNC
