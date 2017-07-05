/**
 * @file NOR_TZ10xx.h
 * @brief a header file for TZ10xx NOR driver
 * @date $Date:: 2014-03-12 10:40:51 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef NOR_TZ10XX_H
#define NOR_TZ10XX_H

#include "Driver_Common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NOR_FLASH_NAME  0              /* Flash Device Name */
#define NOR_FLASH_SIZE  0x100000             /* Flash Device Size in bytes 1MB */
#define NOR_FLASH_EVAL  0xFF                  /* Contents of Erased Memory */

#define NOR_FLASH_SECTOR_SIZE    0x1000      /* Sector Size */

#define NOR_FLASH_SECTORS                                             \
  ARM_NOR_SECTOR_INFO(0x00000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x01000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x02000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x03000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x04000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x05000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x06000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x07000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x08000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x09000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x0a000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x0b000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x0c000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x0d000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x0e000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x0f000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x10000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x11000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x12000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x13000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x14000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x15000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x16000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x17000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x18000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x19000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x1a000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x1b000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x1c000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x1d000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x1e000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x1f000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x20000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x21000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x22000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x23000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x24000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x25000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x26000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x27000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x28000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x29000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x2a000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x2b000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x2c000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x2d000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x2e000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x2f000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x30000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x31000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x32000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x33000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x34000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x35000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x36000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x37000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x38000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x39000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x3a000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x3b000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x3c000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x3d000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x3e000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x3f000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x40000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x41000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x42000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x43000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x44000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x45000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x46000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x47000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x48000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x49000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x4a000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x4b000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x4c000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x4d000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x4e000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x4f000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x50000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x51000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x52000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x53000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x54000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x55000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x56000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x57000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x58000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x59000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x5a000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x5b000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x5c000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x5d000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x5e000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x5f000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x60000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x61000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x62000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x63000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x64000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x65000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x66000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x67000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x68000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x69000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x6a000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x6b000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x6c000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x6d000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x6e000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x6f000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x70000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x71000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x72000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x73000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x74000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x75000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x76000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x77000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x78000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x79000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x7a000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x7b000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x7c000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x7d000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x7e000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x7f000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x80000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x81000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x82000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x83000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x84000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x85000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x86000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x87000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x88000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x89000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x8a000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x8b000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x8c000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x8d000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x8e000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x8f000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x90000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x91000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x92000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x93000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x94000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x95000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x96000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x97000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x98000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x99000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x9a000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x9b000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x9c000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x9d000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x9e000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0x9f000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xa0000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xa1000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xa2000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xa3000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xa4000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xa5000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xa6000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xa7000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xa8000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xa9000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xaa000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xab000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xac000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xad000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xae000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xaf000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xb0000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xb1000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xb2000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xb3000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xb4000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xb5000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xb6000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xb7000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xb8000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xb9000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xba000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xbb000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xbc000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xbd000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xbe000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xbf000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xc0000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xc1000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xc2000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xc3000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xc4000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xc5000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xc6000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xc7000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xc8000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xc9000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xca000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xcb000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xcc000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xcd000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xce000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xcf000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xd0000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xd1000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xd2000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xd3000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xd4000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xd5000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xd6000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xd7000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xd8000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xd9000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xda000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xdb000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xdc000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xdd000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xde000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xdf000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xe0000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xe1000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xe2000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xe3000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xe4000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xe5000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xe6000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xe7000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xe8000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xe9000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xea000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xeb000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xec000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xed000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xee000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xef000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xf0000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xf1000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xf2000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xf3000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xf4000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xf5000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xf6000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xf7000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xf8000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xf9000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xfa000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xfb000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xfc000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xfd000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xfe000, 0x1000),   /* Sector Size 4kB */ \
  ARM_NOR_SECTOR_INFO(0xff000, 0x1000),   /* Sector Size 4kB */ 

#define NOR_FLASH_SECTOR_NUM    256           /* Number of Sectors */


/**
 * @brief size of write protection
 */
typedef enum _NOR_WRITE_PROTECT_SIZE {
	NOR_0KB = 0, /**< unprotect */
	NOR_4KB,    /**< protect 4Kbyte  */
	NOR_8KB,    /**< protect 8Kbyte  */
	NOR_16KB,    /**< protect 16Kbyte  */
	NOR_32KB,    /**< protect 32Kbyte  */
	NOR_64KB,    /**< protect 64Kbyte  */
	NOR_128KB,    /**< protect 128Kbyte  */
	NOR_256KB,    /**< protect 256Kbyte  */
	NOR_512KB,    /**< protect 512Kbyte  */
	NOR_768KB,    /**< protect 768Kbyte  */
	NOR_896KB,    /**< protect 896Kbyte  */
	NOR_960KB,    /**< protect 960Kbyte  */
	NOR_992KB,    /**< protect 992Kbyte  */
	NOR_1008KB,    /**< protect 1008Kbyte  */
	NOR_1016KB,    /**< protect 1016Kbyte  */
	NOR_1020KB,    /**< protect 1020Kbyte  */
	NOR_1024KB,      /**< Protect all */
} NOR_WRITE_PROTECT_SIZE;


/**
 * @brief write protect from top or bottom of NOR Flash
 */
typedef enum _NOR_WRITE_PROTECT_TB {
	NOR_TOP = 0,   /**< protect from top of array */
	NOR_BOTTOM,    /**< Protect from bottom of array */
} NOR_WRITE_PROTECT_TB;

/**
 * @brief driver function pointers
 */
typedef struct __TZ10XX_DRIVER_NOR {
  ARM_DRIVER_VERSION (*GetVersion)   (void);                                               ///< Pointer to \ref ARM_NOR_GetVersion : Get driver version.
  ARM_NOR_STATUS  (*Initialize)   (uint32_t base_addr);                                 ///< Pointer to \ref ARM_NOR_Initialize : Initialize NOR Flash Interface.
  ARM_NOR_STATUS  (*Uninitialize) (void);                                               ///< Pointer to \ref ARM_NOR_Uninitialize : De-initialize NOR Flash Interface.
  ARM_NOR_STATUS  (*PowerControl) (ARM_POWER_STATE state);                              ///< Pointer to \ref ARM_NOR_PowerControl : Control NOR Flash Interface Power.
  ARM_NOR_STATUS  (*ReadData)     (uint32_t addr,       uint8_t *data, uint32_t size);  ///< Pointer to \ref ARM_NOR_ReadData : Read data from NOR Flash. Optional, NULL for parallel memory-mapped devices.
  ARM_NOR_STATUS  (*WriteData)    (uint32_t addr, const uint8_t *data, uint32_t size);  ///< Pointer to \ref ARM_NOR_WriteData : Write data to NOR Flash.
  ARM_NOR_STATUS  (*EraseSector)  (uint32_t addr);                                      ///< Pointer to \ref ARM_NOR_EraseSector : Erase NOR Flash Sector.
  ARM_NOR_STATUS  (*EraseChip)    (void);                                               ///< Pointer to \ref ARM_NOR_EraseChip : Erase complete NOR Flash. Optional function.
	ARM_NOR_STATUS  (*WriteProtect)    (NOR_WRITE_PROTECT_SIZE size, NOR_WRITE_PROTECT_TB tb);
  ARM_NOR_STATUS  (*ReadSecurityRegister)   (uint32_t addr,  uint8_t *data, uint32_t size);  ///< Pointer to \ref ARM_NOR_ReadSecurityRegister : Read data from NOR Flash SecurityRegister.
  ARM_NOR_STATUS  (*WriteSecurityRegister)   (uint32_t addr,  const uint8_t *data, uint32_t size);  ///< Pointer to \ref ARM_NOR_WriteSecurityRegister : Write data to NOR Flash SecurityRegister.
	ARM_NOR_STATUS  (*EraseSecurityRegister)  (uint32_t addr);                    ///< Pointer to \ref ARM_NOR_EraseSecurityRegister : Erase NOR Flash SecurityRegister.
  ARM_NOR_STATUS  (*GetID)     (uint8_t *manufacturer, uint8_t *device, uint8_t *unique);  ///< Pointer to \ref ARM_NOR_ReadID : Get IDs of NOR Flash.
  ARM_NOR_STATUS  (*GetQuad)  (uint8_t *qe);                          ///< Pointer to \ref ARM_NOR_Quad : Get quad status.
  ARM_NOR_STATUS  (*SetQuad)  (uint8_t qe);                          ///< Pointer to \ref ARM_NOR_SetQuad : Set quad status.
  } const TZ10XX_DRIVER_NOR;



	#ifdef __cplusplus
}
#endif

#endif /* NOR_TZ10XX_H */
