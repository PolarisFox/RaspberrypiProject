/*****************************************************************************
* | File        :   DEV_Config.h
* | Author      :   Waveshare team (modified)
* | Function    :   Hardware underlying interface
* | Info        :   GPIO via bcm2835, SPI via kernel spidev.
*                   SPI is exposed as whole transactions (CS low -> N bytes ->
*                   CS high in a single ioctl) so the kernel cannot interleave
*                   another device's (MCP2515) traffic in the middle of an
*                   ADS1263 command.
*----------------
* Original: V1.0 2020-12-21, MIT-style license (see ADS1263.c)
******************************************************************************/
#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include "Debug.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

/* ---------------------------------------------------------------------------
 * Build-time configuration (override with -D... or via Makefile variables)
 *
 * ADC_SPI_DEVICE  : spidev node used for the ADS1263.
 * ADC_KERNEL_CS   : 1 = the kernel drives CS (GPIO22 declared as cs-gpio in
 *                       the device tree, e.g. dtoverlay=spi0-2cs,cs1_pin=22).
 *                       This is the fully safe mode when sharing SPI0 with the
 *                       MCP2515 driver.
 *                   0 = (default) user space drives GPIO22 around each
 *                       transaction. Works with the current wiring/config, but
 *                       a kernel CAN transfer can still slip in between the
 *                       GPIO toggle and the ioctl. Corrupted reads are caught
 *                       by the ADS1263 checksum and reported as errors.
 * ------------------------------------------------------------------------- */
#ifndef ADC_SPI_DEVICE
#define ADC_SPI_DEVICE "/dev/spidev0.1"
#endif
#ifndef ADC_KERNEL_CS
#define ADC_KERNEL_CS 0
#endif
#ifndef ADC_SPI_SPEED_HZ
#define ADC_SPI_SPEED_HZ 2000000u
#endif

extern int DEV_RST_PIN;
extern int DEV_CS_PIN;
extern int DEV_DRDY_PIN;

void  DEV_Digital_Write(UWORD Pin, UBYTE Value);
UBYTE DEV_Digital_Read(UWORD Pin);

/* Full-duplex SPI transaction with CS asserted for the whole buffer.
 * rx may be NULL. Returns 0 on success, -1 on failure. Thread-safe. */
int DEV_SPI_Transfer(const UBYTE *tx, UBYTE *rx, size_t len);

UBYTE DEV_Module_Init(void);
void  DEV_Module_Exit(void);

void DEV_Delay_ms(UDOUBLE xms);

/* Monotonic milliseconds, used for timeouts. */
uint64_t DEV_Millis(void);

#ifdef __cplusplus
}
#endif
#endif
