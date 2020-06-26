/* --------------------------------------------------------------------------
 * Copyright (c) 2013-2016 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * $Date:        02. March 2016
 * $Revision:    V2.1
 *
 * Project:      I2C Driver Definitions for NXP LPC17xx
 * -------------------------------------------------------------------------- */

#ifndef __I2C_LPC17XX_H
#define __I2C_LPC17XX_H

#if defined (LPC175x_6x)
  #include "LPC17xx.h"
  #include "PIN_LPC17xx.h"
#elif defined (LPC177x_8x)
  #include "LPC177x_8x.h"
  #include "PIN_LPC177x_8x.h"
#endif

#include "Driver_I2C.h"

/* I2C Driver state flags */
#define I2C_FLAG_INIT       (1 << 0)        // Driver initialized
#define I2C_FLAG_POWER      (1 << 1)        // Driver power on
#define I2C_FLAG_SETUP      (1 << 2)        // Master configured, clock set
#define I2C_FLAG_SLAVE_RX   (1 << 3)        // Slave receive registered

/* I2C Common Control flags */
#define I2C_CON_AA          (1 << 2)        // Assert acknowledge bit
#define I2C_CON_SI          (1 << 3)        // I2C interrupt bit
#define I2C_CON_STO         (1 << 4)        // STOP bit
#define I2C_CON_STA         (1 << 5)        // START bit
#define I2C_CON_I2EN        (1 << 6)        // I2C interface enable
#define I2C_CON_FLAGS       (I2C_CON_AA | I2C_CON_SI | I2C_CON_STO | I2C_CON_STA)

/* I2C Stalled Status flags */
#define I2C_MASTER          (1 << 0)        // Master stalled
#define I2C_SLAVE_TX        (1 << 1)        // Slave stalled on transmit
#define I2C_SLAVE_RX        (1 << 2)        // Slave stalled on receive
#define I2C_SLAVE           (I2C_SLAVE_TX | I2C_SLAVE_RX)

/* I2C Status Miscellaneous states */
#define I2C_STAT_BUSERR      0x00           // I2C Bus error

/* I2C Status Master mode */
#define I2C_STAT_MA_START    0x08           // START transmitted
#define I2C_STAT_MA_RSTART   0x10           // Repeated START transmitted
#define I2C_STAT_MA_SLAW_A   0x18           // SLA+W transmitted, ACK received
#define I2C_STAT_MA_SLAW_NA  0x20           // SLA+W transmitted, no ACK recvd
#define I2C_STAT_MA_DT_A     0x28           // Data transmitted, ACK received
#define I2C_STAT_MA_DT_NA    0x30           // Data transmitted, no ACK recvd
#define I2C_STAT_MA_ALOST    0x38           // Arbitration lost SLA+W or data
#define I2C_STAT_MA_SLAR_A   0x40           // SLA+R transmitted, ACK received
#define I2C_STAT_MA_SLAR_NA  0x48           // SLA+R transmitted, no ACK recvd
#define I2C_STAT_MA_DR_A     0x50           // Data received, ACK returned
#define I2C_STAT_MA_DR_NA    0x58           // Data received, no ACK returned

/* I2C Status Slave mode */
#define I2C_STAT_SL_SLAW_A   0x60           // SLA+W received, ACK returned
#define I2C_STAT_SL_ALOST_MW 0x68           // Arbitration lost SLA+W in Master mode
#define I2C_STAT_SL_GCA_A    0x70           // General address recvd, ACK returned
#define I2C_STAT_SL_ALOST_GC 0x78           // Arbitration lost in General call
#define I2C_STAT_SL_DR_A     0x80           // Data received, ACK returned
#define I2C_STAT_SL_DR_NA    0x88           // Data received, no ACK returned
#define I2C_STAT_SL_DRGC_A   0x90           // Data recvd General call, ACK returned
#define I2C_STAT_SL_DRGC_NA  0x98           // Data recvd General call, no ACK returned
#define I2C_STAT_SL_STOP     0xA0           // STOP received while addressed
#define I2C_STAT_SL_SLAR_A   0xA8           // SLA+R received, ACK returned
#define I2C_STAT_SL_ALOST_MR 0xB0           // Arbitration lost SLA+R in Master mode
#define I2C_STAT_SL_DT_A     0xB8           // Data transmitted, ACK received
#define I2C_STAT_SL_DT_NA    0xC0           // Data transmitted, no ACK received
#define I2C_STAT_SL_LDT_A    0xC8           // Last data transmitted, ACK received


/* Clocks Configuration */
typedef const struct _I2C_CLOCK {
  uint32_t              reg_pwr_val;        // I2C block power control register value
  volatile uint32_t    *reg_pwr;            // I2C block power control  register
#if defined (LPC175x_6x)
  uint32_t              peri_cfg_pos;       // I2C peripheral clock configuration position
  uint32_t              peri_cfg_val;       // I2C peripheral clock configuration value
  volatile uint32_t    *peri_cfg;           // I2C peripheral clock configuration register
#endif
} I2C_CLOCKS;

/* I2C Control Information */
typedef struct {
  ARM_I2C_SignalEvent_t cb_event;           // Event callback
  ARM_I2C_STATUS        status;             // Status flags
  uint8_t               flags;              // Control and state flags
  uint8_t               sla_rw;             // Slave address and RW bit
  bool                  pending;            // Transfer pending (no STOP)
  uint8_t               stalled;            // Stall mode status flags
  uint8_t               con_aa;             // I2C slave CON flag
  int32_t               cnt;                // Master transfer count
  uint8_t              *data;               // Master data to transfer
  uint32_t              num;                // Number of bytes to transfer
  uint8_t              *sdata;              // Slave data to transfer
  uint32_t              snum;               // Number of bytes to transfer
} I2C_CTRL;

typedef struct
{
  __IO uint32_t I2CONSET;                   // Offset: 0x000 I2C Control Set Register (R/W)
  __I  uint32_t I2STAT;                     // Offset: 0x004 I2C Status Register (R/ )
  __IO uint32_t I2DAT;                      // Offset: 0x008 I2C Data Register (R/W)
  __IO uint32_t I2ADR0;                     // Offset: 0x00C I2C Slave Address Register 0 (R/W)
  __IO uint32_t I2SCLH;                     // Offset: 0x010 SCH Duty Cycle Register High Half Word (R/W)
  __IO uint32_t I2SCLL;                     // Offset: 0x014 SCL Duty Cycle Register Low Half Word (R/W)
  __O  uint32_t I2CONCLR;                   // Offset: 0x018 I2C Control Clear Register ( /W)
  __IO uint32_t MMCTRL;                     // Offset: 0x01C Monitor mode control register (R/W)
  __IO uint32_t I2ADR1;                     // Offset: 0x020 I2C Slave Address Register 1 (R/W)
  __IO uint32_t I2ADR2;                     // Offset: 0x024 I2C Slave Address Register 2 (R/W)
  __IO uint32_t I2ADR3;                     // Offset: 0x028 I2C Slave Address Register 3 (R/W)
  __I  uint32_t I2DATA_BUFFER;              // Offset: 0x02C Data buffer register ( /W)
  __IO uint32_t I2MASK0;                    // Offset: 0x030 I2C Slave address mask register 0 (R/W)
  __IO uint32_t I2MASK1;                    // Offset: 0x034 I2C Slave address mask register 1 (R/W)
  __IO uint32_t I2MASK2;                    // Offset: 0x038 I2C Slave address mask register 2 (R/W)
  __IO uint32_t I2MASK3;                    // Offset: 0x03C I2C Slave address mask register 3 (R/W)
} I2C_TypeDef;

/* I2C Resource Configuration */
typedef struct {
  I2C_TypeDef           *reg;               // I2C register interface
  IRQn_Type             i2c_ev_irq;         // I2C Event IRQ Number
  PIN                   *scl;               // I2C SCL pin
  PIN                   *sda;               // I2C SDA pin
  uint8_t               func_scl;           // SCL Pin function
  uint8_t               func_sda;           // SDA Pin function
#if defined (LPC177x_8x)
  uint8_t            i2c_pad_scl;           // SCL I2C PAD use
  uint8_t            i2c_pad_sda;           // SDA I2C PAD use
  uint8_t            i2c_fast_plus;         // I2C Fast Plus Mode support
#endif
  I2C_CLOCKS            clk;                // I2C clocks configuration 
  I2C_CTRL              *ctrl;              // Run-Time control information
} const I2C_RESOURCES;

#endif /* __I2C_LPC17XX_H */
