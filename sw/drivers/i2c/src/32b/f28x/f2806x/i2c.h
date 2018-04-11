/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \defgroup I2C I2C
//@{


#ifndef _I2C_H_
#define _I2C_H_

//! \file   drivers/i2c/src/32b/f28x/f2806x/i2c.h
//!
//! \brief  Contains public interface to various functions related to the 
//!         I2C object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "sw/modules/types/src/types.h"


#ifdef __cplusplus
extern "C" {
#endif

/*
//----------------------------------------------------
// I2C interrupt vector register bit definitions
struct I2CISRC_BITS {         // bits   description
   uint16_t INTCODE:3;          // 2:0    Interrupt code
   uint16_t rsvd1:13;           // 15:3   reserved
};

union I2CISRC_REG {
   uint16_t                 all;
   struct I2CISRC_BITS    bit;
};

//----------------------------------------------------
// I2C interrupt mask register bit definitions
struct I2CIER_BITS {          // bits   description
   uint16_t ARBL:1;               // 0      Arbitration lost interrupt
   uint16_t NACK:1;             // 1      No ack interrupt
   uint16_t ARDY:1;             // 2      Register access ready interrupt
   uint16_t RRDY:1;             // 3      Recieve data ready interrupt
   uint16_t XRDY:1;             // 4      Transmit data ready interrupt
   uint16_t SCD:1;              // 5      Stop condition detection
   uint16_t AAS:1;              // 6      Address as slave
   uint16_t rsvd:9;             // 15:7   reserved
};

union I2CIER_REG {
   uint16_t                 all;
   struct I2CIER_BITS     bit;
};

//----------------------------------------------------
// I2C status register bit definitions
struct I2CSTR_BITS {          // bits   description
   uint16_t ARBL:1;               // 0      Arbitration lost interrupt
   uint16_t NACK:1;             // 1      No ack interrupt
   uint16_t ARDY:1;             // 2      Register access ready interrupt
   uint16_t RRDY:1;             // 3      Recieve data ready interrupt
   uint16_t XRDY:1;             // 4      Transmit data ready interrupt
   uint16_t SCD:1;              // 5      Stop condition detection
   uint16_t rsvd1:2;            // 7:6    reserved
   uint16_t AD0:1;              // 8      Address Zero
   uint16_t AAS:1;              // 9      Address as slave
   uint16_t XSMT:1;             // 10     XMIT shift empty
   uint16_t RSFULL:1;           // 11     Recieve shift full
   uint16_t BB:1;               // 12     Bus busy
   uint16_t NACKSNT:1;          // 13     A no ack sent
   uint16_t SDIR:1;             // 14     Slave direction
   uint16_t rsvd2:1;            // 15     reserved
};

union I2CSTR_REG {
   uint16_t                 all;
   struct I2CSTR_BITS     bit;
};

//----------------------------------------------------
// I2C mode control register bit definitions
struct I2CMDR_BITS {          // bits   description
   uint16_t BC:3;               // 2:0    Bit count
   uint16_t FDF:1;              // 3      Free data format
   uint16_t STB:1;              // 4      Start byte
   uint16_t IRS:1;              // 5      I2C Reset not
   uint16_t DLB:1;              // 6      Digital loopback
   uint16_t RM:1;               // 7      Repeat mode
   uint16_t XA:1;               // 8      Expand address
   uint16_t TRX:1;              // 9      Transmitter/reciever
   uint16_t MST:1;              // 10     Master/slave
   uint16_t STP:1;              // 11     Stop condition
   uint16_t rsvd1:1;            // 12     reserved
   uint16_t STT:1;              // 13     Start condition
   uint16_t FREE:1;             // 14     Emulation mode
   uint16_t NACKMOD:1;          // 15     No Ack mode
};

union I2CMDR_REG {
   uint16_t                 all;
   struct I2CMDR_BITS     bit;
};

//----------------------------------------------------
// I2C extended mode control register bit definitions
struct I2CEMDR_BITS {          // bits   description
   uint16_t BCM:1;               // 0      Bit count
   uint16_t rsvd1:15;            // 15:1   reserved
};

union I2CEMDR_REG {
   uint16_t                 all;
   struct I2CEMDR_BITS    bit;
};

//----------------------------------------------------
// I2C pre-scaler register bit definitions
struct I2CPSC_BITS {         // bits   description
   uint16_t IPSC:8;            // 7:0    pre-scaler
   uint16_t rsvd1:8;           // 15:8   reserved
};

union I2CPSC_REG {
   uint16_t                 all;
   struct I2CPSC_BITS     bit;
};

//----------------------------------------------------
// TX FIFO control register bit definitions
struct I2CFFTX_BITS {         // bits   description
   uint16_t TXFFIL:5;           // 4:0    FIFO interrupt level
   uint16_t TXFFIENA:1;         // 5      FIFO interrupt enable/disable
   uint16_t TXFFINTCLR:1;       // 6      FIFO clear
   uint16_t TXFFINT:1;          // 7      FIFO interrupt flag
   uint16_t TXFFST:5;           // 12:8   FIFO level status
   uint16_t TXFFRST:1;          // 13     FIFO reset
   uint16_t I2CFFEN:1;          // 14     enable/disable TX & RX FIFOs
   uint16_t rsvd1:1;            // 15     reserved

};

union I2CFFTX_REG {
   uint16_t                 all;
   struct I2CFFTX_BITS    bit;
};

//----------------------------------------------------
// RX FIFO control register bit definitions
struct I2CFFRX_BITS {         // bits   description
   uint16_t RXFFIL:5;           // 4:0    FIFO interrupt level
   uint16_t RXFFIENA:1;         // 5      FIFO interrupt enable/disable
   uint16_t RXFFINTCLR:1;       // 6      FIFO clear
   uint16_t RXFFINT:1;          // 7      FIFO interrupt flag
   uint16_t RXFFST:5;           // 12:8   FIFO level
   uint16_t RXFFRST:1;          // 13     FIFO reset
   uint16_t rsvd1:2;            // 15:14  reserved
};

union I2CFFRX_REG {
   uint16_t                 all;
   struct I2CFFRX_BITS    bit;
};

//----------------------------------------------------

struct I2C_REGS {
   uint16_t              I2COAR;    // Own address register
   union  I2CIER_REG   I2CIER;    // Interrupt enable
   union  I2CSTR_REG   I2CSTR;    // Interrupt status
   uint16_t              I2CCLKL;   // Clock divider low
   uint16_t              I2CCLKH;   // Clock divider high
   uint16_t              I2CCNT;    // Data count
   uint16_t              I2CDRR;    // Data recieve
   uint16_t              I2CSAR;    // Slave address
   uint16_t              I2CDXR;    // Data transmit
   union  I2CMDR_REG   I2CMDR;    // Mode
   union  I2CISRC_REG  I2CISRC;   // Interrupt source
   union  I2CEMDR_REG  I2CEMDR;   // Extended mode
   union  I2CPSC_REG   I2CPSC;    // Pre-scaler
   uint16_t              rsvd2[19]; // reserved
   union  I2CFFTX_REG  I2CFFTX;   // Transmit FIFO
   union  I2CFFRX_REG  I2CFFRX;   // Recieve FIFO
};



//---------------------------------------------------------------------------
// External References & Function Declarations:
//
extern volatile struct I2C_REGS I2caRegs;
*/


// **************************************************************************
// the defines


//! \brief Defines the base address of the I2C registers
//!
#define I2C_BASE_ADDR              (0x00007900)

//! \brief Defines the location of the I2COAR9-0  bits in the I2COAR register
//!
#define I2C_I2COAR_OAR_BITS   (0x3ff << 0)

//! \brief Defines the location of the Arbitration-lost interrupt enable bit in the I2CIER register
//!
#define I2C_I2CIER_AL_ENA_BITS          (1 << 0)
//! \brief Defines the location of the No-acknowledgement interrupt enable bit in the I2CIER register
//!
#define I2C_I2CIER_NACK_ENA_BITS        (1 << 1)
//! \brief Defines the location of the Register-access-ready interrupt enable bit in the I2CIER register
//!
#define I2C_I2CIER_ARDY_ENA_BITS        (1 << 2)
//! \brief Defines the location of the Receive-data-ready interrupt enable bit in the I2CIER register
//!
#define I2C_I2CIER_RRDY_ENA_BITS        (1 << 3)
//! \brief Defines the location of the Transmit-data-ready interrupt enable bit in the I2CIER register
//!
#define I2C_I2CIER_XRDY_ENA_BITS        (1 << 4)
//! \brief Defines the location of the Stop condition detected interrupt enable bit in the I2CIER register
//!
#define I2C_I2CIER_SCD_ENA_BITS        (1 << 5)
//! \brief Defines the location of the Address as slave interrupt enable bit in the I2CIER register
//!
#define I2C_I2CIER_AAS_ENA_BITS        (1 << 6)


//! \brief Defines the location of the Arbitration-lost interrupt flag bit in the I2CSTR register
//!
#define I2C_I2CSTR_AL_BITS            (1 << 0)
//! \brief Defines the location of the No-acknowledgment interrupt flag bit in the I2CSTR register
//!
#define I2C_I2CSTR_NACK_BITS          (1 << 1)
//! \brief Defines the location of the Register-access-ready interrupt flag bit in the I2CSTR register
//!
#define I2C_I2CSTR_ARDY_BITS          (1 << 2)
//! \brief Defines the location of the Receive-data-ready interrupt flag bit in the I2CSTR register
//!
#define I2C_I2CSTR_RRDY_BITS          (1 << 3)
//! \brief Defines the location of the Transmit-data-ready interrupt flag bit in the I2CSTR register
//!
#define I2C_I2CSTR_XRDY_BITS          (1 << 4)
//! \brief Defines the location of the Stop condition detected bit in the I2CSTR register
//!
#define I2C_I2CSTR_SCD_BITS           (1 << 5)
//! \brief Defines the location of the Address 0 bits in the I2CSTR register
//!
#define I2C_I2CSTR_AD0_BITS           (1 << 8)
//! \brief Defines the location of the Addressed-as-slave bit in the I2CSTR register
//!
#define I2C_I2CSTR_AAS_BITS           (1 << 9)
//! \brief Defines the location of the Transmit shift register empty bit in the I2CSTR register
//!
#define I2C_I2CSTR_XSMT_BITS          (1 << 10)
//! \brief Defines the location of the Receive shift register full bit in the I2CSTR register
//!
#define I2C_I2CSTR_RSFULL_BITS        (1 << 11)
//! \brief Defines the location of the Bus busy bit in the I2CSTR register
//!
#define I2C_I2CSTR_BB_BITS            (1 << 12)
//! \brief Defines the location of the NACK sent bit in the I2CSTR register
//!
#define I2C_I2CSTR_NACKSNT_BITS       (1 << 13)
//! \brief Defines the location of the Slave direction bit in the I2CSTR register
//!
#define I2C_I2CSTR_SDIR_BITS          (1 << 14)


//! \brief Defines the location of the Bit count bits in the I2CMDR register
//!
#define I2C_I2CMDR_BC_BITS            (7 << 0)
//! \brief Defines the location of the Free data format mode bit in the I2CMDR register
//!
#define I2C_I2CMDR_FDF_BITS            (1 << 3)
//! \brief Defines the location of the START byte mode bit in the I2CMDR register
//!
#define I2C_I2CMDR_STB_BITS            (1 << 4)
//! \brief Defines the location of the I2C module reset bit in the I2CMDR register
//!
#define I2C_I2CMDR_IRS_BITS            (1 << 5)
//! \brief Defines the location of Digital loopback mode bit. in the I2CMDR register
//!
#define I2C_I2CMDR_DLB_BITS            (1 << 6)
//! \brief Defines the location of Repeat mode bit. in the I2CMDR register
//!
//#define I2C_I2CMDR_RM_BITS            (1 << 7)
//! \brief Defines the location of Expanded address enable bit. in the I2CMDR register
//!
#define I2C_I2CMDR_XA_BITS            (1 << 8)
//! \brief Defines the location of Transmitter mode bit. in the I2CMDR register
//!
#define I2C_I2CMDR_TRX_BITS            (1 << 9)
//! \brief Defines the location of Master mode bit. in the I2CMDR register
//!
#define I2C_I2CMDR_MST_BITS            (1 << 10)
//! \brief Defines the location of STOP condition bit. in the I2CMDR register
//!
#define I2C_I2CMDR_STP_BITS            (1 << 11)
//! \brief Defines the location of START condition bit. in the I2CMDR register
//!
#define I2C_I2CMDR_STT_BITS            (1 << 13)
//! \brief Defines the location of START condition bit. in the I2CMDR register
//!
#define I2C_I2CMDR_FREE_BITS            (1 << 14)
//! \brief Defines the location of NACK mode bit. in the I2CMDR register
//!
#define I2C_I2CMDR_NACKMOD_BITS         (1 << 15)

#define I2C_I2CISRC_INTCODE_BITS		(7 << 0)

#define I2C_I2CEMDR_BCM_BITS			(1 << 0)

//! \brief Defines the location of Transmit FIFO interrupt level in the I2CFFTX register
//!
#define I2C_I2CFFTX_TXFFIL_BITS            (0x1f << 0)
//! \brief Defines the location of Transmit FIFO interrupt enable bit in the I2CFFTX register
//!
#define I2C_I2CFFTX_TXFFIENA_BITS            (1 << 5)
//! \brief Defines the location of Transmit FIFO interrupt flag clear bit in the I2CFFTX register
//!
#define I2C_I2CFFTX_TXFFINTCLR_BITS          (1 << 6)
//! \brief Defines the location of Transmit FIFO interrupt flag in the I2CFFTX register
//!
#define I2C_I2CFFTX_TXFFINT_BITS            (1 << 7)
//! \brief Defines the location of Contains the status of the transmit FIFO in the I2CFFTX register
//!
#define I2C_I2CFFTX_TXFFST_BITS            (0x1f << 8)
//! \brief Defines the location of I2C transmit FIFO reset bit in the I2CFFTX register
//!
#define I2C_I2CFFTX_TXFFRST_BITS            (1 << 13)
//! \brief Defines the location of I2C FIFO mode enable bit in the I2CFFTX register
//!
#define I2C_I2CFFTX_I2CFFEN_BITS            (1 << 14)


//! \brief Defines the location of Receive FIFO interrupt level in the I2CFFRX register
//!
#define I2C_I2CFFRX_RXFFIL_BITS            (0x1f << 0)
//! \brief Defines the location of Receive FIFO interrupt enable bit in the I2CFFRX register
//!
#define I2C_I2CFFRX_RXFFIENA_BITS            (1 << 5)
//! \brief Defines the location of Receive FIFO interrupt flag clear bit in the I2CFFRX register
//!
#define I2C_I2CFFRX_RXFFINTCLR_BITS            (1 << 6)
//! \brief Defines the location of Receive FIFO interrupt flag in the I2CFFRX register
//!
#define I2C_I2CFFRX_RXFFINT_BITS            (1 << 7)
//! \brief Defines the location of Contains the status of the Receive FIFO in the I2CFFRX register
//!
#define I2C_I2CFFRX_RXFFST_BITS            (0x1f << 8)
//! \brief Defines the location of I2C Receive FIFO reset bit in the I2CFFRX register
//!
#define I2C_I2CFFRX_RXFFRST_BITS            (1 << 13)


//! \brief Enumeration to define the sI2C character lengths
//!
typedef enum
{
  I2C_CharLength_1_Bit= (1 << 0),     //!< Denotes a character length of 1 bit
  I2C_CharLength_2_Bits=(2 << 0),    //!< Denotes a character length of 2 bits
  I2C_CharLength_3_Bits=(3 << 0),    //!< Denotes a character length of 3 bits
  I2C_CharLength_4_Bits=(4 << 0),    //!< Denotes a character length of 4 bits
  I2C_CharLength_5_Bits=(5 << 0),    //!< Denotes a character length of 5 bits
  I2C_CharLength_6_Bits=(6 << 0),    //!< Denotes a character length of 6 bits
  I2C_CharLength_7_Bits=(7 << 0),    //!< Denotes a character length of 7 bits
  I2C_CharLength_8_Bits=(0 << 0)     //!< Denotes a character length of 8 bits
} I2C_CharLength_e;

typedef enum
{
  I2C_Mode_Slave=(0<< 10),      //!< Denotes slave mode
  I2C_Mode_Master=(1<< 10)      //!< Denotes master mode
} I2C_Mode_e;

typedef enum
{
  I2C_Mode_Receive=(0<< 9),      //!< Denotes Receive mode
  I2C_Mode_Transmit=(1<< 9)      //!< Denotes Transmit mode
} I2C_ModeTxRx_e;

typedef enum
{
	 I2C_Mode_NonRepeat= (0 << 7),
	 I2C_Mode_Repeat = (1 << 7)
} I2C_ModeRepeat_e;


typedef enum
{
  I2C_Mode_7Address=(0<< 8),      //!< Denotes 7 bit address mode
  I2C_Mode_10Address=(1<< 8)      //!< Denotes 10 bit address mode
} I2C_ModeAddress_e;


//! \brief Enumeration to define the serial peripheral interface (SPI) FIFO level
//!
typedef enum
{
  I2C_FifoLevel_Empty=(0 << 0),      //!< Denotes the fifo is empty
  I2C_FifoLevel_1_Word=(1 << 0),     //!< Denotes the fifo contains 1 word
  I2C_FifoLevel_2_Words=(2 << 0),    //!< Denotes the fifo contains 2 words
  I2C_FifoLevel_3_Words=(3 << 0),    //!< Denotes the fifo contains 3 words
  I2C_FifoLevel_4_Words=(4 << 0)     //!< Denotes the fifo contains 4 words
} I2C_FifoLevel_e;

//! \brief Enumeration to define the serial peripheral interface (SPI) FIFO status
//!
typedef enum
{
  I2C_FifoStatus_Empty=(0 << 8),      //!< Denotes the fifo is empty
  I2C_FifoStatus_1_Word=(1 << 8),     //!< Denotes the fifo contains 1 word
  I2C_FifoStatus_2_Words=(2 << 8),    //!< Denotes the fifo contains 2 words
  I2C_FifoStatus_3_Words=(3 << 8),    //!< Denotes the fifo contains 3 words
  I2C_FifoStatus_4_Words=(4 << 8)     //!< Denotes the fifo contains 4 words
}  I2C_FifoStatus_e;


typedef enum
{
  I2C_None= 0,
  I2C_ArbitrationLost = 1,
  I2C_NackDection = 2,
  I2C_RegisterReady = 3,
  I2C_ReceiveReady = 4,
  I2C_TransmitReady = 5,
  I2C_StopDection = 6,
  I2C_AddressSlave = 7
} I2C_InterruptCode_e;

typedef enum
{
	I2C_StatusFlag_ArbitLost = (1 << 0),
	I2C_StatusFlag_Nack = (1 << 1),
	I2C_StatusFlaag_RegistReady = (1 << 2),
	I2C_StatusFlag_RxReady = (1 << 3),
	I2C_StatusFlag_TxReady = (1 << 4),
	I2C_StatusFlag_StopCond = (1 << 5),
	I2C_StatusFlag_Add0 = (1 << 8),
	I2C_StatusFlag_AddSlave = (1 << 9),
	I2C_StatusFlag_NoUnderflow = (1 << 10),
	I2C_StatusFlag_Overrun = (1 << 11),
	I2C_StatusFlag_BusBusy = (1 << 12),
	I2C_StatusFlag_NackSend = (1 << 13),
	I2C_StatusFlag_SlaaveDir = (1 << 14)
} I2C_StatusFlag_e;




//! \brief Defines the I2C object
//!
typedef struct _I2C_Obj_
{
  volatile uint16_t     I2COAR;    // Own address register
  volatile uint16_t     I2CIER;    // Interrupt enable
  volatile uint16_t     I2CSTR;    // Interrupt status
  volatile uint16_t     I2CCLKL;   // Clock divider low
  volatile uint16_t     I2CCLKH;   // Clock divider high
  volatile uint16_t     I2CCNT;    // Data count
  volatile uint16_t     I2CDRR;    // Data recieve
  volatile uint16_t     I2CSAR;    // Slave address
  volatile uint16_t     I2CDXR;    // Data transmit
  volatile uint16_t     I2CMDR;    // Mode
  volatile uint16_t     I2CISRC;   // Interrupt source
  volatile uint16_t     I2CEMDR;   // Extended mode
  volatile uint16_t     I2CPSC;    // Pre-scaler
  volatile uint16_t     rsvd2[19]; // reserved
  volatile uint16_t     I2CFFTX;   // Transmit FIFO
  volatile uint16_t     I2CFFRX;   // Recieve FIFO
} I2C_Obj;


typedef struct _I2C_Handle_ *I2C_Handle;


//! \brief     Initializes the I2C object handle
//! \param[in] pMemory     A pointer to the base address of the I2C registers
//! \param[in] numBytes    The number of bytes allocated for the I2C object, bytes
//! \return    The IC object handle
extern I2C_Handle I2C_init(void *pMemory,const size_t numBytes);

extern void I2C_disableNAck(I2C_Handle i2cHandle);

extern void I2C_disableFreeMode(I2C_Handle i2cHandle);

extern void I2C_NonRepeatMode(I2C_Handle i2cHandle);

extern void I2C_disableLookbackMode(I2C_Handle i2cHandle);

extern void I2C_Reset(I2C_Handle i2cHandle);

extern void I2C_disableStartByteMode(I2C_Handle i2cHandle);

extern void I2C_disableFreeDataFormat(I2C_Handle i2cHandle);


extern void I2C_disableAddSlaveInt(I2C_Handle i2cHandle);

extern void I2C_disableStopCondDectInt(I2C_Handle i2cHandle);

extern void I2C_disableTXReadyInt(I2C_Handle i2cHandle);

extern void I2C_disableRXReadyInt(I2C_Handle i2cHandle);

extern void I2C_disableRegAccessReadyInt(I2C_Handle i2cHandle);

extern void I2C_disableNoAckInt(I2C_Handle i2cHandle);

extern void I2C_disableArbiLostInt(I2C_Handle i2cHandle);


extern void I2C_disableTxFifoMode(I2C_Handle i2cHandle);

extern void I2C_resetTxFifo(I2C_Handle i2cHandle);

extern void I2C_disableTxFifoInt(I2C_Handle i2cHandle);


extern void I2C_resetRxFifo(I2C_Handle i2cHandle);

extern void I2C_disableRxFifoInt(I2C_Handle i2cHandle);

extern void I2C_enableAddSlaveInt(I2C_Handle i2cHandle);

extern void I2C_enableStopCondDectInt(I2C_Handle i2cHandle);

extern void I2C_enableTXReadyInt(I2C_Handle i2cHandle);

extern void I2C_enableRXReadyInt(I2C_Handle i2cHandle);

extern void I2C_enableRegAccessReadyInt(I2C_Handle i2cHandle);

extern void I2C_enableNoAckInt(I2C_Handle i2cHandle);

extern void I2C_enableArbiLostInt(I2C_Handle i2cHandle);


extern void I2C_enableFifoMode(I2C_Handle i2cHandle);

extern void I2C_enableTxFifo(I2C_Handle i2cHandle);

extern void I2C_clearTxFifoIntFlag(I2C_Handle i2cHandle);

extern void I2C_enableTxFifoInt(I2C_Handle i2cHandle);

extern void I2C_enableRxFifo(I2C_Handle i2cHandle);

extern uint_least8_t I2C_getReceiveFifoNo(I2C_Handle i2cHandle);

extern void I2C_clearRxFifoIntFlag(I2C_Handle i2cHandle);

extern void I2C_enableRxFifoInt(I2C_Handle i2cHandle);

extern void I2C_clearNackStatus(I2C_Handle i2cHandle);
extern void I2C_clearNackSend(I2C_Handle i2cHandle);

extern bool I2C_getStopStatus(I2C_Handle i2cHandle);

extern I2C_StatusFlag_e I2C_getStatusFlag(I2C_Handle i2cHandle);
extern I2C_InterruptCode_e I2C_getIntFlagStatus(I2C_Handle i2cHandle);

extern bool I2C_getFifoTxInt(I2C_Handle i2cHandle);

extern bool I2C_getFifoRxInt(I2C_Handle i2cHandle);


extern void I2C_setNAckMode(I2C_Handle i2cHandle);

extern void I2C_setFreeMode(I2C_Handle i2cHandle);

extern void I2C_StartCond(I2C_Handle i2cHandle);

extern void I2C_StopCond(I2C_Handle i2cHandle);

//extern void I2C_setRepeatMode(I2C_Handle i2cHandle);
void I2C_setRepeatMode(I2C_Handle , const I2C_ModeRepeat_e );

extern void I2C_setLookbackMode(I2C_Handle i2cHandle);

extern void I2C_Enable(I2C_Handle i2cHandle);

extern void I2C_setStartByteMode(I2C_Handle i2cHandle);

extern void I2C_setFreeDataFormat(I2C_Handle i2cHandle);

extern void I2C_setBitCountBits(I2C_Handle i2cHandle,const I2C_CharLength_e i2cCharLen);

extern void I2C_setMode(I2C_Handle i2cHandle, const I2C_Mode_e i2cMode);

extern void I2C_setTxRxMode(I2C_Handle i2cHandle, I2C_ModeTxRx_e i2cModeTxRx);

extern void I2C_setAddressMode(I2C_Handle i2cHandle, I2C_ModeAddress_e i2cModeAddress);

extern void I2C_setTxFifoIntLevel(I2C_Handle i2cHandle, const I2C_FifoLevel_e i2cFifoLevel);

extern void I2C_setRxFifoIntLevel(I2C_Handle i2cHandle, const I2C_FifoLevel_e i2cFifoLevel);

extern void I2C_setCMDR(I2C_Handle i32Handle, uint16_t u16Value);
extern void I2C_setAddress(I2C_Handle i2cHandle,const uint16_t u16Address);
extern void I2C_setClockLowTime(I2C_Handle i2cHandle,const uint16_t u16Value);
extern void I2C_setClockHighTime(I2C_Handle i2cHandle,const uint16_t u16Value);
extern void I2C_setPrescaler(I2C_Handle i2cHandle,const uint_least8_t u16ScalerValue);

extern void I2C_setDataCount(I2C_Handle i2cHandle,const uint_least8_t u16Value);
//extern I2C_CommunicationStatus I2C_WriteMessage(I2C_Handle i2cHandle, I2C_Message *pI2CMessage);
//extern I2C_CommunicationStatus I2C_ReadMessage(I2C_Handle i2cHandle, I2C_Message *pI2CMessage);

//extern void I2C_TimerCounter();
//extern I2C_MessageCode_e I2C_Synchronize();

//extern void I2C_interrupt(I2C_Handle i2cHandle);

static inline uint16_t I2C_read(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	uint16_t u16Data = pI2c->I2CDRR;
	return (u16Data);
}

static inline void I2C_write(I2C_Handle i2cHandle,const uint16_t u16Data)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;
	// set the bits
	pI2c->I2CDXR = u16Data;

}




#ifdef __cplusplus
}
#endif /* extern "C" */

#endif  // end of _I2C_H_ definition

//@}
