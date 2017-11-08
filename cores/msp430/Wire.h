/*
  ************************************************************************
  *	Wire.h
  *
  *	Arduino core files for MSP430
  *		Copyright (c) 2012 Robert Wessels. All right reserved.
  *
  *
  ***********************************************************************
  Derived from:
  TwoWire.h - TWI/I2C library for Arduino & Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts

  Modified 2017 by Theia Space to support multiple interfaces.
*/

#ifndef TwoWire_h
#define TwoWire_h
#include <Energia.h>
#include <inttypes.h>
#include "Stream.h"
#include <msp430.h>

#define BUFFER_LENGTH 16

#ifndef TWI_FREQ
#define TWI_FREQ 100000L
#endif

#define TWI_BUFFER_LENGTH BUFFER_LENGTH

#define TWI_READY 0
#define TWI_MRX   1
#define TWI_MTX   2
#define TWI_SRX   3
#define TWI_STX   4

#define TWI_SND_START 0
#define TWI_PREP_SLA_ADDR_ACK 1
#define TWI_MT_PROC_ADDR_ACK 2
#define TWI_MT_PREP_DATA_ACK 3
#define TWI_MT_PROC_DATA_ACK 4
#define TWI_MR_PREP_DATA_RECV 5
#define TWI_MR_PROC_DATA_RECV 6
#define TWI_MR_PREP_STOP 7

#define TWI_SL_START 8
#define TWI_SL_PROC_ADDR 9
#define TWI_SL_SEND_BYTE 10
#define TWI_SL_PREP_DATA_ACK 11
#define TWI_SL_PROC_DATA_ACK 12
#define TWI_SL_RECV_BYTE 13
#define TWI_SL_PROC_BYTE 14
#define TWI_SL_RESET 15
#define TWI_EXIT 16
#define TWI_IDLE 17

#define TWI_ERRROR_NO_ERROR 0
#define TWI_ERROR_BUF_TO_LONG 1
#define TWI_ERROR_ADDR_NACK 2
#define TWI_ERROR_DATA_NACK 3
#define TWI_ERROR_OTHER 4

#define TWI_WAIT_ITERATIONS 0xFFFF

class TwoWire : public Stream
{
  private:
    uint8_t module;
    uint8_t rxBuffer[BUFFER_LENGTH];
    uint8_t rxBufferIndex;
    uint8_t rxBufferLength;

    uint8_t txAddress;
    uint8_t txBuffer[BUFFER_LENGTH];
    uint8_t txBufferIndex;
    uint8_t txBufferLength;

    uint8_t transmitting;

    volatile unsigned char* UCBxCTL0;
    volatile unsigned char* UCBxCTL1;
    volatile unsigned char* UCBxBR0;
    volatile unsigned char* UCBxBR1;
    volatile unsigned char* UCBxRXBUF;
    volatile unsigned char* UCBxTXBUF;
    volatile unsigned int* UCBxI2COA;
    volatile unsigned int* UCBxI2CSA;
    volatile unsigned char* UCBxIE;
    volatile unsigned char* UCBxIFG;
    uint8_t TWISCLx;
    uint16_t TWISCLx_SET_MODE;
    uint8_t TWISDAx;
    uint16_t TWISDAx_SET_MODE;
    volatile uint8_t twi_state;
    volatile uint8_t twi_sendStop; // should the transaction end with a stop
    volatile uint8_t twi_inRepStart; // in the middle of a repeated start
    void (*twi_onSlaveTransmit)(void);
    void (*twi_onSlaveReceive)(uint8_t*, int);
    uint8_t twi_masterBuffer[TWI_BUFFER_LENGTH];
    volatile uint8_t twi_masterBufferIndex;
    uint8_t twi_masterBufferLength;
    uint8_t twi_txBuffer[TWI_BUFFER_LENGTH];
    volatile uint8_t twi_txBufferIndex;
    volatile uint8_t twi_txBufferLength;
    uint8_t twi_rxBuffer[TWI_BUFFER_LENGTH];
    volatile uint8_t twi_rxBufferIndex;
    volatile uint8_t twi_error;

    void (*user_onRequest)(void);
    void (*user_onReceive)(int);
    void onRequestService(void);
    void onReceiveService(uint8_t*, int);
    static void onRequestService0(void);
    #ifdef ESAT_BOARD_HAS_SECOND_I2C_BUS
    static void onRequestService1(void);
    #endif
    static void onReceiveService0(uint8_t*, int);
    #ifdef ESAT_BOARD_HAS_SECOND_I2C_BUS
    static void onReceiveService1(uint8_t*, int);
    #endif
    void twi_init();
    void twi_init_port();
    void twi_setAddress(uint8_t);
    uint8_t twi_readFrom(uint8_t, uint8_t*, uint8_t, uint8_t);
    uint8_t twi_writeTo(uint8_t, uint8_t*, uint8_t, uint8_t, uint8_t);
    uint8_t twi_transmit(const uint8_t*, uint8_t);
    void twi_attachSlaveRxEvent(void (*)(uint8_t*, int) );
    void twi_attachSlaveTxEvent(void (*)(void) );

  public:
    TwoWire();
    TwoWire(uint8_t module);
    void begin();
    void begin(uint8_t);
    void begin(int);
    void beginTransmission(uint8_t);
    void beginTransmission(int);
    uint8_t endTransmission(void);
    uint8_t endTransmission(uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
    uint8_t requestFrom(int, int);
    uint8_t requestFrom(int, int, int);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *, size_t);
    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive( void (*)(int) );
    void onRequest( void (*)(void) );
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write;
    boolean i2c_state_isr();
    boolean i2c_txrx_isr();
};

extern TwoWire Wire;
#ifdef ESAT_BOARD_HAS_SECOND_I2C_BUS
extern TwoWire Wire1;
#endif

extern "C" {
  boolean i2c_txrx_isr(uint8_t module);
  boolean i2c_state_isr(uint8_t module);
}

#endif

