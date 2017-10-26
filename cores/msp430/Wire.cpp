/*
  ************************************************************************
  *	Wire.cpp
  *
  *	Arduino core files for MSP430
  *		Copyright (c) 2012 Robert Wessels. All right reserved.
  *
  *
  ***********************************************************************
  Derived from:
  TwoWire.cpp - TWI/I2C library for Wiring & Arduino
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

#include "Energia.h"

extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
  #include "usci_isr_handler.h"
}

#include "Wire.h"

static void empty_onRequest(void)
{
}

static void empty_onReceive(int)
{
}

void TwoWire::onRequestService0(void)
{
  Wire.onRequestService();
}

#ifdef ESAT_BOARD_HAS_SECOND_I2C_BUS
void TwoWire::onRequestService1(void)
{
  Wire1.onRequestService();
}
#endif

void TwoWire::onReceiveService0(uint8_t* inBytes, int numBytes)
{
  Wire.onReceiveService(inBytes, numBytes);
}

#ifdef ESAT_BOARD_HAS_SECOND_I2C_BUS
void TwoWire::onReceiveService1(uint8_t* inBytes, int numBytes)
{
  Wire1.onReceiveService(inBytes, numBytes);
}
#endif

// Constructors ////////////////////////////////////////////////////////////////

TwoWire::TwoWire(): TwoWire(0)
{
}

TwoWire::TwoWire(uint8_t i2cModule):
  module(i2cModule),
  rxBufferIndex(0),
  rxBufferLength(0),
  txAddress(0),
  txBufferIndex(0),
  txBufferLength(0),
  transmitting(0),
  user_onRequest(empty_onRequest),
  user_onReceive(empty_onReceive)
{
  switch (module)
  {
  case 0:
    UCBxCTL0 = &UCB0CTL0;
    UCBxCTL1 = &UCB0CTL1;
    UCBxBR0 = &UCB0BR0;
    UCBxBR1 = &UCB0BR1;
    UCBxRXBUF = &UCB0RXBUF;
    UCBxTXBUF = &UCB0TXBUF;
    UCBxI2COA = &UCB0I2COA;
    UCBxI2CSA = &UCB0I2CSA;
    UCBxIE = &UCB0IE;
    UCBxIFG = &UCB0IFG;
    TWISCLx = TWISCL0;
    TWISCLx_SET_MODE = TWISCL0_SET_MODE;
    TWISDAx = TWISDA0;
    TWISDAx_SET_MODE = TWISDA0_SET_MODE;
    break;
#ifdef ESAT_BOARD_HAS_SECOND_I2C_BUS
  case 1:
    UCBxCTL0 = &UCB1CTL0;
    UCBxCTL1 = &UCB1CTL1;
    UCBxBR0 = &UCB1BR0;
    UCBxBR1 = &UCB1BR1;
    UCBxRXBUF = &UCB1RXBUF;
    UCBxTXBUF = &UCB1TXBUF;
    UCBxI2COA = &UCB1I2COA;
    UCBxI2CSA = &UCB1I2CSA;
    UCBxIE = &UCB1IE;
    UCBxIFG = &UCB1IFG;
    TWISCLx = TWISCL1;
    TWISCLx_SET_MODE = TWISCL1_SET_MODE;
    TWISDAx = TWISDA1;
    TWISDAx_SET_MODE = TWISDA1_SET_MODE;
    break;
#endif
  default:
    break;
  }
}

// Public Methods //////////////////////////////////////////////////////////////

void TwoWire::begin(void)
{
  rxBufferIndex = 0;
  rxBufferLength = 0;

  txBufferIndex = 0;
  txBufferLength = 0;

  twi_init();
}

void TwoWire::begin(uint8_t address)
{
  twi_setAddress(address);
  switch (module)
  {
  case 0:
    twi_attachSlaveTxEvent(onRequestService0);
    twi_attachSlaveRxEvent(onReceiveService0);
    break;
#ifdef ESAT_BOARD_HAS_SECOND_I2C_BUS
  case 1:
    twi_attachSlaveTxEvent(onRequestService1);
    twi_attachSlaveRxEvent(onReceiveService1);
    break;
#endif
  }
  begin();
}

void TwoWire::begin(int address)
{
  begin((uint8_t)address);
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
  // clamp to buffer length
  if(quantity > BUFFER_LENGTH){
    quantity = BUFFER_LENGTH;
  }
  // perform blocking read into buffer
  uint8_t read = twi_readFrom(address, rxBuffer, quantity, sendStop);
  // set rx buffer iterator vars
  rxBufferIndex = 0;
  rxBufferLength = read;

  return read;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t TwoWire::requestFrom(int address, int quantity)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
}

void TwoWire::beginTransmission(uint8_t address)
{
  // indicate that we are transmitting
  transmitting = 1;
  // set address of targeted slave
  txAddress = address;
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
}

void TwoWire::beginTransmission(int address)
{
  beginTransmission((uint8_t)address);
}

//
//	Originally, 'endTransmission' was an f(void) function.
//	It has been modified to take one parameter indicating
//	whether or not a STOP should be performed on the bus.
//	Calling endTransmission(false) allows a sketch to 
//	perform a repeated start. 
//
//	WARNING: Nothing in the library keeps track of whether
//	the bus tenure has been properly ended with a STOP. It
//	is very possible to leave the bus in a hung state if
//	no call to endTransmission(true) is made. Some I2C
//	devices will behave oddly if they do not see a STOP.
//
uint8_t TwoWire::endTransmission(uint8_t sendStop)
{
  // transmit buffer (blocking)
  int8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, 1, sendStop);
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
  // indicate that we are done transmitting
  transmitting = 0;
  return ret;
}

//	This provides backwards compatibility with the original
//	definition, and expected behaviour, of endTransmission
//
uint8_t TwoWire::endTransmission(void)
{
  return endTransmission(true);
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t TwoWire::write(uint8_t data)
{
  if(transmitting){
  // in master transmitter mode
    // don't bother if buffer is full
    if(txBufferLength >= BUFFER_LENGTH){
      setWriteError();
      return 0;
    }
    // put byte in tx buffer
    txBuffer[txBufferIndex] = data;
    ++txBufferIndex;
    // update amount in buffer   
    txBufferLength = txBufferIndex;
  }else{
  // in slave send mode
    // reply to master
    twi_transmit(&data, 1);
  }
  return 1;
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
  if(transmitting){
  // in master transmitter mode
    for(size_t i = 0; i < quantity; ++i){
      write(data[i]);
    }
  }else{
  // in slave send mode
    // reply to master
    twi_transmit(data, quantity);
  }
  return quantity;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWire::available(void)
{
  return rxBufferLength - rxBufferIndex;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWire::read(void)
{
  int value = -1;
  
  // get each successive byte on each call
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }

  return value;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWire::peek(void)
{
  int value = -1;
  
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
  }

  return value;
}

void TwoWire::flush(void)
{
  // XXX: to be implemented.
}

// behind the scenes function that is called when data is received
void TwoWire::onReceiveService(uint8_t* inBytes, int numBytes)
{
  // don't bother if user hasn't registered a callback
  if(!user_onReceive){
    return;
  }
  // don't bother if rx buffer is in use by a master requestFrom() op
  // i know this drops data, but it allows for slight stupidity
  // meaning, they may not have read all the master requestFrom() data yet
  if(rxBufferIndex < rxBufferLength){
    return;
  }
  // copy twi rx buffer into local read buffer
  // this enables new reads to happen in parallel
  for(uint8_t i = 0; i < numBytes; ++i){
    rxBuffer[i] = inBytes[i];    
  }
  // set rx iterator vars
  rxBufferIndex = 0;
  rxBufferLength = numBytes;
  // alert user program
  user_onReceive(numBytes);
}

// behind the scenes function that is called when data is requested
void TwoWire::onRequestService(void)
{
  // don't bother if user hasn't registered a callback
  if(!user_onRequest){
    return;
  }
  // reset tx buffer iterator vars
  // !!! this will kill any pending pre-master sendTo() activity
  txBufferIndex = 0;
  txBufferLength = 0;
  // alert user program
  user_onRequest();
}

// sets function called on slave write
void TwoWire::onReceive( void (*function)(int) )
{
  user_onReceive = function;
}

// sets function called on slave read
void TwoWire::onRequest( void (*function)(void) )
{
  user_onRequest = function;
}

/*
 * Function twi_init
 * Desc     readys twi pins and sets twi bitrate
 * Input    none
 * Output   none
 */
void TwoWire::twi_init(void)
{
  // initialize state
  twi_state = TWI_IDLE;
  twi_sendStop = true; // default value
  twi_inRepStart = false;

  /* Calling this dummy function prevents the linker
   * from stripping the USCI interupt vectors.*/
  usci_isr_install();

  twi_init_port();

  //Disable the USCI module and clears the other bits of control register
  *UCBxCTL1 = UCSWRST;

  /*
   * Configure as I2C Slave.
   * UCMODE_3 = I2C mode
   * UCSYNC = Synchronous mode
   * UCCLK = SMCLK
   */
  *UCBxCTL0 = UCMODE_3 | UCSYNC;
  /*
   * Compute the clock divider that achieves less than or
   * equal to 100kHz.  The numerator is biased to favor a larger
   * clock divider so that the resulting clock is always less than or equal
   * to the desired clock, never greater.
   */
  *UCBxBR0 = (unsigned char)((F_CPU / TWI_FREQ) & 0xFF);
  *UCBxBR1 = (unsigned char)((F_CPU / TWI_FREQ) >> 8);

  *UCBxCTL1 &= ~(UCSWRST);

  /* Set I2C state change interrupt mask and TX/RX interrupts */
  *UCBxIE |= (UCALIE|UCNACKIE|UCSTTIE|UCSTPIE|UCRXIE|UCTXIE);
}

/*
 * Function twi_init_port
 * Desc     initialises the port pins
 * Input    none
 * Output   none
 */
void TwoWire::twi_init_port(void)
{
  /* Set pins to I2C mode */
  pinMode_int(TWISDAx, INPUT_PULLUP);
  if (digitalRead(TWISDAx) == 0){ // toggle SCL if SDA is low at startup
    pinMode_int(TWISCLx, INPUT_PULLUP);
    digitalWrite(TWISCLx, LOW);
    pinMode(TWISCLx, OUTPUT);
    pinMode_int(TWISCLx, INPUT_PULLUP);
  }
  if ((TWISDAx_SET_MODE & INPUT_PULLUP) == 0) {
    pinMode(TWISDAx, INPUT);          // some device do not allow the pull up to be enabled
    pinMode(TWISCLx, INPUT);
  }
  pinMode_int(TWISDAx, TWISDAx_SET_MODE);
  pinMode_int(TWISCLx, TWISCLx_SET_MODE);
}

/*
 * Function twi_setAddress
 * Desc     sets slave address and enables interrupt
 * Input    address: 7bit i2c device address
 * Output   none
 */
void TwoWire::twi_setAddress(uint8_t address)
{
  /* UCGCEN = respond to general Call */
  *UCBxI2COA = (address | UCGCEN);
}


/*
 * Function twi_readFrom
 * Desc     attempts to become twi bus master and read a
 *          series of bytes from a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes to read into array
 * Output   number of bytes read
 */
uint8_t TwoWire::twi_readFrom(uint8_t address, uint8_t* data, uint8_t length, uint8_t sendStop)
{
  uint8_t i;
  uint32_t waitCounter;

  *UCBxCTL1 = UCSWRST;                      // Enable SW reset
  *UCBxCTL1 |= (UCSSEL_2);                  // I2C Master, synchronous mode
  *UCBxCTL0 |= (UCMST | UCMODE_3 | UCSYNC); // I2C Master, synchronous mode
  *UCBxCTL1 &= ~(UCTR);                     // Configure in receive mode
  *UCBxI2CSA = address;                     // Set Slave Address
  *UCBxCTL1 &= ~UCSWRST;                    // Clear SW reset, resume operation
  *UCBxIE |= (UCALIE|UCNACKIE|UCSTPIE|UCRXIE|UCTXIE);  // Enable I2C interrupts
  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < length){
    return 0;
  }

  // initialize buffer iteration vars
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length-1;  // This is not intuitive, read on...
  // On receive, the previously configured ACK/NACK setting is transmitted in
  // response to the received byte before the interrupt is signalled.
  // Therefor we must actually set NACK when the _next_ to last byte is
  // received, causing that NACK to be sent in response to receiving the last
  // expected byte of data.

  twi_state =  TWI_MRX;                     // Master receive mode
  *UCBxCTL1 |= UCTXSTT;                      // I2C start condition

  if(length == 1) {                         // When only receiving 1 byte..
    waitCounter = 0;
    while(*UCBxCTL1 & UCTXSTT)             // Wait for start bit to be sent
    {
      if (waitCounter >= TWI_WAIT_ITERATIONS) {
        return 0;
      }
      waitCounter = waitCounter + 1;
    }
    *UCBxCTL1 |= UCTXSTP;                  // Send I2C stop condition after recv
  }

  /* Wait in low power mode for read operation to complete */
  waitCounter = 0;
  while(twi_state != TWI_IDLE){
    if (waitCounter >= TWI_WAIT_ITERATIONS) {
      return 0;
    }
    waitCounter = waitCounter + 1;
  }

  if (twi_masterBufferIndex < length)
    length = twi_masterBufferIndex;

  for(i = 0; i < length; ++i){
    data[i] = twi_masterBuffer[i];
  }

  /* Ensure stop condition got sent before we exit. */
  waitCounter = 0;
  while (*UCBxCTL1 & UCTXSTP) {
    if (waitCounter >= TWI_WAIT_ITERATIONS) {
      return 0;
    }
  }
  return length;
}

/*
 * Function twi_writeTo
 * Desc     attempts to become twi bus master and write a
 *          series of bytes to a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes in array
 *          wait: boolean indicating to wait for write or not
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
uint8_t TwoWire::twi_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait, uint8_t sendStop)
{
  uint8_t i;
  uint32_t waitCounter;
  twi_error = TWI_ERRROR_NO_ERROR;
  twi_sendStop = sendStop;

  *UCBxCTL1 = UCSWRST;                          // Enable SW reset
  *UCBxCTL1 |= UCSSEL_2;                        // SMCLK
  *UCBxCTL0 |= (UCMST | UCMODE_3 | UCSYNC);     // I2C Master, synchronous mode
  *UCBxCTL1 |= UCTR;                            // Configure in transmit mode
  *UCBxI2CSA = address;                         // Set Slave Address
  *UCBxCTL1 &= ~UCSWRST;                        // Clear SW reset, resume operation
  *UCBxIE |= (UCALIE|UCNACKIE|UCSTPIE|UCTXIE);  // Enable I2C interrupts

  /* Ensure data will fit into buffer */
  if(length > TWI_BUFFER_LENGTH){
    return TWI_ERROR_BUF_TO_LONG;
  }

  /* initialize buffer iteration vars */
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length;

  for(i = 0; i < length; ++i){
    twi_masterBuffer[i] = data[i];
  }

  twi_state =  TWI_MTX;                         // Master Transmit mode
  *UCBxCTL1 |= UCTXSTT;                         // I2C start condition

  /* Wait for the transaction to complete */
  waitCounter = 0;
  while(twi_state != TWI_IDLE) {
    if (waitCounter >= TWI_WAIT_ITERATIONS)
    {
      return TWI_ERROR_OTHER;
    }
    waitCounter = waitCounter + 1;
  }

  /* Ensure stop/start condition got sent before we exit. */
  waitCounter = 0;
  if(sendStop)
  {
    while (*UCBxCTL1 & UCTXSTP) // end with stop condition
    {
      if (waitCounter >= TWI_WAIT_ITERATIONS)
      {
        return TWI_ERROR_OTHER;
      }
      waitCounter = waitCounter + 1;
    }
  } else {
    while (*UCBxCTL1 & UCTXSTT) // end with (re)start condition
    {
      if (waitCounter >= TWI_WAIT_ITERATIONS)
      {
        return TWI_ERROR_OTHER;
      }
      waitCounter = waitCounter + 1;
    }
  }

  return twi_error;
}

/*
 * Function twi_transmit
 * Desc     fills slave tx buffer with data
 *          must be called in slave tx event callback
 * Input    data: pointer to byte array
 *          length: number of bytes in array
 * Output   1 length too long for buffer
 *          2 not slave transmitter
 *          0 ok
 */
uint8_t TwoWire::twi_transmit(const uint8_t* data, uint8_t length)
{
  uint8_t i;

  twi_state =  TWI_STX; // Slave transmit mode

  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < (twi_txBufferLength+length)){
    return 1;
  }

  // copy data into tx buffer and update tx buffer length
  for (i = 0; i < length; ++i) {
    twi_txBuffer[twi_txBufferLength+i] = data[i];
  }
  twi_txBufferLength = twi_txBufferLength + length;

  return 0;
}

/*
 * Function twi_attachSlaveRxEvent
 * Desc     sets function called before a slave read operation
 * Input    function: callback function to use
 * Output   none
 */
void TwoWire::twi_attachSlaveRxEvent(void (*function)(uint8_t*, int) )
{
  twi_onSlaveReceive = function;
}

/*
 * Function twi_attachSlaveTxEvent
 * Desc     sets function called before a slave write operation
 * Input    function: callback function to use
 * Output   none
 */
void TwoWire::twi_attachSlaveTxEvent(void (*function)(void) )
{
  twi_onSlaveTransmit = function;
}

boolean TwoWire::i2c_txrx_isr(void)  // RX/TX Service
{
  boolean stay_active = false;

  /* USCI I2C mode. USCI_B0 receive interrupt flag.
   * UCBxRXIFG is set when UCBxRXBUF has received a complete character. */
  if (*UCBxIFG & UCRXIFG) {
    /* Master receive mode. */
    if (twi_state ==  TWI_MRX) {
      twi_masterBuffer[twi_masterBufferIndex++] = *UCBxRXBUF;
      if(twi_masterBufferIndex == twi_masterBufferLength)
        /* Only one byte left. Generate STOP condition.
         * In master mode a STOP is preceded by a NACK */
        *UCBxCTL1 |= UCTXSTP;
      if (twi_masterBufferIndex > twi_masterBufferLength) {
        /* All bytes received. We are idle */
        stay_active = true;
        twi_state = TWI_IDLE;
      }
      /* Slave receive mode. (twi_state = TWI_SRX) */
    } else {
      // if there is still room in the rx buffer
      if (twi_rxBufferIndex < TWI_BUFFER_LENGTH) {
        // put byte in buffer and ack
        twi_rxBuffer[twi_rxBufferIndex++] = *UCBxRXBUF;
      } else {
        // otherwise nack
        *UCBxCTL1 |= UCTXNACK;   // Generate NACK condition
      }
    }
  }
  /* USCI I2C mode. USCI_Bx transmit interrupt flag.
   * UCBxTXIFG is set when UCBxTXBUF is empty.*/
  if (*UCBxIFG & UCTXIFG){
    /* Master transmit mode */
    if (twi_state == TWI_MTX) {
      // if there is data to send, send it, otherwise stop
      if (twi_masterBufferIndex < twi_masterBufferLength) {
        // Copy data to output register and ack.
        *UCBxTXBUF = twi_masterBuffer[twi_masterBufferIndex++];
      } else {
        if (twi_sendStop) {
          /* All done. Generate STOP condition and IDLE */
          *UCBxCTL1 |= UCTXSTP;
        } else {
          twi_inRepStart = true;  // we're gonna send the START
          // don't enable the interrupt. We'll generate the start, but we
          // avoid handling the interrupt until we're in the next transaction,
          // at the point where we would normally issue the start.
          *UCBxCTL1 |= UCTXSTT;
        }
        twi_state = TWI_IDLE;
        stay_active = true;
      }
      /* Slave transmit mode (twi_state = TWI_STX) */
    } else {
      // copy data to output register
      *UCBxTXBUF = twi_txBuffer[twi_txBufferIndex++];
      // if there is more to send, ack, otherwise nack
      if(twi_txBufferIndex < twi_txBufferLength) {
      } else {
        *UCBxCTL1 |= UCTXNACK;    // Generate NACK condition
      }
    }
  }
  return stay_active;
}

boolean TwoWire::i2c_state_isr(void)  // I2C Service
{
  boolean stay_active = false;

  /* Arbitration lost interrupt flag */
  if (*UCBxIFG & UCALIFG) {
    *UCBxIFG &= ~UCALIFG;
    /* TODO: Handle bus arbitration lost */
    twi_error = TWI_ERROR_OTHER;
    twi_state = TWI_IDLE;
    stay_active = true;
  }
  /* Not-acknowledge received interrupt flag.
   * UCNACKIFG is automatically cleared when a START condition is received.*/
  if (*UCBxIFG & UCNACKIFG) {
    *UCBxIFG &= ~UCNACKIFG;
    ////UCBxCTL1 |= UCTXSTP;
    /* TODO: This can just as well be an address NACK.
     * Figure out a way to distinguish between ANACK and DNACK */
    if (twi_masterBufferIndex == 0) {
      twi_error = TWI_ERROR_ADDR_NACK;
      twi_state = TWI_IDLE;
      stay_active = true;
    }
    else {
      twi_error = TWI_ERROR_DATA_NACK;
      twi_state = TWI_IDLE;
      stay_active = true;
    }
  }
  /* Start condition interrupt flag.
   * UCSTTIFG is automatically cleared if a STOP condition is received. */
  if (*UCBxIFG & UCSTTIFG) {
    *UCBxIFG &= ~UCSTTIFG;
    /* UCTR is automagically set by the USCI module upon a START condition. */
    if (*UCBxCTL1 &  UCTR) {
      /* Slave TX mode. */
      twi_state =  TWI_STX;
      /* Ready the tx buffer index for iteration. */
      twi_txBufferIndex = 0;
      /* Set tx buffer length to be zero, to verify if user changes it. */
      twi_txBufferLength = 0;
      /* Request for txBuffer to be filled and length to be set. */
      /* note: user must call twi_transmit(bytes, length) to do this */
      (*twi_onSlaveTransmit)();
      /* If they didn't change buffer & length, initialize it
       * TODO: Is this right? Shouldn't we reply with a NACK if there is no data to send? */
      if (0 == twi_txBufferLength) {
        twi_txBufferLength = 1;
        twi_txBuffer[0] = 0x00;
      }
    } else {
      /* Slave receive mode. */
      twi_state =  TWI_SRX;
      /* Indicate that rx buffer can be overwritten and ACK */
      twi_rxBufferIndex = 0;
    }
  }
  /* Stop condition interrupt flag.
   * UCSTPIFG is automatically cleared when a STOP condition is received. */
  if (*UCBxIFG & UCSTPIFG) {
    *UCBxIFG &= ~UCSTPIFG;
    if (twi_state ==  TWI_SRX) {
      /* Callback to user defined callback */
      (*twi_onSlaveReceive)(twi_rxBuffer, twi_rxBufferIndex);
    }
    twi_state =  TWI_IDLE;
    stay_active = true;
  }
  return stay_active;
}

boolean i2c_txrx_isr(uint8_t module)
{
#ifdef ESAT_BOARD_HAS_SECOND_I2C_BUS
  if (module == 1) {
    return Wire1.i2c_txrx_isr();
  }
  else {
    return Wire.i2c_txrx_isr();
  }
#else
  return Wire.i2c_txrx_isr();
#endif
}

boolean i2c_state_isr(uint8_t module)
{
#ifdef ESAT_BOARD_HAS_SECOND_I2C_BUS
  if (module == 1) {
    return Wire1.i2c_state_isr();
  }
  else {
    return Wire.i2c_state_isr();
  }
#else
  return Wire.i2c_state_isr();
#endif
}

// Preinstantiate Objects //////////////////////////////////////////////////////
TwoWire Wire = TwoWire(0);
#ifdef ESAT_BOARD_HAS_SECOND_I2C_BUS
TwoWire Wire1 = TwoWire(1);
#endif
