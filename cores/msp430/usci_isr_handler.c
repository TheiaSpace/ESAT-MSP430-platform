#include "Energia.h"
#include "usci_isr_handler.h"

/* This dummy function ensures that, when called from any module that 
 * is interested in having the USCIAB0TX_VECTOR and USCIAB0TX_VECTOR
 * installed, the linker won't strip the vectors.*/
void usci_isr_install(){}

static boolean still_asleep;  // Used to validate whether a user ISR has issued wakeup() inside LPM3/LPM4 sleep modes.

#ifndef USCI_UART_UCRXIFG
#define USCI_UART_UCRXIFG USCI_UCRXIFG
#endif
#ifndef USCI_UART_UCTXIFG
#define USCI_UART_UCTXIFG USCI_UCTXIFG
#endif

#define XUSCI_A1_OFFSET (__MSP430_BASEADDRESS_USCI_A1__ - __MSP430_BASEADDRESS_USCI_A0__)

extern CHardwareSerial *Serial;
#ifdef SERIAL1_AVAILABLE
extern CHardwareSerial *Serial1;
#endif

__attribute__((interrupt(USCI_A0_VECTOR)))
void USCIA0_ISR(void)
{
  still_asleep = stay_asleep;

  switch ( UCA0IV )
  {
  case USCI_UART_UCRXIFG: uart_rx_isr(0); break;
  case USCI_UART_UCTXIFG: uart_tx_isr(0); break;
  }

  if (still_asleep != stay_asleep)
    __bic_SR_register_on_exit(LPM4_bits);
}

__attribute__((interrupt(USCI_A1_VECTOR)))
void USCIA1_ISR(void)
{
  still_asleep = stay_asleep;

  switch ( UCA1IV )
  {
  case USCI_UART_UCRXIFG: uart_rx_isr(XUSCI_A1_OFFSET); break;
  case USCI_UART_UCTXIFG: uart_tx_isr(XUSCI_A1_OFFSET); break;
  }

  if (still_asleep != stay_asleep)
    __bic_SR_register_on_exit(LPM4_bits);
}

__attribute__((interrupt(USCI_B0_VECTOR)))
void USCIB0_ISR(void)
{
  still_asleep = stay_asleep;
  boolean stay_active = false;

  /* USCI_B0 I2C state change interrupt. */
  if ((UCB0CTL0 & UCMODE_3) == UCMODE_3 && (UCB0IFG & (UCALIFG | UCNACKIFG | UCSTTIFG | UCSTPIFG)) != 0)
    stay_active = i2c_state_isr(0);
  /* USCI_B0 I2C TX RX interrupt. */
  if ((UCB0CTL0 & UCMODE_3) == UCMODE_3 && (UCB0IFG & (UCTXIFG | UCRXIFG)) != 0)
    stay_active = i2c_txrx_isr(0);

  if (still_asleep != stay_asleep || stay_active)
    __bic_SR_register_on_exit(LPM4_bits);
}

__attribute__((interrupt(USCI_B1_VECTOR)))
void USCIB1_ISR(void)
{
  still_asleep = stay_asleep;
  boolean stay_active = false;

  /* USCI_B1 I2C state change interrupt. */
  if ((UCB1CTL0 & UCMODE_3) == UCMODE_3 && (UCB1IFG & (UCALIFG | UCNACKIFG | UCSTTIFG | UCSTPIFG)) != 0)
    stay_active = i2c_state_isr(1);
  /* USCI_B1 I2C TX RX interrupt. */
  if ((UCB1CTL0 & UCMODE_3) == UCMODE_3 && (UCB1IFG & (UCTXIFG | UCRXIFG)) != 0)
    stay_active = i2c_txrx_isr(1);

  if (still_asleep != stay_asleep || stay_active)
    __bic_SR_register_on_exit(LPM4_bits);
}
