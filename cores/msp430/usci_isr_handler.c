#include "Energia.h"
#if defined(__MSP430_HAS_USCI__) || defined(__MSP430_HAS_USCI_A0__) || defined(__MSP430_HAS_USCI_A1__) \
 || defined(__MSP430_HAS_EUSCI_A0__) || defined(__MSP430_HAS_USCI_B0__) || defined(__MSP430_HAS_USCI_B1__)
#include "usci_isr_handler.h"
/* This dummy function ensures that, when called from any module that 
 * is interested in having the USCIAB0TX_VECTOR and USCIAB0TX_VECTOR
 * installed, the linker won't strip the vectors.*/
void usci_isr_install(){}

#if defined(__MSP430_HAS_USCI_A0__) || defined(__MSP430_HAS_USCI_A1__) || defined(__MSP430_HAS_EUSCI_A0__)
#ifndef USCI_UART_UCRXIFG
#define USCI_UART_UCRXIFG USCI_UCRXIFG
#endif
#ifndef USCI_UART_UCTXIFG
#define USCI_UART_UCTXIFG USCI_UCTXIFG
#endif

#if defined(__MSP430_HAS_EUSCI_A0__) && defined(__MSP430_HAS_EUSCI_A1__)
#define XUSCI_A1_OFFSET (__MSP430_BASEADDRESS_EUSCI_A1__ - __MSP430_BASEADDRESS_EUSCI_A0__)
#else
#define XUSCI_A1_OFFSET (__MSP430_BASEADDRESS_USCI_A1__ - __MSP430_BASEADDRESS_USCI_A0__)
#endif

__attribute__((interrupt(USCI_A0_VECTOR)))
void USCIA0_ISR(void)
{
	switch ( UCA0IV )
	{
		case USCI_UART_UCRXIFG: uart_rx_isr(0); break;
		case USCI_UART_UCTXIFG: uart_tx_isr(0); break;
	}
}

#if defined( __MSP430_HAS_USCI_A1__ ) || defined(__MSP430_HAS_EUSCI_A1__)
__attribute__((interrupt(USCI_A1_VECTOR)))
void USCIA1_ISR(void)
{
  switch ( UCA1IV ) 
  {
    case USCI_UART_UCRXIFG: uart_rx_isr(XUSCI_A1_OFFSET); break;
    case USCI_UART_UCTXIFG: uart_tx_isr(XUSCI_A1_OFFSET); break;
  }  
}
#endif


#endif /* defined(__MSP430_HAS_USCI__) || defined(__MSP430_HAS_USCI_A0__) || defined(__MSP430_HAS_USCI_A1__) ||
          defined(__MSP430_HAS_EUSCI_A0__) */

#ifdef __MSP430_HAS_USCI__
/* USCI_Ax and USCI_Bx share the same TX interrupt vector.
 * UART: 
 *	USCIAB0TX_VECTOR services the UCA0TXIFG set in UC0IFG.
 *	USCIAB0RX_VECTOR services the UCA0RXIFG set in UC0IFG.
 * I2C: 
 *	USCIAB0TX_VECTOR services both UCB0TXIFG and UCB0RXIFG
 *	set in UC0IFG.
 *	USCIAB0RX_VECTOR services the state change interrupts
 *	UCSTTIFG, UCSTPIFG, UCIFG, UCALIFG set in UCB0STAT.*/

__attribute__((interrupt(USCIAB0TX_VECTOR))) 
void USCIAB0TX_ISR(void)
{
	/* USCI_A0 UART interrupt? */
	if (UC0IFG & UCA0TXIFG)
		uart_tx_isr(0);

	/* USCI_B0 I2C TX RX interrupt. */
	if ((UCB0CTL0 & UCMODE_3) == UCMODE_3 && (UC0IFG & (UCB0TXIFG | UCB0RXIFG)) != 0)
		Wire.I2C_Usci_txrx_handler();
}

__attribute__((interrupt(USCIAB0RX_VECTOR)))
void USCIAB0RX_ISR(void)
{
	/* USCI_A0 UART interrupt? */
	if (UC0IFG & UCA0RXIFG)
		uart_rx_isr(0);

	/* USCI_B0 I2C state change interrupt. */
	if ((UCB0STAT & (UCALIFG | UCNACKIFG | UCSTTIFG | UCSTPIFG)) != 0)
		Wire.I2C_Usci_state_handler();
}
#endif // __MSP430_HAS_USCI__
#endif // entire file
