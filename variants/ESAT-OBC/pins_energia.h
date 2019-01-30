/*
  pins_energia.h - Pin definition functions for Energia.
  Part of Theia Space's ESAT OBC (MSP430) core for Energia.
  Modified for Theia Space's ESAT OBC (MSP430) by Theia Space.
  Copyright (c) 2017, 2018, 2019 Theia Space.

  Original file: pins_energia.h, part of the Energia platform.
  Copyright (c) 2012 Robert Wessels.
  Contribution: Rei VILO.

  Original pins_energia.h derived from:
  pins_arduino.h - Pin definition functions for Arduino.
  Copyright (c) 2007 David A. Mellis.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#ifndef Pins_Energia_h
#define Pins_Energia_h
#ifndef BV
#define BV(x) (1 << (x))
#endif

static const uint8_t SCK     = 48;  /* P4.3 */
static const uint8_t MOSI    = 46;  /* P4.1 */
static const uint8_t MISO    = 47;  /* P4.2 */
static const uint8_t TWISDA0 = 37;  /* P3.0 */
static const uint8_t TWISCL0 = 38;  /* P3.1 */
static const uint8_t AUX_UARTRXD = 52;  /* Receive  Data (RXD) at P4.5 */
static const uint8_t AUX_UARTTXD = 51;  /* Transmit Data (TXD) at P4.4 */
#define TWISDA0_SET_MODE (PORT_SELECTION0)
#define TWISCL0_SET_MODE (PORT_SELECTION0)
#define AUX_UARTRXD_SET_MODE (PORT_SELECTION0 | (PM_UCA1RXD << 8) | INPUT)
#define AUX_UARTTXD_SET_MODE (PORT_SELECTION0 | (PM_UCA1TXD << 8) | OUTPUT)
#define SPISCK_SET_MODE (PORT_SELECTION0)
#define SPIMOSI_SET_MODE (PORT_SELECTION0)
#define SPIMISO_SET_MODE (PORT_SELECTION0)
#define SPI_MODULE 1
#define SPI_AVAILABLE
#define SD_AVAILABLE
/* Define the default I2C settings */
#define DEFAULT_I2C 0
#define TWISDA TWISDA0
#define TWISCL TWISCL0
#define TWISDA_SET_MODE  TWISDA0_SET_MODE
#define TWISCL_SET_MODE  TWISCL0_SET_MODE

#define AUX_UART_MODULE_OFFSET 0x40

#define USE_USCI_A1

/* Analog inputs */
static const uint8_t ADC12 = 5;
static const uint8_t ADC13 = 6;
static const uint8_t ADC14 = 7;
static const uint8_t ADC15 = 8;

/* Wifi module control */
static const uint8_t ESPRST = 36;
static const uint8_t ESP_SLEEP = 25;
static const uint8_t ESP0 = 35;
static const uint8_t RX1 = 52; /* Receive data from the Wifi module */
static const uint8_t TX1 = 51; /* Send data to the Wifi module */
#define SerialWifi (Serial1) /* Serial interface for the Wifi module */

/* General purpose IO */
static const uint8_t GPIO0 = 28;
static const uint8_t GPIO1 = 29;
static const uint8_t GPIO2 = 30;

/* Coarse sun sensors */
static const uint8_t CSSXMINUS = 4; /* Coarse sun sensor on X- panel */
static const uint8_t CSSXPLUS = 10; /* Coarse sun sensor on X+ panel */
static const uint8_t CSSYMINUS = 3; /* Coarse sun sensor on Y- panel */
static const uint8_t CSSYPLUS = 9;  /* Coarse sun sensor on Y+ panel */

/* Magnetorquer pins */
static const uint8_t ENMTQX = 79; /* Enable magnetorquer (X axis) */
static const uint8_t ENMTQY = 78; /* Enable magnetorquer (Y axis) */
static const uint8_t MTQX = 80;   /* Magnetorquer polarity (X axis) */
static const uint8_t MTQY = 1;    /* Magnetorquer polarity (Y axis) */

/* Wheel and tachometer pins */
static const uint8_t PWM = 23; /* Wheel control */
static const uint8_t TCH = 34; /* Tachometer */

/* EPS interrupt line */
static const uint8_t EMG = 33;

/* I2C bus */
static const uint8_t SCL_O = 38;
static const uint8_t SDA_O = 37;
#define WireOBC (Wire) /* OBC I2C bus */

/* SPI bus */
static const uint8_t CLK_O = 48;  /* Clock, OBC SPI bus */
static const uint8_t CS_O = 59;   /* User chip select */
static const uint8_t CS_SD = 45;  /* SD card chip select */
static const uint8_t MISO_O = 47; /* MISO, OBC SPI bus */
static const uint8_t MOSI_O = 46; /* MOSI, OBC SPI bus */

/* Spy-Bi-Wire debugging interface */
static const uint8_t TEST_O = 71;
static const uint8_t RST_O = 76;

/* LED */
static const uint8_t LED_O = 60; /* OBC board LED */
static const uint8_t LED_BUILTIN = 60; /* Same as LED_O */

/* Internal thermometer */
static const uint8_t TEMPSENSOR = 128 + 10;

/* Vcc/2 */
static const uint8_t VCC_2 = 128 + 11;

/*
 * These serial port names are intended to allow libraries and architecture-neutral
 * sketches to automatically default to the correct port name for a particular type
 * of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
 * the first hardware serial port whose RX/TX pins are not dedicated to another use.
 *
 * SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
 *
 * SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
 *
 * SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
 *
 * SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
 *
 * SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
 *                            pins are NOT connected to anything by default.
 */
#define SERIAL_PORT_MONITOR       Serial
#define SERIAL_PORT_USBVIRTUAL    Serial
#define SERIAL_PORT_HARDWARE      Serial1
#define SERIAL_PORT_HARDWARE_OPEN Serial1
#define SerialUSB                 SERIAL_PORT_USBVIRTUAL

#ifdef ARDUINO_MAIN

const uintptr_t port_to_input[] = {
  NOT_A_PORT,
  (uintptr_t) &P1IN,
  (uintptr_t) &P2IN,
  (uintptr_t) &P3IN,
  (uintptr_t) &P4IN,
  (uintptr_t) &P5IN,
  (uintptr_t) &P6IN,
  (uintptr_t) &P7IN,
  (uintptr_t) &P8IN,
};

const uintptr_t port_to_output[] = {
  NOT_A_PORT,
  (uintptr_t) &P1OUT,
  (uintptr_t) &P2OUT,
  (uintptr_t) &P3OUT,
  (uintptr_t) &P4OUT,
  (uintptr_t) &P5OUT,
  (uintptr_t) &P6OUT,
  (uintptr_t) &P7OUT,
  (uintptr_t) &P8OUT,
};

const uintptr_t port_to_dir[] = {
  NOT_A_PORT,
  (uintptr_t) &P1DIR,
  (uintptr_t) &P2DIR,
  (uintptr_t) &P3DIR,
  (uintptr_t) &P4DIR,
  (uintptr_t) &P5DIR,
  (uintptr_t) &P6DIR,
  (uintptr_t) &P7DIR,
  (uintptr_t) &P8DIR,
};

const uintptr_t port_to_ren[] = {
  NOT_A_PORT,
  (uintptr_t) &P1REN,
  (uintptr_t) &P2REN,
  (uintptr_t) &P3REN,
  (uintptr_t) &P4REN,
  (uintptr_t) &P5REN,
  (uintptr_t) &P6REN,
  (uintptr_t) &P7REN,
  (uintptr_t) &P8REN,
};

const uintptr_t port_to_sel0[] = {  /* put this PxSEL register under the group of PxSEL0 */
  NOT_A_PORT,
  (uintptr_t) &P1SEL,
  (uintptr_t) &P2SEL,
  (uintptr_t) &P3SEL,
  (uintptr_t) &P4SEL,
  (uintptr_t) &P5SEL,
  (uintptr_t) &P6SEL,
  (uintptr_t) &P7SEL,
  (uintptr_t) &P8SEL,
};

const uintptr_t port_to_pmap[] = {
  NOT_A_PORT, /* PMAP starts at port P1 */
  NOT_A_PORT,
  NOT_A_PORT,
  NOT_A_PORT,
  (uintptr_t) &P4MAP0,
  NOT_A_PORT,
  NOT_A_PORT,
  NOT_A_PORT,
  NOT_A_PORT,
};

const uint8_t digital_pin_to_timer[] = {
  NOT_ON_TIMER, /*  dummy */
  NOT_ON_TIMER, /*  1 - P6.4/CB4/A4 */
  NOT_ON_TIMER, /*  2 - P6.5/CB5/A5 */
  NOT_ON_TIMER, /*  3 - P6.6/CB6/A6 */
  NOT_ON_TIMER, /*  4 - P6.7/CB7/A7 */
  NOT_ON_TIMER, /*  5 - P7.0/CB8/A12 */
  NOT_ON_TIMER, /*  6 - P7.1/CB9/A13 */
  NOT_ON_TIMER, /*  7 - P7.2/CB10/A14 */
  NOT_ON_TIMER, /*  8 - P7.3/CB11/A15 */
  NOT_ON_TIMER, /*  9 - P5.0/A8/VREF+/VeRERF+ */
  NOT_ON_TIMER, /* 10 - P5.1/A9/VREF-/VeREF- */
  NOT_ON_TIMER, /* 11 - AVCC1 */
  NOT_ON_TIMER, /* 12 - P5.4/XIN */
  NOT_ON_TIMER, /* 13 - P5.5/XOUT */
  NOT_ON_TIMER, /* 14 - AVSS1 */
  NOT_ON_TIMER, /* 15 - P8.0 */
  NOT_ON_TIMER, /* 16 - P8.1 */
  NOT_ON_TIMER, /* 17 - P8.2 */
  NOT_ON_TIMER, /* 18 - DVCC1 */
  NOT_ON_TIMER, /* 19 - DVSS1 */
  NOT_ON_TIMER, /* 20 - VCORE  */
  NOT_ON_TIMER, /* 21 - P1.0/TA0CLK/ACLK */
  T0A0,         /* 22 - P1.1/TA0.0 */
  T0A1,         /* 23 - P1.2/TA0.1 */
  T0A2,         /* 24 - P1.3/TA0.2 */
  T0A3,         /* 25 - P1.4/TA0.3 */
  T0A4,         /* 26 - P1.5/TA0.4 */
  NOT_ON_TIMER, /* 27 - P1.6/TA1CLK/CBOUT */
  T1A0,         /* 28 - P1.7/TA1.0 */
  T1A1,         /* 29 - P2.0/TA1.1 */
  T1A2,         /* 30 - P2.1/TA1.2 */
  NOT_ON_TIMER, /* 31 - P2.2/TA2CLK/SMCLK */
  T2A0,         /* 32 - P2.3/TA2.0 */
  T2A1,         /* 33 - P2.4/TA2.1 */
  T2A2,         /* 34 - P2.5/TA2.2 */
  NOT_ON_TIMER, /* 35 - P2.6/RTCCLK/DMAE0 */
  NOT_ON_TIMER, /* 36 - P2.7/UCB0STE/UCA0CLK */
  NOT_ON_TIMER, /* 37 - P3.0/UCB0SIMO/UCB0SDA */
  NOT_ON_TIMER, /* 38 - P3.1/UCB0SOMI/UCB0SCL */
  NOT_ON_TIMER, /* 39 - P3.2/UCB0CLK/UCA0STE */
  NOT_ON_TIMER, /* 40 - P3.3/UCA0TXD/UCA0SIMO */
  NOT_ON_TIMER, /* 41 - P3.4/UCA0RXD/UCA0SOMI */
  T0B5,         /* 42 - P3.5/TB0.5 */
  T0B6,         /* 43 - P3.6/TB0.6 */
  NOT_ON_TIMER, /* 44 - P3.7/TB0OUTH/SVMOUT */
  NOT_ON_TIMER, /* 45 - P4.0/PM_UCB1STE/PM_UCA1CLK */
  NOT_ON_TIMER, /* 46 - P4.1/PM_UCB1SIMO/PM_UCB1SDA */
  NOT_ON_TIMER, /* 47 - P4.2/PM_UCB1SOMI/PM_UCB1SCL */
  NOT_ON_TIMER, /* 48 - P4.3/PM_UCB1CLK/PM_UCA1STE */
  NOT_ON_TIMER, /* 49 - DVSS2 */
  NOT_ON_TIMER, /* 50 - DVCC2 */
  NOT_ON_TIMER, /* 51 - P4.4/PM_UCA1TXD/PM_UCA1SIMO */
  NOT_ON_TIMER, /* 52 - P4.5/PM_UCA1RXD/PM_UCA1SOMI */
  NOT_ON_TIMER, /* 53 - P4.6/PM_NONE */
  NOT_ON_TIMER, /* 54 - P4.7/PM_NONE */
  T0B0,         /* 55 - P5.6/TB0.0 */
  T0B1,         /* 56 - P5.7/TB0.1 */
  T0B2,         /* 57 - P7.4/TB0.2 */
  T0B3,         /* 58 - P7.5/TB0.3 */
  T0B4,         /* 59 - P7.6/TB0.4 */
  NOT_ON_TIMER, /* 60 - P7.7/TB0CLK/MCLK */
  NOT_ON_TIMER, /* 61 - VSSU */
  NOT_ON_TIMER, /* 62 - PU.0/DP */
  NOT_ON_TIMER, /* 63 - PUR */
  NOT_ON_TIMER, /* 64 - PU.1/DM */
  NOT_ON_TIMER, /* 65 - VBUS */
  NOT_ON_TIMER, /* 66 - VUSB */
  NOT_ON_TIMER, /* 67 - V18 */
  NOT_ON_TIMER, /* 68 - AVSS2 */
  NOT_ON_TIMER, /* 69 - P5.2/XT2IN */
  NOT_ON_TIMER, /* 70 - P5.3/XT2OUT */
  NOT_ON_TIMER, /* 71 - TEST/SBWTCK */
  NOT_ON_TIMER, /* 72 - PJ.0/TD0 */
  NOT_ON_TIMER, /* 73 - PJ.1/TDI/TCLK */
  NOT_ON_TIMER, /* 74 - PJ.2/TMS */
  NOT_ON_TIMER, /* 75 - PJ.3/TCK */
  NOT_ON_TIMER, /* 76 - RST/NMI/SBWTDIO */
  NOT_ON_TIMER, /* 77 - P6.0/CB0/A0  */
  NOT_ON_TIMER, /* 78 - P6.1/CB1/A1 */
  NOT_ON_TIMER, /* 79 - P6.2/CB2/A2 */
  NOT_ON_TIMER, /* 80 - P6.3/CB3/A3 */
};

const uint8_t digital_pin_to_port[] = {
  NOT_A_PIN, /*  dummy */
  P6,        /*  1 - P6.4/CB4/A4 */
  P6,        /*  2 - P6.5/CB5/A5 */
  P6,        /*  3 - P6.6/CB6/A6 */
  P6,        /*  4 - P6.7/CB7/A7 */
  P7,        /*  5 - P7.0/CB8/A12 */
  P7,        /*  6 - P7.1/CB9/A13 */
  P7,        /*  7 - P7.2/CB10/A14 */
  P7,        /*  8 - P7.3/CB11/A15 */
  P5,        /*  9 - P5.0/A8/VREF+/VeRERF+ */
  P5,        /* 10 - P5.1/A9/VREF-/VeREF- */
  NOT_A_PIN, /* 11 - AVCC1 */
  P5,        /* 12 - P5.4/XIN */
  P5,        /* 13 - P5.5/XOUT */
  NOT_A_PIN, /* 14 - AVSS1 */
  P8,        /* 15 - P8.0 */
  P8,        /* 16 - P8.1 */
  P8,        /* 17 - P8.2 */
  NOT_A_PIN, /* 18 - DVCC1 */
  NOT_A_PIN, /* 19 - DVSS1 */
  NOT_A_PIN, /* 20 - VCORE  */
  P1,        /* 21 - P1.0/TA0CLK/ACLK */
  P1,        /* 22 - P1.1/TA0.0 */
  P1,        /* 23 - P1.2/TA0.1 */
  P1,        /* 24 - P1.3/TA0.2 */
  P1,        /* 25 - P1.4/TA0.3 */
  P1,        /* 26 - P1.5/TA0.4 */
  P1,        /* 27 - P1.6/TA1CLK/CBOUT */
  P1,        /* 28 - P1.7/TA1.0 */
  P2,        /* 29 - P2.0/TA1.1 */
  P2,        /* 30 - P2.1/TA1.2 */
  P2,        /* 31 - P2.2/TA2CLK/SMCLK */
  P2,        /* 32 - P2.3/TA2.0 */
  P2,        /* 33 - P2.4/TA2.1 */
  P2,        /* 34 - P2.5/TA2.2 */
  P2,        /* 35 - P2.6/RTCCLK/DMAE0 */
  P2,        /* 36 - P2.7/UCB0STE/UCA0CLK */
  P3,        /* 37 - P3.0/UCB0SIMO/UCB0SDA */
  P3,        /* 38 - P3.1/UCB0SOMI/UCB0SCL */
  P3,        /* 39 - P3.2/UCB0CLK/UCA0STE */
  P3,        /* 40 - P3.3/UCA0TXD/UCA0SIMO */
  P3,        /* 41 - P3.4/UCA0RXD/UCA0SOMI */
  P3,        /* 42 - P3.5/TB0.5 */
  P3,        /* 43 - P3.6/TB0.6 */
  P3,        /* 44 - P3.7/TB0OUTH/SVMOUT */
  P4,        /* 45 - P4.0/PM_UCB1STE/PM_UCA1CLK */
  P4,        /* 46 - P4.1/PM_UCB1SIMO/PM_UCB1SDA */
  P4,        /* 47 - P4.2/PM_UCB1SOMI/PM_UCB1SCL */
  P4,        /* 48 - P4.3/PM_UCB1CLK/PM_UCA1STE */
  NOT_A_PIN, /* 49 - DVSS2 */
  NOT_A_PIN, /* 50 - DVCC2 */
  P4,        /* 51 - P4.4/PM_UCA1TXD/PM_UCA1SIMO */
  P4,        /* 52 - P4.5/PM_UCA1RXD/PM_UCA1SOMI */
  P4,        /* 53 - P4.6/PM_NONE */
  P4,        /* 54 - P4.7/PM_NONE */
  P5,        /* 55 - P5.6/TB0.0 */
  P5,        /* 56 - P5.7/TB0.1 */
  P7,        /* 57 - P7.4/TB0.2 */
  P7,        /* 58 - P7.5/TB0.3 */
  P7,        /* 59 - P7.6/TB0.4 */
  P7,        /* 60 - P7.7/TB0CLK/MCLK */
  NOT_A_PIN, /* 61 - VSSU */
  NOT_A_PIN, /* 62 - PU.0/DP */
  NOT_A_PIN, /* 63 - PUR */
  NOT_A_PIN, /* 64 - PU.1/DM */
  NOT_A_PIN, /* 65 - VBUS */
  NOT_A_PIN, /* 66 - VUSB */
  NOT_A_PIN, /* 67 - V18 */
  NOT_A_PIN, /* 68 - AVSS2 */
  P5,        /* 69 - P5.2/XT2IN */
  P5,        /* 70 - P5.3/XT2OUT */
  NOT_A_PIN, /* 71 - TEST/SBWTCK */
  NOT_A_PIN, /* 72 - PJ.0/TD0 */
  NOT_A_PIN, /* 73 - PJ.1/TDI/TCLK */
  NOT_A_PIN, /* 74 - PJ.2/TMS */
  NOT_A_PIN, /* 75 - PJ.3/TCK */
  NOT_A_PIN, /* 76 - RST/NMI/SBWTDIO */
  P6,        /* 77 - P6.0/CB0/A0  */
  P6,        /* 78 - P6.1/CB1/A1 */
  P6,        /* 79 - P6.2/CB2/A2 */
  P6         /* 80 - P6.3/CB3/A3 */
};

const uint8_t digital_pin_to_bit_mask[] = {
  NOT_A_PIN, /*  dummy */
  BV(4),     /*  1 - P6.4/CB4/A4 */
  BV(5),     /*  2 - P6.5/CB5/A5 */
  BV(6),     /*  3 - P6.6/CB6/A6 */
  BV(7),     /*  4 - P6.7/CB7/A7 */
  BV(0),     /*  5 - P7.0/CB8/A12 */
  BV(1),     /*  6 - P7.1/CB9/A13 */
  BV(2),     /*  7 - P7.2/CB10/A14 */
  BV(3),     /*  8 - P7.3/CB11/A15 */
  BV(0),     /*  9 - P5.0/A8/VREF+/VeRERF+ */
  BV(1),     /* 10 - P5.1/A9/VREF-/VeREF- */
  NOT_A_PIN, /* 11 - AVCC1 */
  BV(4),     /* 12 - P5.4/XIN */
  BV(5),     /* 13 - P5.5/XOUT */
  NOT_A_PIN, /* 14 - AVSS1 */
  BV(0),     /* 15 - P8.0 */
  BV(1),     /* 16 - P8.1 */
  BV(2),     /* 17 - P8.2 */
  NOT_A_PIN, /* 18 - DVCC1 */
  NOT_A_PIN, /* 19 - DVSS1 */
  NOT_A_PIN, /* 20 - VCORE  */
  BV(0),     /* 21 - P1.0/TA0CLK/ACLK */
  BV(1),     /* 22 - P1.1/TA0.0 */
  BV(2),     /* 23 - P1.2/TA0.1 */
  BV(3),     /* 24 - P1.3/TA0.2 */
  BV(4),     /* 25 - P1.4/TA0.3 */
  BV(5),     /* 26 - P1.5/TA0.4 */
  BV(6),     /* 27 - P1.6/TA1CLK/CBOUT */
  BV(7),     /* 28 - P1.7/TA1.0 */
  BV(0),     /* 29 - P2.0/TA1.1 */
  BV(1),     /* 30 - P2.1/TA1.2 */
  BV(2),     /* 31 - P2.2/TA2CLK/SMCLK */
  BV(3),     /* 32 - P2.3/TA2.0 */
  BV(4),     /* 33 - P2.4/TA2.1 */
  BV(5),     /* 34 - P2.5/TA2.2 */
  BV(6),     /* 35 - P2.6/RTCCLK/DMAE0 */
  BV(7),     /* 36 - P2.7/UCB0STE/UCA0CLK */
  BV(0),     /* 37 - P3.0/UCB0SIMO/UCB0SDA */
  BV(1),     /* 38 - P3.1/UCB0SOMI/UCB0SCL */
  BV(2),     /* 39 - P3.2/UCB0CLK/UCA0STE */
  BV(3),     /* 40 - P3.3/UCA0TXD/UCA0SIMO */
  BV(4),     /* 41 - P3.4/UCA0RXD/UCA0SOMI */
  BV(5),     /* 42 - P3.5/TB0.5 */
  BV(6),     /* 43 - P3.6/TB0.6 */
  BV(7),     /* 44 - P3.7/TB0OUTH/SVMOUT */
  BV(0),     /* 45 - P4.0/PM_UCB1STE/PM_UCA1CLK */
  BV(1),     /* 46 - P4.1/PM_UCB1SIMO/PM_UCB1SDA */
  BV(2),     /* 47 - P4.2/PM_UCB1SOMI/PM_UCB1SCL */
  BV(3),     /* 48 - P4.3/PM_UCB1CLK/PM_UCA1STE */
  NOT_A_PIN, /* 49 - DVSS2 */
  NOT_A_PIN, /* 50 - DVCC2 */
  BV(4),     /* 51 - P4.4/PM_UCA1TXD/PM_UCA1SIMO */
  BV(5),     /* 52 - P4.5/PM_UCA1RXD/PM_UCA1SOMI */
  BV(6),     /* 53 - P4.6/PM_NONE */
  BV(7),     /* 54 - P4.7/PM_NONE */
  BV(6),     /* 55 - P5.6/TB0.0 */
  BV(7),     /* 56 - P5.7/TB0.1 */
  BV(4),     /* 57 - P7.4/TB0.2 */
  BV(5),     /* 58 - P7.5/TB0.3 */
  BV(6),     /* 59 - P7.6/TB0.4 */
  BV(7),     /* 60 - P7.7/TB0CLK/MCLK */
  NOT_A_PIN, /* 61 - VSSU */
  NOT_A_PIN, /* 62 - PU.0/DP */
  NOT_A_PIN, /* 63 - PUR */
  NOT_A_PIN, /* 64 - PU.1/DM */
  NOT_A_PIN, /* 65 - VBUS */
  NOT_A_PIN, /* 66 - VUSB */
  NOT_A_PIN, /* 67 - V18 */
  NOT_A_PIN, /* 68 - AVSS2 */
  BV(2),     /* 69 - P5.2/XT2IN */
  BV(3),     /* 70 - P5.3/XT2OUT */
  NOT_A_PIN, /* 71 - TEST/SBWTCK */
  NOT_A_PIN, /* 72 - PJ.0/TD0 */
  NOT_A_PIN, /* 73 - PJ.1/TDI/TCLK */
  NOT_A_PIN, /* 74 - PJ.2/TMS */
  NOT_A_PIN, /* 75 - PJ.3/TCK */
  NOT_A_PIN, /* 76 - RST/NMI/SBWTDIO */
  BV(0),     /* 77 - P6.0/CB0/A0  */
  BV(1),     /* 78 - P6.1/CB1/A1 */
  BV(2),     /* 79 - P6.2/CB2/A2 */
  BV(3)      /* 80 - P6.3/CB3/A3 */
};

const uint32_t digital_pin_to_analog_in[] = {
  NOT_ON_ADC, /*  dummy */
  4,          /*  1 - P6.4/CB4/A4 */
  5,          /*  2 - P6.5/CB5/A5 */
  6,          /*  3 - P6.6/CB6/A6 */
  7,          /*  4 - P6.7/CB7/A7 */
  12,         /*  5 - P7.0/CB8/A12 */
  13,         /*  6 - P7.1/CB9/A13 */
  14,         /*  7 - P7.2/CB10/A14 */
  15,         /*  8 - P7.3/CB11/A15 */
  8,          /*  9 - P5.0/A8/VREF+/VeRERF+ */
  9,          /* 10 - P5.1/A9/VREF-/VeREF- */
  NOT_ON_ADC, /* 11 - AVCC1 */
  NOT_ON_ADC, /* 12 - P5.4/XIN */
  NOT_ON_ADC, /* 13 - P5.5/XOUT */
  NOT_ON_ADC, /* 14 - AVSS1 */
  NOT_ON_ADC, /* 15 - P8.0 */
  NOT_ON_ADC, /* 16 - P8.1 */
  NOT_ON_ADC, /* 17 - P8.2 */
  NOT_ON_ADC, /* 18 - DVCC1 */
  NOT_ON_ADC, /* 19 - DVSS1 */
  NOT_ON_ADC, /* 20 - VCORE  */
  NOT_ON_ADC, /* 21 - P1.0/TA0CLK/ACLK */
  NOT_ON_ADC, /* 22 - P1.1/TA0.0 */
  NOT_ON_ADC, /* 23 - P1.2/TA0.1 */
  NOT_ON_ADC, /* 24 - P1.3/TA0.2 */
  NOT_ON_ADC, /* 25 - P1.4/TA0.3 */
  NOT_ON_ADC, /* 26 - P1.5/TA0.4 */
  NOT_ON_ADC, /* 27 - P1.6/TA1CLK/CBOUT */
  NOT_ON_ADC, /* 28 - P1.7/TA1.0 */
  NOT_ON_ADC, /* 29 - P2.0/TA1.1 */
  NOT_ON_ADC, /* 30 - P2.1/TA1.2 */
  NOT_ON_ADC, /* 31 - P2.2/TA2CLK/SMCLK */
  NOT_ON_ADC, /* 32 - P2.3/TA2.0 */
  NOT_ON_ADC, /* 33 - P2.4/TA2.1 */
  NOT_ON_ADC, /* 34 - P2.5/TA2.2 */
  NOT_ON_ADC, /* 35 - P2.6/RTCCLK/DMAE0 */
  NOT_ON_ADC, /* 36 - P2.7/UCB0STE/UCA0CLK */
  NOT_ON_ADC, /* 37 - P3.0/UCB0SIMO/UCB0SDA */
  NOT_ON_ADC, /* 38 - P3.1/UCB0SOMI/UCB0SCL */
  NOT_ON_ADC, /* 39 - P3.2/UCB0CLK/UCA0STE */
  NOT_ON_ADC, /* 40 - P3.3/UCA0TXD/UCA0SIMO */
  NOT_ON_ADC, /* 41 - P3.4/UCA0RXD/UCA0SOMI */
  NOT_ON_ADC, /* 42 - P3.5/TB0.5 */
  NOT_ON_ADC, /* 43 - P3.6/TB0.6 */
  NOT_ON_ADC, /* 44 - P3.7/TB0OUTH/SVMOUT */
  NOT_ON_ADC, /* 45 - P4.0/PM_UCB1STE/PM_UCA1CLK */
  NOT_ON_ADC, /* 46 - P4.1/PM_UCB1SIMO/PM_UCB1SDA */
  NOT_ON_ADC, /* 47 - P4.2/PM_UCB1SOMI/PM_UCB1SCL */
  NOT_ON_ADC, /* 48 - P4.3/PM_UCB1CLK/PM_UCA1STE */
  NOT_ON_ADC, /* 49 - DVSS2 */
  NOT_ON_ADC, /* 50 - DVCC2 */
  NOT_ON_ADC, /* 51 - P4.4/PM_UCA1TXD/PM_UCA1SIMO */
  NOT_ON_ADC, /* 52 - P4.5/PM_UCA1RXD/PM_UCA1SOMI */
  NOT_ON_ADC, /* 53 - P4.6/PM_NONE */
  NOT_ON_ADC, /* 54 - P4.7/PM_NONE */
  NOT_ON_ADC, /* 55 - P5.6/TB0.0 */
  NOT_ON_ADC, /* 56 - P5.7/TB0.1 */
  NOT_ON_ADC, /* 57 - P7.4/TB0.2 */
  NOT_ON_ADC, /* 58 - P7.5/TB0.3 */
  NOT_ON_ADC, /* 59 - P7.6/TB0.4 */
  NOT_ON_ADC, /* 60 - P7.7/TB0CLK/MCLK */
  NOT_ON_ADC, /* 61 - VSSU */
  NOT_ON_ADC, /* 62 - PU.0/DP */
  NOT_ON_ADC, /* 63 - PUR */
  NOT_ON_ADC, /* 64 - PU.1/DM */
  NOT_ON_ADC, /* 65 - VBUS */
  NOT_ON_ADC, /* 66 - VUSB */
  NOT_ON_ADC, /* 67 - V18 */
  NOT_ON_ADC, /* 68 - AVSS2 */
  NOT_ON_ADC, /* 69 - P5.2/XT2IN */
  NOT_ON_ADC, /* 70 - P5.3/XT2OUT */
  NOT_ON_ADC, /* 71 - TEST/SBWTCK */
  NOT_ON_ADC, /* 72 - PJ.0/TD0 */
  NOT_ON_ADC, /* 73 - PJ.1/TDI/TCLK */
  NOT_ON_ADC, /* 74 - PJ.2/TMS */
  NOT_ON_ADC, /* 75 - PJ.3/TCK */
  NOT_ON_ADC, /* 76 - RST/NMI/SBWTDIO */
  0,          /* 77 - P6.0/CB0/A0  */
  1,          /* 78 - P6.1/CB1/A1 */
  2,          /* 79 - P6.2/CB2/A2 */
  3           /* 80 - P6.3/CB3/A3 */
};
#endif // #ifdef ARDUINO_MAIN
#endif // #ifndef Pins_Energia_h
