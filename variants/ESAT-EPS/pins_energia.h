/*
  pins_energia.h - Pin definition functions for Energia.
  Part of Theia Space's ESAT EPS (MSP430) core for Energia.
  Modified for Theia Space's ESAT EPS (MSP430) by Theia Space.
  Copyright (c) 2017 Theia Space.

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

#if defined(__MSP430_HAS_USCI_A0__) || defined(__MSP430_HAS_USCI_A1__)
static const uint8_t SS      = 36;  /* P2.7 */
static const uint8_t SCK     = 39;  /* P3.2 */
static const uint8_t MOSI    = 37;  /* P3.0 */
static const uint8_t MISO    = 38;  /* P3.1 */
static const uint8_t SS1     = 45;  /* P2.7 */
static const uint8_t SCK1    = 48;  /* P3.2 */
static const uint8_t MOSI1   = 46;  /* P3.0 */
static const uint8_t MISO1   = 47;  /* P3.1 */
static const uint8_t TWISDA  = 37;  /* P3.0 */
static const uint8_t TWISCL  = 38;  /* P3.1 */
static const uint8_t TWISDA2 = 46;  /* P4.1 */
static const uint8_t TWISCL2 = 47;  /* P4.2 */
static const uint8_t DEBUG_UARTRXD = 52;  /* Receive  Data (RXD) at P4.5 */
static const uint8_t DEBUG_UARTTXD = 51;  /* Transmit Data (TXD) at P4.4 */
static const uint8_t AUX_UARTRXD = 41;  /* Receive  Data (RXD) at P4.5 */
static const uint8_t AUX_UARTTXD = 40;  /* Transmit Data (TXD) at P4.4 */
#define TWISDA_SET_MODE (PORT_SELECTION0)
#define TWISCL_SET_MODE (PORT_SELECTION0)
#define TWISDA_SET_MODE1 (PORT_SELECTION0 | (PM_UCB1SDA << 8) | INPUT)
#define TWISCL_SET_MODE1 (PORT_SELECTION0 | (PM_UCB1SCL << 8) | INPUT)
#define DEBUG_UARTRXD_SET_MODE (PORT_SELECTION0 | (PM_UCA1RXD << 8) | INPUT)
#define DEBUG_UARTTXD_SET_MODE (PORT_SELECTION0 | (PM_UCA1TXD << 8) | OUTPUT)
#define AUX_UARTRXD_SET_MODE (PORT_SELECTION0 | INPUT)
#define AUX_UARTTXD_SET_MODE (PORT_SELECTION0 | OUTPUT)
#define SPISCK_SET_MODE (PORT_SELECTION0)
#define SPIMOSI_SET_MODE (PORT_SELECTION0)
#define SPIMISO_SET_MODE (PORT_SELECTION0)
#define SPISCK_SET_MODE1 (PORT_SELECTION0 | (PM_UCB1STE << 8) | INPUT)
#define SPIMOSI_SET_MODE1 (PORT_SELECTION0 | (PM_UCB1SIMO << 8) | OUTPUT)
#define SPIMISO_SET_MODE1 (PORT_SELECTION0 | (PM_UCB1SOMI << 8) | INPUT)
#endif

#define DEBUG_UART_MODULE_OFFSET 0x40
#define AUX_UART_MODULE_OFFSET 0x0
#define SERIAL1_AVAILABLE 1

#if defined(__MSP430_HAS_USCI_A1__)
#define USE_USCI_A1
#endif

/* Current and voltage sensor pins */
static const uint8_t I_12V = 77;    /* 12V line current */
static const uint8_t V_12V = 78;    /* 12V line voltage */
static const uint8_t I_5V = 80;     /* 5V line current */
static const uint8_t V_5V = 1;      /* 5V line voltage */
static const uint8_t I_3V3 = 2;     /* 3V3 line current */
static const uint8_t V_3V3 = 3;     /* 3V3 line voltage */
static const uint8_t I_IN = 4;      /* Input current to power stage */
static const uint8_t V_IN = 79;     /* Input voltage (battery/USB) */
static const uint8_t I_P1_IN = 7;   /* Solar panel 1 input current */
static const uint8_t I_P1_OUT = 6;  /* Solar panel 1 output current */
static const uint8_t V_P1 = 5;      /* Solar panel 1 voltage */
static const uint8_t I_P2_IN = 8;   /* Solar panel 2 input current */
static const uint8_t I_P2_OUT = 10; /* Solar panel 2 output current */
static const uint8_t V_P2 = 9;      /* Solar panel 2 voltage */

/* Switches */
static const uint8_t EN5V = 58;  /* Enable the 5V line */
static const uint8_t EN3V3 = 60; /* Enable the 3V3 line */

/* PWM control for maximum power point tracking */
static const uint8_t PWM1 = 23; /* PWM signal for MPPT 1 */
static const uint8_t PWM2 = 24; /* PWM signal for MPPT 2 */

/* Overcurrent detection */
static const uint8_t OC5V = 33;  /* Overcurrent on the 5V line */
static const uint8_t OC3V3 = 34; /* Overcurrent on the 3V3 line */

/* EPS interrupt line */
static const uint8_t EMG = 28;

/* I2C bus */
static const uint8_t SCL_O = 38; /* SCL, OBC I2C bus */
static const uint8_t SDA_O = 37; /* SDA, OBC I2C bus */
static const uint8_t SCL_E = 47; /* SCL, EPS I2C bus */
static const uint8_t SDA_E = 46; /* SDA, EPS I2C bus */

/* Spy-Bi-Wire debugging interface */
static const uint8_t TEST_E = 71;
static const uint8_t RST_E = 76;

/* Analog pin names (datasheet) */
static const uint8_t A0 = 77;
static const uint8_t A1 = 78;
static const uint8_t A2 = 79;
static const uint8_t A3 = 80;
static const uint8_t A4 = 1;
static const uint8_t A5 = 2;
static const uint8_t A6 = 3;
static const uint8_t A7 = 4;
static const uint8_t A8 = 9;
static const uint8_t A9 = 10;
static const uint8_t A10 = 128 + 10; /* special. This is the internal temp sensor */
static const uint8_t A11 = 128 + 11; /* special. This is Vcc/2 */
static const uint8_t A12 = 5;
static const uint8_t A13 = 6;
static const uint8_t A14 = 7;
static const uint8_t A15 = 8;

/* Pin names (datasheet) */
static const uint8_t P6_4 = 1;
static const uint8_t P6_5 = 2;
static const uint8_t P6_6 = 3;
static const uint8_t P6_7 = 4;
static const uint8_t P7_0 = 5;
static const uint8_t P7_1 = 6;
static const uint8_t P7_2 = 7;
static const uint8_t P7_3 = 8;
static const uint8_t P5_0 = 9;
static const uint8_t P5_1 = 10;
static const uint8_t P5_4 = 12;
static const uint8_t P5_5 = 13;
static const uint8_t P8_0 = 15;
static const uint8_t P8_1 = 16;
static const uint8_t P8_2 = 17;
static const uint8_t P1_0 = 21;
static const uint8_t P1_1 = 22;
static const uint8_t P1_2 = 23;
static const uint8_t P1_3 = 24;
static const uint8_t P1_4 = 25;
static const uint8_t P1_5 = 26;
static const uint8_t P1_6 = 27;
static const uint8_t P1_7 = 28;
static const uint8_t P2_0 = 29;
static const uint8_t P2_1 = 30;

static const uint8_t P2_2 = 31;
static const uint8_t P2_3 = 32;
static const uint8_t P2_4 = 33;
static const uint8_t P2_5 = 34;
static const uint8_t P2_6 = 35;
static const uint8_t P2_7 = 36;
static const uint8_t P3_0 = 37;
static const uint8_t P3_1 = 38;
static const uint8_t P3_2 = 39;
static const uint8_t P3_3 = 40;

static const uint8_t P3_4 = 41;
static const uint8_t P3_5 = 42;
static const uint8_t P3_6 = 43;
static const uint8_t P3_7 = 44;
static const uint8_t P4_0 = 45;
static const uint8_t P4_1 = 46;
static const uint8_t P4_2 = 47;
static const uint8_t P4_3 = 48;
static const uint8_t P4_4 = 51;
static const uint8_t P4_5 = 52;
static const uint8_t P4_6 = 53;
static const uint8_t P4_7 = 54;
static const uint8_t P5_6 = 55;
static const uint8_t P5_7 = 56;
static const uint8_t P7_4 = 57;
static const uint8_t P7_5 = 58;
static const uint8_t P7_6 = 59;
static const uint8_t P7_7 = 60;
static const uint8_t P5_2 = 69;
static const uint8_t P5_3 = 70;
static const uint8_t P6_0 = 77;
static const uint8_t P6_1 = 78;
static const uint8_t P6_2 = 79;
static const uint8_t P6_3 = 80;

/* Internal thermometer */
static const uint8_t TEMPSENSOR = 128 + 10;

#ifdef ARDUINO_MAIN

const uint16_t port_to_input[] = {
  NOT_A_PORT,
  (uint16_t) &P1IN,
  (uint16_t) &P2IN,
#ifdef __MSP430_HAS_PORT3_R__
  (uint16_t) &P3IN,
#endif
#ifdef __MSP430_HAS_PORT4_R__
  (uint16_t) &P4IN,
#endif
#ifdef __MSP430_HAS_PORT5_R__
  (uint16_t) &P5IN,
#endif
#ifdef __MSP430_HAS_PORT6_R__
  (uint16_t) &P6IN,
#endif
#ifdef __MSP430_HAS_PORT7_R__
  (uint16_t) &P7IN,
#endif
#ifdef __MSP430_HAS_PORT8_R__
  (uint16_t) &P8IN,
#endif
};

const uint16_t port_to_output[] = {
  NOT_A_PORT,
  (uint16_t) &P1OUT,
  (uint16_t) &P2OUT,
#ifdef __MSP430_HAS_PORT3_R__
  (uint16_t) &P3OUT,
#endif
#ifdef __MSP430_HAS_PORT4_R__
  (uint16_t) &P4OUT,
#endif
#ifdef __MSP430_HAS_PORT5_R__
  (uint16_t) &P5OUT,
#endif
#ifdef __MSP430_HAS_PORT6_R__
  (uint16_t) &P6OUT,
#endif
#ifdef __MSP430_HAS_PORT7_R__
  (uint16_t) &P7OUT,
#endif
#ifdef __MSP430_HAS_PORT8_R__
  (uint16_t) &P8OUT,
#endif
};

const uint16_t port_to_dir[] = {
  NOT_A_PORT,
  (uint16_t) &P1DIR,
  (uint16_t) &P2DIR,
#ifdef __MSP430_HAS_PORT3_R__
  (uint16_t) &P3DIR,
#endif
#ifdef __MSP430_HAS_PORT4_R__
  (uint16_t) &P4DIR,
#endif
#ifdef __MSP430_HAS_PORT5_R__
  (uint16_t) &P5DIR,
#endif
#ifdef __MSP430_HAS_PORT6_R__
  (uint16_t) &P6DIR,
#endif
#ifdef __MSP430_HAS_PORT7_R__
  (uint16_t) &P7DIR,
#endif
#ifdef __MSP430_HAS_PORT8_R__
  (uint16_t) &P8DIR,
#endif
};

const uint16_t port_to_ren[] = {
  NOT_A_PORT,
  (uint16_t) &P1REN,
  (uint16_t) &P2REN,
#ifdef __MSP430_HAS_PORT3_R__
  (uint16_t) &P3REN,
#endif
#ifdef __MSP430_HAS_PORT4_R__
  (uint16_t) &P4REN,
#endif
#ifdef __MSP430_HAS_PORT5_R__
  (uint16_t) &P5REN,
#endif
#ifdef __MSP430_HAS_PORT6_R__
  (uint16_t) &P6REN,
#endif
#ifdef __MSP430_HAS_PORT7_R__
  (uint16_t) &P7REN,
#endif
#ifdef __MSP430_HAS_PORT8_R__
  (uint16_t) &P8REN,
#endif
};

const uint16_t port_to_sel0[] = { /* put this PxSEL register under the group of PxSEL0 */
  NOT_A_PORT,
  (uint16_t) &P1SEL,
  (uint16_t) &P2SEL,
#ifdef __MSP430_HAS_PORT3_R__
  (uint16_t) &P3SEL,
#endif
#ifdef __MSP430_HAS_PORT4_R__
  (uint16_t) &P4SEL,
#endif
#ifdef __MSP430_HAS_PORT5_R__
  (uint16_t) &P5SEL,
#endif
#ifdef __MSP430_HAS_PORT6_R__
  (uint16_t) &P6SEL,
#endif
#ifdef __MSP430_HAS_PORT7_R__
  (uint16_t) &P7SEL,
#endif
#ifdef __MSP430_HAS_PORT8_R__
  (uint16_t) &P8SEL,
#endif
};

const uint16_t port_to_pmap[] = {
  NOT_A_PORT, /* PMAP starts at port P1 */
  NOT_A_PORT,
  NOT_A_PORT,
  NOT_A_PORT,
  (uint16_t) &P4MAP0,
  NOT_A_PORT,
  NOT_A_PORT,
  NOT_A_PORT,
  NOT_A_PORT,
};

const uint8_t digital_pin_to_timer[] = {
  NOT_ON_TIMER, /*  dummy */
  NOT_ON_TIMER, /*  1 -  */
  NOT_ON_TIMER, /*  2 - P5.6 */
  NOT_ON_TIMER, /*  3 - P3.4 */
  NOT_ON_TIMER, /*  4 - P3.3 */
  NOT_ON_TIMER, /*  5 - P1.6 */
  NOT_ON_TIMER, /*  6 - P6.6 */
  NOT_ON_TIMER, /*  7 - P3.2 */
  NOT_ON_TIMER, /*  8 - P2.7 */
  NOT_ON_TIMER, /*  9 - P4.2 */
  NOT_ON_TIMER, /* 10 - P4.1 */
  NOT_ON_TIMER, /* 11 - P8.1 */
  NOT_ON_TIMER, /* 12 - P2.3 */
  NOT_ON_TIMER, /* 13 - P2.6 */
  NOT_ON_TIMER, /* 14 - P3.2 */
  NOT_ON_TIMER, /* 15 - P3.0 */
  NOT_ON_TIMER, /* 16 - RST */
  NOT_ON_TIMER, /* 17 - P7.4 */
  NOT_ON_TIMER, /* 18 - P2.2 */
  NOT_ON_TIMER, /* 19 - P2.0 */
  NOT_ON_TIMER, /* 20 - GND  */
  NOT_ON_TIMER, /* 21 - 5.0v */
  T0A0,         /* 22 - GND */
  T0A1,         /* 23 - P6.0 */
  T0A2,         /* 24 - P6.1 */
  T0A3,         /* 25 - P6.2 */
  T0A4,         /* 26 - P6.3 */
  NOT_ON_TIMER, /* 27 - P6.4 */
  T1A0,         /* 28 - P7.0 */
  T1A1,         /* 29 - P3.6 */
  T1A2,         /* 30 - P3.5 */
  NOT_ON_TIMER, /* 31 - P8.2 */
  T2A0,         /* 32 - P3.7 */
  T2A1,         /* 33 - P4.0 */
  T2A2,         /* 34 - P4.3 */
  NOT_ON_TIMER, /* 35 - P1.2 */
  NOT_ON_TIMER, /* 36 - P1.3 */
  NOT_ON_TIMER, /* 37 - P1.4 */
  NOT_ON_TIMER, /* 38 - P1.5 */
  NOT_ON_TIMER, /* 39 - P2.4 */
  NOT_ON_TIMER, /* 40 - P2.5 */
  NOT_ON_TIMER, /* 41 - P2.1 */
  T0B5,         /* 42 - P1.1 */
  T0B6,         /* 43 - P1.0 */
  NOT_ON_TIMER, /* 44 - P4.7 */
  NOT_ON_TIMER, /* 45 */
  NOT_ON_TIMER, /* 46 */
  NOT_ON_TIMER, /* 47 */
  NOT_ON_TIMER, /* 48 */
  NOT_ON_TIMER, /* 49 */
  NOT_ON_TIMER, /* 50 */
  NOT_ON_TIMER, /* 51 */
  NOT_ON_TIMER, /* 52 */
  NOT_ON_TIMER, /* 53 */
  NOT_ON_TIMER, /* 54 */
  T0B0,         /* 55 */
  T0B1,         /* 56 */
  T0B2,         /* 57 */
  T0B3,         /* 58 */
  T0B4,         /* 59 */
  NOT_ON_TIMER, /* 60 */
  NOT_ON_TIMER, /* 61 */
  NOT_ON_TIMER, /* 62 */
  NOT_ON_TIMER, /* 63 */
  NOT_ON_TIMER, /* 64 */
  NOT_ON_TIMER, /* 65 */
  NOT_ON_TIMER, /* 66 */
  NOT_ON_TIMER, /* 67 */
  NOT_ON_TIMER, /* 68 */
  NOT_ON_TIMER, /* 69 */
  NOT_ON_TIMER, /* 70 */
  NOT_ON_TIMER, /* 71 */
  NOT_ON_TIMER, /* 72 */
  NOT_ON_TIMER, /* 73 */
  NOT_ON_TIMER, /* 74 */
  NOT_ON_TIMER, /* 75 */
  NOT_ON_TIMER, /* 76 */
  NOT_ON_TIMER, /* 76 */
  NOT_ON_TIMER, /* 77 */
  NOT_ON_TIMER, /* 78 */
  NOT_ON_TIMER, /* 79 */
  NOT_ON_TIMER  /* 80 */
};

const uint8_t digital_pin_to_port[] = {
  NOT_A_PIN, /* dummy */
  P6,        /* 1 */
  P6,        /* 2 */
  P6,        /* 3 */
  P6,        /* 4 */
  P7,        /* 5 */
  P7,        /* 6 */
  P7,        /* 7 */
  P7,        /* 8 */
  P5,        /* 9 */
  P5,        /* 10 */
  NOT_A_PIN, /* 11 */
  P5,        /* 12 */
  P5,        /* 13 */
  NOT_A_PIN, /* 14 */
  P8,        /* 15 */
  P8,        /* 16 */
  P8,        /* 17 */
  NOT_A_PIN, /* 18 */
  NOT_A_PIN, /* 19 */
  NOT_A_PIN, /* 20 */
  P1,        /* 21 */
  P1,        /* 22 */
  P1,        /* 23 */
  P1,        /* 24 */
  P1,        /* 25 */
  P1,        /* 26 */
  P1,        /* 27 */
  P1,        /* 28 */
  P2,        /* 29 */
  P2,        /* 30 */
  P2,        /* 31 */
  P2,        /* 32 */
  P2,        /* 33 */
  P2,        /* 34 */
  P2,        /* 35 */
  P2,        /* 36 */
  P3,        /* 37 */
  P3,        /* 38 */
  P3,        /* 39 */
  P3,        /* 40 */
  P3,        /* 41 */
  P3,        /* 42 */
  P3,        /* 43 */
  P3,        /* 44 */
  P4,        /* 45 */
  P4,        /* 46 */
  P4,        /* 47 */
  P4,        /* 48 */
  NOT_A_PIN, /* 49 */
  NOT_A_PIN, /* 50 */
  P4,        /* 51 */
  P4,        /* 52 */
  P4,        /* 53 */
  P4,        /* 54 */
  P5,        /* 55 */
  P5,        /* 56 */
  P7,        /* 57 */
  P7,        /* 58 */
  P7,        /* 59 */
  P7,        /* 60 */
  NOT_A_PIN, /* 61 */
  NOT_A_PIN, /* 62 */
  NOT_A_PIN, /* 63 */
  NOT_A_PIN, /* 64 */
  NOT_A_PIN, /* 65 */
  NOT_A_PIN, /* 66 */
  NOT_A_PIN, /* 67 */
  NOT_A_PIN, /* 68 */
  P5,        /* 69 */
  P5,        /* 70 */
  NOT_A_PIN, /* 71 */
  NOT_A_PIN, /* 72 */
  NOT_A_PIN, /* 73 */
  NOT_A_PIN, /* 74 */
  NOT_A_PIN, /* 75 */
  NOT_A_PIN, /* 76 */
  P6,        /* 77 */
  P6,        /* 78 */
  P6,        /* 79 */
  P6         /* 80 */
};

const uint8_t digital_pin_to_bit_mask[] = {
  NOT_A_PIN, /* 0, pin count starts at 1 */
  BV(4),     /* 1 */
  BV(5),     /* 2 */
  BV(6),     /* 3 */
  BV(7),     /* 4 */
  BV(0),     /* 5 */
  BV(1),     /* 6 */
  BV(2),     /* 7 */
  BV(3),     /* 8 */
  BV(0),     /* 9 */
  BV(1),     /* 10 */
  NOT_A_PIN, /* 11 */
  BV(4),     /* 12 */
  BV(5),     /* 13 */
  NOT_A_PIN, /* 14 */
  BV(0),     /* 15 */
  BV(1),     /* 16 */
  BV(2),     /* 17 */
  NOT_A_PIN, /* 18 */
  NOT_A_PIN, /* 19 */
  NOT_A_PIN, /* 20 */
  BV(0),     /* 21 */
  BV(1),     /* 22 */
  BV(2),     /* 23 */
  BV(3),     /* 24 */
  BV(4),     /* 25 */
  BV(5),     /* 26 */
  BV(6),     /* 27 */
  BV(7),     /* 28 */
  BV(0),     /* 29 */
  BV(1),     /* 30 */
  BV(2),     /* 31 */
  BV(3),     /* 32 */
  BV(4),     /* 33 */
  BV(5),     /* 34 */
  BV(6),     /* 35 */
  BV(7),     /* 36 */
  BV(0),     /* 37 */
  BV(1),     /* 38 */
  BV(2),     /* 39 */
  BV(3),     /* 40 */
  BV(4),     /* 41 */
  BV(5),     /* 42 */
  BV(6),     /* 43 */
  BV(7),     /* 44 */
  BV(0),     /* 45 */
  BV(1),     /* 46 */
  BV(2),     /* 47 */
  BV(3),     /* 48 */
  NOT_A_PIN, /* 49 */
  NOT_A_PIN, /* 50 */
  BV(4),     /* 51 */
  BV(5),     /* 52 */
  BV(6),     /* 53 */
  BV(7),     /* 54 */
  BV(6),     /* 55 */
  BV(7),     /* 56 */
  BV(4),     /* 57 */
  BV(5),     /* 58 */
  BV(6),     /* 59 */
  BV(7),     /* 60 */
  NOT_A_PIN, /* 61 */
  NOT_A_PIN, /* 62 */
  NOT_A_PIN, /* 63 */
  NOT_A_PIN, /* 64 */
  NOT_A_PIN, /* 65 */
  NOT_A_PIN, /* 66 */
  NOT_A_PIN, /* 67 */
  NOT_A_PIN, /* 68 */
  BV(2),     /* 69 */
  BV(3),     /* 70 */
  NOT_A_PIN, /* 71 */
  NOT_A_PIN, /* 72 */
  NOT_A_PIN, /* 73 */
  NOT_A_PIN, /* 74 */
  NOT_A_PIN, /* 75 */
  NOT_A_PIN, /* 76 */
  BV(0),     /* 77 */
  BV(1),     /* 78 */
  BV(2),     /* 79 */
  BV(3)      /* 80 */
};

const uint32_t digital_pin_to_analog_in[] = {
  NOT_ON_ADC, /* dummy */
  4,          /* 1 - */
  5,          /* 2 - A5 */
  6,          /* 3 - P3.4 */
  7,          /* 4 - P3.3 */
  12,         /* 5 - P1.6 */
  13,         /* 6 - A6 */
  14,         /* 7 - P3.2 */
  15,         /* 8 - P2.7 */
  8,          /* 9 - P4.2 */
  9,          /* 10 - P4.1 */
  NOT_ON_ADC, /* 11 - P8.1 */
  NOT_ON_ADC, /* 12 - P2.3 */
  NOT_ON_ADC, /* 13 - P2.6 */
  NOT_ON_ADC, /* 14 - P3.1 */
  NOT_ON_ADC, /* 15 - P3.0 */
  NOT_ON_ADC, /* 16 - RST */
  NOT_ON_ADC, /* 17 - P7.4 */
  NOT_ON_ADC, /* 18 - P2.2 */
  NOT_ON_ADC, /* 19 - P2.0 */
  NOT_ON_ADC, /* 20 - GND */
  NOT_ON_ADC, /* 21 - 5V */
  NOT_ON_ADC, /* 22 - GND */
  NOT_ON_ADC, /* 23 - A0 */
  NOT_ON_ADC, /* 24 - A1 */
  NOT_ON_ADC, /* 25 - A2 */
  NOT_ON_ADC, /* 26 - A3 */
  NOT_ON_ADC, /* 27 - A4 */
  NOT_ON_ADC, /* 28 - A12 */
  NOT_ON_ADC, /* 29 - P3.6 */
  NOT_ON_ADC, /* 30 - P3.5 */
  NOT_ON_ADC, /* 31 - P8.2 */
  NOT_ON_ADC, /* 32 - P3.7 */
  NOT_ON_ADC, /* 33 - P4.0 */
  NOT_ON_ADC, /* 34 - P4.3 */
  NOT_ON_ADC, /* 35 - P1.2 */
  NOT_ON_ADC, /* 36 - P1.3 */
  NOT_ON_ADC, /* 37 - P1.4 */
  NOT_ON_ADC, /* 38 - P1.5 */
  NOT_ON_ADC, /* 39 - P2.4 */
  NOT_ON_ADC, /* 40 - P2.5 */
  NOT_ON_ADC, /* 41 - GND */
  NOT_ON_ADC, /* 42 - 5V */
  NOT_ON_ADC, /* 43 - GND */
  NOT_ON_ADC, /* 44 - A0 */
  NOT_ON_ADC, /* 45 - A1 */
  NOT_ON_ADC, /* 46 - A2 */
  NOT_ON_ADC, /* 47 - A3 */
  NOT_ON_ADC, /* 48 - A4 */
  NOT_ON_ADC, /* 49 - A12 */
  NOT_ON_ADC, /* 50 - P3.6 */
  NOT_ON_ADC, /* 51 - P3.5 */
  NOT_ON_ADC, /* 52 - P8.2 */
  NOT_ON_ADC, /* 53 - P3.7 */
  NOT_ON_ADC, /* 54 - P4.0 */
  NOT_ON_ADC, /* 55 - P4.3 */
  NOT_ON_ADC, /* 56 - P1.2 */
  NOT_ON_ADC, /* 57 - P1.3 */
  NOT_ON_ADC, /* 58 - P1.4 */
  NOT_ON_ADC, /* 59 - P1.5 */
  NOT_ON_ADC, /* 60 - P2.4 */
  NOT_ON_ADC, /* 61 - P2.5 */
  NOT_ON_ADC, /* 62 - GND */
  NOT_ON_ADC, /* 63 - 5V */
  NOT_ON_ADC, /* 64 - GND */
  NOT_ON_ADC, /* 65 - A0 */
  NOT_ON_ADC, /* 66 - A1 */
  NOT_ON_ADC, /* 67 - A2 */
  NOT_ON_ADC, /* 68 - A3 */
  NOT_ON_ADC, /* 69 - A4 */
  NOT_ON_ADC, /* 70 - A12 */
  NOT_ON_ADC, /* 71 - P3.6 */
  NOT_ON_ADC, /* 72 - P3.5 */
  NOT_ON_ADC, /* 73 - P8.2 */
  NOT_ON_ADC, /* 74 - P3.7 */
  NOT_ON_ADC, /* 75 - P4.0 */
  NOT_ON_ADC, /* 76 - P4.3 */
  0,          /* 77 - P1.2 */
  1,          /* 78 - P1.3 */
  2,          /* 79 - P1.4 */
  3           /* 80 - P1.5 */
};
#endif // #ifdef ARDUINO_MAIN
#endif // #ifndef Pins_Energia_h
