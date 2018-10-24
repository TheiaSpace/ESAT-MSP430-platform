/*
 * Copyright (C) 2018 Theia Space, Universidad Politécnica de Madrid
 *
 * dtostrf - Emulation for dtostrf function from avr-libc.
 *
 * This file is part of Theia Space's ESAT core for Arduino for
 * MSP430-based ESAT boards.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef dtostrf_h
#define dtostrf_h

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Print the decimal representation of a floating-point number
 * into an output string.
 * Use the specified number of decimal digits.
 * The second argument to the original dtostrf is the desired width of
 * the output, but we ignore it.
 */
char* dtostrf(double value,
              signed char ignored_argument,
              unsigned char decimal_digits,
              char *output);

#ifdef __cplusplus
}
#endif

#endif /* dtostrf_h */
