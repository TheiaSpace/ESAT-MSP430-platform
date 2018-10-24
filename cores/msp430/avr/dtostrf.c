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

#include <stdint.h>
#include <math.h>

/*
 * State of execution of the dtostrf function.
 * The helper functions for dtostrf use and modify this structure to
 * keep track of the state.
 */
struct dtostrf_state
{
  double value;
  int sign;
  int position;
  int integer_digits;
  int decimal_digits;
};

/*
 * Count the number of integer digits.
 * Helper function for dtostrf.
 */
static void count_integer_digits(struct dtostrf_state* state)
{
  state->integer_digits = 0;
  while (state->value >= 1)
  {
    state->value = state->value / 10;
    state->integer_digits = state->integer_digits + 1;
  }
}

/*
 * Prepare the dtostrf state.
 * Helper function for dtostrf.
 */
static void init_state(struct dtostrf_state* state,
                       double value,
                       unsigned char decimal_digits)
{
  if (value > 0)
  {
    state->value = value;
    state->sign = 1;
  }
  else
  {
    state->value = -value;
    state->sign = -1;
  }
  state->position = 0;
  count_integer_digits(state);
  state->decimal_digits = decimal_digits;
}

/*
 * Print a character to the output string.
 * Helper function for dtostrf.
 */
static void print_character(struct dtostrf_state* state,
                             char character,
                             char* output)
{
  output[state->position] = character;
  state->position = state->position + 1;
}

/*
 * Print the negative sign to the output string if necessary.
 * Helper function for dtostrf.
 */
static void print_sign(struct dtostrf_state* state,
                        char* output)
{
  if (state->sign < 0)
  {
    print_character(state, '-', output);
  }
}

/*
 * Print the next digit to the output string.
 * Helper function for dtostrf.
 */
static void print_digit(struct dtostrf_state* state,
                        char* output)
{
  const char digit_to_character[] = "0123456789";
  state->value = 10 * state->value;
  int digit = (int) state->value;
  state->value = state->value - digit;
  print_character(state, digit_to_character[digit], output);
}

/*
 * Print the integer part to the output string.
 * Helper function for dtostrf.
 */
static void print_integer_part(struct dtostrf_state* state,
                               char* output)
{
  if ((state->integer_digits == 0) && (state->sign > 0))
  {
    print_character(state, '0', output);
  }
  for (int i = 0; i < state->integer_digits; i++)
  {
    print_digit(state, output);
  }
}

/*
 * Print the decimal separator to the output string if necessary.
 * Helper function for dtostrf.
 */
static void print_decimal_separator(struct dtostrf_state* state,
                                    char* output)
{
  if (state->decimal_digits > 0)
  {
    print_character(state, '.', output);
  }
}

/*
 * Print the decimal part to the output string.
 * Helper function for dtostrf.
 */
static void print_decimal_part(struct dtostrf_state* state,
                               char* output)
{
  for (int i = 0; i < state->decimal_digits; i++)
  {
    print_digit(state, output);
  }
}

/*
 * Terminate the output string.
 * Helper function for dtostrf.
 */
static void terminate_output(struct dtostrf_state* state,
                             char* output)
{
  print_character(state, '\0', output);
}

/*
 * Print the decimal representation of a floating-point number
 * into an output string.
 * Use the specified number of decimal digits.
 */
char *dtostrf(double value,
              signed char ignored_argument,
              unsigned char decimal_digits,
              char *output)
{
  (void) ignored_argument;
  struct dtostrf_state state;
  init_state(&state, value, decimal_digits);
  print_sign(&state, output);
  print_integer_part(&state, output);
  print_decimal_separator(&state, output);
  print_decimal_part(&state, output);
  terminate_output(&state, output);
  return output;
}
