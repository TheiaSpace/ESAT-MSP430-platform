/*
 * Copyright (c) 2018 Theia Space, Universidad Politécnica de Madrid
 * <info@theiaspace.com>.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

unsigned char const abromStringDescriptor[] =
{
  // String index0, language support
  4, // Length of language descriptor ID
  3, // LANGID tag
  0x09, 0x04, // 0x0409 for English

  // String index1, Manufacturer
  2 * sizeof("Theia Space"), // Length of this string descriptor
  3, // bDescriptorType
  'T',0, 'h',0, 'e',0, 'i',0, 'a',0,
  ' ',0,
  'S',0, 'p',0, 'a',0, 'c',0, 'e',0,

  // String index2, Product
  2 * sizeof("ESAT Electrical Power Subsystem"), // Length of this string descriptor
  3, // bDescriptorType
  'E',0, 'S',0, 'A',0, 'T',0,
  ' ',0,
  'E',0, 'l',0, 'e',0, 'c',0, 't',0, 'r',0, 'i',0, 'c',0, 'a',0, 'l',0,
  ' ',0,
  'P',0, 'o',0, 'w',0, 'e',0, 'r',0,
  ' ',0,
  'S',0, 'u',0, 'b',0, 's',0, 'y',0, 's',0, 't',0, 'e',0, 'm',0,

  // String index3, Serial Number
  2 * sizeof("0"), // Length of this string descriptor
  3, // bDescriptorType
  '0',0,

  // String index4, Configuration String
  2 * sizeof("ESAT Electrical Power Subsystem"), // Length of this string descriptor
  3, // bDescriptorType
  'E',0, 'S',0, 'A',0, 'T',0,
  ' ',0,
  'E',0, 'l',0, 'e',0, 'c',0, 't',0, 'r',0, 'i',0, 'c',0, 'a',0, 'l',0,
  ' ',0,
  'P',0, 'o',0, 'w',0, 'e',0, 'r',0,
  ' ',0,
  'S',0, 'u',0, 'b',0, 's',0, 'y',0, 's',0, 't',0, 'e',0, 'm',0,

  // String index5, Interface String
  2 * sizeof("Virtual COM Port (CDC)"), // Length of this string descriptor
  3, // bDescriptorType
  'V',0, 'i',0, 'r',0, 't',0, 'u',0, 'a',0, 'l',0,
  ' ',0,
  'C',0, 'O',0, 'M',0,
  ' ',0,
  'P',0, 'o',0, 'r',0, 't',0,
  ' ',0,
  '(',0, 'C',0, 'D',0, 'C',0, ')',0
};
