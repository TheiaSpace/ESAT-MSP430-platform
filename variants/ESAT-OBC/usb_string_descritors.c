/*
 * Copyright (C) 2018 Theia Space, Universidad Politécnica de Madrid
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

unsigned char abromStringDescriptor[] =
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
  2 * sizeof("ESAT On-Board Computer"), // Length of this string descriptor
  3, // bDescriptorType
  'E',0, 'S',0, 'A',0, 'T',0,
  ' ',0,
  'O',0, 'n',0, '-',0, 'B',0, 'o',0, 'a',0, 'r',0, 'd',0,
  ' ',0,
  'C',0, 'o',0, 'm',0, 'p',0, 'u',0, 't',0, 'e',0, 'r',0,

  // String index3, Serial Number
  2 * sizeof("0"), // Length of this string descriptor
  3, // bDescriptorType
  '0',0,

  // String index4, Configuration String
  2 * sizeof("ESAT On-Board Computer"), // Length of this string descriptor
  3, // bDescriptorType
  'E',0, 'S',0, 'A',0, 'T',0,
  ' ',0,
  'O',0, 'n',0, '-',0, 'B',0, 'o',0, 'a',0, 'r',0, 'd',0,
  ' ',0,
  'C',0, 'o',0, 'm',0, 'p',0, 'u',0, 't',0, 'e',0, 'r',0,

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
