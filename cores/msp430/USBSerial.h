/*

USBSerial.h (formerly NewSoftSerial.h) - 
 Multi-instance USB serial library for Arduino/Wiring
 
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
 */

#ifndef USBSerial_h
#define USBSerial_h
#include <inttypes.h>
#include <Stream.h>
///////////////////////

/******************************************************************************
 * Definitions
 ******************************************************************************/


class USBSerial : public Stream {
private:


  // static data

  // private methods

public:
  // public methods
  USBSerial(uint16_t port);
  ~USBSerial();
  void begin(uint32_t unusedBaudrate = 0, uint8_t unusedConfig = 0);
  void end();
  virtual int available(void);
  virtual int peek();
  virtual size_t write(uint8_t byte);
  using Print::write;
  virtual int read();
  virtual void flush();

  using Print::write;

  uint32_t baudrate();
  boolean readDTR();
  boolean readRTS();

  operator bool();

  // public only for easy access by interrupt handlers
  static inline void handle_interrupt();
};
extern USBSerial Serial;
#endif

