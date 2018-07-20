
#include <Arduino.h>
#include <IPAddress.h>


const IPAddress INADDR_NONE(0,0,0,0);

IPAddress::IPAddress()
{
    memset(_address, 0, sizeof(_address));
}

IPAddress::IPAddress(uint8_t first_octet, uint8_t second_octet, uint8_t third_octet, uint8_t fourth_octet)
{
    _address[0] = first_octet;
    _address[1] = second_octet;
    _address[2] = third_octet;
    _address[3] = fourth_octet;
}

IPAddress::IPAddress(uint32_t address)
{
    memcpy(_address, &address, sizeof(_address));
}

IPAddress::IPAddress(const uint8_t *address)
{
    memcpy(_address, address, sizeof(_address));
}

IPAddress::operator uint32_t()
{
    return ((uint32_t)_address[0])
      | (((uint32_t)_address[1])<<8)
      | (((uint32_t)_address[2])<<16)
      | (((uint32_t)_address[3])<<24);
}

IPAddress& IPAddress::operator=(const uint8_t *address)
{
    memcpy(_address, address, sizeof(_address));
    return *this;
}

IPAddress& IPAddress::operator=(uint32_t address)
{
    memcpy(_address, (const uint8_t *)&address, sizeof(_address));
    return *this;
}

bool IPAddress::operator==(const IPAddress& addr)
{
   return memcmp(addr._address, _address, sizeof(_address)) == 0;
}

bool IPAddress::operator==(const uint8_t* addr)
{
    return memcmp(addr, _address, sizeof(_address)) == 0;
}

size_t IPAddress::printTo(Print& p) const
{
    size_t n = 0;
    for (int i =0; i < 3; i++)
    {
        n += p.print(_address[i], DEC);
        n += p.print('.');
    }
    n += p.print(_address[3], DEC);
    return n;
}

