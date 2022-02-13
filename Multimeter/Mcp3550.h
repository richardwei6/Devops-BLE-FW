#include <mbed.h>

#ifndef __MCP3550_H__
#define __MCP3550_H__


// MCP3550/1/3 22-bit SPI ADC
class Mcp3550 {
public:
  Mcp3550(SPI& spi, DigitalOut& cs, int frequency = 1000000) : 
      spi_(spi), cs_(cs), frequency_(frequency) {

  }

  // reads ADC as a 22-bit value, returning whether the ADC had new data
  // if the ADC did not have new data, a new conversion will be started
  bool read_raw_u22(uint32_t* outValue) {
    spi_.format(8, 3);
    spi_.frequency(frequency_);

    cs_ = 0;
    // first two clocks are for sampling, then one null bit, then data
    // and last bit (in a 16 bit transfer) is unused
    uint8_t byte0 = spi_.write(0);

    if (byte0 == 0xff) {  // data was not ready
      cs_ = 1;
      return false;
    }

    uint8_t byte1 = spi_.write(0);
    uint8_t byte2 = spi_.write(0);
    cs_ = 1;

    *outValue = (((uint32_t)(byte0 & 0x3f) << 16) |
                 ((uint32_t)byte1 << 8) |
                 byte2);
    return true;
  }

  // reads ADC scaled up to a 32-bit value
  // uint32_t read_u32();

  // reads ADC scaled down to a 16-bit value
  // uint16_t read_u16();

  // sets the SPI frequency
  void spi_frequency(int hz);

protected:
  SPI& spi_;
  DigitalOut& cs_;
  int frequency_;
};

#endif  // __MCP3550_H__
