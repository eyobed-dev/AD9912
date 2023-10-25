
#ifndef AD9912_h
#define AD9912_h

#include <Arduino.h>
#include <inttypes.h>
#include <SPI.h>

#define ad9912_PartID 0x1902
#define ad9912_read 0x1
#define ad9912_write 0x0

//DDS Register Addresses
//LSB
#define FTW0_1 0x01A6
#define FTW0_2 0x01A7
#define FTW0_3 0x01A8
#define FTW0_4 0x01A9
//MSB
#define FTW0_start_1 0x01AA
#define FTW0_start_2 0x01AB
#define DAC_fsc_1 0x040B
#define DAC_fsc_2 0x040C

//DDS Default values and maxes
#define DAC_fcs_default 0x01FF
#define DAC_fcs_max 0x03FF

class AD9912 {
  public:
    void init(int SPICS, int SPISCK, int SPIMOSI, int SPIMISO, int IO_update, uint32_t clkFreq, uint64_t RDAC_REF);
    uint16_t read_PartID();
    uint64_t instruction(short command, uint16_t address, char bytes, uint64_t data);
    uint16_t DAC_read();
    void DAC_write(uint16_t DAC_val);
    uint64_t FTW_read();
    void FTW_write(uint64_t FTW);
    void setFrequency(uint32_t frequency);
    uint32_t getFrequency();
    uint32_t fDDS();
    void updateClkFreq(uint64_t clkFreq);
    float IDAC_REF();
    void setCurrent(float current);
    float getCurrent();
    void setPLL(uint8_t factor);
    uint16_t getPLL();
    float IDAC_FS();
  private:
    // global type variables
    uint32_t _SPISCK;
    uint32_t _SPIMISO;
    uint32_t _SPIMOSI;
    uint32_t _SPICS;
    uint32_t _IO_update;
    uint32_t _fs;
    uint64_t _RDAC_REF;
};
#endif
