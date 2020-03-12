/*
 * This file is part of Magis.
 *
 * Cleanflight and Magis are free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight and Magis are distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif




/* Describes the pin modes a pin of the SC18IS602B can be in */
enum SC18IS601B_GPIOPinMode {
    SC18IS601B_GPIO_MODE_QUASI_BIDIRECTIONAL    = 0B00,
    SC18IS601B_GPIO_MODE_PUSH_PULL              = 0B01,
    SC18IS601B_GPIO_MODE_INPUT_ONLY             = 0B10,
    SC18IS601B_GPIO_MODE_OPEN_DRAIN             = 0B11
};

/* Describes the possible SPI speeds */
enum SC18IS601B_SPI_Speed {
    SC18IS601B_SPICLK_1843_kHz      = 0B00,     /* 1.8 MBit/s */
    SC18IS601B_SPICLK_461_kHz       = 0B01,     /* 461 kbit/s */
    SC18IS601B_SPICLK_115_kHz       = 0B10,     /* 115 kbit/s */
    SC18IS601B_SPICLK_58_kHz        = 0B11      /* 58 kbit/s */
};

/* Describes the possible SPI modes */
enum SC18IS601B_SPI_Mode {
    SC18IS601B_SPIMODE_0 = 0B00,        /* CPOL: 0  CPHA: 0 */
    SC18IS601B_SPIMODE_1 = 0B01,        /* CPOL: 0  CPHA: 1 */
    SC18IS601B_SPIMODE_2 = 0B10,        /* CPOL: 1  CPHA: 0 */
    SC18IS601B_SPIMODE_3 = 0B11         /* CPOL: 1  CPHA: 1 */
};

/* Function IDs */
#define SC18IS601B_CONFIG_SPI_CMD           0xF0
#define SC18IS601B_CLEAR_INTERRUPT_CMD      0xF1
#define SC18IS601B_IDLE_CMD                 0xF2
#define SC18IS601B_GPIO_WRITE_CMD           0xF4
#define SC18IS601B_GPIO_READ_CMD            0xF5
#define SC18IS601B_GPIO_ENABLE_CMD          0xF6
#define SC18IS601B_GPIO_CONFIGURATION_CMD   0xF7

#define SC18IS601B_DATABUFFER_DEPTH         200

class SC18IS602B {
public:
    /* By default instantiate it with no attached RESET pin and a0 to a2 to GND */


    /* if resetPin = -1 it will be ignored */
    void init(int slaveNum, bool a0, bool a1, bool a2);

    /* calls into Wire.begin() */
    void begin();

    /* for the ESP8266: Explicit SDA and SCL pins */
#ifdef ARDUINO_ARCH_ESP8266
    void begin(int sda, int scl);
#endif

    /* reset by pulsing RESET */
    void reset();

    /* Enables or disables GPIO functionality for a slave select pin */
    bool enableGPIO(int num, bool enable);

    /* Sets up a specific GPIO in some mode */
    bool setupGPIO(int num, SC18IS601B_GPIOPinMode mode);

    /* Writes a value to an output pin */
    bool writeGPIO(int num, bool value);

    /* Writes a 4-bit value to the entire GPIO bank (GPIO3 to GPIO0) */
    bool writeGPIOBank(uint8_t value);

    /* Same function as above with just all values split up */
    bool writeGPIOBank(bool gpio0, bool gpio1, bool gpio2, bool gpio3);

    /* Reads GPIO value */
    bool readGPIO(int num);

    /* Sets the chip into low power mode */
    bool setLowPowerMode();

    /* Clears the INT pin asserted HIGH after every SPI transfer */
    bool clearInterrupt();

    /* SPI functions. spiMode should be SPI_MODE0 to SPI_MODE3 */
    bool configureSPI(bool lsbFirst, SC18IS601B_SPI_Mode spiMode, SC18IS601B_SPI_Speed clockSpeed);

    /* Executes a SPI transfer.
     * Slave select number `slaveNum` will be used.
     * The txLen bytes from the txData buffer will be sent,
     * then txLen bytes will be read into readBuf.
     * Returns success (true/false).
     * */
    bool spiTransfer(int slaveNum,  uint8_t* txData, uint8_t txLen, uint8_t* readBuf);

    /* Transfers a single byte to a slave. returns the read value. */
    uint8_t spiTransfer(int slaveNum, uint8_t txByte);


    bool write(uint8_t register_address,uint8_t data);

    uint8_t read(uint8_t register_address);


private:

    bool i2c_write(uint8_t cmdByte, uint8_t* data, uint8_t len);
    uint8_t i2c_read(uint8_t* readBuf,uint8_t len);

    int resetPin = -1;      /* reset pin */
    uint8_t address = 0;        /* address of the module */
    uint8_t gpioEnable = 0; /* last value for GPIO enable */
    uint8_t gpioConfig = 0; /* last value for GPIO configuration */
    uint8_t gpioWrite = 0;  /* last value for GPIO write */

    int slaveNumber=0;
    uint8_t slaveSelect = 0;
};




#ifdef __cplusplus
}
#endif
