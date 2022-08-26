/*
 * SPI Master library for Arduino Zero.
 * Copyright (c) 2015 Arduino LLC
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

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <Arduino.h>

// SPI_HAS_TRANSACTION means SPI has
//   - beginTransaction()
//   - endTransaction()
//   - usingInterrupt()
//   - SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1

// SPI_HAS_NOTUSINGINTERRUPT means that SPI has notUsingInterrupt() method
#define SPI_HAS_NOTUSINGINTERRUPT 1

typedef enum {
    SPI_MODE0 = 0,
    SPI_MODE1 = 1,
    SPI_MODE2 = 2,
    SPI_MODE3 = 3,
} SPIMode;

class SPISettings {
public:
    SPISettings(uint32_t clockFreq, BitOrder bitOrder, SPIMode dataMode) :
        m_clock(clockFreq),
        m_control(((bitOrder != MSBFIRST) ? 0x80 : 0) | ((uint32_t)dataMode & 0x03)) {
    }

    SPISettings(uint32_t clockFreq, BitOrder bitOrder, int dataMode) :
        m_clock(clockFreq),
        m_control(((bitOrder != MSBFIRST) ? 0x80 : 0) | ((uint32_t)dataMode & 0x03)) {
    }

    SPISettings() :
        m_clock(4000000),
        m_control(0) {
    } 

    bool operator==(const SPISettings& other) const {
        return ((m_clock == other.m_clock) && (m_control == other.m_control));
    }

    bool operator!=(const SPISettings& other) const {
        return ((m_clock != other.m_clock) || (m_control != other.m_control));
    }
    
    uint32_t getClockFreq() const {
        return m_clock;
    }

    BitOrder getBitOrder() const {
        return (BitOrder)((m_control & 0x80) ? LSBFIRST : MSBFIRST);
    }

    SPIMode getDataMode() const {
        return (SPIMode)(m_control & 0x03);
    }

protected:
    uint32_t m_clock;
    uint32_t m_control;

    friend class SPIClass;
};


#define SPI_STATUS_SUCCESS 0
#define SPI_STATUS_FAILURE 1
#define SPI_STATUS_BUSY    255

class SPIClass {
public:
    SPIClass(struct _stm32wb_spi_t *spi, const struct _stm32wb_spi_params_t *params);
    SPIClass(struct _stm32wb_uart_t *uart, const struct _stm32wb_uart_params_t *params);

    virtual uint8_t transfer(uint8_t data);
    virtual uint16_t transfer16(uint16_t data);
    virtual void transfer(void *buf, size_t count);

    // Transaction Functions
    virtual void usingInterrupt(int interruptNumber);
    virtual void notUsingInterrupt(int interruptNumber);
    virtual void beginTransaction(SPISettings settings);
    virtual void endTransaction(void);

    // SPI Configuration methods
    virtual void attachInterrupt();
    virtual void detachInterrupt();

    virtual void begin();
    virtual void end();

    // STM32WB EXTENSION: transfer with separate read/write buffer
    void transfer(const void *txBuffer, void *rxBuffer, size_t count);
    
    // STM32WB EXTENSTION: asynchronous transfer with separate read/write buffer
    bool transfer(const void *txBuffer, void *rxBuffer, size_t count, volatile uint8_t &status);
    bool transfer(const void *txBuffer, void *rxBuffer, size_t count, volatile uint8_t &status, void(*callback)(void));
    bool transfer(const void *txBuffer, void *rxBuffer, size_t count, volatile uint8_t &status, Callback callback);
    size_t cancel(void);
    
private:
    struct _stm32wb_spi_t * const m_spi;
    const struct SPIInterface * const m_interface;
    bool m_transaction;

    Callback m_done_callback;
    k_work_t m_work;

    uint8_t (*m_data_8)(struct _stm32wb_spi_t *spi, uint8_t data);
    uint16_t (*m_data_16)(struct _stm32wb_spi_t *spi, uint16_t data);

    static uint8_t null_data_8(struct _stm32wb_spi_t *spi, uint8_t data);
    static uint16_t null_data_16(struct _stm32wb_spi_t *spi, uint16_t data);
    
    static void workCallback(class SPIClass *self);
    static void doneCallback(class SPIClass *self);
};

#if SPI_INTERFACES_COUNT > 0
  extern SPIClass SPI;
#endif
#if SPI_INTERFACES_COUNT > 1
  extern SPIClass SPI1;
#endif

#endif
