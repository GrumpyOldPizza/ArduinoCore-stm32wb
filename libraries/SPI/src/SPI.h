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

#define SPI_MODE0 0x00
#define SPI_MODE1 0x01
#define SPI_MODE2 0x02
#define SPI_MODE3 0x03

#define SPI_MIN_CLOCK_DIVIDER 4

#define SPI_STATUS_SUCCESS 0
#define SPI_STATUS_FAILURE 1
#define SPI_STATUS_BUSY    255

class SPISettings {
public:
  SPISettings() : m_clock(4000000), m_control(0) { }
  SPISettings(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) :
  m_clock(clock), m_control((dataMode & (SPI_CR1_CPHA | SPI_CR1_CPOL)) | ((bitOrder != MSBFIRST) ? SPI_CR1_LSBFIRST : 0)) { }
  
private:
  const uint32_t m_clock;
  const uint32_t m_control;

  friend class SPIClass;
};

class SPIClass {
public:
    SPIClass(struct _stm32wb_spi_t *spi, const struct _stm32wb_spi_params_t *params);

    inline uint8_t transfer(uint8_t data) { return m_data_8(m_spi, data); }
    inline uint16_t transfer16(uint16_t data) { return m_data_16(m_spi, data); }
    inline void transfer(void *buffer, size_t count) { m_data_transfer(m_spi, (const uint8_t*)buffer, (uint8_t*)buffer, count); }

    // Transaction Functions
    void usingInterrupt(uint32_t pin);
    void notUsingInterrupt(uint32_t pin);
    void beginTransaction(SPISettings settings);
    void endTransaction(void);

    // SPI Configuration methods
    void attachInterrupt();
    void detachInterrupt();

    void begin();
    void end();

    void setBitOrder(BitOrder bitOrder);
    void setDataMode(uint8_t dataMode);
    void setClockDivider(uint8_t divider);

    // STM32WB EXTENSION: transfer with separate read/write buffer
    void transfer(const void *txBuffer, void *rxBuffer, size_t count) { m_data_transfer(m_spi, (const uint8_t*)txBuffer, (uint8_t*)rxBuffer, count); }
    
    // STM32WB EXTENSTION: asynchronous transfer with separate read/write buffer
    bool transfer(const void *txBuffer, void *rxBuffer, size_t count, volatile uint8_t &status);
    bool transfer(const void *txBuffer, void *rxBuffer, size_t count, volatile uint8_t &status, void(*callback)(void));
    bool transfer(const void *txBuffer, void *rxBuffer, size_t count, volatile uint8_t &status, Callback callback);
    size_t cancel(void);
    
private:
    struct _stm32wb_spi_t *m_spi;
    bool m_active;
    uint32_t m_clock;
    uint32_t m_control;

    Callback m_done_callback;
    k_work_t m_work;

    uint8_t (*m_data_8)(struct _stm32wb_spi_t *spi, uint8_t data);
    uint16_t (*m_data_16)(struct _stm32wb_spi_t *spi, uint16_t data);
    void (*m_data_transfer)(struct _stm32wb_spi_t *spi, const uint8_t *tx_data, uint8_t *rx_data, uint32_t count);

    static uint8_t select_data_8(struct _stm32wb_spi_t *spi, uint8_t data);
    static uint16_t select_data_16(struct _stm32wb_spi_t *spi, uint16_t data);
    static void select_data_transfer(struct _stm32wb_spi_t *spi, const uint8_t *tx_data, uint8_t *rx_data, uint32_t count);

    static uint8_t null_data_8(struct _stm32wb_spi_t *spi, uint8_t data);
    static uint16_t null_data_16(struct _stm32wb_spi_t *spi, uint16_t data);
    static void null_data_transfer(struct _stm32wb_spi_t *spi, const uint8_t *tx_data, uint8_t *rx_data, uint32_t count);
    
    void workCallback();
    void doneCallback();
};

#if SPI_INTERFACES_COUNT > 0
  extern SPIClass SPI;
#endif
#if SPI_INTERFACES_COUNT > 1
  extern SPIClass SPI1;
#endif

#endif
