/*
 * TWI/I2C library for Arduino Zero
 * Copyright (c) 2015 Arduino LLC. All rights reserved.
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

#ifndef TwoWire_h
#define TwoWire_h

#include "Callback.h"
#include "Stream.h"
#include "variant.h"

#define WIRE_BUFFER_LENGTH 32

 // WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1

enum WireStatus {
    WIRE_STATUS_SUCCESS = 0,
    WIRE_STATUS_FAILURE,
    WIRE_STATUS_ADDRESS_NACK,
    WIRE_STATUS_DATA_NACK,
    WIRE_STATUS_ARIBTRATION_LOST,
    WIRE_STATUS_TIMEOUT,
    WIRE_STATUS_BUSY,
};

class TwoWire : public Stream
{
public:
    TwoWire(struct _stm32wb_i2c_t *i2c, const struct _stm32wb_i2c_params_t *params, struct _stm32wb_i2c_transaction_t *transaction);
    TwoWire(const TwoWire&) = delete;
    TwoWire& operator=(const TwoWire&) = delete;

    void begin();
    void begin(uint8_t address, bool enableGeneralCall = false);
    void end();
    void setClock(uint32_t clock);

    void beginTransmission(uint8_t address);
    uint8_t endTransmission(bool stopBit = true);

    size_t requestFrom(uint8_t address, size_t size, bool stopBit = true);
    size_t requestFrom(uint8_t address, size_t size, uint32_t iaddress, uint8_t isize, bool stopBit = true);

    int available(void) override;
    int peek(void) override;
    int read(void) override;
    int read(uint8_t *buffer, size_t size) override;

    int availableForWrite(void) override;
    size_t write(uint8_t data) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    void flush(void) override;

    void onReceive(void(*)(int));
    void onRequest(void(*)(void));
    void onTransmit(void(*)(int));

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write;

    // STM32WB EXTENSTION: clock low timeout in milliseconds
    void setClockLowTimeout(unsigned long timeout);

    // STM32WB EXTENSTION: general call status
    bool isGeneralCall();

    // STM32WB EXTENSTION: synchronous/asynchronous composite transmit/receive 
    uint8_t transfer(uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *rxBuffer, size_t rxSize, bool stopBit = true);
    bool transfer(uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *rxBuffer, size_t rxSize, bool stopBit, volatile uint8_t &status);
    bool transfer(uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *rxBuffer, size_t rxSize, bool stopBit, volatile uint8_t &status, void(*callback)(void));
    bool transfer(uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *rxBuffer, size_t rxSize, bool stopBit, volatile uint8_t &status, Callback callback);

    // STM32WB EXTENSTION: reset I2C bus
    void reset();

  private:
    struct _stm32wb_i2c_t *m_i2c;
    struct _stm32wb_i2c_transaction_t *m_transaction;
    uint32_t m_option;
    uint32_t m_timeout;
    uint8_t m_ev_address;
    uint8_t m_xf_address;

    uint8_t m_rx_data[WIRE_BUFFER_LENGTH];
    uint8_t m_rx_read;
    uint8_t m_rx_count;
    uint8_t m_rx_address;

    uint8_t m_tx_data[WIRE_BUFFER_LENGTH];
    uint8_t m_tx_write;
    uint8_t m_tx_address;
    bool m_tx_active;

    void (*m_receive_callback)(int);
    void (*m_request_callback)(void);
    void (*m_transmit_callback)(int);

    volatile uint8_t *m_xf_status;
    Callback m_done_callback;
    k_work_t m_work;
    
    static void workRoutine(class TwoWire *self);
    static void doneCallback(class TwoWire *self);
    static void eventCallback(class TwoWire *self, uint32_t events);

    static const uint32_t TWI_CLOCK = 100000;
};

#if WIRE_INTERFACES_COUNT > 0
  extern TwoWire Wire;
#endif
#if WIRE_INTERFACES_COUNT > 1
  extern TwoWire Wire1;
#endif

#endif
