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

#include "stm32wb_i2c.h"

#define WIRE_BUFFER_LENGTH 128

 // WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1

// WIRE_HAS_CLOCK_LOW_TIMEOUT means Wire support a SCL timeout
#define WIRE_HAS_CLOCK_LOW_TIMEOUT 1

// WIRE_HAS_GENERAL_CALL means Wire isGeneralCall() slave extensions
#define WIRE_HAS_GENERAL_CALL 1

// WIRE_HAS_TRANSFER means Wire has composite transfer()
#define WIRE_HAS_TRANSFER 1

// WIRE_HAS_TRANSACTION means Wire has asynchronous TwoWireTransaction class support
#define WIRE_HAS_TRANSACTION 1

// endTransmission() / sendTransmission() error codes:
//
// 0 success
// 1 paramater
// 2 address NACK
// 3 data NACK
// 4 failure
// 5 timeout

class TwoWire : public Stream
{
public:
    TwoWire(stm32wb_i2c_t *i2c, const stm32wb_i2c_params_t *params);
    TwoWire(const TwoWire&) = delete;
    TwoWire& operator=(const TwoWire&) = delete;

    void begin();
    void begin(uint8_t address);
    void begin(uint8_t address, bool enableGeneralCall);
    void end();
    void setClock(uint32_t clock);

    void beginTransmission(uint8_t address);
    uint8_t endTransmission();
    uint8_t endTransmission(bool stopBit);
  
    size_t requestFrom(uint8_t address, size_t size);
    size_t requestFrom(uint8_t address, size_t size, bool stopBit);
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

    // STM32WB EXTENSION: clock low timeout in milliseconds
    void setClockLowTimeout(unsigned long timeout);

    // STM32WB EXTENSION: general call status
    bool isGeneralCall();
  
    // STM32WB EXTENSTION: synchronous composite transmit/receive 
    uint8_t transfer(uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *xfBuffer, size_t xfSize, bool request = true, bool stopBit = true);
  
    // STM32WB EXTENSION: reset I2C bus
    void reset();

private:
    stm32wb_i2c_t *m_i2c;
    uint32_t m_option;
    uint32_t m_timeout;
    uint8_t m_ev_address;
    bool m_ev_broadcast;

    uint8_t m_xf_address;

    uint8_t m_tx_data[WIRE_BUFFER_LENGTH];
    uint8_t m_tx_address;
    uint8_t m_tx_index;
    bool m_tx_busy;

    uint8_t m_rx_data[WIRE_BUFFER_LENGTH];
    uint8_t m_rx_index;
    uint8_t m_rx_count;

    void (*m_receive_callback)(int);
    void (*m_request_callback)(void);
    void (*m_transmit_callback)(int);

    static void doneCallback(void *context);
    static void eventCallback(void *context, uint32_t events);

    static const uint32_t TWI_CLOCK = 100000;

    friend class TwoWireTransaction;
};


// status() return codes:
//
// 0 success
// 1 failure
// 2 address NACK
// 3 data NACK
// 4 failure
// 5 timeout
// 255 busy

class TwoWireTransaction {
public:
    TwoWireTransaction();
    ~TwoWireTransaction();
    TwoWireTransaction(const TwoWireTransaction&) = delete;
    TwoWireTransaction& operator=(const TwoWireTransaction&) = delete;

    bool submit(TwoWire &wire, uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *xfBuffer, size_t xfSize, void(*callback)(void) = nullptr, bool request = true, bool stopBit = true);
    bool submit(TwoWire &wire, uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *xfBuffer, size_t xfSize, Callback callback, bool request = true, bool stopBit = true);

    uint8_t status();
    bool busy();

private:
    volatile uint8_t m_busy;
    stm32wb_i2c_transaction_t m_transaction;
    TwoWireTransaction * volatile m_notify;
    Callback m_callback;

    static struct TwoWireTransactionQueue {
        TwoWireTransaction * volatile notify;      
        k_work_t                      work;
    } m_queue;

    static void done(void *context);
    static void notify(void *context);

    friend class TwoWire;
};


#if WIRE_INTERFACES_COUNT > 0
extern TwoWire Wire;
#endif
#if WIRE_INTERFACES_COUNT > 1
extern TwoWire Wire1;
#endif

#endif
