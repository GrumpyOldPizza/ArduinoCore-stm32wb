/*
 * Copyright (c) 2020-2021 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#ifndef BLE_H
#define BLE_H

#include <Arduino.h>
#include <type_traits>

enum BLEOption : uint32_t {
    BLE_OPTION_RANDOM_STATIC_ADDRESS                         = 0x00000001,
    BLE_OPTION_PRIVACY                                       = 0x00000002,
    BLE_OPTION_NO_SERVICE_CHANGED                            = 0x00000004,
    BLE_OPTION_READ_ONLY_DEVICE_NAME                         = 0x00000008,
};


enum BLEAppearance : uint16_t {
    BLE_APPEARANCE_UNKNOWN                                   = 0,
    BLE_APPEARANCE_GENERIC_PHONE                             = 64,
    BLE_APPEARANCE_GENERIC_COMPUTER                          = 128,
    BLE_APPEARANCE_GENERIC_WATCH                             = 192,
    BLE_APPEARANCE_WATCH_SPORTS_WATCH                        = 193,
    BLE_APPEARANCE_GENERIC_CLOCK                             = 256,
    BLE_APPEARANCE_GENERIC_DISPLAY                           = 320,
    BLE_APPEARANCE_GENERIC_REMOTE_CONTROL                    = 384,
    BLE_APPEARANCE_GENERIC_EYE_GLASSES                       = 448,
    BLE_APPEARANCE_GENERIC_TAG                               = 512,
    BLE_APPEARANCE_GENERIC_KEYRING                           = 576,
    BLE_APPEARANCE_GENERIC_MEDIA_PLAYER                      = 640,
    BLE_APPEARANCE_GENERIC_BARCODE_SCANNER                   = 704,
    BLE_APPEARANCE_GENERIC_THERMOMETER                       = 768,
    BLE_APPEARANCE_THERMOMETER_EAR                           = 769,
    BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR                 = 832,
    BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT         = 833,
    BLE_APPEARANCE_GENERIC_BLOOD_PRESSURE                    = 896,
    BLE_APPEARANCE_BLOOD_PRESSURE_ARM                        = 897,
    BLE_APPEARANCE_BLOOD_PRESSURE_WRIST                      = 898,
    BLE_APPEARANCE_GENERIC_HID                               = 960,
    BLE_APPEARANCE_HID_KEYBOARD                              = 961,
    BLE_APPEARANCE_HID_MOUSE                                 = 962,
    BLE_APPEARANCE_HID_JOYSTICK                              = 963,
    BLE_APPEARANCE_HID_GAMEPAD                               = 964,
    BLE_APPEARANCE_HID_DIGITIZERSUBTYPE                      = 965,
    BLE_APPEARANCE_HID_CARD_READER                           = 966,
    BLE_APPEARANCE_HID_DIGITAL_PEN                           = 967,
    BLE_APPEARANCE_HID_BARCODE                               = 968,
    BLE_APPEARANCE_GENERIC_GLUCOSE_METER                     = 1024,
    BLE_APPEARANCE_GENERIC_RUNNING_WALKING_SENSOR            = 1088,
    BLE_APPEARANCE_RUNNING_WALKING_SENSOR_IN_SHOE            = 1089,
    BLE_APPEARANCE_RUNNING_WALKING_SENSOR_ON_SHOE            = 1090,
    BLE_APPEARANCE_RUNNING_WALKING_SENSOR_ON_HIP             = 1091,
    BLE_APPEARANCE_GENERIC_CYCLING                           = 1152,
    BLE_APPEARANCE_CYCLING_CYCLING_COMPUTER                  = 1153,
    BLE_APPEARANCE_CYCLING_SPEED_SENSOR                      = 1154,
    BLE_APPEARANCE_CYCLING_CADENCE_SENSOR                    = 1155,
    BLE_APPEARANCE_CYCLING_POWER_SENSOR                      = 1156,
    BLE_APPEARANCE_CYCLING_SPEED_CADENCE_SENSOR              = 1157,
    BLE_APPEARANCE_GENERIC_PULSE_OXIMETER                    = 3136,
    BLE_APPEARANCE_PULSE_OXIMETER_FINGERTIP                  = 3137,
    BLE_APPEARANCE_PULSE_OXIMETER_WRIST_WORN                 = 3138,
    BLE_APPEARANCE_GENERIC_WEIGHT_SCALE                      = 3200,
    BLE_APPEARANCE_GENERIC_OUTDOOR_SPORTS_ACT                = 5184,
    BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_DISP               = 5185,
    BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_AND_NAV_DISP       = 5186,
    BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_POD                = 5187,
    BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_AND_NAV_POD        = 5188,
};

enum BLEAdType : uint8_t {
    BLE_AD_TYPE_FLAGS                                        = 0x01,
    BLE_AD_TYPE_16_BIT_SERV_UUID                             = 0x02,
    BLE_AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST                  = 0x03,
    BLE_AD_TYPE_32_BIT_SERV_UUID                             = 0x04,
    BLE_AD_TYPE_32_BIT_SERV_UUID_CMPLT_LIST                  = 0x05,
    BLE_AD_TYPE_128_BIT_SERV_UUID                            = 0x06,
    BLE_AD_TYPE_128_BIT_SERV_UUID_CMPLT_LIST                 = 0x07,
    BLE_AD_TYPE_SHORTENED_LOCAL_NAME                         = 0x08,
    BLE_AD_TYPE_COMPLETE_LOCAL_NAME                          = 0x09,
    BLE_AD_TYPE_TX_POWER_LEVEL                               = 0x0a,
    BLE_AD_TYPE_CLASS_OF_DEVICE                              = 0x0d,
    BLE_AD_TYPE_SLAVE_CONN_INTERVAL                          = 0x12,
    BLE_AD_TYPE_SERV_SOLICIT_16_BIT_UUID_LIST                = 0x14,
    BLE_AD_TYPE_SERV_SOLICIT_128_BIT_UUID_LIST               = 0x15,
    BLE_AD_TYPE_SERVICE_DATA                                 = 0x16,
    BLE_AD_TYPE_APPEARANCE                                   = 0x19,
    BLE_AD_TYPE_ADVERTISING_INTERVAL                         = 0x1a,
    BLE_AD_TYPE_LE_ROLE                                      = 0x1c,
    BLE_AD_TYPE_SERV_SOLICIT_32_BIT_UUID_LIST                = 0x1f,
    BLE_AD_TYPE_MANUFACTURER_SPECIFIC_DATA                   = 0xff
};

enum BLEProperty : uint8_t {
    BLE_PROPERTY_READ                                        = 0x02,
    BLE_PROPERTY_WRITE_WITHOUT_RESPONSE                      = 0x04,
    BLE_PROPERTY_WRITE                                       = 0x08,
    BLE_PROPERTY_NOTIFY                                      = 0x10,
    BLE_PROPERTY_INDICATE                                    = 0x20
};

enum BLEPermission : uint8_t {
    BLE_PERMISSION_READ_UNENCRYPTED                          = 0x00,
    BLE_PERMISSION_READ_UNAUTHENTICATED                      = 0x01,
    BLE_PERMISSION_READ_AUTHENTICATED                        = 0x02,
    BLE_PERMISSION_READ_AUTHENTICATED_SECURE_CONNECTION      = 0x03,
    BLE_PERMISSION_WRITE_UNENCRYPTED                         = 0x00,
    BLE_PERMISSION_WRITE_UNAUTHENTICATED                     = 0x04,
    BLE_PERMISSION_WRITE_AUTHENTICATED                       = 0x08,
    BLE_PERMISSION_WRITE_AUTHENTICATED_SECURE_CONNECTION     = 0x0c,
    BLE_PERMISSION_SUBSCRIBE_UNENCRYPTED                     = 0x00,
    BLE_PERMISSION_SUBSCRIBE_UNAUTHENTICATED                 = 0x10,
    BLE_PERMISSION_SUBSCRIBE_AUTHENTICATED                   = 0x20,
    BLE_PERMISSION_SUBSCRIBE_AUTHENTICATED_SECURE_CONNECTION = 0x30
};

enum BLESecurity : uint8_t {
    BLE_SECURITY_UNENCRYPTED                                 = 0x00,
    BLE_SECURITY_UNAUTHENTICATED                             = 0x01,
    BLE_SECURITY_AUTHENTICATED                               = 0x02,
    BLE_SECURITY_AUTHENTICATED_SECURE_CONNECTION             = 0x03,
};

enum BLEPhy : uint8_t {
    BLE_PHY_1M                                               = 0x01,
    BLE_PHY_2M                                               = 0x02,
    BLE_PHY_CODED                                            = 0x04
};

enum BLEPhyOption : uint16_t {
    BLE_PHY_OPTION_S2                                        = 0x01,
    BLE_PHY_OPTION_S8                                        = 0x02,
};

enum BLEDiscoverable : uint8_t {
    BLE_DISCOVERABLE_LIMITED                                 = 0x01,
    BLE_DISCOVERABLE_GENERAL                                 = 0x02,
};

enum BLEStatus : uint8_t {
    BLE_STATUS_SUCCESS                                       = 0,
    BLE_STATUS_TIMEOUT                                       = 254,
    BLE_STATUS_BUSY                                          = 255
};
  

class BLEDescriptor {
public:
    BLEDescriptor();
    BLEDescriptor(const char *uuid, uint8_t permissions, const void *value, int valueLength);
    BLEDescriptor(const char* uuid, uint8_t permissions, const char* value);
    BLEDescriptor(const char* uuid, const char* value);
    ~BLEDescriptor();

    BLEDescriptor(const BLEDescriptor &other);
    BLEDescriptor &operator=(const BLEDescriptor &other);
    BLEDescriptor(BLEDescriptor &&other);
    BLEDescriptor &operator=(BLEDescriptor &&other);

    operator bool() const;

    const char *uuid() const;
    uint8_t permissions() const;

    int valueLength() const;
    const void *value() const;
    int getValue(void *value, int length) const;

protected:
    BLEDescriptor(class BLEDescriptorInstance *instance);
    class BLEDescriptorInstance *instance();

    friend class BLEDescriptorInstance;
    
private:
    static BLEDescriptorInstance *instance(const char *uuid, uint8_t permissions, const void *value, int valueLength);

    BLEDescriptorInstance *m_instance;

public:
    template<typename T>
    typename std::enable_if<std::is_standard_layout<T>::value, int>::type
    getValue(T& value) {
        return getValue(reinterpret_cast<void*>(&value), sizeof(T));
    }
};


class BLECharacteristic {
public:
    BLECharacteristic();
    BLECharacteristic(const char *uuid, uint8_t properties, uint8_t permissions, const void *value, int valueLength, int valueSize, bool fixedLength);
    BLECharacteristic(const char *uuid, uint8_t properties, uint8_t permissions, int valueSize, bool fixedLength);
    BLECharacteristic(const char* uuid, uint8_t properties, uint8_t permissions, const char* value);
    BLECharacteristic(const char* uuid, uint8_t properties, const char* value);
    ~BLECharacteristic();

    BLECharacteristic(const BLECharacteristic &other);
    BLECharacteristic &operator=(const BLECharacteristic &other);
    BLECharacteristic(BLECharacteristic &&other);
    BLECharacteristic &operator=(BLECharacteristic &&other);
    
    operator bool() const;

    const char *uuid() const;
    uint8_t properties() const;
    uint8_t permissions() const;
    bool fixedLength() const;

    int valueSize() const;
    int valueLength();
    const void* value();
    int getValue(void *value, int length);

    bool setValue(const void *value, int length);

    bool writeValue(const void *value, int length);
    bool writeValue(const void *value, int length, volatile uint8_t &status);
    bool writeValue(const void *value, int length, volatile uint8_t &status, void(*callback)(void));
    bool writeValue(const void *value, int length, volatile uint8_t &status, Callback callback);

    bool busy();
    bool written();
    bool subscribed();

    unsigned int descriptorCount();
    BLEDescriptor descriptor(unsigned int index);
    bool addDescriptor(BLEDescriptor &descriptor);
    bool removeDescriptor(BLEDescriptor &descriptor);
    
    void onRead(void(*callback)(void));
    void onRead(Callback callback);
    void onWritten(void(*callback)(void));
    void onWritten(Callback callback);
    void onSubscribed(void(*callback)(void));
    void onSubscribed(Callback callback);

protected:
    BLECharacteristic(class BLECharacteristicInstance *instance);
    class BLECharacteristicInstance *instance();

    friend class BLECharacteristicInstance;
    
private:
    static BLECharacteristicInstance *instance(const char *uuid, uint8_t properties, uint8_t permissions, const void *value, int valueLength, int valueSize, bool fixedLength, bool constant);

    BLECharacteristicInstance *m_instance;

public:
    template<typename T>
    BLECharacteristic(const char *uuid, uint8_t properties, const T& value) :
        BLECharacteristic(uuid, properties, 0, reinterpret_cast<const void*>(&value), sizeof(T), sizeof(T), false) { };

    template<typename T>
    BLECharacteristic(const char *uuid, uint8_t properties, uint8_t permissions, const T& value) :
        BLECharacteristic(uuid, properties, permissions, reinterpret_cast<const void*>(&value), sizeof(T), sizeof(T), false) { };
    
    template<typename T>
    typename std::enable_if<std::is_standard_layout<T>::value, int>::type
    getValue(T& value) {
        return getValue(reinterpret_cast<void*>(&value), sizeof(T));
    }
    
    template<typename T>
    typename std::enable_if<std::is_standard_layout<T>::value, bool>::type
    setValue(const T& value) {
        return setValue(reinterpret_cast<const void*>(&value), sizeof(T));
    }
    
    template<typename T>
    typename std::enable_if<std::is_standard_layout<T>::value, bool>::type
    writeValue(const T& value) {
        return writeValue(reinterpret_cast<const void*>(&value), sizeof(T));
    }

    template<typename T>
    typename std::enable_if<std::is_standard_layout<T>::value, bool>::type
    writeValue(const T& value, volatile uint8_t &status) {
        return writeValue(reinterpret_cast<const void*>(&value), sizeof(T), status);
    }

    template<typename T>
    typename std::enable_if<std::is_standard_layout<T>::value, bool>::type
    writeValue(const T& value, volatile uint8_t &status, void(*callback)(void)) {
        return writeValue(reinterpret_cast<const void*>(&value), sizeof(T), status, callback);
    }

    template<typename T>
    typename std::enable_if<std::is_standard_layout<T>::value, bool>::type
    writeValue(const T& value, volatile uint8_t &status, Callback callback) {
        return writeValue(reinterpret_cast<const void*>(&value), sizeof(T), status, callback);
    }
};


class BLEService {
public:
    BLEService();
    BLEService(const char *uuid);
    ~BLEService();

    BLEService(const BLEService &other);
    BLEService &operator=(const BLEService &other);
    BLEService(BLEService &&other);
    BLEService &operator=(BLEService &&other);
    
    operator bool() const;

    const char *uuid() const;

    unsigned int characteristicCount();
    BLECharacteristic characteristic(unsigned int index);
    bool addCharacteristic(BLECharacteristic &characteristic);
    bool removeCharacteristic(BLECharacteristic &characteristic);

protected:
    BLEService(class BLEServiceInstance *instance);
    class BLEServiceInstance *instance();

    friend class BLEServiceInstance;
    
private:
    static BLEServiceInstance *instance(const char *uuid);
    
    BLEServiceInstance *m_instance;
};


class BLEDevice {
public:
    BLEDevice();
    ~BLEDevice();

    BLEDevice(const BLEDevice &other);
    BLEDevice &operator=(const BLEDevice &other);
    BLEDevice(BLEDevice &&other);
    BLEDevice &operator=(BLEDevice &&other);
    
    operator bool() const;

    String address() const;
    bool connected();
    int rssi();
    int mtu();

    bool setConnectionInterval(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval);
    bool setConnectionParameters(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval, uint16_t slaveLatency);
    bool setConnectionParameters(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval, uint16_t slaveLatency, uint16_t supervisionTimeout);
    void getConnectionParameters(uint16_t &connectionInterval, uint16_t &slaveLatency, uint16_t &supervisionTimeout); 
    
    void onConnectionUpdate(void(*callback)(void));
    void onConnectionUpdate(Callback callback);
    
protected:
    BLEDevice(class BLEDeviceInstance *instance);
    class BLEDeviceInstance *instance();

    friend class BLEDeviceInstance;
    
private:
    BLEDeviceInstance *m_instance;
};


class BLEClass {
public:
    static bool begin(int mtu = 23, uint32_t options = 0);
    static void end();
    
    static String address();
    
    static bool setTxPowerLevel(int txPower);
    static bool setPreferredPhy(BLEPhy txPhy, BLEPhy rxPhy, BLEPhyOption phyOptions = (BLEPhyOption)0);

    // GAP/GATT Server: Standard Services/Attributes
    static void setDeviceName(const char *deviceName);
    static void setAppearance(BLEAppearance appearance);
    static bool setConnectionInterval(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval);
    static bool setConnectionParameters(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval, uint16_t slaveLatency);
    static bool setConnectionParameters(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval, uint16_t slaveLatency, uint16_t supervisionTimeout);
    
    // GATT Server: Custom Services
    static unsigned int serviceCount();
    static BLEService service(unsigned int index);
    static bool addService(BLEService &service);
    static bool removeService(BLEService &service);

    // GAP: Authentication
    static bool setSecurity(BLESecurity security);
    static bool setBonding(bool bonding);
    static bool setPin(const char *pin);
    static void clearBondStorage();
    
    // GAP: Advertising
    static bool setIncludeTxPowerLevel(bool enable);
    static bool setIncludeConnectionInterval(bool enable);
    static bool setLocalName(const char *localName);
    static bool setAdvertisedServiceUuid(const char *uuid, bool complete = true);
    static bool setAdvertisedServiceUuids(const char *uuid[], int count, bool complete = true);
    static bool setManufacturerData(const uint8_t data[], int length);
    static bool setBeacon(const char *uuid, uint16_t major, uint16_t minor, int8_t rssi);
    static bool setAdvertisingData(const uint8_t data[], int length);
    static bool setScanResponseData(const uint8_t data[], int length);

    static bool setAdvertisingInterval(uint16_t advertisingInterval);
    static bool setAdvertisingInterval(uint16_t minimumAdvertisingInterval, uint16_t maximumAdvertisingInterval);
    static void setConnectable(bool connectable);
    static void setDiscoverable(BLEDiscoverable discoverable);

    static bool advertise();
    static void stopAdvertise();
    static bool advertising();
    
    // GAP: Peripheral 
    static BLEDevice central();
    static bool connected();
    static void disconnect();

    static void onStop(void(*callback)(void));
    static void onStop(Callback callback);
    static void onConnect(void(*callback)(void));
    static void onConnect(Callback callback);
    static void onDisconnect(void(*callback)(void));
    static void onDisconnect(Callback callback);
};

extern BLEClass BLE;


enum BLEUartProtocol : uint8_t {
    BLE_UART_PROTOCOL_BLUEST = 0,
    BLE_UART_PROTOCOL_NORDIC,
    BLE_UART_PROTOCOL_HMSOFT
};

#define BLE_UART_RX_BUFFER_SIZE 128
#define BLE_UART_TX_BUFFER_SIZE 128

class BLEUart : public Stream, public BLEService {
public:
    BLEUart(BLEUartProtocol protocol);
    ~BLEUart();
    
    operator bool();

    int available() override;
    int peek() override;
    int read() override;
    int read(uint8_t *buffer, size_t size) override;

    int availableForWrite() override;
    size_t write(uint8_t data) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    void flush() override;

    void setNonBlocking(bool enable);

    void onReceive(void(*callback)(void));
    void onReceive(Callback callback);
    
    using Print::write;
    
private:
    BLECharacteristic m_rx_characteristic;
    BLECharacteristic m_tx_characteristic;
    BLECharacteristic m_tx2_characteristic;

    uint8_t m_rx_data[BLE_UART_RX_BUFFER_SIZE];
    uint8_t m_tx_data[BLE_UART_TX_BUFFER_SIZE];
    volatile uint16_t m_rx_write;
    volatile uint16_t m_rx_read;
    volatile uint16_t m_rx_count;
    volatile uint16_t m_tx_write;
    volatile uint16_t m_tx_read;
    volatile uint16_t m_tx_count;
    volatile uint16_t m_tx_size;
    volatile uint8_t m_tx_busy;
    volatile uint8_t m_connected;
    volatile uint8_t m_status;

    const BLEUartProtocol m_protocol;
    uint8_t m_packet_size;
    uint8_t m_nonblocking;

    Callback m_receive_callback;

    void transmit();
    void receive();
    void connect();
};

#endif // BLE_H
