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

#include "Arduino.h"
#include "BLE.h"
#include "wiring_private.h"

#include "stm32wb_ipcc.h"

extern "C" {
#include "BLE/ble_core.h"
}

#define BLE_TRACE_SUPPORTED                              0

#define BLE_STATUS_REQUEST_BUSY                          0xff

#define BLE_ADDRESS_LENGTH                               6
#define BLE_UUID16_LENGTH                                2
#define BLE_UUID128_LENGTH                               16
#define BLE_MAX_UUID_LENGTH                              16
#define BLE_MIN_DEVICE_NAME_LENGTH                       32
#define BLE_MAX_DEVICE_NAME_LENGTH                       248

#define BLE_SLAVE_SCA                                    500
#define BLE_MASTER_SCA                                   0     /* 251 ppm to 500 ppm */
#define BLE_LSE_SOURCE                                   0     /* OSC32K */
#define BLE_HSE_STARTUP_TIME                             0x148 /* units of 625/256 us (~2.44 us), 800us */

#define BLE_NUM_LINK                                     1
#define BLE_DATA_LENGTH_EXTENSION                        1
#define BLE_MAX_ATT_MTU                                  156
#define BLE_VITERBI_MODE                                 1
#define BLE_MAX_CONN_EVENT_LENGTH                        0xffffffff

#define BLE_NUM_GATT_ATTRIBUTES                          68
#define BLE_NUM_GATT_SERVICES                            8
#define BLE_ATT_VALUE_ARRAY_SIZE                         1344

#define BLE_PREPARE_WRITE_LIST_SIZE                      (BLE_PREP_WRITE_X_ATT(BLE_MAX_ATT_MTU))
#define BLE_MBLOCK_COUNT                                 (BLE_MBLOCKS_CALC(BLE_PREPARE_WRITE_LIST_SIZE, BLE_MAX_ATT_MTU, BLE_NUM_LINK))

#define BLE_OPTION_LL_ONLY                               0x01
#define BLE_OPTION_NO_SERVICE_CHANGED                    0x02
#define BLE_OPTION_READ_ONLY_DEVICE_NAME                 0x04
#define BLE_OPTION_POWER_CLASS_1                         0x80

#define BLE_VALUE_ATTRIB_HANDLE_OFFSET                   1
#define BLE_CCCD_ATTRIB_HANDLE_OFFSET                    2

#define BLE_ATT_SERVICE_HANDLE                           0x0001
#define BLE_SERVICE_CHANGED_HANDLE                       0x0002

#define BLE_BUSY_MASK                                    0x0000ffff
#define BLE_BUSY_HCI_LE_SET_PHY                          0x00000001
#define BLE_BUSY_L2CAP_CONNECTION_PARAMETER_UPDATE       0x00000002
#define BLE_BUSY_GAP_SLAVE_SECURITY                      0x00000004
#define BLE_BUSY_GAP_ALLOW_REBOND                        0x00000008
#define BLE_BUSY_GAP_TERMINATE                           0x00000010
#define BLE_BUSY_GATT_UPDATE_CHAR_VALUE_EXT              0x00000020
#define BLE_BUSY_GATT_READ_HANDLE_VALUE                  0x00000040
#define BLE_BUSY_GATT_CONFIRM_INDICATION                 0x00000080
#define BLE_BUSY_GATT_ALLOW_READ                         0x00000100

#define BLE_REQUEST_MASK                                 0x0fff0000
#define BLE_REQUEST_HCI_LE_SET_PHY                       0x00010000
#define BLE_REQUEST_L2CAP_CONNECTION_PARAMETER_UPDATE    0x00020000
#define BLE_REQUEST_GAP_SLAVE_SECURITY                   0x00040000
#define BLE_REQUEST_GAP_ALLOW_REBOND                     0x00080000
#define BLE_REQUEST_GAP_TERMINATE                        0x00100000
#define BLE_REQUEST_GATT_CONFIRM_INDICATION              0x00200000
#define BLE_REQUEST_GATT_ALLOW_READ                      0x00400000

#define BLE_EVENT_MASK                                   0xf0000000
#define BLE_EVENT_GAP_PAIRING_COMPLETE                   0x10000000
#define BLE_EVENT_GATT_TX_POOL_AVAILABLE                 0x20000000
#define BLE_EVENT_GATT_SERVER_CONFIRMATION               0x40000000

#define BLE_PEER_DEVICE_NOT_CONNECTED                    0xffff

#define BLE_FIXED_PIN_UNDEFINED                          0xffffffff

static const void *allocate_noconst(const void *data, uint32_t size) {
    void *_data;
    
    if (((uint32_t)data >= FLASH_BASE) && ((uint32_t)data <= (FLASH_BASE + FLASH_SIZE))) {
        return data;
    }

    _data = malloc(size);

    if (_data) {
        memcpy(_data, data, size);
    }

    return (const void*)_data;
}

static void deallocate_noconst(const void *data) {
    if (!(((uint32_t)data >= FLASH_BASE) && ((uint32_t)data <= (FLASH_BASE + FLASH_SIZE)))) {
        free((void*)data);
    }
}
        
static int ascii2hex(char c) {
    if ((c >= '0') && (c <= '9')) { return ((c - '0') + 0x00); }
    if ((c >= 'a') && (c <= 'f')) { return ((c - 'a') + 0x0a); }
    if ((c >= 'A') && (c <= 'F')) { return ((c - 'A') + 0x0a); }

    return -1;
}

static const char hex2ascii[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };

/**********************************************************************************
 * Address
 */

enum class BLEAddressType : uint8_t {
    PUBLIC                        = 0,
    RANDOM_STATIC                 = 1,
    RANDOM_PRIVATE_RESOLVABLE     = 2,
    RANDOM_PRIVATE_NON_RESOLVABLE = 3,
    UNDEFINED                     = 255,
};
  
class BLEAddress {
public:
    BLEAddress();
    BLEAddress(const char *cstring, BLEAddressType type);
    BLEAddress(const uint8_t data[BLE_ADDRESS_LENGTH], BLEAddressType type);

    operator bool() const;
    
    BLEAddressType type() const;
    const uint8_t *data() const;
    uint32_t toString(char* cstring, uint32_t size, bool stripped = false) const;
    String toString(bool stripped = false) const;
    
private:
    BLEAddressType m_type;
    uint8_t m_data[BLE_ADDRESS_LENGTH];
};

BLEAddress::BLEAddress() {
    m_type = BLEAddressType::UNDEFINED;
}

BLEAddress::BLEAddress(const char *cstring, BLEAddressType type) {
    int index, length, data_l, data_h;

    memset(&m_data[0], 0, sizeof(m_data));
    
    m_type = BLEAddressType::UNDEFINED;

    length = strlen(cstring);

    if ((length == 12) || (length == 17)) {
        for (index = 5; index >= 0; index--) {
            data_h = ascii2hex(*cstring++);
            data_l = ascii2hex(*cstring++);

            if ((data_h < 0) || (data_l < 0)) {
                break;
            }

            m_data[index] = (data_h << 4) | (data_l << 0);

            if (length == 17) {
                if (*cstring++ != ':') {
                    break;
                }
            }
        }
        
        if (index == -1) {
            m_type = type;
        }
    }
}

BLEAddress::BLEAddress(const uint8_t data[BLE_ADDRESS_LENGTH], BLEAddressType type) {
    int index;

    m_type = type;

    if (type != BLEAddressType::UNDEFINED) {
        for (index = 5; index >= 0; index--) {
            m_data[index] = data[index];
        }
    }
}

BLEAddress::operator bool() const {
    return m_type != BLEAddressType::UNDEFINED;
}

BLEAddressType BLEAddress::type() const {
    return m_type;
}

const uint8_t* BLEAddress::data() const {
    return m_data;
}

uint32_t BLEAddress::toString(char* cstring, uint32_t size, bool stripped) const {
    int index;

    if (m_type != BLEAddressType::UNDEFINED) {
        if (size >= (stripped ? 13 : 18)) {
            for (index = 15; index >= 0; index--) {
                *cstring++ = hex2ascii[m_data[index] >> 4];
                *cstring++ = hex2ascii[m_data[index] & 15];

                if (!stripped) {
                    *cstring++ = ':';
                }
            }

            *cstring++ = '\0';

            return (stripped ? 12 : 17);
        }
    }

    return 0;
}

String BLEAddress::toString(bool stripped) const {
    char cstring[18];

    if (toString(&cstring[0], sizeof(cstring), stripped)) {
        return String(&cstring[0]);
    } else {
        return String();
    }
}

/**********************************************************************************
 * UUID
 */

enum class BLEUuidType : uint8_t {
    UUID16    = 1,
    UUID128   = 2,
    UNDEFINED = 255,
};
  
class BLEUuid {
public:
    BLEUuid();
    BLEUuid(const char *cstring, BLEUuidType type = BLEUuidType::UNDEFINED);
    BLEUuid(const uint8_t data[], BLEUuidType type);

    operator bool() const;
    
    BLEUuidType type() const;
    const uint8_t *data() const;
    uint32_t toString(char* cstring, uint32_t size, bool stripped = false) const;
    String toString(bool stripped = false) const;
    
private:
    BLEUuidType m_type;
    uint8_t m_data[BLE_MAX_UUID_LENGTH];
};

BLEUuid::BLEUuid() {
    m_type = BLEUuidType::UNDEFINED;
}

BLEUuid::BLEUuid(const char *cstring, BLEUuidType type) {
    int index, length, data_l, data_h;

    memset(&m_data[0], 0, sizeof(m_data));
    
    m_type = BLEUuidType::UNDEFINED;

    length = strlen(cstring);

    if (length == 4) {
        if ((type == BLEUuidType::UNDEFINED) || (type == BLEUuidType::UUID16)) {
            for (index = 1; index >= 0; index--) {
                data_h = ascii2hex(*cstring++);
                data_l = ascii2hex(*cstring++);
                
                if ((data_h < 0) || (data_l < 0)) {
                    break;
                }
                
                m_data[index] = (data_h << 4) | (data_l << 0);
            }
            
            if (index == -1) {
                m_type = BLEUuidType::UUID16;
            }
        }
    }

    if ((length == 32) || (length == 36)) {
        if ((type == BLEUuidType::UNDEFINED) || (type == BLEUuidType::UUID128)) {
            for (index = 15; index >= 0; index--) {
                data_h = ascii2hex(*cstring++);
                data_l = ascii2hex(*cstring++);
                
                if ((data_h < 0) || (data_l < 0)) {
                    break;
                }
                
                m_data[index] = (data_h << 4) | (data_l << 0);
                
                if ((length == 36) && ((1 << index) & ((1 << 12) | (1 << 10) | (1 << 8) | (1 << 6)))) {
                    if (*cstring++ != '-') {
                        break;
                    }
                }
            }

            if (index == -1) {
                m_type = BLEUuidType::UUID128;
            }
        }
    }
}

BLEUuid::BLEUuid(const uint8_t uuid[], BLEUuidType type) {
    int index;

    m_type = type;

    if (type != BLEUuidType::UNDEFINED) {
        for (index = ((type == BLEUuidType::UUID16) ? 1 : 15); index >= 0; index--) {
            m_data[index] = uuid[index];
        }
    }
}

BLEUuid::operator bool() const {
    return m_type != BLEUuidType::UNDEFINED;
}

BLEUuidType BLEUuid::type() const {
    return m_type;
}

const uint8_t* BLEUuid::data() const {
    return m_data;
}

uint32_t BLEUuid::toString(char* cstring, uint32_t size, bool stripped) const {
    int index;

    if (m_type == BLEUuidType::UUID16) {
        if (size >= 5) {
            for (index = 1; index >= 0; index--) {
                *cstring++ = hex2ascii[m_data[index] >> 4];
                *cstring++ = hex2ascii[m_data[index] & 15];
            }

            *cstring++ = '\0';
            
            return 4;
        }
    }

    if (m_type == BLEUuidType::UUID128) {
        if (size >= (stripped ? 33 : 37)) {
            for (index = 15; index >= 0; index--) {
                *cstring++ = hex2ascii[m_data[index] >> 4];
                *cstring++ = hex2ascii[m_data[index] & 15];

                if (!stripped && ((1 << index) & ((1 << 12) | (1 << 10) | (1 << 8) | (1 << 6)))) {
                    *cstring++ = '-';
                }
            }

            *cstring++ = '\0';

            return (stripped ? 32 : 36);
        }
    }

    return 0;
}

String BLEUuid::toString(bool stripped) const {
    char cstring[37];

    if (toString(&cstring[0], sizeof(cstring), stripped)) {
        return String(&cstring[0]);
    } else {
        return String();
    }
}

/**********************************************************************************
 * GAP/GATT instance
 */

class BLEDescriptorInstance {
public:
    virtual ~BLEDescriptorInstance() = default;

    virtual void reference();
    virtual void unreference();

    virtual class BLEServerDescriptor* server();
  
    virtual const char* uuid() const;
    virtual uint8_t permissions() const;

    virtual int valueLength() const;
    virtual const void *value() const;
    virtual int getValue(void *value, int length) const;

    inline BLEDescriptor descriptor() { return BLEDescriptor(this); }
    inline static BLEDescriptorInstance *instance(BLEDescriptor &descriptor) { return descriptor.instance(); }
};

class BLECharacteristicInstance {
public:
    virtual ~BLECharacteristicInstance() = default;

    virtual void reference();
    virtual void unreference();

    virtual class BLEServerCharacteristic* server();
  
    virtual const char *uuid() const;
    virtual uint8_t properties() const;
    virtual uint8_t permissions() const;
    virtual bool fixedLength() const;

    virtual int valueSize() const;
    virtual int valueLength();
    virtual const void *value();
    virtual int getValue(void *value, int length);

    virtual bool setValue(const void *value, int length);
    virtual bool writeValue(const void *value, int length, volatile uint8_t *p_status_return, Callback callback);
    
    virtual bool busy();
    virtual bool written();
    virtual bool subscribed();
    
    virtual unsigned int descriptorCount();
    virtual BLEDescriptor descriptor(unsigned int index);
    virtual bool addDescriptor(BLEDescriptor &descriptor);
    virtual bool removeDescriptor(BLEDescriptor &descriptor);
    
    virtual void onRead(Callback callback);
    virtual void onWritten(Callback callback);
    virtual void onSubscribed(Callback callback);

    inline BLECharacteristic characteristic() { return BLECharacteristic(this); }
    inline static BLECharacteristicInstance *instance(BLECharacteristic &characteristic) { return characteristic.instance(); }
};

class BLEServiceInstance {
public:
    virtual ~BLEServiceInstance() = default;
  
    virtual void reference();
    virtual void unreference();

    virtual class BLEServerService* server();
  
    virtual const char *uuid() const;

    virtual unsigned int characteristicCount();
    virtual BLECharacteristic characteristic(unsigned int index);
    virtual bool addCharacteristic(BLECharacteristic &characteristic);
    virtual bool removeCharacteristic(BLECharacteristic &characteristic);

    inline BLEService service() { return BLEService(this); }
    inline static BLEServiceInstance *instance(BLEService &service) { return service.instance(); }
};

class BLEDeviceInstance {
public:
    virtual ~BLEDeviceInstance() = default;

    virtual void reference();
    virtual void unreference();

    virtual String address() const;
    virtual bool connected();
    virtual int rssi();
    virtual int mtu();

    virtual bool setConnectionParameters(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval, uint16_t slaveLatency, uint16_t supervisionTimeout);
    virtual void getConnectionParameters(uint16_t &connectionInterval, uint16_t &slaveLatency, uint16_t &supervisionTimeout);  

    virtual void onConnectionUpdate(Callback callback);

    inline BLEDevice device() { return BLEDevice(this); }
    inline static BLEDeviceInstance *instance(BLEDevice &device) { return device.instance(); }
};

static BLEDescriptorInstance BLENullDescriptor;
static BLECharacteristicInstance BLENullCharacteristic;
static BLEServiceInstance BLENullService;
static BLEDeviceInstance BLENullDevice;


/**********************************************************************************
 * GATT client
 */



/**********************************************************************************
 * GATT server
 */

class BLEServerDescriptor final : public BLEDescriptorInstance {
public:
    BLEServerDescriptor(const char *uuid, uint8_t permissions, uint8_t *value, int valueLength);
    ~BLEServerDescriptor() override;

    void reference() override;
    void unreference() override;

    class BLEServerDescriptor* server() override;
    
    const char *uuid() const override;
    uint8_t permissions() const override;

    int valueLength() const override;
    const void *value() const override;
    int getValue(void *value, int length) const override;
    
private:
    volatile uint32_t m_refcount;


protected:
    const char * const m_uuid;
    const uint8_t m_permissions;
 
    const uint8_t * m_value;
    const uint16_t m_value_length;

    struct {
        uint16_t handle;
        class BLEServerCharacteristic *parent;
        class BLEServerDescriptor *sibling;
    } m_server;
    
    friend class BLELocalDevice;
    friend class BLEServerService;
    friend class BLEServerCharacteristic;
};

class BLEServerCharacteristic final : public BLECharacteristicInstance {
public:
    BLEServerCharacteristic(const char *uuid, uint8_t properties, uint8_t permissions, uint8_t *value, int valueLength, int valueSize, bool fixedLength, bool constant);
    ~BLEServerCharacteristic() override;

    void reference() override;
    void unreference() override;

    class BLEServerCharacteristic* server() override;
    
    const char *uuid() const override;
    uint8_t properties() const override;
    uint8_t permissions() const override;
    bool fixedLength() const override;

    int valueSize() const override;
    int valueLength() override;
    const void *value() override;
    int getValue(void *value, int length) override;

    bool setValue(const void *value, int length) override;
    bool writeValue(const void *value, int length, volatile uint8_t *p_status_return, Callback callback) override;
    
    bool busy() override;
    bool written() override;
    bool subscribed() override;

    unsigned int descriptorCount() override;
    BLEDescriptor descriptor(unsigned int index) override;
    bool addDescriptor(BLEDescriptor &descriptor) override;
    bool removeDescriptor(BLEDescriptor &descriptor) override;
    
    void onRead(Callback callback) override;
    void onWritten(Callback callback) override;
    void onSubscribed(Callback callback) override;
    
private:
    volatile uint32_t m_refcount;
  
protected:
    const char * const m_uuid;
    const uint8_t m_properties;
    const uint8_t m_permissions : 6;
    const uint8_t m_constant : 1;
    const uint8_t m_fixed_length : 1;

    volatile uint8_t m_subscribed : 2;
    volatile uint8_t m_update_type : 2;
    volatile uint8_t m_update_value : 1;
    volatile uint8_t m_sync_value : 1;
    volatile uint8_t m_sync_subscribed : 1;
    
    volatile uint8_t m_busy;
    volatile uint16_t m_written;
    
    uint8_t * const m_value;
    uint16_t m_value_length;
    const uint16_t m_value_size;

    volatile uint8_t *m_status;
    Callback m_done_callback;

    Callback m_read_callback;
    Callback m_written_callback;
    Callback m_subscribed_callback;

    struct {
        uint16_t handle;
        uint16_t count;
        class BLEServerService *parent;
        class BLEServerCharacteristic *sibling;
        class BLEServerDescriptor *children;
    } m_server;

    struct {
        class BLEServerCharacteristic * volatile request;
    } m_process;
    
    friend class BLELocalDevice;
    friend class BLEServerService;
    friend class BLEServerDescriptor;
};

class BLEServerService final : public BLEServiceInstance {
public:
    BLEServerService(const char *uuid);
    ~BLEServerService() override;

    void reference() override;
    void unreference() override;

    class BLEServerService* server() override;
    
    const char *uuid() const override;

    unsigned int characteristicCount() override;
    BLECharacteristic characteristic(unsigned int index) override;
    bool addCharacteristic(BLECharacteristic &characteristic) override;
    bool removeCharacteristic(BLECharacteristic &characteristic) override;
  
private:
    volatile uint32_t m_refcount;

protected:
    const char * const m_uuid;

    struct {
        uint16_t handle;
        uint16_t count;
        uint16_t attrib[2];
        class BLEServerService *sibling;
        class BLEServerCharacteristic *children;
    } m_server;
    
    friend class BLELocalDevice;
    friend class BLEServerCharacteristic;
    friend class BLEServerDescriptor;
};


/**********************************************************************************
 * GAP peer
 */

class BLEPeerDevice final : public BLEDeviceInstance {
public:
    BLEPeerDevice();
    BLEPeerDevice(uint16_t handle, const uint8_t *address, uint8_t type, uint16_t connectionInterval, uint16_t slaveLatency, uint16_t supervisionTimeout);
    ~BLEPeerDevice() override;

    void reference() override;
    void unreference() override;
    
    String address() const override;
    bool connected() override;
    int rssi() override;
    int mtu() override;

    bool setConnectionParameters(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval, uint16_t slaveLatency, uint16_t supervisionTimeout) override;
    void getConnectionParameters(uint16_t &connectionInterval, uint16_t &slaveLatency, uint16_t &supervisionTimeout) override; 

    void onConnectionUpdate(Callback callback) override;
    
private:
    volatile uint32_t m_refcount;

protected:
    uint16_t m_handle;
    BLEAddress m_address;
    uint16_t m_mtu;
    uint16_t m_connection_interval;
    uint16_t m_slave_latency;
    uint16_t m_supervision_timeout;
    
    Callback m_connection_update_callback;

    friend class BLELocalDevice;
};


/**********************************************************************************
 * GAP host
 */

class BLELocalDevice {
public:
    BLELocalDevice();
    ~BLELocalDevice() { };

    inline bool begin(int mtu, uint32_t options);
    inline void end();
    
    inline String address() const;
    
    inline bool setTxPowerLevel(int txPower);
    inline bool setPreferredPhy(BLEPhy txPhy, BLEPhy rxPhy, BLEPhyOption options);

    inline void setDeviceName(const char *deviceName);
    inline void setAppearance(enum BLEAppearance appearance);
    inline bool setConnectionParameters(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval, uint16_t slaveLatency, uint16_t supervisionTimeout);
    
    inline unsigned int serviceCount();
    inline BLEService service(unsigned int index);
    inline bool addService(BLEService &service);
    inline bool removeService(BLEService &service);

    inline bool setSecurity(BLESecurity security);
    inline bool setBonding(bool bonding);
    inline bool setPin(const char *pin);
    inline void clearBondStorage();
    
    inline bool setIncludeTxPowerLevel(bool enable);
    inline bool setIncludeConnectionInterval(bool enable);
    inline bool setLocalName(const char *localName);
    inline bool setAdvertisedServiceUuids(const char *uuids[], int count, bool complete = true);
    inline bool setManufacturerData(const uint8_t data[], int length);
    inline bool setBeacon(const char *uuid, uint16_t major, uint16_t minor, int8_t rssi);
    inline bool setAdvertisingData(const uint8_t data[], int length);
    inline bool setScanResponseData(const uint8_t data[], int length);

    inline bool setAdvertisingInterval(uint16_t minimumAdvertisingInterval, uint16_t maximumAdvertisingInterval);
    inline void setConnectable(bool connectable);
    inline void setDiscoverable(BLEDiscoverable discoverable);

    inline bool advertise();
    inline void stopAdvertise();
    inline bool advertising();
    
    inline BLEDevice central();
    inline bool connected();
    inline void disconnect();

    inline void onStop(Callback callback);
    inline void onConnect(Callback callback);
    inline void onDisconnect(Callback callback);
    
private:
    bool m_reset;

    uint8_t m_mtu;
    uint32_t m_options;
    
    BLEAddress m_address;

    const char *m_device_name;
    uint16_t m_device_name_length;
    uint16_t m_appearance;
    struct {
        uint16_t connection_interval_min;
        uint16_t connection_interval_max;
        uint16_t slave_latency;
        uint16_t supervision_timeout;
    } m_ppcp;
    uint8_t m_pa_level;
    int8_t m_tx_power_level;
    uint8_t m_tx_phy;
    uint8_t m_rx_phy;
    uint16_t m_phy_options;
    
    uint16_t m_gap_service_handle;
    uint16_t m_device_name_handle;
    uint16_t m_appearance_handle;
    uint16_t m_ppcp_handle;
    
    Callback m_stop_callback;
    Callback m_connect_callback;
    Callback m_disconnect_callback;

    BLEPeerDevice *m_central;
    
    struct {
        BLESecurity security;
        uint16_t count;
        BLEServerService *children;
    } m_server;

    struct {
        bool advertising;
        bool bonding;
        BLESecurity security;
        uint32_t fixed_pin;
        bool connectable;
        BLEDiscoverable discoverable;
        uint16_t advertising_interval_min;
        uint16_t advertising_interval_max;
    } m_peripheral;
  
    struct {
        uint8_t data_length;
        uint8_t local_name_length;
        uint8_t service_uuids_length;
        uint8_t connection_interval_length;
        uint8_t tx_power_level_length;
        uint8_t manufacturer_data_length;
        uint8_t custom_length;
        uint8_t data[MAX_ADV_DATA_LEN];
        uint8_t scan_response_data_length;
        uint8_t scan_response_data[MAX_ADV_DATA_LEN];
    } m_advertising;

    struct {
        volatile uint32_t sequence;
        uint16_t connection;
        volatile uint16_t connection_interval_min;
        volatile uint16_t connection_interval_max;
        volatile uint16_t slave_latency;
        volatile uint16_t supervision_timeout;

        bool published;
        uint8_t service_count;
        BLEServerService *service_table[BLE_NUM_GATT_SERVICES];
        
        BLEServerCharacteristic *request_head;
        BLEServerCharacteristic *request_tail;
        BLEServerCharacteristic *request_current;
        BLEServerCharacteristic * volatile request_submit;
        uint16_t request_offset;
        uint16_t request_count;
        uint16_t request_length;

        volatile bool sync_value;
        BLEServerCharacteristic *value_current;
        uint16_t value_offset;
        uint16_t value_count;
        uint16_t value_length;

        volatile bool sync_subscribed;
        BLEServerCharacteristic *subscribed_current;
        
        stm32wb_ipcc_ble_command_t command;
        uint8_t data[256];
        k_work_t work;
    } m_process;

    void reset();
    bool publish();
    bool insert(BLEServerService *service); 
    void remove(BLEServerService *service); 
    void request(BLEServerCharacteristic *request); 
    BLEPeerDevice *complete(uint16_t handle, const uint8_t *address, uint8_t type, uint16_t connectionInterval, uint16_t slaveLatency, uint16_t supervisionTimeout);
    void terminate();
    void connection(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval, uint16_t slaveLatency, uint16_t supervisionTimeout);
    void phy(BLEPhy txPhy, BLEPhy rxPhy);
    void process();
    void done();
    void event();

    friend class BLEServerService;
    friend class BLEServerCharacteristic;
    friend class BLEServerDescriptor;
    friend class BLEPeerDevice;
};

static BLELocalDevice BLEHost;


/**********************************************************************************
 * GAP/GATT instance
 */

void BLEDescriptorInstance::reference() {
}

void BLEDescriptorInstance::unreference() {
}

class BLEServerDescriptor* BLEDescriptorInstance::server() {
    return nullptr;
}

const char *BLEDescriptorInstance::uuid() const {
    return nullptr;
}

uint8_t BLEDescriptorInstance::permissions() const {
    return 0;
}

int BLEDescriptorInstance::valueLength() const {
    return 0;
}

const void * BLEDescriptorInstance::value() const {
    return nullptr;
}

int BLEDescriptorInstance::getValue(void *value __attribute__((unused)), int length __attribute__((unused))) const {
    return 0;
}


void BLECharacteristicInstance::reference() {
}

void BLECharacteristicInstance::unreference() {
}

class BLEServerCharacteristic* BLECharacteristicInstance::server() {
    return nullptr;
}

const char *BLECharacteristicInstance::uuid() const {
    return nullptr;
}

uint8_t BLECharacteristicInstance::properties() const {
    return 0;
}

uint8_t BLECharacteristicInstance::permissions() const {
    return 0;
}

bool BLECharacteristicInstance::fixedLength() const {
    return false;
}

int BLECharacteristicInstance::valueSize() const {
    return 0;
}

int BLECharacteristicInstance::valueLength() {
    return 0;
}

const void * BLECharacteristicInstance::value() {
    return nullptr;
}

int BLECharacteristicInstance::getValue(void *value __attribute__((unused)), int length __attribute__((unused))) {
    return 0;
}

bool BLECharacteristicInstance::setValue(const void *value __attribute__((unused)), int length __attribute__((unused))) {
    return false;
}

bool BLECharacteristicInstance::writeValue(const void *value __attribute__((unused)), int length __attribute__((unused)), volatile uint8_t *p_status_return __attribute__((unused)), Callback callback __attribute__((unused))) {
    return false;
}

bool BLECharacteristicInstance::busy() {
    return false;
}

bool BLECharacteristicInstance::written() {
    return false;
}

bool BLECharacteristicInstance::subscribed() {
    return false;
}

unsigned int BLECharacteristicInstance::descriptorCount() {
    return 0;
}

BLEDescriptor BLECharacteristicInstance::descriptor(unsigned int index __attribute__((unused))) {
    return BLEDescriptor();
}

bool BLECharacteristicInstance::addDescriptor(BLEDescriptor &descriptor __attribute__((unused))) {
    return false;
}

bool BLECharacteristicInstance::removeDescriptor(BLEDescriptor &descriptor __attribute__((unused))) {
    return false;
}

void BLECharacteristicInstance::onRead(Callback callback __attribute__((unused))) {
}

void BLECharacteristicInstance::onWritten(Callback callback __attribute__((unused))) {
}

void BLECharacteristicInstance::onSubscribed(Callback callback __attribute__((unused))) {
}


void BLEServiceInstance::reference() {
}

void BLEServiceInstance::unreference() {
}

class BLEServerService* BLEServiceInstance::server() {
    return nullptr;
}

const char *BLEServiceInstance::uuid() const {
    return nullptr;
}

unsigned int BLEServiceInstance::characteristicCount() {
    return 0;
}

BLECharacteristic BLEServiceInstance::characteristic(unsigned int index __attribute__((unused))) {
    return BLECharacteristic();
}

bool BLEServiceInstance::addCharacteristic(BLECharacteristic &characteristic __attribute__((unused))) {
    return false;
}

bool BLEServiceInstance::removeCharacteristic(BLECharacteristic &characteristic __attribute__((unused))) {
    return false;
}


void BLEDeviceInstance::reference() {
}

void BLEDeviceInstance::unreference() {
}

String BLEDeviceInstance::address() const {
    return String();
}

bool BLEDeviceInstance::connected() {
    return 0;
}

int BLEDeviceInstance::rssi() {
    return 0;
}

int BLEDeviceInstance::mtu() {
    return 0;
}

bool BLEDeviceInstance::setConnectionParameters(uint16_t minimumConnectionInterval __attribute__((unused)), uint16_t maximumConnectionInterval __attribute__((unused)), uint16_t slaveLatency __attribute__((unused)), uint16_t supervisionTimeout __attribute__((unused))) {
    return false;
}

void BLEDeviceInstance::getConnectionParameters(uint16_t &connectionInterval, uint16_t &slaveLatency, uint16_t &supervisionTimeout) {
    connectionInterval = 0;
    slaveLatency = 0;
    supervisionTimeout = 0;
}

void BLEDeviceInstance::onConnectionUpdate(Callback callback __attribute__((unused))) {
}

/**********************************************************************************
 * GATT client
 */



/**********************************************************************************
 * GATT server
 */

BLEServerDescriptor::BLEServerDescriptor(const char *uuid, uint8_t permissions, uint8_t *value, int valueLength) :
    m_uuid(uuid),
    m_permissions(permissions),
    m_value(value),
    m_value_length(valueLength)
{
    m_refcount = 1;

    m_server.handle = 0;
    m_server.parent = nullptr;
    m_server.sibling = nullptr;
}

BLEServerDescriptor::~BLEServerDescriptor() {
    deallocate_noconst(const_cast<char*>(m_uuid));
    deallocate_noconst(m_value);
}

void BLEServerDescriptor::reference() {
    __armv7m_atomic_inc(&m_refcount, 0xffffffff);
}

void BLEServerDescriptor::unreference() {
    if (__armv7m_atomic_dec(&m_refcount) == 1) {
        delete this;
    }
}

class BLEServerDescriptor* BLEServerDescriptor::server() {
    return this;
}

const char *BLEServerDescriptor::uuid() const {
    return m_uuid;
}

uint8_t BLEServerDescriptor::permissions() const {
    return m_permissions;
}

int BLEServerDescriptor::valueLength() const {
    return m_value_length;
}

const void * BLEServerDescriptor::value() const {
    return m_value;
}

int BLEServerDescriptor::getValue(void *value, int length) const {
    if (length > m_value_length) {
        length = m_value_length;
    }

    memcpy(value, &m_value[0], length);

    return length;
}


BLEServerCharacteristic::BLEServerCharacteristic(const char *uuid, uint8_t properties, uint8_t permissions, uint8_t *value, int valueLength, int valueSize, bool fixedLength, bool constant) :
    m_uuid(uuid),
    m_properties(properties),
    m_permissions(permissions),
    m_constant(constant),
    m_fixed_length(fixedLength),
    m_value(value),
    m_value_length(valueLength),
    m_value_size(valueSize)
{
    m_refcount = 1;

    m_subscribed = 0;
    m_update_type = 0;
    m_update_value = false;
    m_sync_value = false;
    m_sync_subscribed = false;
    
    m_busy = 0;
    m_written = 0;

    m_done_callback = Callback(__emptyCallback);
    m_read_callback = Callback(__emptyCallback);
    m_written_callback = Callback(__wakeupCallback);
    m_subscribed_callback = Callback(__wakeupCallback);
    
    m_server.handle = 0;
    m_server.count = 0;
    m_server.parent = nullptr;
    m_server.sibling = nullptr;
    m_server.children = nullptr;

    armv7m_atomic_store((volatile uint32_t*)&m_process.request, (uint32_t)nullptr);
}

BLEServerCharacteristic::~BLEServerCharacteristic() {
    deallocate_noconst(const_cast<char*>(m_uuid));
    deallocate_noconst(const_cast<uint8_t*>(m_value));
}

void BLEServerCharacteristic::reference() {
    __armv7m_atomic_inc(&m_refcount, 0xffffffff);
}

void BLEServerCharacteristic::unreference() {
    if (__armv7m_atomic_dec(&m_refcount) == 1) {
        delete this;
    }
}

class BLEServerCharacteristic* BLEServerCharacteristic::server() {
    return this;
}
    
const char *BLEServerCharacteristic::uuid() const {
    return m_uuid;
}

uint8_t BLEServerCharacteristic::properties() const {
    return m_properties;
}

uint8_t BLEServerCharacteristic::permissions() const {
    return m_permissions;
}

bool BLEServerCharacteristic::fixedLength() const {
    return m_fixed_length;
}

int BLEServerCharacteristic::valueSize() const {
    return m_value_size;
}

int BLEServerCharacteristic::valueLength() {
    return m_value_length;
}

const void * BLEServerCharacteristic::value() {
    return m_value;
}

int BLEServerCharacteristic::getValue(void *value, int length) {
    if (length > m_value_length) {
        length = m_value_length;
    }

    memcpy(value, &m_value[0], length);

    return length;
}
  
bool BLEServerCharacteristic::setValue(const void *value, int length) {
    if (m_constant) {
        return false;
    }
    
    if (m_fixed_length) {
        if (length != m_value_size) {
            return false;
        }
    } else {
        if (length > m_value_size) {
            return false;
        }
    }

    if (armv7m_atomic_casb((volatile uint8_t*)&m_busy, 0, 1) != 0) {
        return false;
    }
    
    if (m_written & 0x8000) {
        armv7m_atomic_storeb(&m_busy, 0);
        
        return false;
    }

    m_value_length = length;
    
    memcpy(&m_value[0], value, length);

    m_update_value = true;

    armv7m_atomic_storeb(&m_busy, 0);
    
    return true;
}

bool BLEServerCharacteristic::writeValue(const void *value, int length, volatile uint8_t *p_status_return, Callback callback) {
    if (m_constant) {
        return false;
    }

    if (m_fixed_length) {
        if (length != m_value_size) {
            return false;
        }
    } else {
        if (length > m_value_size) {
            return false;
        }
    }

    if (armv7m_atomic_casb((volatile uint8_t*)&m_busy, 0, 1) != 0) {
        return false;
    }
    
    if (m_written & 0x8000) {
        armv7m_atomic_storeb(&m_busy, 0);
        
        return false;
    }

    m_value_length = length;
    
    memcpy(&m_value[0], value, length);
    
    m_update_value = true;

    m_update_type = m_subscribed;
    m_update_value = false;

    if (p_status_return) {
        *p_status_return = BLE_STATUS_BUSY;
    }
    
    m_status = p_status_return;
    m_done_callback = callback ? callback : Callback(__emptyCallback);
        
    BLEHost.request(this);

    return true;
}

bool BLEServerCharacteristic::busy() {
    return m_busy;
}

bool BLEServerCharacteristic::written() {
    if (!m_written) {
        return false;
    }

    do {
        m_value_length = armv7m_atomic_swaph(&m_written, 0x0000) & ~0x8000;
    
        memcpy(&m_value[0], (void*)&m_value[m_value_size], m_value_length);
    } while (m_written);

    return true;
}

bool BLEServerCharacteristic::subscribed() {
    return !!m_subscribed;
}
    
unsigned int BLEServerCharacteristic::descriptorCount() {
    return m_server.count;
}

BLEDescriptor BLEServerCharacteristic::descriptor(unsigned int index) {
    class BLEServerDescriptor *serverDescriptor;
    unsigned int n;
    
    if (index >= m_server.count) {
        return BLEDescriptor();
    }

    for (serverDescriptor = m_server.children, n = 0; n < index; n++) {
        serverDescriptor = serverDescriptor->m_server.sibling;
    }

    return serverDescriptor->descriptor();
}

bool BLEServerCharacteristic::addDescriptor(BLEDescriptor &descriptor) {
    class BLEServerDescriptor *serverDescriptor, *currentDescriptor, **previousDescriptor;

    if (m_server.parent) {
        return false;
    }
    
    serverDescriptor = BLEDescriptorInstance::instance(descriptor)->server();

    if (!serverDescriptor) {
        return false;
    }
    
    if (serverDescriptor->m_server.parent) {
        return false;
    }

    serverDescriptor->reference();
    
    serverDescriptor->m_server.parent = this;

    for (previousDescriptor = &m_server.children, currentDescriptor = *previousDescriptor;
         currentDescriptor;
         previousDescriptor = &currentDescriptor->m_server.sibling, currentDescriptor = *previousDescriptor)
    {
    }

    *previousDescriptor = serverDescriptor;

    m_server.count++;
    
    return true;
}

bool BLEServerCharacteristic::removeDescriptor(BLEDescriptor &descriptor) {
    class BLEServerDescriptor *serverDescriptor, *currentDescriptor, **previousDescriptor;

    if (m_server.parent) {
        return false;
    }
    
    serverDescriptor = BLEDescriptorInstance::instance(descriptor)->server();

    if (!serverDescriptor) {
        return false;
    }
    
    if (serverDescriptor->m_server.parent != this) {
        return false;
    }

    m_server.count--;

    for (previousDescriptor = &m_server.children, currentDescriptor = *previousDescriptor;
         currentDescriptor;
         previousDescriptor = &currentDescriptor->m_server.sibling, currentDescriptor = *previousDescriptor)
    {
        if (currentDescriptor == serverDescriptor) {
            *previousDescriptor = serverDescriptor->m_server.sibling;
            break;
        }
    }

    serverDescriptor->m_server.parent = NULL;
        
    serverDescriptor->unreference();

    return true;
}

void BLEServerCharacteristic::onRead(Callback callback) {
    m_read_callback = (callback ? callback : Callback(__emptyCallback));
}

void BLEServerCharacteristic::onWritten(Callback callback) {
    m_written_callback = (callback ? callback : Callback(__wakeupCallback));
}

void BLEServerCharacteristic::onSubscribed(Callback callback) {
    m_subscribed_callback = (callback ? callback : Callback(__wakeupCallback));
}


BLEServerService::BLEServerService(const char *uuid) :
    m_uuid(uuid)
{
    m_refcount = 1;

    m_server.handle = 0;
    m_server.count = 0;
    m_server.attrib[0] = 0;
    m_server.attrib[1] = 0;
    m_server.sibling = nullptr;
    m_server.children = nullptr;
}

BLEServerService::~BLEServerService() {
    deallocate_noconst(const_cast<char*>(m_uuid));
}

void BLEServerService::reference() {
    __armv7m_atomic_inc(&m_refcount, 0xffffffff);
}

void BLEServerService::unreference() {
    if (__armv7m_atomic_dec(&m_refcount) == 1) {
        delete this;
    }
}

class BLEServerService* BLEServerService::server() {
    return this;
}

const char *BLEServerService::uuid() const {
    return m_uuid;
}

unsigned int BLEServerService::characteristicCount() {
    return m_server.count;
}

BLECharacteristic BLEServerService::characteristic(unsigned int index) {
    class BLEServerCharacteristic *serverCharacteristic;
    unsigned int n;
    
    if (index >= m_server.count) {
        return BLECharacteristic();
    }

    for (serverCharacteristic = m_server.children, n = 0; n < index; n++) {
        serverCharacteristic = serverCharacteristic->m_server.sibling;
    }

    return serverCharacteristic->characteristic();
}

bool BLEServerService::addCharacteristic(BLECharacteristic &characteristic) {
    class BLEServerCharacteristic *serverCharacteristic, *currentCharacteristic, **previousCharacteristic;

    if (m_server.attrib[0]) {
        return false;
    }
    
    serverCharacteristic = BLECharacteristicInstance::instance(characteristic)->server();

    if (!serverCharacteristic) {
        return false;
    }
    
    if (serverCharacteristic->m_server.parent) {
        return false;
    }

    serverCharacteristic->reference();

    serverCharacteristic->m_server.parent = this;

    for (previousCharacteristic = &m_server.children, currentCharacteristic = *previousCharacteristic;
         currentCharacteristic;
         previousCharacteristic = &currentCharacteristic->m_server.sibling, currentCharacteristic = *previousCharacteristic)
    {
    }

    *previousCharacteristic = serverCharacteristic;

    m_server.count++;

    return true;
}

bool BLEServerService::removeCharacteristic(BLECharacteristic &characteristic) {
    class BLEServerCharacteristic *serverCharacteristic, *currentCharacteristic, **previousCharacteristic;

    if (m_server.attrib[0]) {
        return false;
    }
    
    serverCharacteristic = BLECharacteristicInstance::instance(characteristic)->server();

    if (!serverCharacteristic) {
        return false;
    }
    
    if (serverCharacteristic->m_server.parent != this) {
        return false;
    }

    m_server.count--;

    for (previousCharacteristic = &m_server.children, currentCharacteristic = *previousCharacteristic;
         currentCharacteristic;
         previousCharacteristic = &currentCharacteristic->m_server.sibling, currentCharacteristic = *previousCharacteristic)
    {
        if (currentCharacteristic == serverCharacteristic) {
            *previousCharacteristic = serverCharacteristic->m_server.sibling;
            break;
        }
    }

    serverCharacteristic->m_server.parent = NULL;
        
    serverCharacteristic->unreference();

    return true;
}


/**********************************************************************************
 * GAP peer
 */

BLEPeerDevice::BLEPeerDevice() {
    m_refcount = 1;

    m_handle = BLE_PEER_DEVICE_NOT_CONNECTED;
    m_address = BLEAddress();
    m_mtu = ATT_MTU;
    m_connection_interval = 0;
    m_slave_latency = 0;
    m_supervision_timeout = 0;

    m_connection_update_callback = Callback(__wakeupCallback);
}

BLEPeerDevice::BLEPeerDevice(uint16_t handle, const uint8_t *address, uint8_t type, uint16_t connectionInterval, uint16_t slaveLatency, uint16_t supervisionTimeout) {
    m_refcount = 1;

    m_handle = handle;
    m_address = BLEAddress(address, (BLEAddressType)type);
    m_mtu = ATT_MTU;
    m_connection_interval = connectionInterval;
    m_slave_latency = slaveLatency;
    m_supervision_timeout = supervisionTimeout;

    m_connection_update_callback = Callback(__wakeupCallback);
}

BLEPeerDevice::~BLEPeerDevice() {
}

void BLEPeerDevice::reference() {
    __armv7m_atomic_inc(&m_refcount, 0xffffffff);
}

void BLEPeerDevice::unreference() {
    if (__armv7m_atomic_dec(&m_refcount) == 1) {
        delete this;
    }
}

String BLEPeerDevice::address() const {
    return m_address.toString();
}

bool BLEPeerDevice::connected() {
    return (m_handle != BLE_PEER_DEVICE_NOT_CONNECTED);
}

int BLEPeerDevice::rssi() {
    tBleStatus status;
    int8_t rssi;

    status = hci_read_rssi(m_handle, (uint8_t*)&rssi);

    if ((status == BLE_STATUS_SUCCESS) && (rssi <= 0)) {
        return rssi;
    }

    return 0;
}

int BLEPeerDevice::mtu() {
    return m_mtu;
}

bool BLEPeerDevice::setConnectionParameters(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval, uint16_t slaveLatency, uint16_t supervisionTimeout) {
    if (minimumConnectionInterval > maximumConnectionInterval) {
        return false;
    }
    
    if ((minimumConnectionInterval < 0x0006) || (minimumConnectionInterval > 0x0c80)) {
        return false;
    }

    if ((maximumConnectionInterval < 0x0006) || (maximumConnectionInterval > 0x0c80)) {
        return false;
    }

    if (slaveLatency > 0x01f3) {
        return false;
    }
    
    if ((supervisionTimeout < 0x000a) || (supervisionTimeout > 0x0c80)) {
        return false;
    }

    BLEHost.connection(minimumConnectionInterval, maximumConnectionInterval, slaveLatency, supervisionTimeout);

    return true;
}

void BLEPeerDevice::getConnectionParameters(uint16_t &connectionInterval, uint16_t &slaveLatency, uint16_t &supervisionTimeout) {
    connectionInterval = m_connection_interval;
    slaveLatency = m_slave_latency;
    supervisionTimeout = m_supervision_timeout;
}

void BLEPeerDevice::onConnectionUpdate(Callback callback) {
    m_connection_update_callback = (callback ? callback : Callback(__wakeupCallback));
}


/**********************************************************************************
 * GAP host
 */

static const uint8_t ER[16] = { 0xfe,0xdc,0xba,0x09,0x87,0x65,0x43,0x21,0xfe,0xdc,0xba,0x09,0x87,0x65,0x43,0x21 };
static const uint8_t IR[16] = { 0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xf0,0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xf0 };

int hci_send_req(stm32wb_ipcc_ble_command_t *command, bool async) {
    command->rsize = command->rlen;

    if (!stm32wb_ipcc_ble_command(command)) {
        return BLE_STATUS_ERROR;
    }

    if (async) {
        return BLE_STATUS_SUCCESS;
    }

    while (command->status == STM32WB_IPCC_BLE_STATUS_BUSY) {
        __WFE();
    }

    return (command->status == STM32WB_IPCC_BLE_STATUS_SUCCESS) ? BLE_STATUS_SUCCESS : BLE_STATUS_TIMEOUT;
}

BLELocalDevice::BLELocalDevice() {
    m_reset = true;

    m_address = BLEAddress();
    
    m_device_name = NULL;
    m_device_name_length = BLE_MIN_DEVICE_NAME_LENGTH;
    m_appearance = 0;
    m_ppcp.connection_interval_min = 0x0010; /* 20ms */
    m_ppcp.connection_interval_max = 0x0020; /* 40ms */
    m_ppcp.slave_latency = 0x0000;
    m_ppcp.supervision_timeout = 0x00c8; /* 2000ms */
    m_pa_level = 0x19;
    m_tx_power_level = 0;
    m_tx_phy = BLE_PHY_1M;
    m_rx_phy = BLE_PHY_1M;
    m_phy_options = 0;

    m_stop_callback = Callback(__wakeupCallback);
    m_connect_callback = Callback(__wakeupCallback);
    m_disconnect_callback = Callback(__wakeupCallback);

    m_server.count = 0;
    m_server.children = nullptr;

    m_peripheral.advertising = false;
    m_peripheral.bonding = false;
    m_peripheral.security = BLE_SECURITY_UNENCRYPTED;
    m_peripheral.fixed_pin = BLE_FIXED_PIN_UNDEFINED;
    m_peripheral.connectable = true;
    m_peripheral.discoverable = BLE_DISCOVERABLE_GENERAL;
    m_peripheral.advertising_interval_min = 0x0080; /*  80ms */
    m_peripheral.advertising_interval_max = 0x00a0; /* 100ms */

    m_advertising.data_length = 3;
    m_advertising.local_name_length = 0;
    m_advertising.service_uuids_length = 0;
    m_advertising.connection_interval_length = 0;
    m_advertising.tx_power_level_length = 0;
    m_advertising.manufacturer_data_length = 0;
    m_advertising.custom_length = 0;
    m_advertising.scan_response_data_length = 0;

    Callback done_callback = Callback(&BLELocalDevice::done, this);
    
    m_process.command.next = NULL;
    m_process.command.opcode = 0;
    m_process.command.event = 0;
    m_process.command.cparam = &m_process.data[0];
    m_process.command.rparam = &m_process.data[0];
    m_process.command.rsize = 255;
    m_process.command.status = STM32WB_IPCC_BLE_STATUS_SUCCESS;
    m_process.command.callback = done_callback.callback();
    m_process.command.context = done_callback.context();
    
    Callback process_callback = Callback(&BLELocalDevice::process, this);

    k_work_create(&m_process.work, process_callback.callback(), process_callback.context());
}

bool BLELocalDevice::begin(int mtu, uint32_t options) {
    tBleStatus status = BLE_STATUS_SUCCESS;
    uint16_t maxTxOctets, maxTxTime, maxRxOctets, maxRxTime;
    
    if (!m_reset) {
        return false;
    }
    
    if ((mtu < 23) || (mtu > 247)) {
        return false;
    }
    
    stm32wb_ipcc_ble_init_params_t init_params = {
        NULL,                                                                                          /* pBleBufferAddress          */
        0,                                                                                             /* BleBufferSize              */
        BLE_NUM_GATT_ATTRIBUTES,                                                                       /* NumAttrRecord              */
        BLE_NUM_GATT_SERVICES,                                                                         /* NumAttrServ                */
        BLE_ATT_VALUE_ARRAY_SIZE,                                                                      /* AttrValueArrSize           */
        BLE_NUM_LINK,                                                                                  /* NumOfLinks                 */
        BLE_DATA_LENGTH_EXTENSION,                                                                     /* ExtendedPacketLengthEnable */
        BLE_PREP_WRITE_X_ATT(BLE_DEFAULT_MAX_ATT_SIZE),                                                /* PrWriteListSize            */
        (uint8_t)BLE_MBLOCKS_CALC(BLE_PREP_WRITE_X_ATT(BLE_DEFAULT_MAX_ATT_SIZE), mtu, BLE_NUM_LINK),  /* MblockCount                */
        (uint8_t)mtu,                                                                                  /* AttMtu                     */
        BLE_SLAVE_SCA,                                                                                 /* SlaveSca                   */
        BLE_MASTER_SCA,                                                                                /* MasterSca                  */
        BLE_LSE_SOURCE,                                                                                /* LsSource                   */
        BLE_MAX_CONN_EVENT_LENGTH,                                                                     /* MaxConnEventLength         */
        BLE_HSE_STARTUP_TIME,                                                                          /* HsStartupTime              */
        BLE_VITERBI_MODE,                                                                              /* ViterbiEnable              */
        (uint8_t)(((options & BLE_OPTION_NO_SERVICE_CHANGED) ? BLE_OPTION_NO_SERVICE_CHANGED : 0) |    /* Options                    */
                  ((options & BLE_OPTION_READ_ONLY_DEVICE_NAME) ? BLE_OPTION_READ_ONLY_DEVICE_NAME : 0)),
        0,                                                                                             /* HwVersion                  */
    };

    if (!stm32wb_ipcc_sys_enable()) {
        return false;
    }

    while (stm32wb_ipcc_sys_state() == STM32WB_IPCC_SYS_STATE_NONE) {
    }
            
    if (stm32wb_ipcc_sys_state() == STM32WB_IPCC_SYS_STATE_WIRELESS) {
        Callback event_callback = Callback(&BLELocalDevice::event, this);
        
        if (!stm32wb_ipcc_ble_enable(&init_params, (stm32wb_ipcc_ble_event_callback_t)event_callback.callback(), event_callback.context())) {
            stm32wb_ipcc_sys_disable();

            return false;
        }
    } else {
        stm32wb_ipcc_sys_disable();

        return false;
    }
    
    status = hci_reset();
    
    if (status == BLE_STATUS_SUCCESS) {
        uint8_t address[BLE_ADDRESS_LENGTH];

        address[0] = *((const uint8_t*)(UID64_BASE + 0));
        address[1] = *((const uint8_t*)(UID64_BASE + 1));
        address[2] = *((const uint8_t*)(UID64_BASE + 2));
        address[3] = 0x26;
        address[4] = 0xe1;
        address[5] = 0x80;

        if (!(options & BLE_OPTION_RANDOM_STATIC_ADDRESS)) {
            m_address = BLEAddress(address, BLEAddressType::PUBLIC);
        }
        
        status = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, address);
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        status = aci_hal_write_config_data(CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, &ER[0]);
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        status = aci_hal_write_config_data(CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, &IR[0]);
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        uint8_t address[BLE_ADDRESS_LENGTH];
        volatile uint8_t busy;

        stm32wb_random(&address[0], sizeof(address), &busy, NULL, NULL);
        
        while (busy == STM32WB_RANDOM_STATUS_BUSY) {
            __WFE();
        }

        address[5] |= 0xc0;
        
        if (options & BLE_OPTION_RANDOM_STATIC_ADDRESS) {
            m_address = BLEAddress(address, BLEAddressType::RANDOM_STATIC);
        }
        
        status = aci_hal_write_config_data(CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, address);
    }

    if (status == BLE_STATUS_SUCCESS) {
        status = aci_gatt_init();
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        if (m_device_name) {
            m_device_name_length = strlen(m_device_name);
            
            if (m_device_name_length < BLE_MIN_DEVICE_NAME_LENGTH) {
                m_device_name_length = BLE_MIN_DEVICE_NAME_LENGTH;
            }

            if (m_device_name_length > BLE_MAX_DEVICE_NAME_LENGTH) {
                m_device_name_length = BLE_MAX_DEVICE_NAME_LENGTH;
            }
        }
        
        status = aci_gap_init(GAP_PERIPHERAL_ROLE,
                              ((m_options & BLE_OPTION_PRIVACY) ? PRIVACY_ENABLED : PRIVACY_DISABLED),
                              m_device_name_length,
                              &m_gap_service_handle,
                              &m_device_name_handle,
                              &m_appearance_handle);
        
        if (status == BLE_STATUS_SUCCESS) {
            m_ppcp_handle = m_appearance_handle + 2;
        }
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        status = aci_gatt_update_char_value(m_gap_service_handle,
                                            m_device_name_handle,
                                            0,
                                            (m_device_name ? strlen(m_device_name) : 0),
                                            reinterpret_cast<const uint8_t*>(m_device_name));
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        status = aci_gatt_update_char_value(m_gap_service_handle,
                                            m_appearance_handle,
                                            0,
                                            2,
                                            reinterpret_cast<const uint8_t*>(&m_appearance));
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        status = aci_gatt_update_char_value(m_gap_service_handle,
                                            m_ppcp_handle,
                                            0,
                                            8,
                                            reinterpret_cast<const uint8_t*>(&m_ppcp));
    }

    if (status == BLE_STATUS_SUCCESS) {
        status = hci_le_set_default_phy(0, m_tx_phy, m_rx_phy);
    }

    if (status == BLE_STATUS_SUCCESS) {
        status = hci_le_read_maximum_data_length(&maxTxOctets, &maxTxTime, &maxRxOctets, &maxRxTime);
    }

    if (status == BLE_STATUS_SUCCESS) {
        if (mtu < (maxTxOctets - 4)) {
            maxTxOctets = mtu + 4;
            maxTxTime = (maxTxOctets + 14) * 8; 
        }
        
        status = hci_le_write_suggested_default_data_length(maxTxOctets, maxTxTime);
    }

    if (status == BLE_STATUS_SUCCESS) {
        stm32wb_system_smps_configure(m_pa_level);

        status = aci_hal_set_tx_power_level(0, m_pa_level);
    }
    
    if (status != BLE_STATUS_SUCCESS) {
        aci_hal_stack_reset();

        stm32wb_ipcc_ble_disable();
        stm32wb_ipcc_sys_disable();

        return false;
    }

    m_reset = false;
    m_mtu = mtu;
    m_options = options;

    return true;
}

void BLELocalDevice::end() {
    if (!m_reset) {
        m_reset = true;

        aci_hal_stack_reset();

        stm32wb_ipcc_ble_disable();
        stm32wb_ipcc_sys_disable();
        
        reset();
    }
}

String BLELocalDevice::address() const {
    return m_address.toString();
}

bool BLELocalDevice::setTxPowerLevel(int txPower) {
    uint32_t advertising_data_offset;
    uint8_t pa_level, tx_power_level; 
    tBleStatus status;

    if (txPower > 6) {
        return false;
    }
    
    if      (txPower >=   6) { pa_level = 0x1f; tx_power_level =   6; }
    else if (txPower >=   5) { pa_level = 0x1e; tx_power_level =   5; }
    else if (txPower >=   4) { pa_level = 0x1d; tx_power_level =   4; }
    else if (txPower >=   3) { pa_level = 0x1c; tx_power_level =   3; }
    else if (txPower >=   2) { pa_level = 0x1b; tx_power_level =   2; }
    else if (txPower >=   1) { pa_level = 0x1a; tx_power_level =   1; }
    else if (txPower >=   0) { pa_level = 0x19; tx_power_level =   0; }
    else if (txPower >=  -1) { pa_level = 0x16; tx_power_level =  -1; }
    else if (txPower >=  -2) { pa_level = 0x14; tx_power_level =  -2; }
    else if (txPower >=  -3) { pa_level = 0x12; tx_power_level =  -3; }
    else if (txPower >=  -4) { pa_level = 0x11; tx_power_level =  -4; }
    else if (txPower >=  -5) { pa_level = 0x10; tx_power_level =  -5; }
    else if (txPower >=  -6) { pa_level = 0x0f; tx_power_level =  -6; }
    else if (txPower >=  -7) { pa_level = 0x0e; tx_power_level =  -7; }
    else if (txPower >=  -8) { pa_level = 0x0d; tx_power_level =  -8; }
    else if (txPower >=  -9) { pa_level = 0x0c; tx_power_level =  -9; }
    else if (txPower >= -10) { pa_level = 0x0b; tx_power_level = -10; }
    else if (txPower >= -11) { pa_level = 0x0a; tx_power_level = -11; }
    else if (txPower >= -12) { pa_level = 0x09; tx_power_level = -12; }
    else if (txPower >= -13) { pa_level = 0x08; tx_power_level = -13; }
    else if (txPower >= -14) { pa_level = 0x07; tx_power_level = -14; }
    else if (txPower >= -15) { pa_level = 0x06; tx_power_level = -15; }
    else if (txPower >= -16) { pa_level = 0x05; tx_power_level = -16; }
    else if (txPower >= -17) { pa_level = 0x04; tx_power_level = -17; }
    else if (txPower >= -18) { pa_level = 0x03; tx_power_level = -18; }
    else if (txPower >= -19) { pa_level = 0x02; tx_power_level = -19; }
    else if (txPower >= -20) { pa_level = 0x01; tx_power_level = -20; }
    else                     { pa_level = 0x00; tx_power_level = -40; }

    if (!m_reset) {
        if (pa_level > m_pa_level) {
            stm32wb_system_smps_configure(pa_level);
        }

        status = aci_hal_set_tx_power_level(0, m_pa_level);

        if (status == BLE_STATUS_SUCCESS) {
            if (pa_level < m_pa_level) {
                stm32wb_system_smps_configure(pa_level);
            }

            m_pa_level = pa_level;
            m_tx_power_level = tx_power_level;

            if (m_advertising.tx_power_level_length) {
                advertising_data_offset = 3 + m_advertising.local_name_length + m_advertising.service_uuids_length + m_advertising.connection_interval_length;

                m_advertising.data[advertising_data_offset+2] = m_tx_power_level;
                
                if (m_peripheral.advertising) {
                    status = hci_le_set_advertising_data(m_advertising.data_length, m_advertising.data);
                }
            }
        }

        if (status != BLE_STATUS_SUCCESS) {
            return false;
        }
    } else {
        m_pa_level = pa_level;
        m_tx_power_level = tx_power_level;
    }

    return true;
}

bool BLELocalDevice::setPreferredPhy(BLEPhy txPhy, BLEPhy rxPhy, BLEPhyOption phyOptions) {
    tBleStatus status;

    if ((txPhy == BLE_PHY_CODED) || (rxPhy == BLE_PHY_CODED)) {
        return false;
    }

    m_tx_phy = txPhy;
    m_rx_phy = rxPhy;
    m_phy_options = phyOptions;

    if (!m_reset) {
        status = hci_le_set_default_phy(0, m_tx_phy, m_rx_phy);

        if (status != BLE_STATUS_SUCCESS) {
            return false;
        }
    }
    
    return true;
}

void BLELocalDevice::setDeviceName(const char *deviceName) {
    uint32_t length;
    
    if (m_device_name) {
        deallocate_noconst(m_device_name);

        m_device_name = NULL;
    }

    if (deviceName) {
        length = strlen(deviceName);

        m_device_name = (const char*)allocate_noconst(deviceName, length +1);
    }

    if (!m_device_name) {
        length = 0;
    }

    if (!m_reset) {
        if (length > m_device_name_length) {
            length = m_device_name_length;
        }

        aci_gatt_update_char_value(m_gap_service_handle,
                                   m_device_name_handle,
                                   0,
                                   length,
                                   reinterpret_cast<const uint8_t*>(m_device_name));
    }
}

void BLELocalDevice::setAppearance(enum BLEAppearance appearance) {
    m_appearance = (uint16_t)appearance;

    if (!m_reset) {
        aci_gatt_update_char_value(m_gap_service_handle,
                                   m_appearance_handle,
                                   0,
                                   2,
                                   reinterpret_cast<const uint8_t*>(&m_appearance));
    }
}

bool BLELocalDevice::setConnectionParameters(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval, uint16_t slaveLatency, uint16_t supervisionTimeout) {
    uint32_t advertising_data_offset;
    tBleStatus status;
    
    if (minimumConnectionInterval > maximumConnectionInterval) {
        return false;
    }
    
    if ((minimumConnectionInterval < 0x0006) || (minimumConnectionInterval > 0x0c80)) {
        return false;
    }

    if ((maximumConnectionInterval < 0x0006) || (maximumConnectionInterval > 0x0c80)) {
        return false;
    }

    if (slaveLatency > 0x01f3) {
        return false;
    }
    
    if ((supervisionTimeout < 0x000a) || (supervisionTimeout > 0x0c80)) {
        return false;
    }

    m_ppcp.connection_interval_min = minimumConnectionInterval;
    m_ppcp.connection_interval_max = maximumConnectionInterval;
    m_ppcp.slave_latency = slaveLatency;
    m_ppcp.supervision_timeout = supervisionTimeout;

    if (!m_reset) {
        status = aci_gatt_update_char_value(m_gap_service_handle,
                                            m_ppcp_handle,
                                            0,
                                            8,
                                            reinterpret_cast<const uint8_t*>(&m_ppcp));

        if (status == BLE_STATUS_SUCCESS) {
            if (m_advertising.connection_interval_length) {
                advertising_data_offset = 3 + m_advertising.local_name_length + m_advertising.service_uuids_length;

                m_advertising.data[advertising_data_offset+2] = (uint8_t)(m_ppcp.connection_interval_min >> 0);
                m_advertising.data[advertising_data_offset+3] = (uint8_t)(m_ppcp.connection_interval_min >> 8);
                m_advertising.data[advertising_data_offset+4] = (uint8_t)(m_ppcp.connection_interval_max >> 0);
                m_advertising.data[advertising_data_offset+5] = (uint8_t)(m_ppcp.connection_interval_max >> 8);
                
                if (m_peripheral.advertising) {
                    status = hci_le_set_advertising_data(m_advertising.data_length, m_advertising.data);
                }
            }
        }
        
        if (status != BLE_STATUS_SUCCESS) {
            return false;
        }
    }

    return true;
}

unsigned int BLELocalDevice::serviceCount() {
    return m_server.count;
}

BLEService BLELocalDevice::service(unsigned int index) {
    BLEServerService *serverService;
    unsigned int n;
    
    if (index >= m_server.count)
    {
        return BLEService();
    }

    for (serverService = m_server.children, n = 0; n < index; n++) {
        serverService = serverService->m_server.sibling;
    }

    return serverService->service();
}

bool BLELocalDevice::addService(BLEService &service) {
    BLEServerService *serverService, *currentService, **previousService;
    BLEServerCharacteristic *characteristic;
    BLEServerDescriptor *descriptor;
    BLESecurity security, security_read, security_write, security_subscribe;

    if (m_process.published && (m_options & BLE_OPTION_NO_SERVICE_CHANGED)) {
        return false;
    }
    
    serverService = BLEServiceInstance::instance(service)->server();

    if (!serverService) {
        return false;
    }
    
    if (serverService->m_server.attrib[0]) {
        return false;
    }

    serverService->reference();

    serverService->m_server.attrib[0] = 0xffff;
    serverService->m_server.attrib[1] = 0x0000;

    for (previousService = &m_server.children, currentService = *previousService;
         currentService;
         previousService = &currentService->m_server.sibling, currentService = *previousService)
    {
    }

    *previousService = serverService;

    for (security = BLE_SECURITY_UNENCRYPTED, serverService = m_server.children; serverService; serverService = serverService->m_server.sibling) {
        for (characteristic = serverService->m_server.children; characteristic; characteristic = characteristic->m_server.sibling) {
            security_read = (BLESecurity)((characteristic->m_permissions >> 0) & 3);
            security_write = (BLESecurity)((characteristic->m_permissions >> 2) & 3);
            security_subscribe = (BLESecurity)((characteristic->m_permissions >> 4) & 3);
            
            if (characteristic->m_properties & (BLE_PROPERTY_READ)) {
                if (security < security_read) {
                    security = security_read;
                }
            }
            
            if (characteristic->m_properties & (BLE_PROPERTY_WRITE_WITHOUT_RESPONSE | BLE_PROPERTY_WRITE)) {
                if (security < security_write) {
                    security = security_write;
                }
            }
            
            if (characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) {
                if (security < security_subscribe) {
                    security = security_subscribe;
                }
            }
            
            for (descriptor = characteristic->m_server.children; descriptor; descriptor = descriptor->m_server.sibling) {
                security_read = (BLESecurity)((descriptor->m_permissions >> 0) & 3);

                if (security < security_read) {
                    security = security_read;
                }
            }
        }
        
    }

    m_server.security = security;
    m_server.count++;
    
    if (m_process.published) {
        if (!insert(serverService)) {
            removeService(service);
            return false;
        }
    }

    return true;
}

bool BLELocalDevice::removeService(BLEService &service) {
    class BLEServerService *serverService, *currentService, **previousService;
    BLEServerCharacteristic *characteristic;
    BLEServerDescriptor *descriptor;
    BLESecurity security, security_read, security_write, security_subscribe;

    if (m_process.published && (m_options & BLE_OPTION_NO_SERVICE_CHANGED)) {
        return false;
    }

    serverService = BLEServiceInstance::instance(service)->server();

    if (!serverService) {
        return false;
    }

    for (previousService = &m_server.children, currentService = *previousService;
         currentService;
         previousService = &currentService->m_server.sibling, currentService = *previousService)
    {
        if (currentService == serverService) {
            *previousService = serverService->m_server.sibling;
            break;
        }
    }

    if (!currentService) {
        return false;
    }
    
    if (m_process.published) {
        if (serverService->m_server.handle) {
            remove(serverService);
        }
    }
    
    for (security = BLE_SECURITY_UNENCRYPTED, currentService = m_server.children; currentService; currentService = currentService->m_server.sibling) {
        for (characteristic = currentService->m_server.children; characteristic; characteristic = characteristic->m_server.sibling) {
            security_read = (BLESecurity)((characteristic->m_permissions >> 0) & 3);
            security_write = (BLESecurity)((characteristic->m_permissions >> 2) & 3);
            security_subscribe = (BLESecurity)((characteristic->m_permissions >> 4) & 3);
            
            if (characteristic->m_properties & (BLE_PROPERTY_READ)) {
                if (security < security_read) {
                    security = security_read;
                }
            }
            
            if (characteristic->m_properties & (BLE_PROPERTY_WRITE_WITHOUT_RESPONSE | BLE_PROPERTY_WRITE)) {
                if (security < security_write) {
                    security = security_write;
                }
            }
            
            if (characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) {
                if (security < security_subscribe) {
                    security = security_subscribe;
                }
            }
            
            for (descriptor = characteristic->m_server.children; descriptor; descriptor = descriptor->m_server.sibling) {
                security_read = (BLESecurity)((descriptor->m_permissions >> 0) & 3);

                if (security < security_read) {
                    security = security_read;
                }
            }
        }
    }

    m_server.security = security;
    m_server.count--;

    serverService->m_server.attrib[0] = 0x0000;
    serverService->m_server.attrib[1] = 0x0000;
        
    serverService->unreference();

    return true;
}

bool BLELocalDevice::setSecurity(BLESecurity security) {
    m_peripheral.security = security;

    return true;
}

bool BLELocalDevice::setBonding(bool bonding) {
    m_peripheral.bonding = bonding;

    return true;
}

bool BLELocalDevice::setPin(const char *pin) {
    uint32_t fixed_pin, index;

    if (!pin) {
        m_peripheral.fixed_pin = BLE_FIXED_PIN_UNDEFINED;
    } else {
        if (strlen(pin) != 6) {
            return false;
        }

        for (fixed_pin =0, index = 0; index < 6; index++) {
            if ((pin[index] < '0') || (pin[index] > '9')) {
                return false;
            }
            
            fixed_pin = (fixed_pin * 10) + (pin[index] - '0');
        }

        m_peripheral.fixed_pin = fixed_pin;
    }
    
    return true;
}

void BLELocalDevice::clearBondStorage(void) {
    aci_gap_clear_security_db();
}

bool BLELocalDevice::setIncludeTxPowerLevel(bool enable) {
    uint32_t advertising_data_offset, length;

    if (m_advertising.custom_length) {
        return false;
    }
    
    if (enable) {
        length = 3;
        
        if (((m_advertising.data_length - m_advertising.tx_power_level_length) + length) > (int)(MAX_ADV_DATA_LEN)) {
            return false;
        }
    }

    advertising_data_offset = 3 + m_advertising.local_name_length + m_advertising.service_uuids_length + m_advertising.connection_interval_length;

    if (m_advertising.tx_power_level_length) {
        memmove(&m_advertising.data[advertising_data_offset],
                &m_advertising.data[advertising_data_offset + m_advertising.tx_power_level_length],
                ((m_advertising.data_length - advertising_data_offset) - m_advertising.tx_power_level_length));

        m_advertising.data_length -= m_advertising.tx_power_level_length;
        m_advertising.tx_power_level_length = 0;
    }

    if (enable) {
        memmove(&m_advertising.data[advertising_data_offset + length],
                &m_advertising.data[advertising_data_offset],
                (m_advertising.data_length - advertising_data_offset));
        
        m_advertising.data[advertising_data_offset++] = length -1;
        m_advertising.data[advertising_data_offset++] = BLE_AD_TYPE_TX_POWER_LEVEL;
        m_advertising.data[advertising_data_offset++] = m_tx_power_level;

        m_advertising.data_length += length;
        m_advertising.tx_power_level_length = length;
    }

    if (m_peripheral.advertising) {
        if (hci_le_set_advertising_data(m_advertising.data_length, m_advertising.data) != BLE_STATUS_SUCCESS) {
            return false;
        }
    }
    
    return true;
}

bool BLELocalDevice::setIncludeConnectionInterval(bool enable) {
    uint32_t advertising_data_offset, length;

    if (m_advertising.custom_length) {
        return false;
    }

    if (enable) {
        length = 6;
        
        if (((m_advertising.data_length - m_advertising.connection_interval_length) + length) > (int)(MAX_ADV_DATA_LEN)) {
            return false;
        }
    } else {
        length = 0;
    }
            

    advertising_data_offset = 3 + m_advertising.local_name_length + m_advertising.service_uuids_length;

    if (m_advertising.connection_interval_length) {
        memmove(&m_advertising.data[advertising_data_offset],
                &m_advertising.data[advertising_data_offset + m_advertising.connection_interval_length],
                ((m_advertising.data_length - advertising_data_offset) - m_advertising.connection_interval_length));

        m_advertising.data_length -= m_advertising.connection_interval_length;
        m_advertising.connection_interval_length = 0;
    }

    if (length) {
        memmove(&m_advertising.data[advertising_data_offset + length],
                &m_advertising.data[advertising_data_offset],
                (m_advertising.data_length - advertising_data_offset));
        
        m_advertising.data[advertising_data_offset++] = length -1;
        m_advertising.data[advertising_data_offset++] = BLE_AD_TYPE_SLAVE_CONN_INTERVAL;
        m_advertising.data[advertising_data_offset++] = (uint8_t)(m_ppcp.connection_interval_min >> 0);
        m_advertising.data[advertising_data_offset++] = (uint8_t)(m_ppcp.connection_interval_min >> 8);
        m_advertising.data[advertising_data_offset++] = (uint8_t)(m_ppcp.connection_interval_max >> 0);
        m_advertising.data[advertising_data_offset++] = (uint8_t)(m_ppcp.connection_interval_max >> 8);

        m_advertising.data_length += length;
        m_advertising.connection_interval_length = length;
    }

    if (m_peripheral.advertising) {
        if (hci_le_set_advertising_data(m_advertising.data_length, m_advertising.data) != BLE_STATUS_SUCCESS) {
            return false;
        }
    }
    
    return true;
}

bool BLELocalDevice::setLocalName(const char *localName) {
    uint32_t advertising_data_offset, length;

    if (m_advertising.custom_length) {
        return false;
    }
    
    if (localName) {
        length = 2 + strlen(localName);

        if (((m_advertising.data_length - m_advertising.local_name_length) + length) > (int)(MAX_ADV_DATA_LEN)) {
            return false;
        }
    } else {
        length = 0;
    }

    advertising_data_offset = 3;

    if (m_advertising.local_name_length) {
        memmove(&m_advertising.data[advertising_data_offset],
                &m_advertising.data[advertising_data_offset + m_advertising.local_name_length],
                ((m_advertising.data_length - advertising_data_offset) - m_advertising.local_name_length));

        m_advertising.data_length -= m_advertising.local_name_length;
        m_advertising.local_name_length = 0;
    }

    if (length) {
        memmove(&m_advertising.data[advertising_data_offset + length],
                &m_advertising.data[advertising_data_offset],
                (m_advertising.data_length - advertising_data_offset));
        
        m_advertising.data[advertising_data_offset++] = length -1;
        m_advertising.data[advertising_data_offset++] = BLE_AD_TYPE_COMPLETE_LOCAL_NAME;
        memcpy(&m_advertising.data[advertising_data_offset], localName, length -2);

        m_advertising.data_length += length;
        m_advertising.local_name_length = length;
    }

    if (m_peripheral.advertising) {
        if (hci_le_set_advertising_data(m_advertising.data_length, m_advertising.data) != BLE_STATUS_SUCCESS) {
            return false;
        }
    }

    return false;
}

bool BLELocalDevice::setAdvertisedServiceUuids(const char *uuids[], int count, bool complete) {
    uint32_t uuid16_data_offset, uuid128_data_offset, advertising_data_offset, length;
    uint8_t uuid16_data[MAX_ADV_DATA_LEN], uuid128_data[MAX_ADV_DATA_LEN];
    BLEUuid service_uuid;
    int index;
    
    if (m_advertising.custom_length) {
        return false;
    }

    uuid16_data_offset = 0;
    uuid128_data_offset = 0;
    
    for (index = 0; index < count; index++) {
        service_uuid = BLEUuid(uuids[index]);
            
        if (service_uuid.type() == BLEUuidType::UNDEFINED) {
            return false;
        }
            
        if (service_uuid.type() == BLEUuidType::UUID16) {
            if (uuid16_data_offset > (MAX_ADV_DATA_LEN - 2)) {
                return false;
            }
                
            memcpy(&uuid16_data[uuid16_data_offset], service_uuid.data(), 2);
            uuid16_data_offset += 2;
        } else {
            if (uuid128_data_offset > (MAX_ADV_DATA_LEN - 16)) {
                return false;
            }
                
            memcpy(&uuid128_data[uuid128_data_offset], service_uuid.data(), 16);
            uuid128_data_offset += 16;
        }
    }
        
    length = 0;
    
    if (uuid16_data_offset) {
        length += (2 + uuid16_data_offset);
    }
    
    if (uuid128_data_offset) {
        length += (2 + uuid128_data_offset);
    }

    if (length) {
        if (((m_advertising.data_length - m_advertising.service_uuids_length) + length) > (int)(MAX_ADV_DATA_LEN)) {
            return false;
        }
    }
    
    advertising_data_offset = 3 + m_advertising.local_name_length + m_advertising.service_uuids_length + m_advertising.connection_interval_length + m_advertising.tx_power_level_length;
    
    if (m_advertising.service_uuids_length) {
        memmove(&m_advertising.data[advertising_data_offset],
                &m_advertising.data[advertising_data_offset + m_advertising.service_uuids_length],
                ((m_advertising.data_length - advertising_data_offset) - m_advertising.service_uuids_length));

        m_advertising.data_length -= m_advertising.service_uuids_length;
        m_advertising.service_uuids_length = 0;
    }

    if (length) {
        memmove(&m_advertising.data[advertising_data_offset + length],
                &m_advertising.data[advertising_data_offset],
                (m_advertising.data_length - advertising_data_offset));

        if (uuid16_data_offset) {
            m_advertising.data[advertising_data_offset++] = 1 + uuid16_data_offset;
            m_advertising.data[advertising_data_offset++] = complete ? BLE_AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST : BLE_AD_TYPE_16_BIT_SERV_UUID;
            
            memcpy(&m_advertising.data[advertising_data_offset], &uuid16_data[0], uuid16_data_offset);
            advertising_data_offset += uuid16_data_offset;
        }

        if (uuid128_data_offset) {
            m_advertising.data[advertising_data_offset++] = 1 + uuid128_data_offset;
            m_advertising.data[advertising_data_offset++] = complete ? BLE_AD_TYPE_128_BIT_SERV_UUID_CMPLT_LIST : BLE_AD_TYPE_128_BIT_SERV_UUID;

            memcpy(&m_advertising.data[advertising_data_offset], &uuid128_data[0], uuid128_data_offset);
            advertising_data_offset += uuid128_data_offset;
        }
        
        m_advertising.data_length += length;
        m_advertising.service_uuids_length = length;
    }

    if (m_peripheral.advertising) {
        if (hci_le_set_advertising_data(m_advertising.data_length, m_advertising.data) != BLE_STATUS_SUCCESS) {
            return false;
        }
    }
    
    return false;
}

bool BLELocalDevice::setManufacturerData(const uint8_t data[], int length) {
    uint32_t advertising_data_offset;

    if (m_advertising.custom_length) {
        return false;
    }

    if (length) {
        length += 2;

        if (((m_advertising.data_length - m_advertising.manufacturer_data_length) + length) > (int)(MAX_ADV_DATA_LEN)) {
            return false;
        }
    } else {
        length = 0;
    }

    advertising_data_offset = 3 + m_advertising.local_name_length + m_advertising.service_uuids_length + m_advertising.connection_interval_length + m_advertising.tx_power_level_length;
    
    if (m_advertising.manufacturer_data_length) {
        memmove(&m_advertising.data[advertising_data_offset],
                &m_advertising.data[advertising_data_offset + m_advertising.manufacturer_data_length],
                ((m_advertising.data_length - advertising_data_offset) - m_advertising.manufacturer_data_length));

        m_advertising.data_length -= m_advertising.manufacturer_data_length;
        m_advertising.manufacturer_data_length = 0;
    }

    if (length) {
        memmove(&m_advertising.data[advertising_data_offset + length],
                &m_advertising.data[advertising_data_offset],
                (m_advertising.data_length - advertising_data_offset));
        
        m_advertising.data[advertising_data_offset++] = length -1;
        m_advertising.data[advertising_data_offset++] = BLE_AD_TYPE_MANUFACTURER_SPECIFIC_DATA;
        memcpy(&m_advertising.data[advertising_data_offset], data, length -2);

        m_advertising.data_length += length;
        m_advertising.manufacturer_data_length = length;
    }

    if (m_peripheral.advertising) {
        if (hci_le_set_advertising_data(m_advertising.data_length, m_advertising.data) != BLE_STATUS_SUCCESS) {
            return false;
        }
    }
    
    return true;
}

bool BLELocalDevice::setBeacon(const char *uuid, uint16_t major, uint16_t minor, int8_t rssi) {
    uint32_t length, index;
    uint8_t data[25];

    if (uuid) {
        length = 25;
        
        BLEUuid beacon_uuid = BLEUuid(uuid);

        if (beacon_uuid.type() != BLEUuidType::UUID128) {
            return false;
        }

        data[0] = 0x4c;
        data[1] = 0x00;
        data[2] = 0x02;
        data[3] = 0x15;

        for (index = 0; index < 16; index++) {
            data[4+index] = *(beacon_uuid.data() + (15 - index));
        }

        data[20] = (uint8_t)(major >> 8);
        data[21] = (uint8_t)(major >> 0);
        data[22] = (uint8_t)(minor >> 8);
        data[23] = (uint8_t)(minor >> 0);
        data[24] = (uint8_t)rssi;
    } else {
        length = 0;
    }

    return setManufacturerData(data, length);
}

bool BLELocalDevice::setAdvertisingData(const uint8_t data[], int length) {
    if (length > 28) {
        return false;
    }

    m_advertising.data_length = (3 + length);
    m_advertising.local_name_length = 0;
    m_advertising.service_uuids_length = 0;
    m_advertising.connection_interval_length = 0;
    m_advertising.tx_power_level_length = 0;
    m_advertising.manufacturer_data_length = 0;
    m_advertising.custom_length = length;

    if (length) {
        memcpy(&m_advertising.data[3], data, length);
    }

    if (m_peripheral.advertising) {
        if (hci_le_set_advertising_data(m_advertising.data_length, m_advertising.data) != BLE_STATUS_SUCCESS) {
            return false;
        }
    }

    return true;
}

bool BLELocalDevice::setScanResponseData(const uint8_t data[], int length) {
    if (length > 31) {
        return false;
    }

    m_advertising.scan_response_data_length = length;

    if (length) {
        memcpy(&m_advertising.scan_response_data[0], data, length);
    }

    if (m_peripheral.advertising) {
        if (hci_le_set_scan_response_data(m_advertising.scan_response_data_length, m_advertising.scan_response_data) != BLE_STATUS_SUCCESS) {
            return false;
        }
    }
    
    return true;
}

bool BLELocalDevice::setAdvertisingInterval(uint16_t minimumAdvertisingInterval, uint16_t maximumAdvertisingInterval) {

    if (minimumAdvertisingInterval > maximumAdvertisingInterval) {
        return false;
    }

    if ((minimumAdvertisingInterval < 0x0020) || (minimumAdvertisingInterval > 0x4000)) {
        return false;
    }

    if ((maximumAdvertisingInterval < 0x0020) || (maximumAdvertisingInterval > 0x4000)) {
        return false;
    }

    m_peripheral.advertising_interval_min = minimumAdvertisingInterval;
    m_peripheral.advertising_interval_max = maximumAdvertisingInterval;

    return true;
}

void BLELocalDevice::setConnectable(bool connectable) {
    m_peripheral.connectable = connectable;
}

void BLELocalDevice::setDiscoverable(BLEDiscoverable discoverable) {
    m_peripheral.discoverable = discoverable;
}

bool BLELocalDevice::advertise() {
    uint32_t index, fixed_pin;
    uint8_t bonded_device_count, resolving_device_count;
    uint8_t mitm, sc, min_key_size, max_key_size, use_fixed_pin;
    BLESecurity security;
    Bonded_Device_Entry_t bonded_devices[((BLE_EVT_MAX_PARAM_LEN - 3) - 2) / sizeof(Bonded_Device_Entry_t)];
    tBleStatus status = BLE_STATUS_SUCCESS;

    security = m_peripheral.security;
    
    if (m_peripheral.security < m_server.security) {
        security = m_server.security;
    }
    
    switch (security) {
    case BLE_SECURITY_UNENCRYPTED:
        mitm = MITM_PROTECTION_NOT_REQUIRED;
        sc = SC_IS_SUPPORTED;
        min_key_size = 7;
        max_key_size = 16;
        use_fixed_pin = USE_FIXED_PIN_FOR_PAIRING_FORBIDDEN;
        fixed_pin = 0;
        break;
        
    case BLE_SECURITY_UNAUTHENTICATED:
        mitm = MITM_PROTECTION_NOT_REQUIRED;
        sc = SC_IS_SUPPORTED;
        min_key_size = 7;
        max_key_size = 16;
        use_fixed_pin = USE_FIXED_PIN_FOR_PAIRING_FORBIDDEN;
        fixed_pin = 0;
        break;
        
    case BLE_SECURITY_AUTHENTICATED:
        if (m_peripheral.fixed_pin == BLE_FIXED_PIN_UNDEFINED) {
            return false;
        }
        
        mitm = MITM_PROTECTION_REQUIRED;
        sc = SC_IS_SUPPORTED;
        min_key_size = 7;
        max_key_size = 16;
        use_fixed_pin = USE_FIXED_PIN_FOR_PAIRING_ALLOWED;
        fixed_pin = m_peripheral.fixed_pin;
        break;

    case BLE_SECURITY_AUTHENTICATED_SECURE_CONNECTION:
        if (m_peripheral.fixed_pin == BLE_FIXED_PIN_UNDEFINED) {
            return false;
        }

        mitm = MITM_PROTECTION_REQUIRED;
        sc = SC_IS_MANDATORY;
        min_key_size = 16;
        max_key_size = 16;
        use_fixed_pin = USE_FIXED_PIN_FOR_PAIRING_ALLOWED;
        fixed_pin = m_peripheral.fixed_pin;
        break;

    default:
        return false;
    }

    if (m_central) {
        return false;
    }
    
    if (m_peripheral.advertising) {
        m_peripheral.advertising = false;
        
        aci_gap_set_non_discoverable();
    }

    publish();

    if (status == BLE_STATUS_SUCCESS) {
        status = aci_gap_set_io_capability((use_fixed_pin == USE_FIXED_PIN_FOR_PAIRING_ALLOWED) ? IO_CAP_DISPLAY_ONLY : IO_CAP_NO_INPUT_NO_OUTPUT);
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        status = aci_gap_set_authentication_requirement(m_peripheral.bonding,
                                                        mitm,
                                                        sc,
                                                        KEYPRESS_NOT_SUPPORTED,
                                                        min_key_size,
                                                        max_key_size,
                                                        use_fixed_pin,
                                                        fixed_pin,
                                                        (m_options & BLE_OPTION_RANDOM_STATIC_ADDRESS));
    }

    if (status == BLE_STATUS_SUCCESS) {
        if ((m_options & BLE_OPTION_PRIVACY) || m_peripheral.bonding) {
            status = aci_gap_configure_whitelist();
        }
    }

    if (status == BLE_STATUS_SUCCESS) {
        if (m_options & BLE_OPTION_PRIVACY) {
            status = aci_gap_get_bonded_devices(&bonded_device_count, &bonded_devices[0]);

            if (status == BLE_STATUS_SUCCESS) {
                for (resolving_device_count = 0, index = 0; index < bonded_device_count; index++) {
                    if (bonded_devices[index].Address_Type == 0x00) {
                        bonded_devices[resolving_device_count++] = bonded_devices[index];
                    }
                }
                
                status = aci_gap_add_devices_to_resolving_list(resolving_device_count, (Whitelist_Identity_Entry_t*)&bonded_devices[0], true);
            }
        }
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        if (m_peripheral.discoverable == BLE_DISCOVERABLE_GENERAL) {
            status = aci_gap_set_discoverable(((m_peripheral.connectable) ? GAP_ADV_IND : (m_advertising.scan_response_data_length ? GAP_ADV_SCAN_IND : GAP_ADV_NONCONN_IND)),
                                              m_peripheral.advertising_interval_min,
                                              m_peripheral.advertising_interval_max,
                                              (m_options & (BLE_OPTION_RANDOM_STATIC_ADDRESS | BLE_OPTION_PRIVACY)),
                                              NO_WHITE_LIST_USE,
                                              0, NULL,
                                              0, NULL,
                                              0, 0);
        } else {
            status = aci_gap_set_limited_discoverable(((m_peripheral.connectable) ? GAP_ADV_IND : (m_advertising.scan_response_data_length ? GAP_ADV_SCAN_IND : GAP_ADV_NONCONN_IND)),
                                                      m_peripheral.advertising_interval_min,
                                                      m_peripheral.advertising_interval_max,
                                                      (m_options & (BLE_OPTION_RANDOM_STATIC_ADDRESS | BLE_OPTION_PRIVACY)),
                                                      NO_WHITE_LIST_USE,
                                                      0, NULL,
                                                      0, NULL,
                                                      0, 0);
        }
    }

    if (status == BLE_STATUS_SUCCESS) {
        status = aci_gap_delete_ad_type(BLE_AD_TYPE_FLAGS);
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        status = aci_gap_delete_ad_type(BLE_AD_TYPE_TX_POWER_LEVEL);
    }

    m_advertising.data[0] = 0x02;
    m_advertising.data[1] = BLE_AD_TYPE_FLAGS;
    m_advertising.data[2] = FLAG_BIT_BR_EDR_NOT_SUPPORTED | ((m_peripheral.discoverable == BLE_DISCOVERABLE_GENERAL) ? FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE : FLAG_BIT_LE_LIMITED_DISCOVERABLE_MODE);
    
    if (status == BLE_STATUS_SUCCESS) {
        status = hci_le_set_advertising_data(m_advertising.data_length, m_advertising.data);
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        status = hci_le_set_scan_response_data(m_advertising.scan_response_data_length, m_advertising.scan_response_data);
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        m_peripheral.advertising = true;
    }

    return m_peripheral.advertising;
}

void BLELocalDevice::stopAdvertise() {
    if (m_peripheral.advertising) {
        m_peripheral.advertising = false;
        
        aci_gap_set_non_discoverable();
    }
}

bool BLELocalDevice::advertising() {
    return m_peripheral.advertising;
}

BLEDevice BLELocalDevice::central() {
    if (m_central) {
        return m_central->device();
    }

    return BLEDevice();
}

bool BLELocalDevice::connected() {
    return (m_central != nullptr);
}

void BLELocalDevice::disconnect() {
    if (m_central) {
        aci_gap_terminate(m_central->m_handle, HCI_REMOTE_USER_TERMINATED_CONNECTION_ERR_CODE);
    }
}

void BLELocalDevice::onStop(Callback callback) {
    m_stop_callback = (callback ? callback : Callback(__wakeupCallback));
}

void BLELocalDevice::onConnect(Callback callback) {
    m_connect_callback = (callback ? callback : Callback(__wakeupCallback));
}

void BLELocalDevice::onDisconnect(Callback callback) {
    m_disconnect_callback = (callback ? callback : Callback(__wakeupCallback));
}

void BLELocalDevice::reset() {
    BLEServerService *service;
    BLEServerCharacteristic *characteristic, *request, *request_next;
    BLEServerDescriptor *descriptor;
    uint32_t index;

    for (index = 0; index < BLE_NUM_GATT_SERVICES; index++) {
        if (m_process.service_table[index] != nullptr) {
            service = m_process.service_table[index];

            if (service->m_server.handle) {
                for (characteristic = service->m_server.children; characteristic; characteristic = characteristic->m_server.sibling) {
                    for (descriptor = characteristic->m_server.children; descriptor; descriptor = descriptor->m_server.sibling) {
                        if (descriptor->m_server.handle) {
                            descriptor->m_server.handle = 0x0000;

                            descriptor->unreference();
                        }
                    }

                    characteristic->m_subscribed = 0;
                    characteristic->m_update_value = false;
                    characteristic->m_sync_value = false;
                    characteristic->m_sync_subscribed = false;
                    
                    if (characteristic->m_server.handle) {
                        characteristic->m_server.handle = 0x0000;

                        armv7m_atomic_storeh(&characteristic->m_written, 0x0000);
            
                        characteristic->unreference();
                    }
                }
        
                service->m_server.handle = 0x0000;
                service->m_server.attrib[0] = 0xffff;
                service->m_server.attrib[1] = 0x0000;
        
                service->unreference();
            }
        }
    }

    if (m_process.request_current) {
        request = m_process.request_current;

        armv7m_atomic_store((volatile uint32_t*)&request->m_process.request, (uint32_t)nullptr);
        armv7m_atomic_storeb(&request->m_busy, 0);

        if (request->m_status) {
            *request->m_status = HCI_REMOTE_USER_TERMINATED_CONNECTION_ERR_CODE;
        }

        request->m_done_callback();
    }

    for (request = m_process.request_head; request; request = request_next) {
        request_next = request->m_process.request;

        armv7m_atomic_store((volatile uint32_t*)&request->m_process.request, (uint32_t)nullptr);
        armv7m_atomic_storeb(&request->m_busy, 0);

        if (request->m_status) {
            *request->m_status = HCI_REMOTE_USER_TERMINATED_CONNECTION_ERR_CODE;
        }

        request->m_done_callback();
    }

    for (request = m_process.request_submit; request; request = request_next) {
        request_next = request->m_process.request;

        armv7m_atomic_store((volatile uint32_t*)&request->m_process.request, (uint32_t)nullptr);
        armv7m_atomic_storeb(&request->m_busy, 0);

        if (request->m_status) {
            *request->m_status = HCI_REMOTE_USER_TERMINATED_CONNECTION_ERR_CODE;
        }

        request->m_done_callback();
    }


    armv7m_atomic_store(&m_process.sequence, 0);

    m_process.connection = BLE_PEER_DEVICE_NOT_CONNECTED;
    m_process.published = false;
    m_process.service_count = 0;

    for (index = 0; index < BLE_NUM_GATT_SERVICES; index++) {
        m_process.service_table[index] = nullptr;
    }
    
    m_process.request_head = nullptr;
    m_process.request_tail = nullptr;
    m_process.request_current = nullptr;
    armv7m_atomic_store((volatile uint32_t*)&m_process.request_submit, (uint32_t)nullptr);

    m_process.sync_value = false;
    m_process.value_current = nullptr;

    m_process.sync_subscribed = false;
    m_process.subscribed_current = nullptr;
}

bool BLELocalDevice::publish() {
    BLEServerService *service;
    
    if (m_process.published) {
        return true;
    }

    for (service = m_server.children; service; service = service->m_server.sibling) {
        if (!insert(service)) {
            for (service = m_server.children; service; service = service->m_server.sibling) {
                if (service->m_server.handle) {
                    remove(service);
                }
            }

            return false;
        }
    }

    m_process.published = true;
    
    return true;
}

bool BLELocalDevice::insert(BLEServerService *service) {
    BLEServerCharacteristic *characteristic;
    BLEServerDescriptor *descriptor;
    BLESecurity security_read, security_write, security_subscribe;
    unsigned int index, attribute_count;
    uint16_t service_changed[2];
    uint8_t value_permissions, cccd_permissions, notify;
    tBleStatus status;
    
    if (m_process.service_count == BLE_NUM_GATT_SERVICES) {
        return false;
    }

    m_process.service_count++;
    
    for (attribute_count = 1, characteristic = service->m_server.children; characteristic; characteristic = characteristic->m_server.sibling) {
        attribute_count += 2;
    
        if (characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) {
            attribute_count += 1;
        }

        for (descriptor = characteristic->m_server.children; descriptor; descriptor = descriptor->m_server.sibling) {
            attribute_count += 2;
        }
    }

    BLEUuid service_uuid(service->m_uuid);
        
    status = aci_gatt_add_service(((service_uuid.type() == BLEUuidType::UUID16) ? UUID_TYPE_16 : UUID_TYPE_128),
                                  reinterpret_cast<const Service_UUID_t*>(service_uuid.data()),
                                  0x01,
                                  attribute_count,
                                  &service->m_server.handle);

    if (status == BLE_STATUS_SUCCESS) {
#if (BLE_TRACE_SUPPORTED == 1)
        printf("ADD-SERVICE \"%s\", %04x\r\n", service->m_uuid, service->m_server.handle);
#endif
      
        service->reference();

        service->m_server.attrib[0] = service->m_server.handle +1;
        service->m_server.attrib[1] = service->m_server.handle +1 + (attribute_count -1);
        
        for (characteristic = service->m_server.children; characteristic; characteristic = characteristic->m_server.sibling) {
            BLEUuid characteristic_uuid(characteristic->m_uuid);
            
            value_permissions = 0;
            cccd_permissions = 0;
            
            notify = GATT_DONT_NOTIFY_EVENTS;
            
            security_read = (BLESecurity)((characteristic->m_permissions >> 0) & 3);
            security_write = (BLESecurity)((characteristic->m_permissions >> 2) & 3);
            security_subscribe = (BLESecurity)((characteristic->m_permissions >> 4) & 3);
            
            if (characteristic->m_properties & (BLE_PROPERTY_READ)) {
                notify |= GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP;

                switch (security_read) {
                case BLE_SECURITY_UNENCRYPTED:
                    break;
                case BLE_SECURITY_UNAUTHENTICATED:
                    value_permissions |= ATTR_PERMISSION_ENCRY_READ;
                    break;
                case BLE_SECURITY_AUTHENTICATED:
                    value_permissions |= (ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_AUTHEN_READ);
                    break;
                case BLE_SECURITY_AUTHENTICATED_SECURE_CONNECTION:
                    value_permissions |= (ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_AUTHEN_READ);
                    break;
                };
            }
            
            if (characteristic->m_properties & (BLE_PROPERTY_WRITE_WITHOUT_RESPONSE | BLE_PROPERTY_WRITE)) {
                notify |= GATT_NOTIFY_ATTRIBUTE_WRITE;

                switch (security_write) {
                case BLE_SECURITY_UNENCRYPTED:
                break;
                case BLE_SECURITY_UNAUTHENTICATED:
                    value_permissions |= ATTR_PERMISSION_ENCRY_WRITE;
                    break;
                case BLE_SECURITY_AUTHENTICATED:
                    value_permissions |= (ATTR_PERMISSION_ENCRY_WRITE | ATTR_PERMISSION_AUTHEN_WRITE);
                    break;
                case BLE_SECURITY_AUTHENTICATED_SECURE_CONNECTION:
                    value_permissions |= (ATTR_PERMISSION_ENCRY_WRITE | ATTR_PERMISSION_AUTHEN_WRITE);
                    break;
                };
            }
            
            if (characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) {
                notify |= GATT_NOTIFY_ATTRIBUTE_WRITE;

                switch (security_subscribe) {
                case BLE_SECURITY_UNENCRYPTED:
                    break;
                case BLE_SECURITY_UNAUTHENTICATED:
                    cccd_permissions |= (ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE);
                    break;
                case BLE_SECURITY_AUTHENTICATED:
                    cccd_permissions |= (ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_AUTHEN_READ | ATTR_PERMISSION_ENCRY_WRITE | ATTR_PERMISSION_AUTHEN_WRITE);
                    break;
                case BLE_SECURITY_AUTHENTICATED_SECURE_CONNECTION:
                    cccd_permissions |= (ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_AUTHEN_READ | ATTR_PERMISSION_ENCRY_WRITE | ATTR_PERMISSION_AUTHEN_WRITE);
                    break;
                };
            }

            if (status == BLE_STATUS_SUCCESS) {
                status = aci_gatt_add_char(service->m_server.handle,
                                           ((characteristic_uuid.type() == BLEUuidType::UUID16) ? UUID_TYPE_16 : UUID_TYPE_128),
                                           reinterpret_cast<const Char_UUID_t*>(characteristic_uuid.data()),
                                           characteristic->m_value_size,
                                           characteristic->m_properties,
                                           value_permissions,
                                           notify,
                                           7,
                                           !characteristic->m_fixed_length,
                                           &characteristic->m_server.handle);
            }
            
            if (status == BLE_STATUS_SUCCESS) {
#if (BLE_TRACE_SUPPORTED == 1)
                printf("ADD-CHARACTERISTIC \"%s\", %04x\r\n", characteristic->m_uuid, characteristic->m_server.handle);
#endif
                characteristic->reference();

                if (characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) {
                    status = aci_gatt_set_security_permission(service->m_server.handle,
                                                              (characteristic->m_server.handle + BLE_CCCD_ATTRIB_HANDLE_OFFSET),
                                                              cccd_permissions);
                }
            }

            for (descriptor = characteristic->m_server.children; descriptor; descriptor = descriptor->m_server.sibling) {
                BLEUuid descriptor_uuid(descriptor->m_uuid);

                value_permissions = 0;

                security_read = (BLESecurity)((descriptor->m_permissions >> 0) & 3);

                switch (security_read) {
                case BLE_SECURITY_UNENCRYPTED:
                    break;
                case BLE_SECURITY_UNAUTHENTICATED:
                    value_permissions |= ATTR_PERMISSION_ENCRY_READ;
                    break;
                case BLE_SECURITY_AUTHENTICATED:
                    value_permissions |= (ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_AUTHEN_READ);
                    break;
                case BLE_SECURITY_AUTHENTICATED_SECURE_CONNECTION:
                    value_permissions |= (ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_AUTHEN_READ);
                    break;
                };
                
                status = aci_gatt_add_char_desc(service->m_server.handle,
                                                characteristic->m_server.handle,
                                                ((descriptor_uuid.type() == BLEUuidType::UUID16) ? UUID_TYPE_16 : UUID_TYPE_128),
                                                reinterpret_cast<const Char_Desc_Uuid_t*>(descriptor_uuid.data()),
                                                descriptor->m_value_length,
                                                descriptor->m_value_length,
                                                descriptor->m_value,
                                                value_permissions,
                                                ATTR_ACCESS_READ_ONLY,
                                                GATT_DONT_NOTIFY_EVENTS,
                                                7,
                                                false,
                                                &descriptor->m_server.handle);
                
                if (status != BLE_STATUS_SUCCESS) {
                    descriptor->reference();
                }
            }
        }
    }
    
    if (status != BLE_STATUS_SUCCESS) {
        remove(service);
        
        return false;
    }

    if (m_central) {
        for (characteristic = service->m_server.children; characteristic; characteristic = characteristic->m_server.sibling) {
            if (characteristic->m_properties & BLE_PROPERTY_READ) {
                characteristic->m_update_value = true;
            }
            
            if (characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) {
                characteristic->m_sync_subscribed = true;
                m_process.sync_subscribed = true;
            }
        }
    }
    
    for (index = 0; index < BLE_NUM_GATT_SERVICES; index++) {
        if (m_process.service_table[index] == nullptr) {
            m_process.service_table[index] = service;
            break;
        }
    }

    if (m_central) {
        if (!(m_options & BLE_OPTION_NO_SERVICE_CHANGED)) {
            service_changed[0] = service->m_server.handle;
            service_changed[1] = service->m_server.attrib[1];
            
            aci_gatt_update_char_value(BLE_ATT_SERVICE_HANDLE,
                                       BLE_SERVICE_CHANGED_HANDLE,
                                       0,
                                       4,
                                       (const uint8_t*)&service_changed[0]);
        }
        
        if (m_process.sync_value || m_process.sync_subscribed) {
            k_work_submit(&m_process.work);
        }
    }
    
    return true;
}

void BLELocalDevice::remove(BLEServerService *service) {
    BLEServerCharacteristic *characteristic;
    BLEServerDescriptor *descriptor;
    uint16_t service_changed[2];
    int index;

    for (index = 0; index < BLE_NUM_GATT_SERVICES; index++) {
        if (m_process.service_table[index] == service) {
            m_process.service_table[index] = nullptr;
            break;
        }
    }

    m_process.service_count--;

    if (service->m_server.handle) {
        service_changed[0] = service->m_server.handle;
        service_changed[1] = service->m_server.attrib[1];

        for (characteristic = service->m_server.children; characteristic; characteristic = characteristic->m_server.sibling) {
            for (descriptor = characteristic->m_server.children; descriptor; descriptor = descriptor->m_server.sibling) {
                if (descriptor->m_server.handle) {
                    descriptor->m_server.handle = 0x0000;

                    descriptor->unreference();
                }
            }

            characteristic->m_subscribed = 0;
            characteristic->m_update_type = 0;
            characteristic->m_update_value = false;
            characteristic->m_sync_value = false;
            characteristic->m_sync_subscribed = false;
            
            if (characteristic->m_server.handle) {
                aci_gatt_del_char(service->m_server.handle, characteristic->m_server.handle);
            
                characteristic->m_server.handle = 0x0000;
            
                characteristic->unreference();
            }
        }
        
        aci_gatt_del_service(service->m_server.handle);

        service->m_server.handle = 0x0000;
        service->m_server.attrib[0] = 0xffff;
        service->m_server.attrib[1] = 0x0000;
        
        service->unreference();
        
        if (m_central) {
            if (!(m_options & BLE_OPTION_NO_SERVICE_CHANGED)) {
                aci_gatt_update_char_value(BLE_ATT_SERVICE_HANDLE,
                                           BLE_SERVICE_CHANGED_HANDLE,
                                           0,
                                           4,
                                           (const uint8_t*)&service_changed[0]);
            }
        }
    }
}

void BLELocalDevice::request(class BLEServerCharacteristic *request) {
    class BLEServerCharacteristic *request_submit;

    do {
        request_submit = m_process.request_submit;

        armv7m_atomic_store((volatile uint32_t*)&request->m_process.request, (uint32_t)request_submit);
    } while ((BLEServerCharacteristic *)armv7m_atomic_cas((volatile uint32_t*)&m_process.request_submit, (uint32_t)request_submit, (uint32_t)request) != request_submit);

    if (!request_submit) {
        k_work_submit(&m_process.work);
    }
}

BLEPeerDevice *BLELocalDevice::complete(uint16_t handle, const uint8_t *address, uint8_t type, uint16_t connectionInterval, uint16_t slaveLatency, uint16_t supervisionTimeout) {
    BLEPeerDevice *central;
    BLEServerService *service;
    BLEServerCharacteristic *characteristic;
    uint32_t index;

    m_process.connection = handle;
                            
    central = new BLEPeerDevice(handle, address, type, connectionInterval, slaveLatency, supervisionTimeout);
    
    if (central) {
        if (m_peripheral.security != BLE_SECURITY_UNENCRYPTED) {
            armv7m_atomic_or(&m_process.sequence, BLE_REQUEST_GAP_SLAVE_SECURITY);
        }

        if ((m_tx_phy != BLE_PHY_1M) || (m_rx_phy != BLE_PHY_1M)) {
            armv7m_atomic_or(&m_process.sequence, BLE_REQUEST_HCI_LE_SET_PHY);
        }
        
        for (index = 0; index < BLE_NUM_GATT_SERVICES; index++) {
            service = m_process.service_table[index];

            if (service) {
                for (characteristic = service->m_server.children; characteristic; characteristic = characteristic->m_server.sibling) {
                    if (characteristic->m_properties & BLE_PROPERTY_READ) {
                        characteristic->m_update_value = true;
                    }

                    if (characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) {
                        characteristic->m_sync_subscribed = true;
                        m_process.sync_subscribed = true;
                    }
                }
            }
        }
        
        m_peripheral.advertising = false;
        
#if (BLE_TRACE_SUPPORTED == 1)
        printf("PROCESS-CONNECT\r\n");
#endif
        
        m_connect_callback();
    } else {
        armv7m_atomic_or(&m_process.sequence, BLE_REQUEST_GAP_TERMINATE);
        
        if (m_peripheral.advertising) {
            m_peripheral.advertising = false;
            
#if (BLE_TRACE_SUPPORTED == 1)
            printf("PROCESS-STOP\r\n");
#endif
            
            m_stop_callback();
        }
    }

    return central;
}

void BLELocalDevice::terminate() {
    BLEServerService *service;
    BLEServerCharacteristic *characteristic;
    uint32_t index;
    
    if (m_central) {
        for (index = 0; index < BLE_NUM_GATT_SERVICES; index++) {
            service = m_process.service_table[index];

            if (service) {
                for (characteristic = service->m_server.children; characteristic; characteristic = characteristic->m_server.sibling) {
                    if (characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) {
                        if (characteristic->m_subscribed) {
                            characteristic->m_subscribed = 0;

#if (BLE_TRACE_SUPPORTED == 1)
                            printf("PROCESS-SUBSCRIBED %02x (\"%s\")\r\n", 0, characteristic->m_uuid);
#endif
                            
                            characteristic->m_subscribed_callback();
                        }
                    }
                    
                    characteristic->m_update_value = false;
                    characteristic->m_sync_value = false;
                    characteristic->m_sync_subscribed = false;
                }
            }
        }
        
        if (m_process.sequence & BLE_EVENT_GATT_SERVER_CONFIRMATION) {
            armv7m_atomic_and(&m_process.sequence, ~BLE_EVENT_GATT_SERVER_CONFIRMATION);
            
            characteristic = m_process.request_current;

            m_process.request_current = nullptr;
            
#if (BLE_TRACE_SUPPORTED == 1)
            printf("PROCESS-DONE %02x (\"%s\", %02x)\r\n", HCI_REMOTE_USER_TERMINATED_CONNECTION_ERR_CODE, characteristic->m_uuid, characteristic->m_update_type);
#endif
            
            armv7m_atomic_storeb(&characteristic->m_busy, 0);

            if (characteristic->m_status) {
                *characteristic->m_status = HCI_REMOTE_USER_TERMINATED_CONNECTION_ERR_CODE;
            }
            
            characteristic->m_done_callback();
        }
        
        m_central->m_handle = BLE_PEER_DEVICE_NOT_CONNECTED;
        
#if (BLE_TRACE_SUPPORTED == 1)
        printf("PROCESS-DISCONNECT\r\n");
#endif
        
        m_disconnect_callback();
        
        m_central->unreference();
        m_central = nullptr;
    }
    
    armv7m_atomic_and(&m_process.sequence, BLE_BUSY_MASK);

    m_process.sync_value = false;
    m_process.value_current = NULL;

    m_process.sync_subscribed = false;
    m_process.subscribed_current = NULL;
}

void BLELocalDevice::connection(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval, uint16_t slaveLatency, uint16_t supervisionTimeout) {
    m_process.connection_interval_min = minimumConnectionInterval;
    m_process.connection_interval_max = maximumConnectionInterval;
    m_process.slave_latency = slaveLatency;
    m_process.supervision_timeout = supervisionTimeout;

    armv7m_atomic_or(&m_process.sequence, BLE_REQUEST_L2CAP_CONNECTION_PARAMETER_UPDATE);

    k_work_submit(&m_process.work);
}

void BLELocalDevice::process() {
    BLEServerService *service;
    BLEServerCharacteristic *characteristic, *request_submit, *request_next, *request_head, *request_tail;
    uint32_t index, entry;
    const hci_uart_pckt *event;    
    const hci_event_pckt *hci_event;
    const evt_le_meta_event *hci_le_event;
    const evt_blue_aci *hci_vs_event;

    if (m_reset) {
        return;
    }

#if (BLE_TRACE_SUPPORTED == 1)
    printf("PROCESS-ENTER %08x\r\n", (unsigned int)m_process.sequence);
#endif
    
    if ((m_process.sequence & BLE_BUSY_MASK) && (m_process.command.status != STM32WB_IPCC_BLE_STATUS_BUSY)) {
#if (BLE_TRACE_SUPPORTED == 1)
        printf("PROCESS-STATUS %02x\r\n", *((uint8_t*)m_process.command.rparam));
#endif
        if (m_process.sequence & BLE_BUSY_GATT_ALLOW_READ) {
        }

        if (m_process.sequence & BLE_BUSY_GATT_CONFIRM_INDICATION) {
        }

        if (m_process.sequence & BLE_BUSY_GAP_TERMINATE) {
        }

        else if (m_process.sequence & BLE_BUSY_GAP_SLAVE_SECURITY) {
            const aci_gap_slave_security_req_rp0 *rparam = (const aci_gap_slave_security_req_rp0*)m_process.command.rparam;

            if (rparam->Status == BLE_STATUS_SUCCESS) {
                armv7m_atomic_or(&m_process.sequence, BLE_EVENT_GAP_PAIRING_COMPLETE);
            } else {
                armv7m_atomic_or(&m_process.sequence, BLE_REQUEST_GAP_TERMINATE);
            }
        }

        else if (m_process.sequence & BLE_BUSY_GAP_ALLOW_REBOND) {
            const aci_gap_slave_security_req_rp0 *rparam = (const aci_gap_slave_security_req_rp0*)m_process.command.rparam;

            if (rparam->Status != BLE_STATUS_SUCCESS) {
                armv7m_atomic_or(&m_process.sequence, BLE_REQUEST_GAP_TERMINATE);
            }
        }
        
        else if (m_process.sequence & BLE_BUSY_L2CAP_CONNECTION_PARAMETER_UPDATE) {
        }

        else if (m_process.sequence & BLE_BUSY_HCI_LE_SET_PHY) {
        }

        else if (m_process.sequence & BLE_BUSY_GATT_UPDATE_CHAR_VALUE_EXT) {
            const aci_gatt_update_char_value_ext_rp0 *rparam = (const aci_gatt_update_char_value_ext_rp0*)m_process.command.rparam;

            if (m_process.request_current) {
                characteristic = m_process.request_current;
            
                if (rparam->Status != BLE_STATUS_INSUFFICIENT_RESOURCES) {
                    if (rparam->Status == BLE_STATUS_SUCCESS) {
                        m_process.request_offset += m_process.request_count;
                        
                        if (m_process.request_offset == m_process.request_length) {
                            if (characteristic->m_update_type & 0x02) {
                                if (m_central) {
                                    armv7m_atomic_or(&m_process.sequence, BLE_EVENT_GATT_SERVER_CONFIRMATION);
                                } else {
                                    m_process.request_current = nullptr;
                                    
#if (BLE_TRACE_SUPPORTED == 1)
                                    printf("PROCESS-DONE %02x (\"%s\", %02x)\r\n", HCI_REMOTE_USER_TERMINATED_CONNECTION_ERR_CODE, characteristic->m_uuid, characteristic->m_update_type);
#endif
                                    
                                    armv7m_atomic_storeb(&characteristic->m_busy, 0);

                                    if (characteristic->m_status) {
                                        *characteristic->m_status = HCI_REMOTE_USER_TERMINATED_CONNECTION_ERR_CODE;
                                    }

                                    characteristic->m_done_callback();
                                }
                            } else {
                                m_process.request_current = nullptr;
                                
#if (BLE_TRACE_SUPPORTED == 1)
                                printf("PROCESS-DONE %02x (\"%s\", %02x)\r\n", rparam->Status, characteristic->m_uuid, characteristic->m_update_type);
#endif
                                
                                armv7m_atomic_storeb(&characteristic->m_busy, 0);

                                if (characteristic->m_status) {
                                    *characteristic->m_status = rparam->Status;
                                }

                                characteristic->m_done_callback();
                            }
                        }
                    } else {
                        m_process.request_current = nullptr;
                        
#if (BLE_TRACE_SUPPORTED == 1)
                        printf("PROCESS-DONE %02x (\"%s\", %02x)\r\n", rparam->Status, characteristic->m_uuid, characteristic->m_update_type);
#endif
                        
                        armv7m_atomic_storeb(&characteristic->m_busy, 0);
                                
                        if (characteristic->m_status) {
                            *characteristic->m_status = rparam->Status;
                        }

                        characteristic->m_done_callback();
                    }

                    if (characteristic->m_written) {
#if (BLE_TRACE_SUPPORTED == 1)
                        printf("PROCESS-WRITTEN (\"%s\")\r\n", characteristic->m_uuid);
#endif
                        characteristic->m_written_callback();
                    }
                } else {
                    if (m_central) {
                        armv7m_atomic_or(&m_process.sequence, BLE_EVENT_GATT_TX_POOL_AVAILABLE);
                    }
                }
            }

            else if (m_process.value_current) {
                characteristic = m_process.value_current;

                if (rparam->Status == BLE_STATUS_SUCCESS) {
                    m_process.value_offset += m_process.value_count;
                    
                    if (m_process.value_offset == m_process.value_length) {
                        m_process.value_current = nullptr;

#if (BLE_TRACE_SUPPORTED == 1)
                        printf("PROCESS-UPDATE %02x (\"%s\")\r\n", rparam->Status, characteristic->m_uuid);
#endif
                    }
                } else {
                    m_process.value_current = nullptr;

#if (BLE_TRACE_SUPPORTED == 1)
                    printf("PROCESS-UPDATE %02x (\"%s\")\r\n", rparam->Status, characteristic->m_uuid);
#endif

                }
            }
        } else if (m_process.sequence & BLE_BUSY_GATT_READ_HANDLE_VALUE) {
            const aci_gatt_read_handle_value_rp0 *rparam = (const aci_gatt_read_handle_value_rp0*)m_process.command.rparam;

            if (m_process.subscribed_current) {
                characteristic = m_process.subscribed_current;
            
                m_process.subscribed_current = nullptr;
                
                if (rparam->Status == BLE_STATUS_SUCCESS) {
                    if (characteristic->m_subscribed != (rparam->Value[0] & 3)) {
                        characteristic->m_subscribed = (rparam->Value[0] & 3);
                    
#if (BLE_TRACE_SUPPORTED == 1)
                        printf("PROCESS-SUBSCRIBED %02x (\"%s\")\r\n", characteristic->m_subscribed, characteristic->m_uuid);
#endif
                        
                        characteristic->m_subscribed_callback();
                    }
                } else {
#if (BLE_TRACE_SUPPORTED == 1)
                    printf("PROCESS-SUBSCRIBED %02x (\"%s\")\r\n", characteristic->m_subscribed, characteristic->m_uuid);
#endif
                }
            }
        }
        
        armv7m_atomic_and(&m_process.sequence, ~BLE_BUSY_MASK);
    }
    
    event = (const hci_uart_pckt*)stm32wb_ipcc_ble_event();

    if (event) {
        do {
            if (event->type == HCI_EVENT_PKT_TYPE) {
                hci_event = (const hci_event_pckt*)&event->data[0];

                switch (hci_event->evt) {

                case HCI_DISCONNECTION_COMPLETE_EVENT: {
                    const hci_disconnection_complete_event_rp0 *eparams = (const hci_disconnection_complete_event_rp0*)&hci_event->data[0];

                    if (m_process.connection == eparams->Connection_Handle) {
                        m_process.connection = BLE_PEER_DEVICE_NOT_CONNECTED;

                        terminate();
                    }
                    break;
                }
                    
                case HCI_ENCRYPTION_CHANGE_EVENT:
                case HCI_READ_REMOTE_VERSION_INFORMATION_COMPLETE_EVENT:
                case HCI_HARDWARE_ERROR_EVENT:
                case HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT:
                    break;
                    
                case HCI_LE_META_EVENT:
                    hci_le_event = (const evt_le_meta_event*)&hci_event->data[0];
                    
                    switch (hci_le_event->subevent) {
                        
                    case HCI_LE_CONNECTION_COMPLETE_EVENT: {
                        const hci_le_connection_complete_event_rp0 *eparam = (const hci_le_connection_complete_event_rp0*)&hci_le_event->data[0];

                        if (eparam->Status == BLE_STATUS_SUCCESS) {
                            m_central = complete(eparam->Connection_Handle,
                                                 eparam->Peer_Address,
                                                 eparam->Peer_Address_Type,
                                                 eparam->Conn_Interval,
                                                 eparam->Conn_Latency,
                                                 eparam->Supervision_Timeout);
                        } else {
                            if (m_peripheral.advertising) {
                                m_peripheral.advertising = false;

#if (BLE_TRACE_SUPPORTED == 1)
                                printf("PROCESS-STOP\r\n");
#endif
                                
                                m_stop_callback();
                            }
                        }
                        break;
                    }
                        
                    case HCI_LE_ENHANCED_CONNECTION_COMPLETE_EVENT: {
                        const hci_le_enhanced_connection_complete_event_rp0 *eparam = (const hci_le_enhanced_connection_complete_event_rp0*)&hci_le_event->data[0];

                        if (eparam->Status == BLE_STATUS_SUCCESS) {
                            m_central = complete(eparam->Connection_Handle,
                                                 eparam->Peer_Address,
                                                 eparam->Peer_Address_Type,
                                                 eparam->Conn_Interval,
                                                 eparam->Conn_Latency,
                                                 eparam->Supervision_Timeout);
                        } else {
                            if (m_peripheral.advertising) {
                                m_peripheral.advertising = false;
                                
#if (BLE_TRACE_SUPPORTED == 1)
                                printf("PROCESS-STOP\r\n");
#endif
                                
                                m_stop_callback();
                            }
                        }
                        break;
                    }
                        
                    case HCI_LE_CONNECTION_UPDATE_COMPLETE_EVENT: {
                        const hci_le_connection_update_complete_event_rp0 *eparam = (const hci_le_connection_update_complete_event_rp0*)&hci_le_event->data[0];
                        
                        if (m_central && (m_central->m_handle == eparam->Connection_Handle)) {
                            m_central->m_connection_interval = eparam->Conn_Interval;
                            m_central->m_slave_latency = eparam->Conn_Latency;
                            m_central->m_supervision_timeout = eparam->Supervision_Timeout;

#if (BLE_TRACE_SUPPORTED == 1)
                            printf("PROCESS-CONNECTION-UPDATE\r\n");
#endif
                            
                            m_central->m_connection_update_callback();
                        }
                        break;
                    }

                    case HCI_LE_LONG_TERM_KEY_REQUEST_EVENT: {
                        // #### negative reply ? or what ?
                        __BKPT();
                        break;
                    }
                        
                    case HCI_LE_READ_REMOTE_FEATURES_COMPLETE_EVENT:
                    case HCI_LE_DATA_LENGTH_CHANGE_EVENT:
                    case HCI_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE_EVENT:
                    case HCI_LE_GENERATE_DHKEY_COMPLETE_EVENT:
                    case HCI_LE_PHY_UPDATE_COMPLETE_EVENT:
                        break;
                    
                    default:
#if (BLE_TRACE_SUPPORTED == 1)
                        printf("PROCESS-UNEXPTECT HCI LE EVENT !!!\n");
#endif
                      //__BKPT();
                        break;
                    }
                    break;

                case HCI_VENDOR_SPECIFIC_EVENT:
                    hci_vs_event = (const evt_blue_aci*)&hci_event->data[0];

                    switch (hci_vs_event->ecode) {

                      /* HAL */
                    case ACI_HAL_END_OF_RADIO_ACTIVITY_EVENT:
                    case ACI_HAL_FW_ERROR_EVENT:
                        break;
                          
                      /* GAP */
                    case ACI_GAP_PAIRING_COMPLETE_EVENT: {
                        const aci_gap_pairing_complete_event_rp0 *eparam = (const aci_gap_pairing_complete_event_rp0*)&hci_vs_event->data[0];

                        armv7m_atomic_and(&m_process.sequence, ~BLE_EVENT_GAP_PAIRING_COMPLETE);

#if (BLE_TRACE_SUPPORTED == 1)
                        printf("PROCESS-PAIRING %02x\r\n", eparam->Status);
#endif
                        
                        if (eparam->Status == BLE_STATUS_SUCCESS) {
                        } else {
                            armv7m_atomic_or(&m_process.sequence, BLE_REQUEST_GAP_TERMINATE);
                        }
                        break;
                    }

                    case ACI_GAP_BOND_LOST_EVENT: {
                        if (m_peripheral.bonding) {
                            if (m_central) {
                                armv7m_atomic_or(&m_process.sequence, BLE_REQUEST_GAP_ALLOW_REBOND);
                            }
                        } else {
                            armv7m_atomic_or(&m_process.sequence, BLE_REQUEST_GAP_TERMINATE);
                        }
                        break;
                    }

                    case ACI_GAP_LIMITED_DISCOVERABLE_EVENT: {
                        if (m_peripheral.advertising) {
                            m_peripheral.advertising = false;

#if (BLE_TRACE_SUPPORTED == 1)
                            printf("PROCESS-STOP\r\n");
#endif
                            
                            m_stop_callback();
                        }
                        break;
                    }
                        
                    case ACI_GAP_SLAVE_SECURITY_INITIATED_EVENT:
                    case ACI_GAP_PROC_COMPLETE_EVENT:
                        break;

                    case ACI_GAP_ADDR_NOT_RESOLVED_EVENT:
                        // ##### ????
                        __BKPT();
                        break;
                        
                    case ACI_GAP_AUTHORIZATION_REQ_EVENT:
                    case ACI_GAP_PASS_KEY_REQ_EVENT:
                    case ACI_GAP_NUMERIC_COMPARISON_VALUE_EVENT:
                    case ACI_GAP_KEYPRESS_NOTIFICATION_EVENT:
#if (BLE_TRACE_SUPPORTED == 1)
                        printf("PROCESS-UNEXPTECT ACI GAP EVENT !!!\n");
#endif
                      // __BKPT();
                        break;
                        
                        
                      /* L2CAP */
                    case ACI_L2CAP_COMMAND_REJECT_EVENT:
                    case ACI_L2CAP_PROC_TIMEOUT_EVENT:
                        break;
                      
                      /* ATT */
                    case ACI_ATT_EXCHANGE_MTU_RESP_EVENT: {
                        const aci_att_exchange_mtu_resp_event_rp0 *eparam = (const aci_att_exchange_mtu_resp_event_rp0*)&hci_vs_event->data[0];
                        
                        if (m_central && (m_central->m_handle == eparam->Connection_Handle)) {
                            m_central->m_mtu = eparam->Server_RX_MTU;
                        }
                        break;
                    }
                      
                    case ACI_GATT_ATTRIBUTE_MODIFIED_EVENT: {
                        const aci_gatt_attribute_modified_event_rp0 *eparam = (const aci_gatt_attribute_modified_event_rp0 *)&hci_vs_event->data[0];

#if (BLE_TRACE_SUPPORTED == 1)
                        printf("PROCESS-MODIFIED-EVENT %04x\r\n", eparam->Attr_Handle);
#endif
                        
                        for (index = 0; index < BLE_NUM_GATT_SERVICES; index++) {
                            service = m_process.service_table[index];

                            if (service && (service->m_server.attrib[0] <= eparam->Attr_Handle) && (service->m_server.attrib[1] >= eparam->Attr_Handle)) {
                                for (characteristic = service->m_server.children; characteristic; characteristic = characteristic->m_server.sibling) {
                                    if (eparam->Attr_Handle == (characteristic->m_server.handle + BLE_VALUE_ATTRIB_HANDLE_OFFSET)) {
                                        if (!characteristic->m_written) {
                                            memcpy(&characteristic->m_value[characteristic->m_value_size + (eparam->Offset & 0x7fff)],
                                                   &eparam->Attr_Data[0],
                                                   (eparam->Attr_Data_Length - (eparam->Offset & 0x7fff)));
                                            
                                            if (!(eparam->Offset & 0x8000)) {
                                                armv7m_atomic_storeh(&characteristic->m_written, (0x8000 | eparam->Attr_Data_Length));
                                                
                                                if (!characteristic->m_busy) {
#if (BLE_TRACE_SUPPORTED == 1)
                                                    printf("PROCESS-WRITTEN (\"%s\")\r\n", characteristic->m_uuid);
#endif
                                                    characteristic->m_written_callback();
                                                }
                                            }
                                        }
                                        break;
                                    }

                                    if (characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) {
                                        if (eparam->Attr_Handle == (characteristic->m_server.handle + BLE_CCCD_ATTRIB_HANDLE_OFFSET)) {
                                          characteristic->m_sync_subscribed = false;
                                          if (characteristic->m_subscribed != (eparam->Attr_Data[0] & 3)) {
                                                characteristic->m_subscribed = (eparam->Attr_Data[0] & 3);
                                                
#if (BLE_TRACE_SUPPORTED == 1)
                                                printf("PROCESS-SUBSCRIBED %02x (\"%s\")\r\n", characteristic->m_subscribed, characteristic->m_uuid);
#endif
                                                
                                                characteristic->m_subscribed_callback();
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        break;
                    }

                    case ACI_GATT_PROC_TIMEOUT_EVENT: {
                        armv7m_atomic_or(&m_process.sequence, BLE_REQUEST_GAP_TERMINATE);
                        break;
                    }

                    case ACI_GATT_READ_PERMIT_REQ_EVENT: {
                        const aci_gatt_read_permit_req_event_rp0 *eparam = (const aci_gatt_read_permit_req_event_rp0 *)&hci_vs_event->data[0];

#if (BLE_TRACE_SUPPORTED == 1)
                        printf("PROCESS-READ-PERMIT-EVENT %04x\r\n", eparam->Attribute_Handle);
#endif
                        
                        for (index = 0; index < BLE_NUM_GATT_SERVICES; index++) {
                            service = m_process.service_table[index];

                            if (service && (service->m_server.attrib[0] <= eparam->Attribute_Handle) && (service->m_server.attrib[1] >= eparam->Attribute_Handle)) {
                                for (characteristic = service->m_server.children; characteristic; characteristic = characteristic->m_server.sibling) {
                                    if (eparam->Attribute_Handle == (characteristic->m_server.handle + BLE_VALUE_ATTRIB_HANDLE_OFFSET)) {
#if (BLE_TRACE_SUPPORTED == 1)
                                        printf("PROCESS-READ (\"%s\")\r\n", characteristic->m_uuid);
#endif
                                        characteristic->m_read_callback();

                                        if (characteristic->m_update_value) {
                                            characteristic->m_update_value = false;
                                            characteristic->m_sync_value = true;
                                            m_process.sync_value = true;
                                        }
                                        break;
                                    }
                                }
                            }
                        }

                        armv7m_atomic_or(&m_process.sequence, BLE_REQUEST_GATT_ALLOW_READ);
                        break;
                    }

                    case ACI_GATT_READ_MULTI_PERMIT_REQ_EVENT: {
                        const aci_gatt_read_multi_permit_req_event_rp0 *eparam = (const aci_gatt_read_multi_permit_req_event_rp0 *)&hci_vs_event->data[0];

                        for (entry = 0; entry < eparam->Number_of_Handles; entry++) {
#if (BLE_TRACE_SUPPORTED == 1)
                            printf("PROCESS-READ-MULTI-PERMIT-EVENT %04x\r\n", eparam->Handle_Item[entry].Handle);
#endif
                        
                            for (index = 0; index < BLE_NUM_GATT_SERVICES; index++) {
                                service = m_process.service_table[index];
                                
                                if (service && (service->m_server.attrib[0] <= eparam->Handle_Item[entry].Handle) && (service->m_server.attrib[1] >= eparam->Handle_Item[entry].Handle)) {
                                    for (characteristic = service->m_server.children; characteristic; characteristic = characteristic->m_server.sibling) {
                                        if (eparam->Handle_Item[entry].Handle == (characteristic->m_server.handle + BLE_VALUE_ATTRIB_HANDLE_OFFSET)) {
#if (BLE_TRACE_SUPPORTED == 1)
                                            printf("PROCESS-READ (\"%s\")\r\n", characteristic->m_uuid);
#endif
                                            characteristic->m_read_callback();
                                            
                                            if (characteristic->m_update_value) {
                                                characteristic->m_update_value = false;
                                                characteristic->m_sync_value = true;
                                                m_process.sync_value = true;
                                            }
                                            break;
                                        }
                                    }
                                }
                            }
                        }

                        armv7m_atomic_or(&m_process.sequence, BLE_REQUEST_GATT_ALLOW_READ);
                        break;
                    }
                      
                    case ACI_GATT_TX_POOL_AVAILABLE_EVENT: {
                        armv7m_atomic_and(&m_process.sequence, ~BLE_EVENT_GATT_TX_POOL_AVAILABLE);
                        break;
                    }
                        
                    case ACI_GATT_SERVER_CONFIRMATION_EVENT: {
                        if (m_process.sequence & BLE_EVENT_GATT_SERVER_CONFIRMATION) {
                            armv7m_atomic_and(&m_process.sequence, ~BLE_EVENT_GATT_SERVER_CONFIRMATION);

                            characteristic = m_process.request_current;
                            m_process.request_current = nullptr;

#if (BLE_TRACE_SUPPORTED == 1)
                            printf("PROCESS-DONE %02x (\"%s\", %02x)\r\n", BLE_STATUS_SUCCESS, characteristic->m_uuid, characteristic->m_update_type);
#endif
                            
                            armv7m_atomic_storeb(&characteristic->m_busy, 0);
                            
                            if (characteristic->m_status) {
                                *characteristic->m_status = BLE_STATUS_SUCCESS;
                            }

                            characteristic->m_done_callback();
                        }
                        break;
                    }

                    case ACI_GATT_INDICATION_EVENT: {
                        const aci_gatt_indication_event_rp0 *eparam = (const aci_gatt_indication_event_rp0*)&hci_vs_event->data[0];

                        if (m_central && (m_central->m_handle == eparam->Connection_Handle)) {
                            if (eparam->Attribute_Handle == (BLE_SERVICE_CHANGED_HANDLE + BLE_VALUE_ATTRIB_HANDLE_OFFSET)) {
                                armv7m_atomic_or(&m_process.sequence, BLE_REQUEST_GATT_CONFIRM_INDICATION);
                            }
                        }
                        break;
                    }
                    case ACI_GATT_PROC_COMPLETE_EVENT:
                    case ACI_GATT_WRITE_PERMIT_REQ_EVENT:
                    case ACI_GATT_PREPARE_WRITE_PERMIT_REQ_EVENT:
                    default:
#if (BLE_TRACE_SUPPORTED == 1)
                        printf("PROCESS-UNEXPECTED ACI ATT/GATT EVENT !!!\n");
#endif
                      // __BKPT();
                        break;
                    }
                    break;

                case HCI_COMMAND_COMPLETE_EVENT:
                case HCI_COMMAND_STATUS_EVENT:
                default:
#if (BLE_TRACE_SUPPORTED == 1)
                        printf("PROCESS-UNEXPTECT HCI EVENT !!!\n");
#endif
                    __BKPT();
                    break;
                }
            }
            
            event = (const hci_uart_pckt*)stm32wb_ipcc_ble_event();
        } while (event);
    }
    
    if (m_process.request_submit) {
        request_submit = (class BLEServerCharacteristic *)armv7m_atomic_swap((volatile uint32_t*)&m_process.request_submit, (uint32_t)nullptr);

        for (request_head = nullptr, request_tail = request_submit; request_submit != nullptr; request_submit = request_next)
        {
            request_next = request_submit->m_process.request;

            armv7m_atomic_store((volatile uint32_t*)&request_submit->m_process.request, (uint32_t)request_head);

            request_head = request_submit;
        }
        
        if (!m_process.request_head)
        {
            m_process.request_head = request_head;
        }
        else
        {
            armv7m_atomic_store((volatile uint32_t*)&m_process.request_tail->m_process.request, (uint32_t)request_head);
        }
        
        m_process.request_tail = request_tail;
    }
    
    if (!m_process.request_current && m_process.request_head) {
        characteristic = m_process.request_head;
            
        if (m_process.request_head == m_process.request_tail)
        {
            m_process.request_head = NULL;
            m_process.request_tail = NULL;
        }
        else
        {
            m_process.request_head = characteristic->m_process.request;
        }

        m_process.request_current = characteristic;
        m_process.request_offset = 0;
        m_process.request_count = 0;
        m_process.request_length = characteristic->m_value_length;
    }

    if (!m_process.value_current && m_process.sync_value) {
        m_process.sync_value = false;

        for (index = 0; index < BLE_NUM_GATT_SERVICES; index++) {
            service = m_process.service_table[index];
                                    
            if (service) {
                for (characteristic = service->m_server.children; characteristic; characteristic = characteristic->m_server.sibling) {
                    if (characteristic->m_sync_value) {
                        m_process.value_current = characteristic;
                        m_process.value_offset = 0;
                        m_process.value_count = 0;
                        m_process.value_length = characteristic->m_value_length;
                        m_process.sync_value = true;
                            
                        characteristic->m_sync_value = false;
                        break;
                    }
                }
            }

            if (m_process.value_current) {
                break;
            }
        }
    }

    if (!m_process.subscribed_current && m_process.sync_subscribed) {
        m_process.sync_subscribed = false;

        for (index = 0; index < BLE_NUM_GATT_SERVICES; index++) {
            service = m_process.service_table[index];
                                    
            if (service) {
                for (characteristic = service->m_server.children; characteristic; characteristic = characteristic->m_server.sibling) {
                    if (characteristic->m_sync_subscribed) {
                        m_process.subscribed_current = characteristic;
                        m_process.sync_subscribed = true;

                        characteristic->m_sync_subscribed = false;
                        break;
                    }
                }
            }

            if (m_process.subscribed_current) {
                break;
            }
        }
    }
    
    if (!(m_process.sequence & BLE_BUSY_MASK)) {
        if (m_process.sequence & BLE_REQUEST_GAP_TERMINATE) {
            aci_gap_terminate_cp0 *cparam = (aci_gap_terminate_cp0*)m_process.command.cparam;
            
            armv7m_atomic_and(&m_process.sequence, ~BLE_REQUEST_GAP_TERMINATE);
            
            cparam->Connection_Handle = m_process.connection;
            cparam->Reason = HCI_CONNECTION_TERMINATED_BY_LOCAL_HOST_ERR_CODE;
            
            m_process.command.opcode = ACI_GAP_TERMINATE;
            m_process.command.clen = sizeof(aci_gap_terminate_cp0);
            
            stm32wb_ipcc_ble_command(&m_process.command);
            
            armv7m_atomic_or(&m_process.sequence, BLE_BUSY_GAP_TERMINATE);
        } else {
            if (m_process.sequence & BLE_REQUEST_GATT_CONFIRM_INDICATION) {
                aci_gatt_confirm_indication_cp0 *cparam = (aci_gatt_confirm_indication_cp0*)m_process.command.cparam;
            
                armv7m_atomic_and(&m_process.sequence, ~BLE_REQUEST_GATT_CONFIRM_INDICATION);
            
                cparam->Connection_Handle = m_central->m_handle;
            
                m_process.command.opcode = ACI_GATT_CONFIRM_INDICATION;
                m_process.command.clen = sizeof(aci_gatt_confirm_indication_cp0);
            
                stm32wb_ipcc_ble_command(&m_process.command);
            
                armv7m_atomic_or(&m_process.sequence, BLE_BUSY_GATT_CONFIRM_INDICATION);
            }

            else if (m_process.sequence & BLE_REQUEST_GAP_SLAVE_SECURITY) {
                aci_gap_slave_security_req_cp0 *cparam = (aci_gap_slave_security_req_cp0*)m_process.command.cparam;

                armv7m_atomic_and(&m_process.sequence, ~BLE_REQUEST_GAP_SLAVE_SECURITY);
            
                cparam->Connection_Handle = m_central->m_handle;
            
                m_process.command.opcode = ACI_GAP_SLAVE_SECURITY_REQ;
                m_process.command.clen = sizeof(aci_gap_slave_security_req_cp0);
            
                stm32wb_ipcc_ble_command(&m_process.command);
            
                armv7m_atomic_or(&m_process.sequence, BLE_BUSY_GAP_SLAVE_SECURITY);
            }

            else if (m_process.sequence & BLE_REQUEST_GAP_ALLOW_REBOND) {
                aci_gap_allow_rebond_cp0 *cparam = (aci_gap_allow_rebond_cp0*)m_process.command.cparam;
            
                armv7m_atomic_and(&m_process.sequence, ~BLE_REQUEST_GAP_ALLOW_REBOND);
            
                cparam->Connection_Handle = m_central->m_handle;
            
                m_process.command.opcode = ACI_GAP_ALLOW_REBOND;
                m_process.command.clen = sizeof(aci_gap_allow_rebond_cp0);
            
                stm32wb_ipcc_ble_command(&m_process.command);
            
                armv7m_atomic_or(&m_process.sequence, BLE_BUSY_GAP_ALLOW_REBOND);
            }
    
            else if (m_process.sequence & BLE_REQUEST_L2CAP_CONNECTION_PARAMETER_UPDATE) {
                aci_l2cap_connection_parameter_update_req_cp0 *cparam = (aci_l2cap_connection_parameter_update_req_cp0*)m_process.command.cparam;

                armv7m_atomic_and(&m_process.sequence, ~BLE_REQUEST_L2CAP_CONNECTION_PARAMETER_UPDATE);
            
                cparam->Connection_Handle = m_central->m_handle;
                cparam->Conn_Interval_Min = m_process.connection_interval_min;
                cparam->Conn_Interval_Max = m_process.connection_interval_max;
                cparam->Slave_latency = m_process.slave_latency;
                cparam->Timeout_Multiplier = m_process.supervision_timeout;

                m_process.command.opcode = ACI_L2CAP_CONNECTION_PARAMETER_UPDATE_REQ;
                m_process.command.clen = sizeof(aci_l2cap_connection_parameter_update_req_cp0);
            
                stm32wb_ipcc_ble_command(&m_process.command);
            
                armv7m_atomic_or(&m_process.sequence, BLE_BUSY_L2CAP_CONNECTION_PARAMETER_UPDATE);
            }
        
            else if (m_process.sequence & BLE_REQUEST_HCI_LE_SET_PHY) {
                hci_le_set_phy_cp0 *cparam = (hci_le_set_phy_cp0*)m_process.command.cparam;
            
                armv7m_atomic_and(&m_process.sequence, ~BLE_REQUEST_HCI_LE_SET_PHY);
            
                cparam->Connection_Handle = m_central->m_handle;
                cparam->ALL_PHYS = 0;
                cparam->TX_PHYS = m_tx_phy;
                cparam->RX_PHYS = m_rx_phy;
                cparam->PHY_options = m_phy_options;
            
                m_process.command.opcode = HCI_LE_SET_PHY;
                m_process.command.clen = sizeof(hci_le_set_phy_cp0);
            
                stm32wb_ipcc_ble_command(&m_process.command);
            
                armv7m_atomic_or(&m_process.sequence, BLE_BUSY_HCI_LE_SET_PHY);
            }

            else if (m_process.request_current && !(m_process.sequence & (BLE_EVENT_GATT_TX_POOL_AVAILABLE | BLE_EVENT_GATT_SERVER_CONFIRMATION))) {
                aci_gatt_update_char_value_ext_cp0 *cparam = (aci_gatt_update_char_value_ext_cp0*)m_process.command.cparam;

                characteristic = m_process.request_current;
                
                m_process.request_count = m_process.request_length - m_process.request_offset;
            
                if (m_process.request_count > 243) {
                    m_process.request_count = 243;
                }
            
                cparam->Conn_Handle_To_Notify = m_central ? m_central->m_handle : 0x0000;
                cparam->Service_Handle = characteristic->m_server.parent->m_server.handle;
                cparam->Char_Handle = characteristic->m_server.handle;
                cparam->Update_Type = (m_central && ((m_process.request_offset + m_process.request_count) == m_process.request_length)) ? characteristic->m_update_type : 0x00;
                cparam->Char_Length = m_process.request_length;
                cparam->Value_Offset = m_process.request_offset;
                cparam->Value_Length = m_process.request_count;
                memcpy(&cparam->Value[0], characteristic->m_value, m_process.request_count);
            
                m_process.command.opcode = ACI_GATT_UPDATE_CHAR_VALUE_EXT;
                m_process.command.clen = 12 + m_process.request_count;
            
                stm32wb_ipcc_ble_command(&m_process.command);
            
                armv7m_atomic_or(&m_process.sequence, BLE_BUSY_GATT_UPDATE_CHAR_VALUE_EXT);
            }

            else if (m_process.value_current) {
                aci_gatt_update_char_value_ext_cp0 *cparam = (aci_gatt_update_char_value_ext_cp0*)m_process.command.cparam;

                characteristic = m_process.value_current;
                
                m_process.value_count = m_process.value_length - m_process.value_offset;
            
                if (m_process.value_count > 243) {
                    m_process.value_count = 243;
                }
            
                cparam->Conn_Handle_To_Notify = m_central ? m_central->m_handle : 0x0000;
                cparam->Service_Handle = characteristic->m_server.parent->m_server.handle;
                cparam->Char_Handle = characteristic->m_server.handle;
                cparam->Update_Type = 0x00;
                cparam->Char_Length = m_process.value_length;
                cparam->Value_Offset = m_process.value_offset;
                cparam->Value_Length = m_process.value_count;
                memcpy(&cparam->Value[0], characteristic->m_value, m_process.value_count);
            
                m_process.command.opcode = ACI_GATT_UPDATE_CHAR_VALUE_EXT;
                m_process.command.clen = 12 + m_process.value_count;
            
                stm32wb_ipcc_ble_command(&m_process.command);
            
                armv7m_atomic_or(&m_process.sequence, BLE_BUSY_GATT_UPDATE_CHAR_VALUE_EXT);
            }
            
            else if (m_process.subscribed_current) {
                aci_gatt_read_handle_value_cp0 *cparam = (aci_gatt_read_handle_value_cp0*)m_process.command.cparam;

                characteristic = m_process.subscribed_current;

                cparam->Attr_Handle = characteristic->m_server.handle + BLE_CCCD_ATTRIB_HANDLE_OFFSET;
                cparam->Offset = 0;
                cparam->Value_Length_Requested = 1;

                m_process.command.opcode = ACI_GATT_READ_HANDLE_VALUE;
                m_process.command.clen = sizeof(aci_gatt_read_handle_value_cp0);
                
                stm32wb_ipcc_ble_command(&m_process.command);
                
                armv7m_atomic_or(&m_process.sequence, BLE_BUSY_GATT_READ_HANDLE_VALUE);
            }

            else if (m_process.sequence & BLE_REQUEST_GATT_ALLOW_READ) {
                aci_gatt_allow_read_cp0 *cparam = (aci_gatt_allow_read_cp0*)m_process.command.cparam;

                armv7m_atomic_and(&m_process.sequence, ~BLE_REQUEST_GATT_ALLOW_READ);
                
                cparam->Connection_Handle = m_central ? m_central->m_handle : 0x0000;
            
                m_process.command.opcode = ACI_GATT_ALLOW_READ;
                m_process.command.clen = sizeof(aci_gatt_allow_read_cp0);
            
                stm32wb_ipcc_ble_command(&m_process.command);
            
                armv7m_atomic_or(&m_process.sequence, BLE_BUSY_GATT_ALLOW_READ);
            }
        }
    }

#if (BLE_TRACE_SUPPORTED == 1)
    printf("PROCESS-EXIT %08x\r\n", (unsigned int)m_process.sequence);
#endif
}

void BLELocalDevice::done() {
    k_work_submit(&m_process.work);
}

void BLELocalDevice::event() {
    k_work_submit(&m_process.work);
}

/**********************************************************************************
 * BLE API
 */

BLEDescriptorInstance *BLEDescriptor::instance(const char *uuid, uint8_t permissions, const void *value, int valueLength) {
    BLEDescriptorInstance *instance;
    uint8_t *data;
    
    uuid = (const char*)allocate_noconst(uuid, strlen(uuid) +1);

    data = (uint8_t*)allocate_noconst(value, valueLength);
    
    if (uuid && value) {
        instance = new BLEServerDescriptor(uuid, permissions, data, valueLength);
    } else {
        instance = nullptr;
    }

    if (instance == nullptr) {
        if (uuid) {
            deallocate_noconst(uuid);
        }

        if (data) {
            deallocate_noconst(data);
        }

        instance = &BLENullDescriptor;
    }

    return instance;
}

BLEDescriptor::BLEDescriptor() {
    m_instance = &BLENullDescriptor;
}

BLEDescriptor::BLEDescriptor(const char *uuid, uint8_t permissions, const void *value, int valueLength) {
    m_instance = instance(uuid, permissions, value, valueLength);
}

BLEDescriptor::BLEDescriptor(const char *uuid, uint8_t permissions, const char *value) {
    m_instance = instance(uuid, permissions, (const void*)value, strlen(value));
}

BLEDescriptor::BLEDescriptor(const char *uuid, const char *value) {
    m_instance = instance(uuid, 0, (const void*)value, strlen(value));
}

BLEDescriptor::~BLEDescriptor() {
    (*m_instance).unreference();
}

BLEDescriptor::BLEDescriptor(const BLEDescriptor &other) {
    m_instance = other.m_instance;

    (*m_instance).reference();
}

BLEDescriptor &BLEDescriptor::operator=(const BLEDescriptor &other) {
    BLEDescriptorInstance *descriptor = m_instance;

    m_instance = other.m_instance;

    (*m_instance).reference();

    descriptor->unreference();

    return *this;
}

BLEDescriptor::BLEDescriptor(BLEDescriptor &&other) {
    m_instance = other.m_instance;

    other.m_instance = &BLENullDescriptor;
}

BLEDescriptor &BLEDescriptor::operator=(BLEDescriptor &&other) {
    BLEDescriptorInstance *descriptor = m_instance;

    m_instance = other.m_instance;

    other.m_instance = &BLENullDescriptor;

    descriptor->unreference();

    return *this;
}

BLEDescriptor::BLEDescriptor(BLEDescriptorInstance *instance) {
    m_instance = instance;

    instance->reference();
}

BLEDescriptorInstance *BLEDescriptor::instance() {
    return m_instance;
}

BLEDescriptor::operator bool() const {
    return m_instance != &BLENullDescriptor;
}

const char *BLEDescriptor::uuid() const {
    return (*m_instance).uuid();
}

uint8_t BLEDescriptor::permissions() const {
    return (*m_instance).permissions();
}

int BLEDescriptor::valueLength() const {
    return (*m_instance).valueLength();
}

const void * BLEDescriptor::value() const {
    return (*m_instance).value();
}

int BLEDescriptor::getValue(void *value, int length) const {
    return (*m_instance).getValue(value, length);
}


BLECharacteristicInstance *BLECharacteristic::instance(const char *uuid, uint8_t properties, uint8_t permissions, const void *value, int valueLength, int valueSize, bool fixedLength, bool constant) {
    BLECharacteristicInstance *instance;
    uint8_t *data;
    
    if (constant) {
        if (properties & (BLE_PROPERTY_WRITE_WITHOUT_RESPONSE | BLE_PROPERTY_WRITE | BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) {
            return &BLENullCharacteristic;
        }

        data = (uint8_t*)allocate_noconst(value, valueSize);
    } else {
        data = (uint8_t*)malloc(((properties & (BLE_PROPERTY_WRITE_WITHOUT_RESPONSE | BLE_PROPERTY_WRITE)) ? (2 * valueSize) : valueSize));

        if (value) {
            memcpy(data, value, valueLength);
        }
    }

    uuid = (const char*)allocate_noconst(uuid, strlen(uuid) +1);
    
    if (uuid && data) {
        instance = new BLEServerCharacteristic(uuid, properties, permissions, data, valueLength, valueSize, fixedLength, constant);
    } else {
        instance = nullptr;
    }

    if (instance == nullptr) {
        if (uuid) {
            deallocate_noconst(uuid);
        }
        
        if (data) {
            deallocate_noconst(data);
        }
        
        instance = &BLENullCharacteristic;
    }

    return instance;
}

BLECharacteristic::BLECharacteristic() {
    m_instance = &BLENullCharacteristic;
}

BLECharacteristic::BLECharacteristic(const char *uuid, uint8_t properties, uint8_t permissions, const void *value, int valueLength, int valueSize, bool fixedLength) {
    m_instance = instance(uuid, properties, permissions, value, valueLength, valueSize, fixedLength, false);
}

BLECharacteristic::BLECharacteristic(const char *uuid, uint8_t properties, uint8_t permissions, int valueSize, bool fixedLength) {
    m_instance = instance(uuid, properties, permissions, NULL, 0, valueSize, fixedLength, false);
}

BLECharacteristic::BLECharacteristic(const char* uuid, uint8_t properties, uint8_t permissions, const char* value) {
    int valueSize = strlen(value);

    m_instance = instance(uuid, properties, permissions, (uint8_t*)value, valueSize, valueSize, true, true);
}

BLECharacteristic::BLECharacteristic(const char* uuid, uint8_t properties, const char* value) {
    int valueSize = strlen(value);

    m_instance = instance(uuid, properties, 0, (uint8_t*)value, valueSize, valueSize, true, true);
}

BLECharacteristic::~BLECharacteristic() {
    (*m_instance).unreference();
}

BLECharacteristic::BLECharacteristic(const BLECharacteristic &other) {
    m_instance = other.m_instance;

    (*m_instance).reference();
}

BLECharacteristic &BLECharacteristic::operator=(const BLECharacteristic &other) {
    BLECharacteristicInstance *characteristic = m_instance;

    m_instance = other.m_instance;

    (*m_instance).reference();

    characteristic->unreference();

    return *this;
}

BLECharacteristic::BLECharacteristic(BLECharacteristic &&other) {
    m_instance = other.m_instance;

    other.m_instance = &BLENullCharacteristic;
}

BLECharacteristic &BLECharacteristic::operator=(BLECharacteristic &&other) {
    BLECharacteristicInstance *characteristic = m_instance;

    m_instance = other.m_instance;

    other.m_instance = &BLENullCharacteristic;

    characteristic->unreference();

    return *this;
}

BLECharacteristic::BLECharacteristic(BLECharacteristicInstance *instance) {
    m_instance = instance;

    instance->reference();
}

BLECharacteristicInstance *BLECharacteristic::instance() {
    return m_instance;
}

BLECharacteristic::operator bool() const {
    return m_instance != &BLENullCharacteristic;
}

const char *BLECharacteristic::uuid() const {
    return (*m_instance).uuid();
}

uint8_t BLECharacteristic::properties() const {
    return (*m_instance).properties();
}

uint8_t BLECharacteristic::permissions() const {
    return (*m_instance).permissions();
}

bool BLECharacteristic::fixedLength() const {
    return (*m_instance).fixedLength();
}

int BLECharacteristic::valueSize() const {
    return (*m_instance).valueSize();
}

int BLECharacteristic::valueLength() {
    return (*m_instance).valueLength();
}

const void * BLECharacteristic::value() {
    return (*m_instance).value();
}

int BLECharacteristic::getValue(void *value, int length) {
    return (*m_instance).getValue(value, length);
}

bool BLECharacteristic::setValue(const void *value, int length) {
    return (*m_instance).setValue(value, length);
}

bool BLECharacteristic::writeValue(const void *value, int length) {
    return (*m_instance).writeValue(value, length, NULL, Callback(__emptyCallback));
}

bool BLECharacteristic::writeValue(const void *value, int length, volatile uint8_t &status) {
    return (*m_instance).writeValue(value, length, &status, Callback(__wakeupCallback));
}

bool BLECharacteristic::writeValue(const void *value, int length, volatile uint8_t &status, void(*callback)(void)) {
    return (*m_instance).writeValue(value, length, &status, Callback(callback));
}

bool BLECharacteristic::writeValue(const void *value, int length, volatile uint8_t &status, Callback callback) {
    return (*m_instance).writeValue(value, length, &status, callback);
}

bool BLECharacteristic::busy() {
    return (*m_instance).busy();
}

bool BLECharacteristic::written() {
    return (*m_instance).written();
}

bool BLECharacteristic::subscribed() {
    return (*m_instance).subscribed();
}

unsigned int BLECharacteristic::descriptorCount() {
    return (*m_instance).descriptorCount();
}

BLEDescriptor BLECharacteristic::descriptor(unsigned int index) {
    return (*m_instance).descriptor(index);
}

bool BLECharacteristic::addDescriptor(BLEDescriptor &descriptor) {
    return (*m_instance).addDescriptor(descriptor);
}

bool BLECharacteristic::removeDescriptor(BLEDescriptor &descriptor) {
    return (*m_instance).removeDescriptor(descriptor);
}

void BLECharacteristic::onRead(void(*callback)(void)) {
    (*m_instance).onRead(Callback(callback));
}

void BLECharacteristic::onRead(Callback callback) {
    (*m_instance).onRead(callback);
}

void BLECharacteristic::onWritten(void(*callback)(void)) {
    (*m_instance).onWritten(Callback(callback));
}

void BLECharacteristic::onWritten(Callback callback) {
    (*m_instance).onWritten(callback);
}

void BLECharacteristic::onSubscribed(void(*callback)(void)) {
    (*m_instance).onSubscribed(Callback(callback));
}

void BLECharacteristic::onSubscribed(Callback callback) {
    (*m_instance).onSubscribed(callback);
}


BLEServiceInstance *BLEService::instance(const char *uuid) {
    BLEServiceInstance *instance;
    uuid = (const char*)allocate_noconst(uuid, strlen(uuid) +1);

    if (uuid) {
        instance = new BLEServerService(uuid);
    } else {
        instance = nullptr;
    }

    if (instance == nullptr) {
        if (uuid) {
            deallocate_noconst(uuid);
        }

        instance = &BLENullService;
    }

    return instance;
}

BLEService::BLEService() {
    m_instance = &BLENullService;
}

BLEService::BLEService(const char *uuid) {
    m_instance = instance(uuid);
}

BLEService::~BLEService() {
    (*m_instance).unreference();
}

BLEService::BLEService(const BLEService &other) {
    m_instance = other.m_instance;

    (*m_instance).reference();
}

BLEService &BLEService::operator=(const BLEService &other) {
    BLEServiceInstance *service = m_instance;

    m_instance = other.m_instance;

    (*m_instance).reference();

    service->unreference();

    return *this;
}

BLEService::BLEService(BLEService &&other) {
    m_instance = other.m_instance;

    other.m_instance = &BLENullService;
}

BLEService &BLEService::operator=(BLEService &&other) {
    BLEServiceInstance *service = m_instance;

    m_instance = other.m_instance;

    other.m_instance = &BLENullService;

    service->unreference();

    return *this;
}

BLEService::BLEService(BLEServiceInstance *instance) {
    m_instance = instance;

    instance->reference();
}

BLEServiceInstance *BLEService::instance() {
    return m_instance;
}

BLEService::operator bool() const {
    return m_instance != &BLENullService;
}

const char *BLEService::uuid() const {
    return (*m_instance).uuid();
}

unsigned int BLEService::characteristicCount() {
    return (*m_instance).characteristicCount();
}

BLECharacteristic BLEService::characteristic(unsigned int index) {
    return (*m_instance).characteristic(index);
}

bool BLEService::addCharacteristic(BLECharacteristic &characteristic) {
    return (*m_instance).addCharacteristic(characteristic);
}

bool BLEService::removeCharacteristic(BLECharacteristic &characteristic) {
    return (*m_instance).removeCharacteristic(characteristic);
}


BLEDevice::BLEDevice() {
    m_instance = &BLENullDevice;
}

BLEDevice::~BLEDevice() {
    m_instance->unreference();
}

BLEDevice::BLEDevice(const BLEDevice &other) {
    m_instance = other.m_instance;

    (*m_instance).reference();
}

BLEDevice &BLEDevice::operator=(const BLEDevice &other) {
    BLEDeviceInstance *device = m_instance;

    m_instance = other.m_instance;

    (*m_instance).reference();

    device->unreference();
    
    return *this;
}

BLEDevice::BLEDevice(BLEDevice &&other) {
    m_instance = other.m_instance;

    other.m_instance = &BLENullDevice;
}

BLEDevice &BLEDevice::operator=(BLEDevice &&other) {
    BLEDeviceInstance *device = m_instance;

    m_instance = other.m_instance;

    other.m_instance = &BLENullDevice;

    device->unreference();

    return *this;
}

BLEDevice::BLEDevice(BLEDeviceInstance *instance) {
    m_instance = instance;

    instance->reference();
}

BLEDeviceInstance *BLEDevice::instance() {
    return m_instance;
}

BLEDevice::operator bool() const {
    return m_instance != &BLENullDevice;
}

String BLEDevice::address() const {
    return (*m_instance).address();
}

bool BLEDevice::connected() {
    return (*m_instance).connected();
}

int BLEDevice::rssi() {
    return (*m_instance).rssi();
}

int BLEDevice::mtu() {
    return (*m_instance).mtu();
}

bool BLEDevice::setConnectionInterval(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval) {
    uint16_t slaveLatency, supervisionTimeout;

    slaveLatency = 0;
    supervisionTimeout = ((((1 + slaveLatency) * (2 * maximumConnectionInterval * 125 + 124)) / 100) + 9) / 10;

    if (supervisionTimeout < 0x00c8) {
        supervisionTimeout = 0x00c8;
    }
    
    return (*m_instance).setConnectionParameters(minimumConnectionInterval, maximumConnectionInterval, slaveLatency, supervisionTimeout);
}

bool BLEDevice::setConnectionParameters(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval, uint16_t slaveLatency) {
    uint16_t supervisionTimeout;
    
    supervisionTimeout = ((((1 + slaveLatency) * (2 * maximumConnectionInterval * 125 + 124)) / 100) + 9) / 10;

    if (supervisionTimeout < 0x00c8) {
        supervisionTimeout = 0x00c8;
    }
    
    return (*m_instance).setConnectionParameters(minimumConnectionInterval, maximumConnectionInterval, slaveLatency, supervisionTimeout);
}

bool BLEDevice::setConnectionParameters(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval, uint16_t slaveLatency, uint16_t supervisionTimeout) {
    return (*m_instance).setConnectionParameters(minimumConnectionInterval, maximumConnectionInterval, slaveLatency, supervisionTimeout);
}

void BLEDevice::getConnectionParameters(uint16_t &connectionInterval, uint16_t &slaveLatency, uint16_t &supervisionTimeout) {
    (*m_instance).getConnectionParameters(connectionInterval, slaveLatency, supervisionTimeout);
}

void BLEDevice::onConnectionUpdate(void(*callback)(void)) {
    (*m_instance).onConnectionUpdate(Callback(callback));
}

void BLEDevice::onConnectionUpdate(Callback callback) {
    (*m_instance).onConnectionUpdate(callback);
}


bool BLEClass::begin(int mtu, uint32_t options) {
    return BLEHost.begin(mtu, options);
}

void BLEClass::end() {
    BLEHost.end();
}

String BLEClass::address() {
    return BLEHost.address();
}

bool BLEClass::setTxPowerLevel(int txPower) {
    return BLEHost.setTxPowerLevel(txPower);
}

bool BLEClass::setPreferredPhy(BLEPhy txPhy, BLEPhy rxPhy, BLEPhyOption phyOptions) {
    return BLEHost.setPreferredPhy(txPhy, rxPhy, phyOptions);
}

void BLEClass::setDeviceName(const char *deviceName) {
    BLEHost.setDeviceName(deviceName);
}

void BLEClass::setAppearance(BLEAppearance appearance) {
    BLEHost.setAppearance(appearance);
}

bool BLEClass::setConnectionInterval(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval) {
    uint16_t slaveLatency, supervisionTimeout;

    slaveLatency = 0;
    supervisionTimeout = ((((1 + slaveLatency) * (2 * maximumConnectionInterval * 125 + 124)) / 100) + 9) / 10;

    if (supervisionTimeout < 0x00c8) {
        supervisionTimeout = 0x00c8;
    }
    
    return setConnectionParameters(minimumConnectionInterval, maximumConnectionInterval, slaveLatency, supervisionTimeout);
}

bool BLEClass::setConnectionParameters(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval, uint16_t slaveLatency) {
    uint16_t supervisionTimeout;

    supervisionTimeout = ((((1 + slaveLatency) * (2 * maximumConnectionInterval * 125 + 124)) / 100) + 9) / 10;

    if (supervisionTimeout < 0x00c8) {
        supervisionTimeout = 0x00c8;
    }

    return setConnectionParameters(minimumConnectionInterval, maximumConnectionInterval, slaveLatency, supervisionTimeout);
}

bool BLEClass::setConnectionParameters(uint16_t minimumConnectionInterval, uint16_t maximumConnectionInterval, uint16_t slaveLatency, uint16_t supervisionTimeout) {
    return BLEHost.setConnectionParameters(minimumConnectionInterval, maximumConnectionInterval, slaveLatency, supervisionTimeout);
}

unsigned int BLEClass::serviceCount() {
    return BLEHost.serviceCount();
}

BLEService BLEClass::service(unsigned int index) {
    return BLEHost.service(index);
}

bool BLEClass::addService(BLEService &service) {
    return BLEHost.addService(service);
}

bool BLEClass::removeService(BLEService &service) {
    return BLEHost.removeService(service);
}

bool BLEClass::setSecurity(BLESecurity security) {
    return BLEHost.setSecurity(security);
}

bool BLEClass::setBonding(bool bonding) {
    return BLEHost.setBonding(bonding);
}

bool BLEClass::setPin(const char *pin) {
    return BLEHost.setPin(pin);
}

void BLEClass::clearBondStorage() {
    BLEHost.clearBondStorage();
}

bool BLEClass::setIncludeTxPowerLevel(bool enable) {
    return BLEHost.setIncludeTxPowerLevel(enable);
}

bool BLEClass::setIncludeConnectionInterval(bool enable) {
    return BLEHost.setIncludeConnectionInterval(enable);
}

bool BLEClass::setLocalName(const char *localName) {
    return BLEHost.setLocalName(localName);
}

bool BLEClass::setAdvertisedServiceUuid(const char *uuid, bool complete) {
    return BLE.setAdvertisedServiceUuids(&uuid, 1, complete);
}

bool BLEClass::setAdvertisedServiceUuids(const char *uuids[], int count, bool complete) {
    return BLEHost.setAdvertisedServiceUuids(uuids, count, complete);
}

bool BLEClass::setManufacturerData(const uint8_t data[], int length) {
    return BLEHost.setManufacturerData(data, length);
}

bool BLEClass::setBeacon(const char *uuid, uint16_t major, uint16_t minor, int8_t rssi) {
    return BLEHost.setBeacon(uuid, major, minor, rssi);
}

bool BLEClass::setAdvertisingData(const uint8_t data[], int length) {
    return BLEHost.setAdvertisingData(data, length);
}

bool BLEClass::setScanResponseData(const uint8_t data[], int length) {
    return BLEHost.setScanResponseData(data, length);
}

bool BLEClass::setAdvertisingInterval(uint16_t advertisingInterval) {
    return BLE.setAdvertisingInterval(advertisingInterval, advertisingInterval);
}

bool BLEClass::setAdvertisingInterval(uint16_t minimumAdvertisingInterval, uint16_t maximumAdvertisingInterval) {
    return BLEHost.setAdvertisingInterval(minimumAdvertisingInterval, maximumAdvertisingInterval);
}

void BLEClass::setConnectable(bool connectable) {
    BLEHost.setConnectable(connectable);
}

void BLEClass::setDiscoverable(BLEDiscoverable discoverable) {
    BLEHost.setDiscoverable(discoverable);
}

bool BLEClass::advertise() {
    return BLEHost.advertise();
}

void BLEClass::stopAdvertise() {
    BLEHost.stopAdvertise();
}

bool BLEClass::advertising() {
    return BLEHost.advertising();
}

BLEDevice BLEClass::central() {
    return BLEHost.central();
}

bool BLEClass::connected() {
    return BLEHost.connected();
}

void BLEClass::disconnect() {
    return BLEHost.disconnect();
}

void BLEClass::onStop(void(*callback)(void)) {
    BLE.onStop(Callback(callback));
}

void BLEClass::onStop(Callback callback) {
    BLEHost.onStop(callback);
}

void BLEClass::onConnect(void(*callback)(void)) {
    BLE.onConnect(Callback(callback));
}

void BLEClass::onConnect(Callback callback) {
    BLEHost.onConnect(callback);
}

void BLEClass::onDisconnect(void(*callback)(void)) {
    BLE.onDisconnect(Callback(callback));
}

void BLEClass::onDisconnect(Callback callback) {
    BLEHost.onDisconnect(callback);
}

BLEClass BLE;




static const char *BLEUartServiceUUIDs[] = {
    "00000000-000E-11E1-9AB4-0002A5D5C51B",
    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E",
    "0000FFE0-0000-1000-8000-00805F9B34FB",
};

BLEUart::BLEUart(BLEUartProtocol protocol) :
    BLEService(BLEUartServiceUUIDs[(unsigned int)protocol]),
    m_protocol(protocol)
{
    switch (protocol) {
    case BLE_UART_PROTOCOL_BLUEST:
        m_packet_size = 20;

        m_rx_characteristic = BLECharacteristic("00000001-000E-11E1-AC36-0002A5D5C51B", (BLE_PROPERTY_WRITE_WITHOUT_RESPONSE | BLE_PROPERTY_NOTIFY), 0, m_packet_size, false);
        m_tx_characteristic = m_rx_characteristic;
        m_tx2_characteristic = BLECharacteristic("00000002-000E-11E1-AC36-0002A5D5C51B", BLE_PROPERTY_NOTIFY, 0, m_packet_size, false);

        addCharacteristic(m_rx_characteristic);
        addCharacteristic(m_tx2_characteristic);
        break;

    case BLE_UART_PROTOCOL_NORDIC:
        m_packet_size = 20;

        m_rx_characteristic = BLECharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLE_PROPERTY_WRITE_WITHOUT_RESPONSE, 0, m_packet_size, false);
        m_tx_characteristic = BLECharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLE_PROPERTY_NOTIFY, 0, m_packet_size, false);

        addCharacteristic(m_rx_characteristic);
        addCharacteristic(m_tx_characteristic);
        break;

    case BLE_UART_PROTOCOL_HMSOFT:
        m_packet_size = 20;

        m_rx_characteristic = BLECharacteristic("0000FFE1-0000-1000-8000-00805F9B34FB", (BLE_PROPERTY_WRITE_WITHOUT_RESPONSE | BLE_PROPERTY_NOTIFY), 0, m_packet_size, false);
        m_tx_characteristic = m_rx_characteristic;

        addCharacteristic(m_rx_characteristic);
        break;
    }
    
    m_rx_characteristic.onWritten(Callback(&BLEUart::receive, this));
    m_tx_characteristic.onSubscribed(Callback(&BLEUart::connect, this));

    m_receive_callback = Callback(__wakeupCallback);
}

BLEUart::~BLEUart() {
}

BLEUart::operator bool() {
    return m_connected;
}

int BLEUart::available() {
    return m_rx_count;
}

int BLEUart::peek() {
    if (m_rx_count == 0) {
        return -1;
    }
        
    return m_rx_data[m_rx_read];
}

int BLEUart::read() {
    uint32_t rx_read, rx_read_next;
    uint8_t data;

    do {
        if (!m_rx_count) {
            return -1;
        }
        
        rx_read = m_rx_read;
        rx_read_next = (unsigned int)(rx_read + 1) & (BLE_UART_RX_BUFFER_SIZE -1);
    } while (armv7m_atomic_cash(&m_rx_read, rx_read, rx_read_next) != rx_read);

    data = m_rx_data[rx_read];
    
    armv7m_atomic_subh(&m_rx_count, 1);

    return data;
}

int BLEUart::read(uint8_t *buffer, size_t size) {
    uint32_t count, rx_count, rx_read, rx_read_next;

    count = 0;

    while (count < size) {
        do {
            rx_count = m_rx_count;

            if (!rx_count) {
                return count;
            }
            
            rx_read = m_rx_read;
            
            if (rx_count > (BLE_UART_RX_BUFFER_SIZE - rx_read)) {
                rx_count = (BLE_UART_RX_BUFFER_SIZE - rx_read);
            }
            
            if (rx_count > (size - count)) {
                rx_count = (size - count);
            }

            rx_read_next = (unsigned int)(rx_read + count) & (BLE_UART_RX_BUFFER_SIZE -1);
        } while (armv7m_atomic_cash(&m_rx_read, rx_read, rx_read_next) != rx_read);
    
        memcpy(&buffer[count], &m_rx_data[rx_read], rx_count);
        count += rx_count;
    
        armv7m_atomic_subh(&m_rx_count, rx_count);
    }

    return count;
}

int BLEUart::availableForWrite() {
    return BLE_UART_TX_BUFFER_SIZE - m_tx_count;
}

size_t BLEUart::write(uint8_t data) {
    return write(&data, 1);
}

size_t BLEUart::write(const uint8_t *buffer, size_t size) {
    uint32_t count, tx_read, tx_write, tx_write_next, tx_count, tx_size;

    if (size == 0) {
        return 0;
    }

    if (!m_connected) {
        return size;
    }
    
    count = 0;

    while (count < size) {
        tx_count = BLE_UART_TX_BUFFER_SIZE - m_tx_count;

        if (!tx_count) {
            if (m_nonblocking || !armv7m_core_is_in_thread() || k_work_is_in_progress()) {
                break;
            }

            if (!armv7m_atomic_casb(&m_tx_busy, false, true) == false) {
                tx_size = m_tx_count;
                tx_read = m_tx_read;

                if (tx_size > (BLE_UART_TX_BUFFER_SIZE - tx_read)) {
                    tx_size = (BLE_UART_TX_BUFFER_SIZE - tx_read);
                }
                
                if (tx_size > m_packet_size) {
                    tx_size = m_packet_size;
                }
                
                m_tx_size = tx_size;
                
                while (!m_tx_characteristic.writeValue(&m_tx_data[tx_read], tx_size, m_status, Callback(&BLEUart::transmit, this))) { }
            }

            while (m_tx_busy) { __WFE(); }
        } else {
            tx_write = m_tx_write;
            
            if (tx_count > (BLE_UART_TX_BUFFER_SIZE - tx_write)) {
                tx_count = (BLE_UART_TX_BUFFER_SIZE - tx_write);
            }
            
            if (tx_count > (size - count)) {
                tx_count = (size - count);
            }
            
            tx_write_next = (unsigned int)(tx_write + tx_count) & (BLE_UART_TX_BUFFER_SIZE -1);
            
            if (armv7m_atomic_cash(&m_tx_write, tx_write, tx_write_next) == tx_write) {
                memcpy(&m_tx_data[tx_write], &buffer[count], tx_count);
                count += tx_count;

                armv7m_atomic_addh(&m_tx_count, tx_count);
            }
        }
    }

    if (armv7m_atomic_casb(&m_tx_busy, false, true) == false) {
        tx_size = m_tx_count;
        
        if (tx_size) {
            tx_read = m_tx_read;
            
            if (tx_size > (BLE_UART_TX_BUFFER_SIZE - tx_read)) {
                tx_size = (BLE_UART_TX_BUFFER_SIZE - tx_read);
            }
            
            if (tx_size > m_packet_size) {
                tx_size = m_packet_size;
            }
            
            m_tx_size = tx_size;
            
            if (!m_tx_characteristic.writeValue(&m_tx_data[tx_read], tx_size, m_status, Callback(&BLEUart::transmit, this))) {
                armv7m_atomic_storeb(&m_tx_busy, false);
            }
        } else {
            armv7m_atomic_storeb(&m_tx_busy, false);
        }
    }

    return count;
}

void BLEUart::flush() {
    if (armv7m_core_is_in_thread() && !k_work_is_in_progress()) {
        while (m_tx_busy) {
            __WFE();
        }
    }
}

void BLEUart::transmit() {
    unsigned int tx_read, tx_size;

    m_tx_read = (m_tx_read + m_tx_size) & (BLE_UART_TX_BUFFER_SIZE -1);
    armv7m_atomic_subh(&m_tx_count, m_tx_size);
        
    m_tx_size = 0;

    tx_size = m_tx_count;

    if (tx_size) {
        tx_read = m_tx_read;

        if (tx_size > (BLE_UART_TX_BUFFER_SIZE - tx_read)) {
            tx_size = (BLE_UART_TX_BUFFER_SIZE - tx_read);
        }
        
        if (tx_size > m_packet_size) {
            tx_size = m_packet_size;
        }
        
        m_tx_size = tx_size;
        
        while (!m_tx_characteristic.writeValue(&m_tx_data[tx_read], tx_size, m_status, Callback(&BLEUart::transmit, this))) {
            receive();
        }
    } else {
        armv7m_atomic_storeb(&m_tx_busy, false);
    }
}

void BLEUart::receive() {
    uint32_t rx_count, rx_size, rx_write;
    const uint8_t *rx_data;

    if (m_rx_characteristic.written()) {
        rx_count = m_rx_characteristic.valueLength();
        rx_data = (const uint8_t *)m_rx_characteristic.value();

        rx_write = m_rx_write;
        
        if (rx_count) {
            if (rx_count > (unsigned)(BLE_UART_RX_BUFFER_SIZE - m_rx_count)) {
                rx_count = (BLE_UART_RX_BUFFER_SIZE - m_rx_count);
            }
            
            rx_size = rx_count;
            
            if (rx_size > (BLE_UART_RX_BUFFER_SIZE - rx_write)) {
                rx_size = BLE_UART_RX_BUFFER_SIZE - rx_write;
            }
            
            if (rx_size) {
                memcpy(&m_rx_data[rx_write], &rx_data[0], rx_size);
                
                rx_count -= rx_size;
                rx_write += rx_size;
                
                if (rx_write == BLE_UART_RX_BUFFER_SIZE) {
                    rx_write = 0;
                }
            }
        
            if (rx_count) {
                memcpy(&m_rx_data[rx_write], &rx_data[rx_size], rx_count);
            
                rx_write += rx_count;
                
                if (rx_write == BLE_UART_RX_BUFFER_SIZE) {
                    rx_write = 0;
                }
            }

            armv7m_atomic_addh(&m_rx_count, (rx_size + rx_count));

            m_rx_write = rx_write;

            m_receive_callback();
        }
    }
}

void BLEUart::connect() {
    m_connected = !!m_tx_characteristic.subscribed();
}

void BLEUart::setNonBlocking(bool enabled) {
    m_nonblocking = enabled;
}

void BLEUart::onReceive(void(*callback)(void)) {
    onReceive(Callback(callback));
}

void BLEUart::onReceive(Callback callback) {
    m_receive_callback = (callback ? callback : Callback(__wakeupCallback));
}

