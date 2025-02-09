#include "Arduino.h"
#include "BLE.h"

typedef struct __attribute__((packed)) {
    uint16_t timestamp;
    uint8_t led;
} blue_st_led_t;

blue_st_led_t ledFeature;

BLEService sensorService("00000000-0001-11e1-9ab4-0002a5d5c51b");
BLECharacteristic ledCharacteristic("20000000-0001-11e1-ac36-0002a5d5c51b", (BLE_PROPERTY_READ | BLE_PROPERTY_NOTIFY), ledFeature);

BLEService commandService("00000000-000F-11e1-9ab4-0002a5d5c51b");
BLECharacteristic commandCharacteristic("00000002-000F-11e1-ac36-0002a5d5c51b", (BLE_PROPERTY_WRITE | BLE_PROPERTY_NOTIFY), 0, 20, true);

BLEOta OTA;

static const uint8_t manufacturer_data[6] = {
    0x01,
    0x00,
    (uint8_t)(0x20000000 >> 24),
    (uint8_t)(0x20000000 >> 16),
    (uint8_t)(0x20000000 >> 8),
    (uint8_t)(0x20000000 >> 0),
};

void subscribedCallback() {
     if (ledCharacteristic.subscribed()) {
         ledCharacteristic.writeValue(ledFeature);
     }
}

void commandCallback() {
    uint8_t data[20];
    uint32_t feature;
    
    if (commandCharacteristic.written()) {
        commandCharacteristic.getValue(data, sizeof(data));

        feature = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3] << 0);

        if (feature == 0x20000000) {
            if (ledFeature.led != (data[4] & 1)) {
                ledFeature.timestamp = millis() / 8;
                ledFeature.led = (data[4] & 1);

                digitalWrite(LED_BUILTIN, (ledFeature.led != 0));

                ledCharacteristic.writeValue(ledFeature);
            }
        }
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, (ledFeature.led != 0));

    Serial.begin(9600);
    
    while (!Serial) { }

    OTA.begin();
    
    BLE.begin(247);
    BLE.setIncludeTxPowerLevel(true);
    BLE.setLocalName("STM32WB");
    BLE.setManufacturerData(manufacturer_data, sizeof(manufacturer_data));

    sensorService.addCharacteristic(ledCharacteristic);

    BLE.addService(sensorService);

    commandService.addCharacteristic(commandCharacteristic);

    BLE.addService(commandService);
    BLE.addService(OTA);

    ledCharacteristic.onSubscribed(subscribedCallback);
    commandCharacteristic.onWritten(commandCallback);
    
    BLE.advertise();

    delay(1000);
}

void loop()
{ 
    if (!BLE.advertising() && !BLE.connected()) {
        BLE.advertise();
    }
    
    delay(2000);
}
