#include "Arduino.h"
#include "STM32WB.h"
#include "BLE.h"

BLEUart SerialBLE(BLE_UART_PROTOCOL_BLUEST);

static const uint8_t manufacturer_data[6] = {
    0x01,
    0x00,
    (uint8_t)(0x00000000 >> 24),
    (uint8_t)(0x00000000 >> 16),
    (uint8_t)(0x00000000 >> 8),
    (uint8_t)(0x00000000 >> 0),
};

void setup()
{
    Serial.begin(9600);
    
    while (!Serial) { }

    BLE.begin();
    BLE.setIncludeTxPowerLevel(true);
    BLE.setLocalName("STM32WB");
    BLE.setManufacturerData(manufacturer_data, sizeof(manufacturer_data));

    BLE.addService(SerialBLE);
    
    BLE.advertise();

    delay(1000);
}

void loop()
{ 
    int c;
    
    if (!BLE.advertising() && !BLE.connected()) {
        BLE.advertise();
    }

    SerialBLE.print("Temperature = ");
    SerialBLE.print(STM32WB.readTemperature());
    SerialBLE.println(" *C");

    SerialBLE.print("Battery = ");
    SerialBLE.print(STM32WB.readBattery());
    SerialBLE.println(" V");

    SerialBLE.println();

    while ((c = SerialBLE.read()) >= 0) { if (c == '\n') { Serial.write('\r'); } Serial.write(c); }
    
    delay(2000);
}
