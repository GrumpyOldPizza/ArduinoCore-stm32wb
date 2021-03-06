#include "Arduino.h"
#include "SFLASH.h"

uint8_t mid;
uint16_t did;

uint32_t start, end;

uint32_t failures;
uint32_t address;
uint8_t data[256];

void setup() {
    Serial.begin(9600);

    while (!Serial) { }

    delay(2000);

    SFLASH.begin();

    SFLASH.identify(mid, did);

    Serial.print("MID = ");
    Serial.println(mid, HEX); 

    Serial.print("DID = ");
    Serial.println(did, HEX); 
    
    Serial.print("CAPACITY = ");
    Serial.println(SFLASH.capacity());

    Serial.print("BLOCKSIZE = ");
    Serial.println(SFLASH.blockSize());

    Serial.print("PAGESIZE = ");
    Serial.println(SFLASH.pageSize());

    Serial.print("LENGTH = ");
    Serial.println(SFLASH.length());

    Serial.println();
    Serial.print("ERASING: ");
    
    start = millis();

    for (address = 0; address < 4096 * 1024; address += SFLASH.blockSize()) {
        SFLASH.erase(address);
    }

    while (SFLASH.busy()) { }

    end = millis();

    Serial.print((4096.0 * 1024.0) / ((end - start) / 1000.0));
    Serial.print(" bytes/second");

    for (failures = 0, address = 0; address < 4096 * 1024; address += sizeof(data)) {
        SFLASH.read(address, data, sizeof(data));
        
        for (int i = 0; i < sizeof(data); i++) { if (data[i] != 0xff) { failures++; } }
    }

    if (failures) {
        Serial.print(", ");
        Serial.print(failures);
        Serial.println(" mismatches.");
    } else {
        Serial.println(".");
    }

    for (int i = 0; i < sizeof(data); i++) { data[i] = i; }

    Serial.println();
    Serial.print("PROGRAMMING: ");
    
    start = millis();

    for (address = 0; address < 4096 * 1024; address += sizeof(data)) {
        SFLASH.program(address, data, sizeof(data));
    }

    while (SFLASH.busy()) { }
    
    end = millis();

    Serial.print((4096.0 * 1024.0) / ((end - start) / 1000.0));
    Serial.print(" bytes/second");

    for (failures = 0, address = 0; address < 4096 * 1024; address += sizeof(data)) {
        SFLASH.read(address, data, sizeof(data));
        
        for (int i = 0; i < sizeof(data); i++) { if (data[i] != i) { failures++; } }
    }

    if (failures) {
        Serial.print(", ");
        Serial.print(failures);
        Serial.println(" mismatches.");
    } else {
        Serial.println(".");
    }

    Serial.println();
    Serial.print("READING: ");
    
    start = millis();

    for (address = 0; address < 4096 * 1024; address += sizeof(data)) {
        SFLASH.read(address, data, sizeof(data));
    }

    end = millis();

    Serial.print((4096.0 * 1024.0) / ((end - start) / 1000.0));
    Serial.println(" bytes/second.");

    Serial.println();
    Serial.println("DONE.");
}

void loop() {
}
