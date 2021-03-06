#include "Arduino.h"
#include "SFLASH.h"

uint8_t mid;
uint16_t did;

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
}

void loop() {
}
