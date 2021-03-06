/*
 This example reads audio data from an Invensense's ICS41350 I2S microphone
 breakout board, and prints out the samples to the Serial console.
 
 It is assumed that the L/R pin is connected do D6.
 */

#include <PDM.h>

void setup()
{
    Serial.begin(115200);

    while (!Serial) { }

    delay(5000); // time to turn the sound on ...

    PDM.setGain(24.0);
    PDM.begin(1, 16000);
}

void loop()
{
    static uint8_t data[PDM_BUFFER_SIZE];
    volatile uint8_t status;
    
    // Read the PDM data stream and write it to serial
  
    if (PDM.available() == PDM_BUFFER_SIZE)
    {
        PDM.read(&data[0], PDM_BUFFER_SIZE);
        Serial.write(&data[0], PDM_BUFFER_SIZE, status);

        while (status == SERIAL_STATUS_BUSY)
        {
        }
    }
}
