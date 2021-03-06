/* DOSFS SFLASH example
 *
 * This example shows how format a SFLASH device with DOSFS.
 *    
 * This example code is in the public domain.
 */

#include <DOSFS.h>

void setup ( void )
{
    Serial.begin(9600);

    while (!Serial) { }

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 1);
    
    DOSFS.begin();

    if (!DOSFS)
    {
        DOSFS.hardformat(8 * 1024 * 1024);
    }

    DOSFS.end();

    digitalWrite(LED_BUILTIN, 0);

    Serial.println("Done.");
}

void loop( void )
{
}
