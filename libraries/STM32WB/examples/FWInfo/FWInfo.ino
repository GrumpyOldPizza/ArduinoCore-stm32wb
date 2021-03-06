#include "Arduino.h"
#include "wiring_private.h"
#include "stm32wb_ipcc.h"

void setup(void) {
    stm32wb_ipcc_sys_info_t info;

    Serial.begin(38400);

    while (!Serial) { }

    delay(2000);

    if (!stm32wb_ipcc_sys_enable()) {
        Serial.println("FAILED TO BOOT CPU2\r\n");
    } else {
        while (stm32wb_ipcc_sys_state() == STM32WB_IPCC_SYS_STATE_NONE) { }

        stm32wb_ipcc_sys_info(&info);

        Serial.println();
        Serial.println();
        Serial.print("State                  = ");
        Serial.println((stm32wb_ipcc_sys_state() == STM32WB_IPCC_SYS_STATE_FUS) ? "FUS" : "Wireless");
        Serial.print("FUS Version            = ");
        Serial.print((int)((info.FusVersion >> 24) & 255));
        Serial.print(".");
        Serial.print((int)((info.FusVersion >> 16) & 255));
        Serial.print(".");
        Serial.println((int)((info.FusVersion >> 8) & 255));
        Serial.print("Wireless Stack Version = ");
        Serial.print((int)((info.WirelessStackVersion >> 24) & 255));
        Serial.print(".");
        Serial.print((int)((info.WirelessStackVersion >> 16) & 255));
        Serial.print(".");
        Serial.println((int)((info.WirelessStackVersion >> 8) & 255));
        Serial.print("Wireless Stack Type    = ");
        Serial.println((int)(info.WirelessStackType & 255));
        Serial.println();
    }
}

void loop(void) {
}

