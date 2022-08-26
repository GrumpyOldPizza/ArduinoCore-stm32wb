#include "STM32WB.h"
#include "stm32wb_ipcc.h"

#include "../Firmware/stm32wb5x_FUS_fw.h"
#include "../Firmware/stm32wb5x_FUS_fw_for_fus_0_5_3.h"
//#include "../Firmware/stm32wb5x_BLE_Stack_full_fw_1_13_3.h"
#include "../Firmware/stm32wb5x_BLE_Stack_full_fw_1_14_0.h"
//#include "../Firmware/stm32wb5x_BLE_Stack_full_fw_1_14_1.h"

bool connected = false;
bool success = true;
uint32_t code = 0;

#if defined(USBCONN)
void initUSB() {
}
#endif

void setup(void) { 
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 1);
    
    if (!stm32wb_ipcc_sys_enable()) {
        success = false;
    }

    if (success) {
        while (stm32wb_ipcc_sys_state() == STM32WB_IPCC_SYS_STATE_NONE) { }
        
        success = stm32wb_ipcc_sys_firmware(WirelessStackVersion, WirelessStackType, WirelessStackAddress, WirelessStackImage, sizeof(WirelessStackImage), FusImage, FusImage_for_0_5_3, &code);
    }

    digitalWrite(LED_BUILTIN, 0);

#if defined(USBCONN)
    USBDevice.begin();
    USBDevice.start();
#endif

    Serial.begin(9600);
}

void loop(void) {
    if (Serial) {
        if (!connected) {
            connected = true;

            Serial.println();
                            
            if (!success) {
                Serial.print("FAILURE: ");
                Serial.println(code, HEX);
            }
            
            if (stm32wb_ipcc_sys_state() != STM32WB_IPCC_SYS_STATE_NONE) {
                stm32wb_ipcc_sys_info_t info;
                
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
    }
    
    delay(500);
    digitalWrite(LED_BUILTIN, (success ? 1 : 0));
    delay(500);
    digitalWrite(LED_BUILTIN, 0);
}


