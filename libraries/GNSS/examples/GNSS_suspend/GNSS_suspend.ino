#include "GNSS.h"
#include "STM32WB.h"
#include "RTC.h"
#include "TimerMillis.h"


bool isPeriodic = false;

unsigned int myAcqTime =  45;
unsigned int myOnTime  =  15;
unsigned int myPeriod  = 120;

GNSSLocation myLocation;
GNSSSatellites mySatellites;

#define PIN_PPS 2

volatile bool isTracking = false;

TimerMillis myTimerSuspend;
TimerMillis myTimerResume;

void callbackSuspend(void)
{
    Serial.println("SUSPEND");
    
    GNSS.suspend();
}

void callbackResume(void)
{
    Serial.println("RESUME");

    GNSS.resume();

    isTracking = false;

    myTimerSuspend.start(callbackSuspend, myAcqTime * 1000);
}

void setup( void )
{
    Serial.begin(9600);

    GNSS.begin(GNSS_MODE_UBLOX, GNSS_RATE_1HZ, Serial1, PIN_PPS);

    while (GNSS.busy()) { }

    GNSS.setConstellation(GNSS_CONSTELLATION_GPS_AND_GLONASS);

    while (GNSS.busy()) { }

    GNSS.setAntenna(GNSS_ANTENNA_INTERNAL);

    while (GNSS.busy()) { }

    GNSS.onLocation(STM32WB.wakeup);
    GNSS.onSatellites(STM32WB.wakeup);
}

void loop( void )
{
    STM32WB.stop();

    if (GNSS.location(myLocation))
    {
        uint8_t year, month, day, hours, minutes, seconds;
        uint16_t milliSeconds;

        static const char *fixTypeString[] = {
            "NONE",
            "TIME",
            "2D",
            "3D",
        };

        static const char *fixQualityString[] = {
            "",
            "",
            "/DIFFERENTIAL",
            "/PRECISE",
            "/RTK_FIXED",
            "/RTK_FLOAT",
            "/ESTIMATED",
            "/MANUAL",
            "/SIMULATION",
        };

        if (!isPeriodic)
        {
            if ((myLocation.fixType() == GNSS_FIX_TYPE_3D) && (myLocation.ehpe() <= 30.0) && myLocation.fullyResolved())
            {
                isPeriodic = true;
                
                myTimerResume.start(callbackResume, 0, myPeriod * 1000);

                isTracking = true;
                
                myTimerSuspend.start(callbackSuspend, myOnTime * 1000);
            }
        }
        else
        {
            if ((myLocation.fixType() >= GNSS_FIX_TYPE_2D) && (myLocation.ehpe() <= 30.0) && myLocation.fullyResolved())
            {
                if (!isTracking)
                {
                    isTracking = true;
                    
                    myTimerSuspend.restart(myOnTime * 1000);
                }
            }
        }

        RTC.getDateTime(day, month, year, hours, minutes, seconds, milliSeconds);

        Serial.print("RTC: ");
        Serial.print(2000 + year);
        Serial.print("/");
        Serial.print(month);
        Serial.print("/");
        Serial.print(day);
        Serial.print(" ");
        if (hours <= 9) {
            Serial.print("0");
        }
        Serial.print(hours);
        Serial.print(":");
        if (minutes <= 9) {
            Serial.print("0");
        }
        Serial.print(minutes);
        Serial.print(":");
        if (seconds <= 9) {
            Serial.print("0");
        }
        Serial.print(seconds);
        Serial.print(".");
        if (milliSeconds <= 9) {
            Serial.print("0");
        }
        if (milliSeconds <= 99) {
            Serial.print("0");
        }
        Serial.print(milliSeconds);

        if (RTC.status() & 0x02) {
            Serial.print(", TIME-SYNCHRONIZED");
        }
        
        if (RTC.status() & 0x08) {
            Serial.print(", UTC_OFFSET-SYNCHRONIZED");
        }

        Serial.println();

        Serial.print("LOCATION: ");
        Serial.print(fixTypeString[myLocation.fixType()]);

        if (myLocation.fixType() != GNSS_FIX_TYPE_NONE)
        {
            Serial.print(fixQualityString[myLocation.fixQuality()]);
            Serial.print(" ");
            Serial.print(myLocation.year());
            Serial.print("/");
            Serial.print(myLocation.month());
            Serial.print("/");
            Serial.print(myLocation.day());
            Serial.print(" ");
            if (myLocation.hours() <= 9) {
                Serial.print("0");
            }
            Serial.print(myLocation.hours());
            Serial.print(":");
            if (myLocation.minutes() <= 9) {
                Serial.print("0");
            }
            Serial.print(myLocation.minutes());
            Serial.print(":");
            if (myLocation.seconds() <= 9) {
                Serial.print("0");
            }
            Serial.print(myLocation.seconds());
            Serial.print(".");
            if (myLocation.millis() <= 9) {
                Serial.print("0");
            }
            if (myLocation.millis() <= 99) {
                Serial.print("0");
            }
            Serial.print(myLocation.millis());

            if (myLocation.leapSeconds() != GNSS_LEAP_SECONDS_UNDEFINED) {
                Serial.print(" ");
                Serial.print(myLocation.leapSeconds());
                if (!myLocation.fullyResolved()) {
                    Serial.print("D");
                }
            }
                
            if (myLocation.fixType() != GNSS_FIX_TYPE_TIME)
            {
                Serial.print(" LLA=");
                Serial.print(myLocation.latitude(), 7);
                Serial.print(",");
                Serial.print(myLocation.longitude(), 7);
                Serial.print(",");
                Serial.print(myLocation.altitude(), 3);
                Serial.print(" EPE=");
                Serial.print(myLocation.ehpe(), 3);
                Serial.print(",");
                Serial.print(myLocation.evpe(), 3);
                Serial.print(" SATELLITES=");
                Serial.print(myLocation.satellites());
                Serial.print(" DOP=");
                Serial.print(myLocation.hdop(), 2);
                Serial.print(",");
                Serial.print(myLocation.vdop(), 2);
            }
        }

        Serial.println();

        GNSS.satellites(mySatellites);

        Serial.print("SATELLITES: ");
        Serial.print(mySatellites.count());
        
        Serial.println();

        for (unsigned int index = 0; index < mySatellites.count(); index++)
        {
            unsigned int svid = mySatellites.svid(index);

            if ((svid >= 1) && (svid <= 32))
            {
                Serial.print("    ");

                if (svid <= 9)
                {
                    Serial.print("  G");
                    Serial.print(svid);
                }
                else
                {
                    Serial.print(" G");
                    Serial.print(svid);
                }
            }
            else if ((svid >= 65) && (svid <= 96))
            {
                Serial.print("    ");

                if ((svid - 64) <= 9)
                {
                    Serial.print("  R");
                    Serial.print(svid -64);
                }
                else
                {
                    Serial.print(" R");
                    Serial.print(svid -64);
                }
            }
            else if ((svid >= 120) && (svid <= 158))
            {
                Serial.print("    ");
                Serial.print("S");
                Serial.print(svid);
            }
            else if ((svid >= 173) && (svid <= 182))
            {
                Serial.print("    ");
                Serial.print("  I");
                Serial.print(svid -172);
            }
            else if ((svid >= 193) && (svid <= 197))
            {
                Serial.print("    ");
                Serial.print("  Q");
                Serial.print(svid -192);
            }
            else if ((svid >= 211) && (svid <= 246))
            {
                Serial.print("    ");

                if ((svid - 210) <= 9)
                {
                    Serial.print("  E");
                    Serial.print(svid -210);
                }
                else
                {
                    Serial.print(" E");
                    Serial.print(svid -210);
                }
            }
            else if (svid == 255)
            {
                Serial.print("    ");
                Serial.print("R???");
            }
            else
            {
                continue;
            }

            Serial.print(": SNR=");
            Serial.print(mySatellites.snr(index));
            Serial.print(", ELEVATION=");
            Serial.print(mySatellites.elevation(index));
            Serial.print(", AZIMUTH=");
            Serial.print(mySatellites.azimuth(index));

            if (mySatellites.unhealthy(index)) {
                Serial.print(", UNHEALTHY");
            }

            if (mySatellites.almanac(index)) {
                Serial.print(", ALMANAC");
            }

            if (mySatellites.ephemeris(index)) {
                Serial.print(", EPHEMERIS");
            }

            if (mySatellites.autonomous(index)) {
                Serial.print(", AUTONOMOUS");
            }

            if (mySatellites.correction(index)) {
                Serial.print(", CORRECTION");
            }

            if (mySatellites.acquired(index)) {
                Serial.print(", ACQUIRED");
            }

            if (mySatellites.locked(index)) {
                Serial.print(", LOCKED");
            }

            if (mySatellites.navigating(index)) {
                Serial.print(", NAVIGATING");
            }

            Serial.println();
        }
    }
}
