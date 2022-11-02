 /*
 * Copyright (c) 2017-2018 Thomas Roell.  All rights reserved.
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

#ifndef _GNSS_H
#define _GNSS_H

#include "Arduino.h"
#include "utility/gnss_api.h"

#define GNSS_RX_BUFFER_SIZE 96

enum GNSSFixType {
    GNSS_FIX_TYPE_NONE = 0,
    GNSS_FIX_TYPE_TIME,
    GNSS_FIX_TYPE_2D,
    GNSS_FIX_TYPE_3D,
};

enum GNSSFixQuality {
    GNSS_FIX_QUALITY_NONE = 0,
    GNSS_FIX_QUALITY_AUTONOMOUS,
    GNSS_FIX_QUALITY_DIFFERENTIAL,
    GNSS_FIX_QUALITY_PRECISE,
    GNSS_FIX_QUALITY_RTK_FIXED,
    GNSS_FIX_QUALITY_RTK_FLOAT,
    GNSS_FIX_QUALITY_ESTIMATED,
    GNSS_FIX_QUALITY_MANUAL,
    GNSS_FIX_QUALITY_SIMULATION,
};

#define GNSS_LEAP_SECONDS_UNDEFINED -128

class GNSSLocation {
public:

    GNSSLocation(const gnss_location_t *location);
    GNSSLocation();

    operator bool() const;

    enum GNSSFixType fixType(void) const;
    enum GNSSFixQuality fixQuality(void) const;
    bool fullyResolved(void) const;
    unsigned int satellites(void) const;

    uint16_t year(void) const;
    uint8_t month(void) const;
    uint8_t day(void) const;
    uint8_t hours(void) const;
    uint8_t minutes(void) const;
    uint8_t seconds(void) const;
    uint16_t millis(void) const;
    int8_t leapSeconds(void) const;

    double latitude(void) const;  // WGS84
    double longitude(void) const; // WGS84
    float height(void) const;   // WGS84
    float altitude(void) const;   // MSL
    float separation(void) const; // WGS84 = MSL + separation

    float speed(void) const;
    float course(void) const;
    float climb(void) const;

    float ehpe(void) const;
    float evpe(void) const;

    float pdop(void) const;
    float hdop(void) const;
    float vdop(void) const;

    void latitude(int32_t &latitude) const { latitude = _location.latitude; };
    void longitude(int32_t &longitude) const { longitude = _location.longitude; };
    void height(int32_t &height) const { height = _location.altitude + _location.separation; };
    void altitude(int32_t &altitude) const { altitude =  _location.altitude; };

private:
    gnss_location_t _location;
};

class GNSSSatellites {
public:
    GNSSSatellites(const gnss_satellites_t *satellites);
    GNSSSatellites();

    unsigned int count() const;

    unsigned int svid(unsigned int index) const;
    unsigned int snr(unsigned int index) const;
    unsigned int elevation(unsigned int index) const;
    unsigned int azimuth(unsigned int index) const;
    bool unhealthy(unsigned int index) const;
    bool almanac(unsigned int index) const;
    bool ephemeris(unsigned int index) const;
    bool autonomous(unsigned int index) const;
    bool correction(unsigned int index) const;
    bool acquired(unsigned int index) const;
    bool locked(unsigned int index) const;
    bool navigating(unsigned int index) const;

private:
    gnss_satellites_t _satellites;
};

enum GNSSmode {
    GNSS_MODE_NMEA = 0,
    GNSS_MODE_UBLOX,
};

enum GNSSrate {
    GNSS_RATE_1HZ = 1,
    GNSS_RATE_5HZ = 5,
    GNSS_RATE_10HZ = 10,
};

enum GNSSantenna {
    GNSS_ANTENNA_INTERNAL = 0,
    GNSS_ANTENNA_EXTERNAL,
};

enum GNSSconstellation {
    GNSS_CONSTELLATION_GPS = 1,
    GNSS_CONSTELLATION_GPS_AND_GLONASS = 3,
};

enum GNSSplatform {
    GNSS_PLATFORM_PORTABLE = 0,
    GNSS_PLATFORM_STATIONARY,
    GNSS_PLATFORM_PEDESTRIAN,
    GNSS_PLATFORM_CAR,
    GNSS_PLATFORM_SEA,
    GNSS_PLATFORM_BALLON,
    GNSS_PLATFORM_AVIATION,
};


class GNSSClass {
public:
    GNSSClass();

    void begin(GNSSmode mode, GNSSrate rate, Uart &uart, uint32_t pinPPS = PIN_NONE, uint32_t pinENABLE = PIN_NONE, uint32_t pinBACKUP = PIN_NONE);
    void end();

    bool setAntenna(GNSSantenna antenna);
    bool setPPS(unsigned int width);
    bool setConstellation(GNSSconstellation constellation);
    bool setSBAS(bool enable);
    bool setQZSS(bool enable);
    bool setAutonomous(bool enable);
    bool setPlatform(GNSSplatform platform);
    bool setPeriodic(unsigned int acqTime, unsigned int onTime, unsigned int period);
    bool suspend();
    bool resume();
    bool busy();
    
    bool location(GNSSLocation &location);
    bool satellites(GNSSSatellites &satellites);

    void onLocation(Callback callback);
    void onLocation(void(*callback)(void)) { onLocation(Callback(callback)); }
    void onSatellites(Callback callback);
    void onSatellites(void(*callback)(void)) { onSatellites(Callback(callback)); }
    
private:
    struct _stm32wb_uart_t *_uart;
    struct {
        uint16_t pps;
        uint16_t enable;
        uint16_t backup;
    } _pins;
    bool _enabled;
    bool _internal;
    uint32_t _baudrate;
    uint8_t _rx_data[GNSS_RX_BUFFER_SIZE];
    gnss_location_t _location_data;
    volatile uint32_t _location_pending;
    gnss_satellites_t _satellites_data;
    volatile uint32_t _satellites_pending;

    Callback _locationCallback;
    Callback _satellitesCallback;

    k_work_t _receiveWork;
    k_work_t _transmitWork;

    void (*_doneCallback)(void);

    void uartBegin(GNSSmode mode, GNSSrate rate, struct _stm32wb_uart_t *uart, const struct _stm32wb_uart_params_t *params, uint16_t pps, uint16_t enable, uint16_t backup, bool internal);
    void uartEnd();
    static void uartReceiveRoutine(class GNSSClass*);
    static void uartTransmitRoutine(class GNSSClass*);
    static void uartEventCallback(class GNSSClass*, uint32_t);
    static void uartDoneCallback(class GNSSClass*);
    static void uartSendRoutine(class GNSSClass*, const uint8_t*, uint32_t, gnss_send_callback_t);
    
    static void enableCallback(class GNSSClass*);
    static void disableCallback(class GNSSClass*);
    static void locationCallback(class GNSSClass*, const gnss_location_t*);
    static void satellitesCallback(class GNSSClass*, const gnss_satellites_t*);
};

extern GNSSClass GNSS;

#endif /* _GNSS_H */
