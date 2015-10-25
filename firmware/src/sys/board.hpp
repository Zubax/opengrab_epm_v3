/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 * Author: Andreas Jochum <Andreas@Nicadrone.com>
 */

#include <cstdint>
#include <array>

namespace board
{

#if __GNUC__
__attribute__((noreturn))
#endif
void die();

typedef std::array<std::uint8_t, 16> UniqueID;
void readUniqueID(UniqueID& out_uid);

void resetWatchdog();

void setStatusLed(bool state);
void setCanLed(bool state);

void setPumpSwitch(bool state);

void setMagnetPos();
void setMagnetNeg();

std::uint8_t readDipSwitch();

/**
 * Whether the button was pressed since last invokation of this function.
 * This function must be called at least every 10 ms.
 */
bool hadButtonPressEvent();

unsigned getSupplyVoltageInMillivolts();

unsigned getOutVoltageInVolts();

unsigned getPwmInputPeriodInMicroseconds();

/**
 * Delays execution in a busyloop for the specified amount of microseconds.
 * The number of microseconds must not exceed 255.
 * The duration is clocked with a hardware timer, so interrupts happening while the busyloop is running
 * will not increase duration of the delay.
 */
void delayUSec(std::uint8_t usec);

/**
 * Delays execution in a busyloop for the specified amount of milliseconds.
 * This function is based on @ref delayUSec(), read its description please.
 */
void delayMSec(unsigned msec);

/**
 * Prints the message to the debug serial.
 * Transmission in blocking.
 */
void syslog(const char* msg);
void syslog(const char* prefix, long long integer_value, const char* suffix = "");

}
