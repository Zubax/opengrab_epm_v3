/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 * Author: Andreas Jochum <Andreas@Nicadrone.com>
 */

#pragma once

#include <cstdint>
#include <array>
#include <uavcan_lpc11c24/clock.hpp>

namespace board
{

namespace clock
{

using uavcan_lpc11c24::clock::getMonotonic;

}

using uavcan::MonotonicTime;
using uavcan::MonotonicDuration;

#if __GNUC__
__attribute__((noreturn))
#endif
void die();

typedef std::array<std::uint8_t, 16> UniqueID;
void readUniqueID(UniqueID& out_uid);

typedef std::array<std::uint8_t, 128> DeviceSignature;
bool tryReadDeviceSignature(DeviceSignature& out_signature);

void resetWatchdog();

void setStatusLed(bool state);
void setCanLed(bool state);

/**
 * Switches the pump specified number of times with specified duty cycle.
 * Warning: this function does not check correctness of the arguments.
 * All arguments MUST BE POSITIVE.
 */
void runPump(std::uint_fast16_t iterations,
             std::uint_fast8_t delay_on,
             std::uint_fast8_t delay_off);

void setMagnetPos();
void setMagnetNeg();

static constexpr std::uint8_t DipSwitchBits = 4;
std::uint8_t readDipSwitch();

/**
 * Whether the button was pressed since last invokation of this function.
 * This function must be called at least every 10 ms.
 */
bool hadButtonPressEvent();

unsigned getSupplyVoltageInMillivolts();

unsigned getOutVoltageInVolts();

/**
 * Status of the PWM input.
 */
enum class PwmInput : std::uint8_t
{
    NoSignal,
    Low,
    Neutral,
    High
};

PwmInput getPwmInput();

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
