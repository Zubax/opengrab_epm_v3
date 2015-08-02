/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 * Author: Andreas Jochum <Andreas@Nicadrone.com>
 */

#include <cstdint>

namespace board
{


constexpr unsigned UniqueIDSize = 16;

void readUniqueID(std::uint8_t out_uid[UniqueIDSize]);

void resetWatchdog();

void setStatusLed(bool state);
void setCanLed(bool state);

void setPumpSwitch(bool state);

void setMagnetPos();

void setMagnetNeg();

std::uint8_t readDipSwitch();

bool hadButtonPressEvent();

unsigned getSupplyVoltageInMillivolts();

unsigned getOutVoltageInVolts();

unsigned getPwmInputPeriodInMicroseconds();

void delayUSec(std::uint8_t usec);

void syslog(const char* msg);

}
