/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include <cstdint>

namespace board
{

constexpr unsigned UniqueIDSize = 16;

void readUniqueID(std::uint8_t out_uid[UniqueIDSize]);

void resetWatchdog();

void setStatusLed(bool state);
void setCanLed(bool state);

void setChargePumpSwitch(bool state);

enum class MagnetBridgeState : std::uint8_t
{
    BothLow,
    RightHighLeftLow,
    RightLowLeftHigh
};
void setMagnetBridge(const MagnetBridgeState state);

std::uint8_t readDipSwitch();

}
