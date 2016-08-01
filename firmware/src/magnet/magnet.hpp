/*
 * OpenGrab EPM - Electropermanent Magnet
 * Copyright (C) 2015  Zubax Robotics <info@zubax.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 *         Andreas Jochum <andreas@nicadrone.com>
 */

#pragma once

#include <cstdint>

namespace magnet
{
/**
 * This function will be called by the application wit maximum possible rate (typically > 1 kHz).
 */
void poll();

/**
 * Maximum number of turn on/off switching cycles.
 */
static constexpr std::uint8_t MinTurnOnCycles = 3;
static constexpr std::uint8_t MaxCycles = 10;

/**
 * Turns the magnet on.
 * @param num_cycles    - number of switch cycles
 */
void turnOn(unsigned num_cycles);

/**
 * Turns the magnet off. The number of switch cycles is fixed.
 */
void turnOff();

bool isTurnedOn();

enum class Health : std::uint8_t
{
    Ok,
    Warning,
    Error
};

Health getHealth();

std::uint8_t getStatusFlags();

}
