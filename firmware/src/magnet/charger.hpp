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

#include <sys/board.hpp>

namespace charger
{

class Charger
{
    const board::MonotonicTime deadline_ = board::clock::getMonotonic() + board::MonotonicDuration::fromMSec(3000);

    unsigned target_output_voltage_ = 0;
    std::uint8_t error_flags_ = 0;

    void addErrorFlags(std::uint8_t x) { error_flags_ |= x; }

public:
    Charger(unsigned target_output_voltage);

    enum class Status : std::uint8_t
    {
        Done,
        InProgress,
        Failure
    };

    Status runAndGetStatus();

    static constexpr std::uint8_t ErrorFlagTimeout             = 1;
    static constexpr std::uint8_t ErrorFlagInputVoltageTooLow  = 2;
    static constexpr std::uint8_t ErrorFlagInputVoltageTooHigh = 4;

    static constexpr std::uint8_t ErrorFlagsBitLength          = 4;

    std::uint8_t getErrorFlags() const { return error_flags_; };
};

}
