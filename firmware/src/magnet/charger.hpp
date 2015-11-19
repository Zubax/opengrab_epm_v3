/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 * Author: Andreas Jochum <Andreas@Nicadrone.com>
 */

#pragma once

#include <sys/board.hpp>

namespace charger
{

class Charger
{
    const board::MonotonicTime deadline_ = board::clock::getMonotonic() + board::MonotonicDuration::fromMSec(1000);

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
