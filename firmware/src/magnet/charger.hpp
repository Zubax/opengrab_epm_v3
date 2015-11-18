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

public:
    Charger(unsigned target_output_voltage);

    enum class Status : std::uint8_t
    {
        Done,
        InProgress,
        Failure
    };

    Status runAndGetStatus();
};

}
