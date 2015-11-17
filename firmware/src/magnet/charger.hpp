/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 * Author: Andreas Jochum <Andreas@Nicadrone.com>
 */

#pragma once

namespace charger
{

class Charger
{
    void cycle(unsigned on, unsigned off);

    unsigned U = 0;

    unsigned time_out = 0;

    bool done = false;

public:
    bool run();

    void reset();

    void set(unsigned U_);

    bool is_done();
};

}
