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
    unsigned ton_ = 0;                          ///< Busy loop units TODO: This is not used, shall be deleted?
    unsigned toff_ = 0;
    unsigned U_ = 0;                            ///< [V] 0-500
    bool done_ = true;                          ///< Charger done

public:
    bool run();

    bool isDone() const { return done_; }

    void restart(unsigned target_U);
};

}
