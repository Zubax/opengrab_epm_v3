/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include <stdint.h>

namespace board
{

static const unsigned UniqueIDSize = 16;

void readUniqueID(uint8_t out_uid[UniqueIDSize]);

void setStatusLed(bool state);
void setCanLed(bool state);

void resetWatchdog();

}
