/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cstdint>

namespace magnet
{

void init();

/**
 * This function will be called by the application wit maximum possible rate (typically > 1 kHz).
 */
void poll();

enum class Health : std::uint8_t
{
    Ok,
    Warning,
    Error
};

Health getHealth();

}
