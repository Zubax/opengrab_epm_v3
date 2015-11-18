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

/**
 * Maximum number of turn on/off switching cycles.
 */
static constexpr std::uint8_t NumCyclesMax = 10;

/**
 * Turns the magnet on.
 * @param num_cycles    - number of switch cycles
 */
void turnOn(std::uint8_t num_cycles);

/**
 * Turns the magnet off. The number of switch cycles is fixed.
 */
void turnOff();

enum class Health : std::uint8_t
{
    Ok,
    Warning,
    Error
};

Health getHealth();

}
