/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
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
