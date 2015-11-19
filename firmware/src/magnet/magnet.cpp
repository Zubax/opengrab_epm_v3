/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

/*
 * TODO
 * VDD CAN is Vin, generate critical error when Vin >6.5V, abs max 7V, CAN transiver limit
 * BOR detect
 */

#include "magnet.hpp"
#include "charger.hpp"
#include <sys/board.hpp>
#include <uavcan/util/lazy_constructor.hpp>

namespace magnet
{
namespace
{

static constexpr std::uint16_t TurnOffCycleArray[][2] =
{
    { 450, 0 },
    { 450, 0 },
    { 300, 1 },
    { 180, 0 },
    { 162, 1 },
    { 146, 0 },
    { 131, 1 },
    { 118, 0 },
    { 106, 1 },
    { 96,  0 },
    { 86,  1 },
    { 77,  0 },
    { 70,  1 },
    { 63,  0 },
    { 56,  1 },
    { 51,  0 },
    { 46,  1 },
    { 41,  0 },
    { 37,  1 },
    { 33,  0 },
    { 30,  1 },
    { 27,  0 },
    { 24,  1 },
    { 22,  0 },
    { 20,  1 },
    { 18,  0 },
    { 16,  1 },
    { 14,  0 },
    { 13,  1 },
    { 12,  0 },
    { 10,  1 }
};

static constexpr unsigned TurnOffCycleArraySize = sizeof(TurnOffCycleArray) / sizeof(TurnOffCycleArray[0]);

static board::MonotonicDuration MinCommandInterval = board::MonotonicDuration::fromMSec(500);

static uavcan::LazyConstructor<charger::Charger> chrg;

/**
 * Positive when turning on
 * Negative when turning off
 * Zero when idle
 */
static int remaining_cycles = 0;

static Health health = Health::Ok;

static std::uint8_t charger_status_flags = 0;

static bool magnet_is_on = false;               ///< This is default

static board::MonotonicTime last_command_ts;


void updateChargerStatusFlags(std::uint8_t x)
{
    charger_status_flags = x;
}

void pollOn()
{
    if (!chrg.isConstructed())
    {
        board::syslog("Mag ON chrg started\r\n");
        chrg.construct<unsigned>(450);
    }

    const auto status = chrg->runAndGetStatus();
    updateChargerStatusFlags(chrg->getErrorFlags());

    if (status == charger::Charger::Status::InProgress)
    {
        ; // Nothing to do
    }
    else if (status == charger::Charger::Status::Done)
    {
        board::setMagnetPos();          // The cap is charged, switching the magnet
        magnet_is_on = true;

        chrg.destroy();                 // Then updating the state
        remaining_cycles--;
        health = Health::Ok;
    }
    else
    {
        chrg.destroy();
        remaining_cycles = 0;
        health = Health::Error;
    }
}

void pollOff()
{
    const unsigned cycle_index = TurnOffCycleArraySize - unsigned(-remaining_cycles);

    const auto cycle_array_item = TurnOffCycleArray[cycle_index];

    if (!chrg.isConstructed())
    {
        board::syslog("Mag OFF chrg started cyc ", cycle_index, "\r\n");
        chrg.construct<unsigned>(cycle_array_item[0]);
    }

    const auto status = chrg->runAndGetStatus();
    updateChargerStatusFlags(chrg->getErrorFlags());

    if (status == charger::Charger::Status::InProgress)
    {
        ; // Nothing to do
    }
    else if (status == charger::Charger::Status::Done)
    {
        if (cycle_array_item[1])        // The cap is charged, switching the magnet
        {
            board::setMagnetPos();
        }
        else
        {
            board::setMagnetNeg();
            magnet_is_on = false;
        }

        chrg.destroy();
        remaining_cycles++;
        health = Health::Ok;
    }
    else
    {
        chrg.destroy();
        remaining_cycles = 0;
        health = Health::Error;
    }
}

} // namespace

void turnOn(unsigned num_cycles)
{
    const auto ts = board::clock::getMonotonic();
    if (magnet_is_on && (ts - last_command_ts < MinCommandInterval))
    {
        return;         // Rate limiting
    }
    last_command_ts = ts;

    if (remaining_cycles <= 0)
    {
        board::syslog("Mag on ", num_cycles, "\r\n");

        if (chrg.isConstructed())
        {
            board::syslog("Charger restart\r\n");
            chrg.destroy();
        }
    }

    remaining_cycles = int(std::min<unsigned>(std::max<unsigned>(1, num_cycles), MaxCycles));
}

void turnOff()
{
    const auto ts = board::clock::getMonotonic();
    if (!magnet_is_on && (ts - last_command_ts < MinCommandInterval))
    {
        return;         // Rate limiting
    }
    last_command_ts = ts;

    if (remaining_cycles >= 0)          // Ignore command if turnning off is already in progress
    {
        board::syslog("Mag off\r\n");

        if (chrg.isConstructed())
        {
            board::syslog("Charger restart\r\n");
            chrg.destroy();
        }

        // TODO Skip the first 3 cycles if the magnet was not on!
        remaining_cycles = -int(TurnOffCycleArraySize);
    }
}

bool isTurnedOn()
{
    return magnet_is_on;
}

void poll()
{
    if (remaining_cycles > 0)
    {
        pollOn();
    }
    else if (remaining_cycles < 0)
    {
        pollOff();
    }
    else
    {
        ;
    }
}

Health getHealth()
{
    return health;
}

std::uint8_t getStatusFlags()
{
    static constexpr std::uint8_t StatusFlagSwitchingOn  = 1 << (charger::Charger::ErrorFlagsBitLength + 0);
    static constexpr std::uint8_t StatusFlagSwitchingOff = 1 << (charger::Charger::ErrorFlagsBitLength + 1);

    std::uint8_t x = charger_status_flags;

    if (remaining_cycles > 0)
    {
        x |= StatusFlagSwitchingOn;
    }
    if (remaining_cycles < 0)
    {
        x |= StatusFlagSwitchingOff;
    }

    return x;
}

}


