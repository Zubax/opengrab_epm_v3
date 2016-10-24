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

/*
 * TODO
 * VDD CAN is Vin, generate critical error when Vin >6.5V, abs max 7V, CAN transiver limit
 * BOR detect
 */

#include "magnet.hpp"
#include "charger.hpp"
#include <sys/board.hpp>
#include <uavcan/util/lazy_constructor.hpp>
#include <build_config.hpp>


namespace magnet
{
namespace
{

static constexpr std::uint16_t TurnOffCycleArray[][2] =
{
    #if defined(PRODROPPER)

    { 475, 0 },
    { 200, 0 },
    { 300, 1 },
    { 290, 0 },
    { 280, 1 },
    { 270, 0 },
    { 260, 1 },
    { 250, 0 },
    { 240, 1 },
    { 230, 0 },
    { 220, 1 },
    { 210, 0 },
    { 200, 1 },
    { 190, 0 },
    { 180, 1 },
    { 170, 0 },
    { 160, 1 },
    { 150, 0 },
    { 140, 1 },
    { 130, 0 },
    { 120, 1 },
    { 110, 0 },
    { 100, 1 },
    { 90,  0 },
    { 80,  1 },
    { 70,  0 },
    { 60,  1 },
    { 50,  0 },
    { 40,  1 },
    { 30,  0 },
    { 20,  1 },
    { 20,  0 },
    { 20,  1 },
    { 20,  0 },
    { 20,  1 },
    { 20,  0 },
    { 20,  1 },
    { 20,  0 },
    { 20,  1 },
    { 20,  0 },
    { 20,  1 },
    { 20,  0 },
    { 20,  1 }

    #else

    { 475, 0 },
    { 475, 0 },
    { 475, 0 },
    { 427, 1 },
    { 427, 1 },
    { 427, 1 },
    { 384, 0 },
    { 384, 0 },
    { 384, 0 },
    { 346, 1 },
    { 346, 1 },
    { 346, 1 },
    { 311, 0 },
    { 311, 0 },
    { 311, 0 },
    { 280, 1 },
    { 280, 1 },
    { 280, 1 },
    { 252, 0 },
    { 252, 0 },
    { 252, 0 },
    { 227, 1 },
    { 227, 1 },
    { 227, 1 },
    { 204, 0 },
    { 204, 0 },
    { 204, 0 },
    { 184, 1 },
    { 184, 1 },
    { 184, 1 },
    { 165, 0 },
    { 165, 0 },
    { 165, 0 },
    { 149, 1 },
    { 149, 1 },
    { 149, 1 },
    { 134, 0 },
    { 134, 0 },
    { 134, 0 },
    { 120, 1 },
    { 120, 1 },
    { 120, 1 },
    { 108, 0 },
    { 108, 0 },
    { 108, 0 },
    { 97,  1 },
    { 97,  1 },
    { 97,  1 },
    { 88,  0 },
    { 88,  0 },
    { 88,  0 },
    { 79,  1 },
    { 79,  1 },
    { 79,  1 }

    #endif
};

static constexpr unsigned TurnOffCycleArraySize = sizeof(TurnOffCycleArray) / sizeof(TurnOffCycleArray[0]);

static board::MonotonicDuration MinCommandInterval =
    board::MonotonicDuration::fromMSec(build_config::CommandRateLimit_ms);

static uavcan::LazyConstructor<charger::Charger> chrg;

/**
 * Positive when turning on
 * Negative when turning off
 * Zero when idle
 */
static int remaining_cycles = 0;

static Health health = Health::Ok;

static std::uint8_t charger_status_flags = 0;

/**
 * Magnet state
 * 0 = off
 * 1 = z_positive
 * 2 = z_negative
 */

static unsigned magnet_state = 0;               ///< This is default

static board::MonotonicTime last_command_ts;

signed duty_cycle_counter = 13000;              // when charging is in progress this counter can run negative, it's a better approximation for the limit

constexpr signed DutyCycleCounterMax = 13000;   // Comparing signed with unsigned is problematic

void updateChargerStatusFlags(std::uint8_t x)
{
    charger_status_flags = x;
}

void pollSetState()
{
    const unsigned cycle_index = TurnOffCycleArraySize - unsigned(-remaining_cycles);

    const auto cycle_array_item = TurnOffCycleArray[cycle_index];

    if (!chrg.isConstructed())
    {
        if (desired_state == 0)
        {
            chrg.construct<unsigned>(cycle_array_item[0]);
        }else{
            chrg.construct<unsigned>(475);
        }

    }

    const auto status = chrg->runAndGetStatus();
    updateChargerStatusFlags(chrg->getErrorFlags());

    if (status == charger::Charger::Status::InProgress)
    {
        duty_cycle_counter--;
    }
    else if (status == charger::Charger::Status::Done)
    {
        if (desired_state == 0)
        {
            if (cycle_array_item[1])        // The cap is charged, switching the magnet
            {
                board::setMagnetPos();
            }
            else
            {
                board::setMagnetNeg();
            }
            remaining_cycles++;
        }
        if (desired_state == 1)
        {
            board::setMagnetPos();
            magnet_state = 1;
            remaining_cycles = 0;
            chrg.destroy();
            health = Health::Ok;
        }
        if (desired_state == 2)
        {
            board::setMagnetNeg();
            magnet_state = 2;
            remaining_cycles = 0;
            chrg.destroy();
            health = Health::Ok;
        }



        // Print some info when capacitor fails to discharge and delcare error
        board::delayMSec(2);                   // Wait until ADC cap settles
        const unsigned Vout = board::getOutVoltageInVolts();
        if (Vout > 100)                        // 1ms is not really enough hence 100V
        {
            board::syslog("\r\nCapacitor failed to discharge \r\n");
            board::syslog("Thyristor D20 on CTRL2 or D23 on CTRL3 failed to fire. Or open magnet winding \r\n");
            board::syslog("Vin  = ", board::getSupplyVoltageInMillivolts(), " mV\r\n");
            board::syslog("Vout = ", Vout, " V\r\n");

            health = Health::Error;
            remaining_cycles = 0;
        }

    }
    else                    // Charger timed out
    {
        chrg.destroy();
        remaining_cycles = 0;
        health = Health::Error;
    }
}

} // namespace

void setState(unsigned blah)
{
    desired_state = blah;
    if (magnet_state == desired_state)
    {
        // Do nothing
        return;
    }
    if (remaining_cycles == 0)          // Ignore the command if switching is already in progress
    {
        // Print some usefull info
        board::syslog("\r\n On command received \r\n");
        board::syslog(" Vin         = ", board::getSupplyVoltageInMillivolts(), " mV\r\n");

        // Check rate limiting
        if (duty_cycle_counter< 0)
        {
            board::syslog("\r\nRate limiting\r\n\r\n");
            return;         // Rate limiting
        }

        /**
         * Magnet state
         * 0 = off
         * 1 = z_positive
         * 2 = z_negative
         */
        if (desired_state == 0 )
        {
            remaining_cycles = -int(TurnOffCycleArraySize);
        } else
        {
            remaining_cycles = 1;
        }
    }
}

unsigned magnetState()
{
    return magnet_state;
}

void poll()
{
    const auto ts = board::clock::getMonotonic();
    static board::MonotonicTime duty_cycle_counter_update_deadline = ts;

    if (ts >= duty_cycle_counter_update_deadline)
    {
        duty_cycle_counter_update_deadline += board::MonotonicDuration::fromMSec(100);
        duty_cycle_counter += 180;

        if (duty_cycle_counter > DutyCycleCounterMax)
        {
            duty_cycle_counter = DutyCycleCounterMax;
        }
    }

    if (remaining_cycles != 0)
    {
        pollSetState();
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


