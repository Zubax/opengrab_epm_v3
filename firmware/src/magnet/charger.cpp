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

#include "charger.hpp"
#include <sys/board.hpp>
#include <chip.h>
#include <config.h>
namespace charger
{

Charger::Charger(unsigned target_output_voltage) :
    target_output_voltage_(target_output_voltage)
{ }

Charger::Status Charger::runAndGetStatus()
{
    /*
     * This is Plan A, run the flyback dynamicly
     * On time is 10000/(Vin-0.5V) [us, mV]
     * Off time is ((Vin*10)/Vout)*(Vin-0.5V) [us, V]
     * e = 2.625uJ / cycle
     * emax = 352,500uJ or 134,000 cycles
     * e = Vout*Vout*1.25 [uJ]
     */
    /*
     * This is Plan B, don't run optimized
     * Use a few fixed timed function, called based on Uout
     * kind of like precomputed lookup tables
     */
    /*
     * Care must be taken not to run too long, if charge has not compleated in 450ms something is wrong
     * more things will break if we keep running
     * note; the LPC11 can be decaped by dumping the 350mJ into a GPIO
     */
    /*
     * Duty cycle of 75% is probably ok, needs verification
     */

    /*
     * Error checks
     */
    const auto supply_voltage_mV = board::getSupplyVoltageInMillivolts();
    const auto ouput_voltage_V   = board::getOutVoltageInVolts();

    if (supply_voltage_mV < VIN_MIN_MV)
    {
        addErrorFlags(ErrorFlagInputVoltageTooLow);
    }

    if (supply_voltage_mV > VIN_MAX_MV)
    {
        addErrorFlags(ErrorFlagInputVoltageTooHigh);
    }

    if (board::clock::getMonotonic() > deadline_)
    {
#if BOARD_OLIMEX_LPC_P11C24
        return Status::Done;                            // This is just a testing mock
#endif
        addErrorFlags(ErrorFlagTimeout);
    }

    if (error_flags_ != 0)
    {
        return Status::Failure;
    }

    /*
     * Calculate on and off time, this will turn into a radom number generator if Vin is out of range
     *
     * This is asuming that the induction is 10uH, thats a bit higher then the datasheet says.
     * The fuse F1 and the 10uF bypass cap keep the sub us peak uner 1100A and RMS under 700mA.
     *
     * Asuming a bigger then 10uH inductor makes 600mV drop after the fuse but runs just fine,
     * charge time of 150ms at Uin = 6.4V.
     *
     * Should be exact for a linear inductor.
     * We are pushing the core right up to saturation so it's not exact science.
     */


    unsigned on_time_ns = PR_INDUCTANCE_PH / (supply_voltage_mV - 500);
    unsigned off_time_ns = ((PR_INDUCTANCE_PH / (supply_voltage_mV - 500)) / (ouput_voltage_V + 1)) * 50;

    unsigned on_time_cy = (on_time_ns - 42) / 104;
    unsigned off_time_cy = 0;

    off_time_ns += 1000;

    if (off_time_ns > 360)
    {
        off_time_cy = (off_time_ns -250) / 104;
    }
    else
    {
        off_time_cy = 1;
    }
    // When output_voltage is relaly low off time is to long
    if (off_time_cy > 120)
    {
        off_time_cy = 120;
    }

    // Sanity check and run a few cycles
    if (on_time_cy > 0 && on_time_cy < 30)
    {
        board::runPump(50, on_time_cy, off_time_cy);
    }

    return (board::getOutVoltageInVolts() >= target_output_voltage_) ? Status::Done : Status::InProgress;
}

}
