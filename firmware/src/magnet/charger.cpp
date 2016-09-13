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

    if (supply_voltage_mV < build_config::VinMin_mV)
    {
        board::syslog("ErrorFlagInputVoltageTooLow\r\n");       // We should keep this, makes it easier to diagnose power supply problems
        board::syslog("Vin  = ", board::getSupplyVoltageInMillivolts(), " mV\r\n");
        addErrorFlags(ErrorFlagInputVoltageTooLow);
    }

    if (supply_voltage_mV > build_config::VinMax_mV)
    {
        board::syslog("ErrorFlagInputVoltageTooHigh\r\n");
        board::syslog("Vin  = ", board::getSupplyVoltageInMillivolts(), " mV\r\n");
        addErrorFlags(ErrorFlagInputVoltageTooLow);
    }

    if (board::clock::getMonotonic() > deadline_)
    {
#if BOARD_OLIMEX_LPC_P11C24
        return Status::Done;                            // This is just a testing mock
#endif
        // Pint some usefull info when charger times out
        const unsigned vout =  board::getOutVoltageInVolts();
        board::syslog("\r\n\r\nCharger timed out\r\n");
        board::syslog("Vin         = ", board::getSupplyVoltageInMillivolts(), " mV\r\n");
        board::syslog("Vout        = ", vout, " V\r\n");
        board::syslog("Vout target = ",target_output_voltage_, " V\r\n");

        if (vout < 160 && vout > 50)
        {
            board::syslog("\r\n    High side Thyristor failure likely \r\n");
        }

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

    // reduce current consumtion when Vin is low, compatiblity with crapy power rails like PixHawk or cell phone chargers

    unsigned on_time_ns = 0;
    unsigned off_time_ns = 0;
    if(supply_voltage_mV < build_config::ReducedCurrentVoltage_mV)
    {
        on_time_ns = (build_config::PRInductance_pH - 3000000) / (supply_voltage_mV - 500);
        off_time_ns = (((build_config::PRInductance_pH - 3000000) / (supply_voltage_mV - 500)) / (ouput_voltage_V + 1)) * 50;
    }else{
        on_time_ns = build_config::PRInductance_pH / (supply_voltage_mV - 500);
        off_time_ns = ((build_config::PRInductance_pH / (supply_voltage_mV - 500)) / (ouput_voltage_V + 1)) * 50;
    }

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

    // Keep track of supply Voltage during switching
    static unsigned supply_volatage_mV_min = 10000;
    if (supply_voltage_mV < supply_volatage_mV_min)
    {
        supply_volatage_mV_min = supply_voltage_mV;
    }

    // Print supply Voltage when below 4.8V
    unsigned const OutputVoltage = board::getOutVoltageInVolts();
    if (OutputVoltage >= target_output_voltage_ && supply_volatage_mV_min <= 4800)
    {
         board::syslog(" Vin min = ", supply_volatage_mV_min , " mV\r\n");
         supply_volatage_mV_min = 10000;
    }

    return (OutputVoltage >= target_output_voltage_) ? Status::Done : Status::InProgress;
}

}
