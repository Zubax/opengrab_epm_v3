/*r
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 * Author: Andreas Jochum <Andreas@Nicadrone.com>
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

    if (supply_voltage_mV < 4300)
    {
        addErrorFlags(ErrorFlagInputVoltageTooLow);
    }

    if (supply_voltage_mV > 6700)
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
    const unsigned on_time = (((10000000 / (supply_voltage_mV - 500)) - 42) / 104);

    /*
     * Pretty close to optimal
     */
    unsigned off_time = ((10000000 / (supply_voltage_mV - 500)) / (board::getOutVoltageInVolts() + 1)) * 50;

    if (off_time > 312)     // Prevent overflow
    {
        off_time = (off_time - 208) / 104;
    }
    else
    {
        off_time = 1;       // We could posible shave off 30ms from the charge time if we had more resolution
    }

    // When output_voltage is relaly low off time is to long
    if (off_time > 120)
    {
        off_time = 120;
    }

    // Sanity check and run a few cycles
    if (on_time > 0 && on_time < 30)
    {
        board::runPump(50, on_time, off_time);
    }

    return (board::getOutVoltageInVolts() >= target_output_voltage_) ? Status::Done : Status::InProgress;
}

}
