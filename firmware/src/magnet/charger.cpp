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

void Charger::set(unsigned U_)
{
    U = U_;
    time_out = 800;       //time out
    done = false;
}

bool Charger::is_done()
{
    return done;
}

bool Charger::run()
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
    while (board::getOutVoltageInVolts() < U)
    {
        //Check input voltage

        const auto supply_voltage_mV = board::getSupplyVoltageInMillivolts();
        if (supply_voltage_mV < 4500 || supply_voltage_mV > 6700)
        {
            //This should make the LED's blink in error patter,...
            //When using a high impedance power source Vin can drop below 4500mV limit,..
            board::syslog("Error input voltage out of range: ", supply_voltage_mV, " mV\r\n");
            return false;
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
        unsigned on_time = (((10000000 / (supply_voltage_mV - 500)) - 42) / 104);

        const auto output_voltage_V = board::getOutVoltageInVolts();

        /*
         * Pretty close ot optimal
         */
        unsigned off_time = ((10000000 / (supply_voltage_mV - 500)) / (output_voltage_V + 1)) * 50;

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

        if (time_out == 0)
        {
            board::syslog("timed out \r\n");
            return false;                       // Timed out, return with error
        }

        time_out--;

        // Sanity check and run a few cycles
        if (on_time > 0 && on_time < 30 && time_out > 0)
        {
            board::runPump(300, on_time, off_time);
        }

        // This can take longer than 1 second
        board::resetWatchdog();

    }

    // We either timed out and existed with error or output voltage is reached
    done = true;
    time_out = 0;                               // Make sure we don't run again without setting _U
    U = 0;

    return true;
}

}
