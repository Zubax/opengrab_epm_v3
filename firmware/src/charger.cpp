/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 * Author: Andreas Jochum <Andreas@Nicadrone.com>
 */

#include <chip.h>
#include "charger.hpp"
#include "util.hpp"
#include "board.hpp"

namespace charger
{

void Charger::run()
{
    /*
     * On time is 5.25/(Vin-0.5V) [us, V]
     * Off time is ((Vin*10)/Vout)*(Vin-0.5V) [us, V]
     * e = 2.625uJ / cycle
     * emax = 352,500uJ or 134,000 cycles
     * e = Vout*Vout*1.25 [uJ]
     */
    unsigned t = 0;

    /*
     * This will run no more then 198k cycles. break when output voltage is reached.
     * if output voltage is not reached report critical charge error, (capacitor leakage, short or Vin to low)
     * t<10 for debug, this takes at worst 1ms then we turn interupt on and run spin Node and other stuff
     */
    for (t = 0; t < 1000; t++)
    {
        for (unsigned i = 0; i < 66; i++) //
        {
            __disable_irq();
            LPC_GPIO[1].DATA[0b010111] = 0b010111; // ON
            board::delayUSec(1);
            LPC_GPIO[1].DATA[0b010111] = 0b000000; // OFF
            board::delayUSec(1);

            LPC_GPIO[1].DATA[0b010111] = 0b010111; // ON
            board::delayUSec(1);
            LPC_GPIO[1].DATA[0b010111] = 0b000000; // OFF
            board::delayUSec(1);

            LPC_GPIO[1].DATA[0b010111] = 0b010111; // ON
            board::delayUSec(1);
            LPC_GPIO[1].DATA[0b010111] = 0b000000; // OFF
            board::delayUSec(1);

            LPC_GPIO[1].DATA[0b010111] = 0b010111; // ON
            board::delayUSec(1);
            LPC_GPIO[1].DATA[0b010111] = 0b000000; // OFF
            board::delayUSec(1);

            LPC_GPIO[1].DATA[0b010111] = 0b010111; // ON
            board::delayUSec(1);
            LPC_GPIO[1].DATA[0b010111] = 0b000000; // OFF
            board::delayUSec(1);
            __enable_irq();                        // interups are enabled again
        }
    }

    const auto output_voltage_V = board::getOutVoltageInVolts();
    char buf[24];

    // this does not work, it is call a bunch of time on a ~1ms interval but only prints once
    util::lltoa(output_voltage_V, buf);
    board::syslog("output_voltage_V =  ");
    board::syslog(static_cast<const char*>(buf));
    board::syslog(" V\r\n");

    if (U_ < board::getOutVoltageInVolts())    // this does not work, even if U is smaller then vout
    {
        done_ = true;
        t = 2000;                              // breaks the loop
    }

    board::resetWatchdog();
}

void Charger::restart(unsigned target_U)
{
    U_ = target_U;
    done_ = false;
}

}

