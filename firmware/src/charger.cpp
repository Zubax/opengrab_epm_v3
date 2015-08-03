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
    for (unsigned i = 0; i < 1000; i++)
    {
        LPC_GPIO[1].DATA[0b010111] = 0b010111; // ON
        board::delayUSec(1);
        LPC_GPIO[1].DATA[0b010111] = 0b000000; // OFF
        board::delayUSec(2);
    }

    for (unsigned i = 0; i < 10000; i++)
    {
        LPC_GPIO[1].DATA[0b010111] = 0b010111; // ON
        board::delayUSec(1);
        LPC_GPIO[1].DATA[0b010111] = 0b000000; // OFF
        board::delayUSec(1);
    }

    // Printing the output voltage
    const auto output_voltage_V = board::getOutVoltageInVolts();
    char buf[24];
    util::lltoa(output_voltage_V, buf);
    board::syslog("output_voltage_V =  ");
    board::syslog(static_cast<const char*>(buf));
    board::syslog(" mV\r\n");
}

}
