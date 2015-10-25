/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 * Author: Andreas Jochum <Andreas@Nicadrone.com>
 */

#include <chip.h>
#include "charger.hpp"
#include "board.hpp"

namespace charger
{

bool Charger::run(unsigned U_)
{
    /*
     * This is Plan A, run the flyback dynamicly
     * On time is 5.25/(Vin-0.5V) [us, V]
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

    //check input voltage

    const auto supply_voltage_mV = board::getSupplyVoltageInMillivolts();
    if(supply_voltage_mV < 4500 ||
        supply_voltage_mV > 6000)
    {
        board::syslog("Error bad voltage: ", supply_voltage_mV, " mV\r\n");
        return false;
    }

    unsigned n=450;                             //max run time in ruffly ms

    cycle1000_7000();                           //U out is very low, transfomers goes in saturation when toff is small

    while(board::getOutVoltageInVolts() < 30)
    {
        n--;
        if(board::getOutVoltageInVolts() > U_)
        {
            return true;
        }
        if(n < 430)                             //we ran 20 ms and Uout is under 30V, report error
        {
            board::syslog("return with error from cycle1500_5000() \r\n");
            return false;
        }
        cycle1500_5000();
    }

    if(board::getOutVoltageInVolts() > U_)
    {
        return true;
    }
    board::resetWatchdog();

    while(board::getOutVoltageInVolts() < 60)
    {
        if(board::getOutVoltageInVolts() > U_)
        {
            return true;
        }

        if(n < 380)                             //we ran 70 ms and Uout is under 60V, report error
        {
            board::syslog("return with error from cycle1500_3000() \r\n");
            return false;
        }
        Charger::cycle1500_3000();
        n--;
    }
    board::resetWatchdog();

    while(board::getOutVoltageInVolts() < 150)
    {
        if(board::getOutVoltageInVolts() > U_)
        {
            return true;
        }

        if(n < 300)                             //we ran 150 ms and Uout is under 150V, report error
        {
            board::syslog("return with error from cycle1500_1500(); \r\n");
            return false;
        }
        Charger::cycle1500_1500();
        n--;
    }
    board::resetWatchdog();

    while(board::getOutVoltageInVolts() < 500)
    {
        if(board::getOutVoltageInVolts() > U_)
        {
            return true;
        }

        if(n == 0)                              //we ran 450 ms and Uout is under 500V, report error
        {
            board::syslog("return with error from cycle2000_500() \r\n");
            return false;
        }
        Charger::cycle2000_500();
        n--;
    }
    board::resetWatchdog();

    return true;
}

void Charger::cycle1000_7000()
{
    __disable_irq();
    for (unsigned i = 0; i < 120; i++)         // this takes ~1ms
    {

        LPC_GPIO[1].DATA[0b010111] = 0b010111; // 1 ON
        board::delayUSec(1);

        LPC_GPIO[1].DATA[0b010111] = 0b000000; // OFF
        board::delayUSec(7);
    }
    __enable_irq();
}

void Charger::cycle1500_5000()
{
    __disable_irq();
    for (unsigned i = 0; i < 150; i++)         // this takes ~1ms
    {

        LPC_GPIO[1].DATA[0b010111] = 0b010111; // 1 ON
        board::delayUSec(1);
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");


        LPC_GPIO[1].DATA[0b010111] = 0b000000; // OFF
        board::delayUSec(5);
    }
    __enable_irq();
}

void Charger::cycle1500_3000()
{
    __disable_irq();
    for (unsigned i = 0; i < 220; i++)         // this takes ~1ms
    {

        LPC_GPIO[1].DATA[0b010111] = 0b010111; // 1 ON
        board::delayUSec(1);
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        LPC_GPIO[1].DATA[0b010111] = 0b000000; // OFF
        board::delayUSec(3);
    }
    __enable_irq();
}

void Charger::cycle1500_1500()
{
    __disable_irq();
    for (unsigned i = 0; i < 330; i++)         // this takes ~1ms
    {

        LPC_GPIO[1].DATA[0b010111] = 0b010111; // 1 ON
        board::delayUSec(1);
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        LPC_GPIO[1].DATA[0b010111] = 0b000000; // OFF
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        board::delayUSec(1);
    }
    __enable_irq();
}

void Charger::cycle2000_500()
{
    __disable_irq();
    for (unsigned i = 0; i < 400; i++)         // this takes ~1ms
    {

        LPC_GPIO[1].DATA[0b010111] = 0b010111; // 1 ON
        board::delayUSec(2);

        LPC_GPIO[1].DATA[0b010111] = 0b000000; // OFF
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");

        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
    }
    __enable_irq();
}

}
