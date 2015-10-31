/*
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

bool Charger::run(unsigned U_)
{
    /*
     * This is Plan A, run the flyback dynamicly
     * On time is 9000/(Vin-0.5V) [us, mV]
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

    //Check input voltage
    
    const auto supply_voltage_mV = board::getSupplyVoltageInMillivolts();
    if(supply_voltage_mV < 4500 || supply_voltage_mV > 6500)        //formated wrong?
    {
        board::syslog("Error bad voltage: ", supply_voltage_mV, " mV\r\n");
        return false;
    }
    
    //Calculate on and off time, this will turn into a radom number generator if Vin is out of range
    
    unsigned on_time=((9000000/(supply_voltage_mV-500))/104);                                      //aproximation, it's probalby good, needs checking
    
    const auto output_voltage_V = board::getOutVoltageInVolts();
 
    
    unsigned off_time=((((9000000/(supply_voltage_mV-500))/(output_voltage_V+1))*50)-208)/104;     //should be correct now
    
    //when output_voltage is relaly low off time is to long
    
    if(off_time > 120)       //8us
    {
        off_time = 120;
    }
    
    //debug
   
    board::syslog("\r\n ", on_time, "\r\n");
    board::syslog("\r\n ", off_time, "\r\n");
    
    //sanity check and run a few cycles 

    if(on_time > 0 && on_time < 30  && off_time >0 )
    {
        board::runPump(10,on_time,off_time);       
    }    
    
    return false;   //debug 
    
    unsigned n=22500;                           //max run time in ruffly ms

    //cycle1000_7000();                           //U out is very low, transfomers goes in saturation when toff is smal

    //cycle(5, 0);                              //just for debug 
    //cycle(5, 0);    //toff is 840ns, we need this to go down to 500ns
    return true;
                      
   


        
    while(board::getOutVoltageInVolts() < 30)
    {
        n--;
        if(board::getOutVoltageInVolts() > U_)
        {
            return true;
        }
        if(n < 21500)                             //we ran 20 ms and Uout is under 30V, report error
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

        if(n < 19000)                             //we ran 70 ms and Uout is under 60V, report error
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

        if(n < 15000)                             //we ran 150 ms and Uout is under 150V, report error
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


void Charger::cycle(unsigned on, unsigned off)
{

    /* on:
    0:350ns
    1:650ns
    2:1000ns
    3:1250ns
    4:1600ns
    5:2000ns
    
    off:
    0:840ns
    1:1200ns
    2:1500ns
    3:1800ns
    4:2200ns
    */

    __disable_irq();
    //board::setPumpSwitch(true);           //too slow
    LPC_GPIO[1].DATA[0b010111] = 0b010111;  // 1 ON
    for (volatile unsigned i=0;  i<on;  i++){
     //__asm__ volatile ("nop"); 
    }

    //board::setPumpSwitch(false);          //too slow
    LPC_GPIO[1].DATA[0b010111] = 0b000000; // OFF
    for (volatile unsigned i=0;  i<off;  i++){
     //__asm__ volatile ("nop");     
    }
    __enable_irq();
}
    

void Charger::cycle1000_7000()
{
    __disable_irq();
    for (unsigned i = 0; i < 3; i++)         // this takes ~50us
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
    for (unsigned i = 0; i < 3; i++)         // this takes ~50us
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
    for (unsigned i = 0; i < 5; i++)         // this takes ~50us
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
    for (unsigned i = 0; i < 7; i++)         // this takes ~50us
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
    for (unsigned i = 0; i < 8; i++)         // this takes ~50us
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
