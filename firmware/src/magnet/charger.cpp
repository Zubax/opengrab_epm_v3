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

void Charger::set(unsigned U_)
{
    U = U_;
    time_out = 200000;       //time out in cycle count
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

    //unsigned i = 0; //debug
 
    while(board::getOutVoltageInVolts() < U)
    {
         //Check input voltage
    
        const auto supply_voltage_mV = board::getSupplyVoltageInMillivolts();
        if(supply_voltage_mV < 4500 || supply_voltage_mV > 6500)        //formated wrong?
        {
            board::syslog("Error bad voltage: ", supply_voltage_mV, " mV\r\n");
            return false;
        }
        
        //Calculate on and off time, this will turn into a radom number generator if Vin is out of range
        
        unsigned on_time=((5000000/(supply_voltage_mV-500))/104);                                      //aproximation, it's probalby good, needs checking
        
        const auto output_voltage_V = board::getOutVoltageInVolts(); 
    
        //i += 20;
        
        unsigned off_time=((5000000/(supply_voltage_mV-500)/(output_voltage_V+1))*50);                 //should be correct now
        if(off_time > 312)                                                                             //prevent overflow 
            off_time = (off_time - 208)/104;
        else   
            off_time = 1;

        //when output_voltage is relaly low off time is to long
        
        if(off_time > 120)       
        {
            off_time = 120;
        }

        time_out--;
        if(time_out == 0)
        {      
            return false;                       //timed out, return with error 
        }
          
        //sanity check and run a few cycles 

        if(on_time > 0 && on_time < 30  && off_time > 0)
        {

            //board::syslog("\r\n U out   : ", output_voltage_V, "\r\n");
            //board::syslog(" V in    : ", supply_voltage_mV, "\r\n");
            //board::syslog(" on_time : ", on_time, "\r\n");
            //board::syslog(" off_time: ", off_time, "\r\n");
            
            board::runPump(100,on_time,off_time);

                   
        }
               
    }

    
    //we either timed out and existed with error or output voltage is reached

    done = true; 
    
    time_out = 0;                               //make sure we don't run again without setting _U
    
    U = 0;

    return true; 
}
}
