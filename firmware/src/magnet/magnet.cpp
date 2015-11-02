/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

/*
 * TODO
 * VDD CAN is Vin, generate critical error when Vin >6.5V, abs max 7V, CAN transiver limit
 * BOR detect
 */

#include "magnet.hpp"
#include "charger.hpp"
#include <sys/board.hpp>

namespace magnet
{
namespace
{

static charger::Charger chrg;

/// TODO: This will be removed
void blinkStatusMs(const unsigned delay_ms, unsigned times = 1)
{
    while (times --> 0)
    {
        board::setStatusLed(true);
        board::delayMSec(delay_ms);
        board::setStatusLed(false);
        if (times > 0)
        {
            board::delayMSec(delay_ms);
        }
    }
}

void magnetOn()
{
    chrg.set(450);
    
    board::syslog(chrg.run() ? "sucess \r\n" : "failed \r\n");

    board::setMagnetPos();
    
    board::delayMSec(5);
    chrg.set(450);
    board::syslog(chrg.run() ? "sucess \r\n" : "failed \r\n");

    board::setMagnetPos();

    //limit duty cycle
    board::delayMSec(250);
    board::delayMSec(250);
}

static bool magnet_state = false;
//Todo, this needs to be calibrated
void magnetOff()
{
    /*
    unsigned blah=5;

    if(magnet_state == true)        //If we are not coming from the on state we skip a few cycles
    {
        board::syslog(chrg.run(450) ? "sucess \r\n" : "failed \r\n");
        board::setMagnetNeg();
        board::delayMSec(blah);
        
        board::syslog(chrg.run(450) ? "sucess \r\n" : "failed \r\n");
        board::setMagnetNeg();
        board::delayMSec(blah);

        board::syslog(chrg.run(300) ? "sucess \r\n" : "failed \r\n");
        board::setMagnetPos();
        board::delayMSec(blah);
    }

    board::syslog(chrg.run(180) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetNeg();
    board::delayMSec(blah);

    board::syslog(chrg.run(162) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetPos();
    board::delayMSec(blah);

    board::syslog(chrg.run(146) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetNeg();
    board::delayMSec(blah);

    board::syslog(chrg.run(131) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetPos();
    board::delayMSec(blah);

    board::syslog(chrg.run(118) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetNeg();
    board::delayMSec(blah);

    board::syslog(chrg.run(106) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetPos();
    board::delayMSec(blah);

    board::syslog(chrg.run(96) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetNeg();
    board::delayMSec(blah);

    board::syslog(chrg.run(86) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetPos();
    board::delayMSec(blah);

    board::syslog(chrg.run(77) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetNeg();
    board::delayMSec(blah);

    board::syslog(chrg.run(70) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetPos();
    board::delayMSec(blah);

    board::syslog(chrg.run(63) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetNeg();
    board::delayMSec(blah);

    board::syslog(chrg.run(56) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetPos();
    board::delayMSec(blah);

    board::syslog(chrg.run(51) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetNeg();
    board::delayMSec(blah);

    board::syslog(chrg.run(46) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetPos();
    board::delayMSec(blah);

    board::syslog(chrg.run(41) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetNeg();
    board::delayMSec(blah);

    board::syslog(chrg.run(37) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetPos();
    board::delayMSec(blah);

    board::syslog(chrg.run(33) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetNeg();
    board::delayMSec(blah);

    board::syslog(chrg.run(30) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetPos();
    board::delayMSec(blah);

    board::syslog(chrg.run(27) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetNeg();
    board::delayMSec(blah);

    board::syslog(chrg.run(24) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetPos();
    board::delayMSec(blah);

    board::syslog(chrg.run(22) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetNeg();
    board::delayMSec(blah);

    board::syslog(chrg.run(20) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetPos();
    board::delayMSec(blah);

    board::syslog(chrg.run(18) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetNeg();
    board::delayMSec(blah);

    board::syslog(chrg.run(16) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetPos();
    board::delayMSec(blah);

    board::syslog(chrg.run(14) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetNeg();
    board::delayMSec(blah);

    board::syslog(chrg.run(13) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetPos();
    board::delayMSec(blah);

    board::syslog(chrg.run(12) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetNeg();
    board::delayMSec(blah);

    board::syslog(chrg.run(10) ? "sucess \r\n" : "failed \r\n");
    board::setMagnetPos();

    //limit duty cycle
    board::delayMSec(250);
    board::delayMSec(250);
    board::delayMSec(250);
    */
}

}

void init()
{

}

void poll()
{
    const auto supply_voltage_mV = board::getSupplyVoltageInMillivolts();

    const auto pwm_input = board::getPwmInputPulseLengthInMicroseconds();

    if (board::hadButtonPressEvent())
    {
        /*
         * Push button is pressed
         * LED status blink 3 times fast
         * Charge the cacitor, for now just pull SW_L and SW_H high twice, ton 2us, toff 10us just for debug
         * Toggle EPM by
         * Pulling  CTRL 1, 4 or 2,3 high ~5us to toggle state
         */
        board::syslog("Toggling the magnet\r\n");

        // Indication
        blinkStatusMs(30, 3);
        board::delayUSec(5);

        // Toggling the magnet
        if (magnet_state == true)
        {

           // board::syslog("Calling mangetOff");
            board::syslog("\r\n");
            magnetOn();
        //    magnetOff();  //debug 
        }

        if (magnet_state == false)
        {

            board::syslog("Calling magnetOn");
            board::syslog("\r\n");
            magnetOn();
        }
        magnet_state = !magnet_state;

        board::syslog("Magnet state:   ", int(magnet_state), "\r\n");
        board::syslog("Supply voltage: ", supply_voltage_mV, "mV\r\n");
        board::syslog("PWM width:      ", pwm_input, " usec\r\n");
    }

    if(pwm_input > 1000 && pwm_input < 1250)
    {
        blinkStatusMs(30, 3);
        board::delayUSec(5);

        //Turn magnet off
        magnetOff();
        magnet_state = false;
    }

    if(pwm_input > 1750 && pwm_input < 2000)
    {
        blinkStatusMs(30, 3);
        board::delayUSec(5);

        //Turn magnet on
        magnetOn();
        magnet_state = true;
    }
}

Health getHealth()
{
    return Health::Ok; // TODO
}

}


