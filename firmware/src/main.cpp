/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 * Author: Andreas Jochum <Andreas@Nicadrone.com>
 */

/* todo
 * VDD CAN is Vin, generate critical error when Vin >6.5V, abs max 7V, CAN transiver limit
 * BOR detect
 */

#include <cstdio>
#include <algorithm>
#include <board.hpp>
#include <chip.h>
#include <uavcan_lpc11c24/uavcan_lpc11c24.hpp>
#include <uavcan/protocol/logger.hpp>
#include "charger.hpp"
#include "util.hpp"

namespace
{

typedef uavcan::Node<2800> Node;

static charger::Charger chrg;

Node& getNode()
{
    static Node node(uavcan_lpc11c24::CanDriver::instance(), uavcan_lpc11c24::SystemClock::instance());
    return node;
}

#if __GNUC__
__attribute__((noinline))
#endif
void init()
{
    board::syslog("Init started \r\n");
    board::resetWatchdog();

    if (uavcan_lpc11c24::CanDriver::instance().init(1000000) < 0)
    {
        board::die();
    }

    board::resetWatchdog();

    getNode().setNodeID(72);
    getNode().setName("org.uavcan.lpc11c24_test");

    uavcan::protocol::SoftwareVersion swver;
    swver.major = FW_VERSION_MAJOR;
    swver.minor = FW_VERSION_MINOR;
    swver.vcs_commit = GIT_HASH;
    swver.optional_field_flags = swver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;
    getNode().setSoftwareVersion(swver);

    uavcan::protocol::HardwareVersion hwver;
    std::uint8_t uid[board::UniqueIDSize] = { };
    board::readUniqueID(uid);
    std::copy(std::begin(uid), std::end(uid), std::begin(hwver.unique_id));
    getNode().setHardwareVersion(hwver);

    board::resetWatchdog();

    while (getNode().start() < 0)
    {
    }

    board::resetWatchdog();
}

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
    board::syslog(chrg.run(450) ? "sucess \r\n" : "failed \r\n");
    
    board::setMagnetPos();
    
    board::delayMSec(5);
    
    board::syslog(chrg.run(450) ? "sucess \r\n" : "failed \r\n");
    
    board::setMagnetPos();
 
    //limit duty cycle	
    board::delayMSec(250);
    board::delayMSec(250);
}

static bool magnet_state = false;
//Todo, this needs to be calibrated 
void magnetOff()
{
    unsigned blah=5;

    if(magnet_state == true)        //If we are not coming from the on state we skip a few cycles     
    {
        board::syslog(chrg.run(450) ? "sucess \r\n" : "failed \r\n");

        board::setMagnetNeg();

        board::delayMSec(blah);
        board::syslog(chrg.run(450) ? "sucess \r\n" : "failed \r\n");
        board::setMagnetNeg();

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
}

void poll()
{
    const auto supply_voltage_mV = board::getSupplyVoltageInMillivolts();

    const auto pwm_input = board::getPwmInputPeriodInMicroseconds();

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
            
            board::syslog("Calling mangetOff");
            board::syslog("\r\n");
            magnetOff();
        }

        if (magnet_state == false)
        {
            
            board::syslog("Calling magnetOn");
            board::syslog("\r\n");
            magnetOn();
        }
        magnet_state = !magnet_state;

        board::syslog("Magnet state: ");
        board::syslog(magnet_state ? "On" : "off");
        board::syslog("\r\n");

        // Printing the supply voltage
        char buf[24];
        util::lltoa(supply_voltage_mV, buf);
        board::syslog("supply_voltage_mV =  ");
        board::syslog(static_cast<const char*>(buf));
        board::syslog(" mV\r\n");

        // Printing the PWM input state
        util::lltoa(pwm_input, buf);
        board::syslog("PWM width: ");
        board::syslog(static_cast<const char*>(buf));
        board::syslog(" usec\r\n");
    }

    
    if(pwm_input < 1250 && pwm_input > 1000)
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

}

int main()
{
    init();

    getNode().setModeOperational();

    board::setStatusLed(false);

    board::syslog("Init OK\r\n");

    while (true)
    {
        const int res = getNode().spinOnce();
        if (res < 0)
        {
            board::syslog("Spin err ");
            char s[] = { static_cast<char>('A' - res), '\r', '\n', '\0' };
            board::syslog(static_cast<const char*>(s));
        }

        poll();

        board::resetWatchdog();
    }
}
