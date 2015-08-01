/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */


/* todo
 * VDD CAN is Vin, generate critical error when Vin >6.5V, abs max 7V, CAN transiver limit
 * BOR detect
 * It would be nice to write critial errors to flash
 */

#include <cstdio>
#include <algorithm>
#include <board.hpp>
#include <chip.h>
#include <uavcan_lpc11c24/uavcan_lpc11c24.hpp>
#include <uavcan/protocol/logger.hpp>

bool magnetState = false; 
namespace
{

struct CriticalSectionLocker
{
    CriticalSectionLocker()  { __disable_irq(); }
    ~CriticalSectionLocker() { __enable_irq(); }
};

typedef uavcan::Node<2800> Node;

Node& getNode()
{
    static Node node(uavcan_lpc11c24::CanDriver::instance(), uavcan_lpc11c24::SystemClock::instance());
    return node;
}

#if __GNUC__
__attribute__((noreturn))
#endif
void die()
{
    while (true) { }
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
        die();
    }

    board::resetWatchdog();

    getNode().setNodeID(72);
    getNode().setName("org.uavcan.lpc11c24_test");

    uavcan::protocol::SoftwareVersion swver;
    swver.major = FW_VERSION_MAJOR;
    swver.minor = FW_VERSION_MINOR;
    swver.vcs_commit = GIT_HASH;
    swver.optional_field_mask = swver.OPTIONAL_FIELD_MASK_VCS_COMMIT;
    getNode().setSoftwareVersion(swver);

    uavcan::protocol::HardwareVersion hwver;
    std::uint8_t uid[board::UniqueIDSize] = {};
    board::readUniqueID(uid);
    std::copy(std::begin(uid), std::end(uid), std::begin(hwver.unique_id));
    getNode().setHardwareVersion(hwver);

    board::resetWatchdog();

    while (getNode().start() < 0)
    {
    }

    board::resetWatchdog();
}

void delayMSec(unsigned msec)
{
    while (msec --> 0)
    {
        board::delayUSec(250);
        board::delayUSec(250);
        board::delayUSec(250);
        board::delayUSec(240);  // Calibration
    }
}

void blinkStatusMs(const unsigned delay_ms, unsigned times = 1)
{
    while (times --> 0)
    {
        board::setStatusLed(true);
        delayMSec(delay_ms);
        board::setStatusLed(false);
        if (times > 0)
        {
            delayMSec(delay_ms);
        }
    }
}

void reverse(char* s)
{
    for (int i = 0, j = int(std::strlen(s)) - 1; i < j; i++, j--)
    {
        const char c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}

void lltoa(long long n, char buf[24])
{
    const short sign = (n < 0) ? -1 : 1;
    if (sign < 0)
    {
        n = -n;
    }
    unsigned i = 0;
    do
    {
        buf[i++] = char(n % 10 + '0');
    }
    while ((n /= 10) > 0);
    if (sign < 0)
    {
        buf[i++] = '-';
    }
    buf[i] = '\0';
    reverse(buf);
}

void magnetOn()
{
    board::setMagnetPos();
}
void magnetOff()
{
    board::setMagnetNeg();
}


void poll()
{
    const auto supply_voltage_mV = board::getSupplyVoltageInMillivolts();
    
    const auto output_voltage_V  = board::getOutVoltageInVolts();

    const auto pwm_input = board::getPwmInputPeriodInMicroseconds();

    if (board::hadButtonPressEvent())
    {
        /*
         * Push button is pressed
         * LED status blink 3 times fast
         * Charge the capacitor, for now just pull SW_L and SW_H high twice, ton 2us, toff 10us just for debug
         * Toggle EPM by
         * Pulling  CTRL 1, 4 or 2,3 high ~5us to toggle state
         */
        board::syslog("Toggling the magnet\r\n");

        // Indication
        blinkStatusMs(30, 3);

        // Charging 
        for (unsigned i = 0; i < 1000; i++)
        {
            
		
				CriticalSectionLocker locker;
				
                LPC_GPIO[1].DATA[0b010111] = 0b010111; // ON
                board::delayUSec(2);
                LPC_GPIO[1].DATA[0b010111] = 0b000000; // OFF
                board::delayUSec(2);
        }  
        for (unsigned i = 0; i < 120000; i++)
        {
            
		
				CriticalSectionLocker locker;
				
                LPC_GPIO[1].DATA[0b010111] = 0b010111; // ON
                board::delayUSec(2);
                LPC_GPIO[1].DATA[0b010111] = 0b000000; // OFF
                board::delayUSec(1);
        }   
        
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
		board::delayUSec(255);
        // Toggling the magnet
		if(magnetState == true) 
		{
			board::syslog("Calling mangetOff");
			board::syslog("\r\n");
			magnetOff();
		}
		if(magnetState == false)
		{
			board::syslog("Calling magnetOn");
			board::syslog("\r\n");
			magnetOn();
		}
		magnetState = !magnetState;
        
        board::syslog("Magnet state: ");
        board::syslog(magnetState ? "On" : "off");
        board::syslog("\r\n");

        // Printing the supply voltage
        char buf[24];
        lltoa(supply_voltage_mV, buf);
        board::syslog("supply_voltage_mV =  ");
        board::syslog(static_cast<const char*>(buf));
        board::syslog(" mV\r\n");
        
		// Printing the output voltage
		lltoa(output_voltage_V, buf);
		board::syslog("output_voltage_V = ");
        board::syslog(static_cast<const char*>(buf));
        board::syslog(" mV\r\n");

        // Printing the PWM input state
        lltoa(pwm_input, buf);
        board::syslog("PWM width: ");
        board::syslog(static_cast<const char*>(buf));
        board::syslog(" usec\r\n");
    }
}

}

int main()
{
    init();

    getNode().setStatusOk();

    board::setStatusLed(false);

    board::syslog("Init OK\r\n");

    while (true)
    {
        const int res = getNode().spinOnce();
        if (res < 0)
        {
            board::syslog("Spin err ");
            char s[] = {static_cast<char>('A' - res), '\r', '\n', '\0'};
            board::syslog(static_cast<const char*>(s));
        }

        poll();

        board::resetWatchdog();
    }
}
