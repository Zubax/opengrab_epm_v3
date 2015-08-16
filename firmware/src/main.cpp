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

struct CriticalSectionLocker
{
    CriticalSectionLocker()
    {
        __disable_irq();
    }
    ~CriticalSectionLocker()
    {
        __enable_irq();
    }
};

typedef uavcan::Node<2800> Node;

static charger::Charger chrg;

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
    while (true)
    {
    }
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

void magnetOn()
{
    chrg.restart(100);
    chrg.run();
    board::setMagnetPos();
}

void magnetOff()
{
    board::setMagnetNeg();
}

void poll()
{
    const auto supply_voltage_mV = board::getSupplyVoltageInMillivolts();

//    const auto output_voltage_V  = board::getOutVoltageInVolts();

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

        static bool magnet_state = false;

        // Toggling the magnet
        if (magnet_state == true)
        {
            magnetOff();
            board::syslog("Calling mangetOff");
            board::syslog("\r\n");
        }

        if (magnet_state == false)
        {
            magnetOn();
            board::syslog("Calling magnetOn");
            board::syslog("\r\n");
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
            char s[] = { static_cast<char>('A' - res), '\r', '\n', '\0' };
            board::syslog(static_cast<const char*>(s));
        }

        poll();

        board::resetWatchdog();
    }
}
