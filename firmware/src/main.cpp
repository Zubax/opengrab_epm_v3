/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include <cstdio>
#include <algorithm>
#include <board.hpp>
#include <chip.h>
#include <uavcan_lpc11c24/uavcan_lpc11c24.hpp>
#include <uavcan/protocol/logger.hpp>

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

void poll1kHz()
{
    const auto supply_voltage_mv = board::getSupplyVoltageInMillivolts();

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
        for (unsigned i = 0; i < 2; i++)
        {
            {
                CriticalSectionLocker locker;
                board::setChargePumpSwitch(true);
                board::delayUSec(2);
                board::setChargePumpSwitch(false);
            }
            board::delayUSec(10);
        }

        // Toggling the magnet
        static bool last_magnet_state = false;
        board::setMagnetBridgeState(last_magnet_state ?
                                    board::MagnetBridgeState::RightHighLeftLow :
                                    board::MagnetBridgeState::RightLowLeftHigh);
        board::delayUSec(5);
        board::setMagnetBridgeState(board::MagnetBridgeState::BothLow);
        last_magnet_state = !last_magnet_state;

        // Printing the supply voltage
        char buf[24];
        lltoa(supply_voltage_mv, buf);
        board::syslog(static_cast<const char*>(buf));
        board::syslog(" mV\r\n");

        // Printing the PWM input state
        lltoa(pwm_input, buf);
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
        // Spin duration defines poll interval for other functions
        const int res = getNode().spin(uavcan::MonotonicDuration::fromMSec(1));
        if (res < 0)
        {
            board::syslog("Spin err ");
            char s[] = {static_cast<char>('A' - res), '\r', '\n', '\0'};
            board::syslog(static_cast<const char*>(s));
        }

        poll1kHz();

        board::resetWatchdog();
    }
}
