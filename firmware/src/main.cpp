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

void poll1kHz()
{
    if (board::hadButtonPressEvent())
    {
        board::syslog("Press\r\n");
        board::setCanLed(true);
        board::setStatusLed(true);
        delayMSec(50);
        board::setCanLed(false);
        board::setStatusLed(false);
        delayMSec(50);
        board::setCanLed(true);
        board::setStatusLed(true);
        delayMSec(50);
        board::setCanLed(false);
        board::setStatusLed(false);
        delayMSec(50);
        board::setCanLed(true);
        board::setStatusLed(true);
        delayMSec(50);
        board::setCanLed(false);
        board::setStatusLed(false);
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
