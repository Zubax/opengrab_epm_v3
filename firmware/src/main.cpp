/*
 * OpenGrab EPM - Electropermanent Magnet
 * Copyright (C) 2015  Zubax Robotics <info@zubax.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 *         Andreas Jochum <andreas@nicadrone.com>
 */

#include <cstdio>
#include <algorithm>
#include <sys/board.hpp>
#include <uavcan_lpc11c24/uavcan_lpc11c24.hpp>
#include <uavcan/equipment/hardpoint/Command.hpp>
#include <uavcan/equipment/hardpoint/Status.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include <magnet/magnet.hpp>

namespace
{

static constexpr unsigned NodeMemoryPoolSize = 2800;

uavcan::Node<NodeMemoryPoolSize>& getNode()
{
    static uavcan::Node<NodeMemoryPoolSize> node(uavcan_lpc11c24::CanDriver::instance(),
                                                 uavcan_lpc11c24::SystemClock::instance());
    return node;
}

struct HwConfig
{
    std::uint8_t hardpoint_id = 0;
    bool use_hardpoint_id_as_node_id = false;

    static constexpr std::uint8_t NodeIDOffset = 100;
};

const HwConfig getHwConfig()
{
    static bool first = true;
    static HwConfig cfg;

    if (first)
    {
        first = false;
        const auto x = board::readDipSwitch();
        cfg.hardpoint_id = x & ((1 << (board::DipSwitchBits - 1)) - 1);
        cfg.use_hardpoint_id_as_node_id = (x >> (board::DipSwitchBits - 1)) != 0;
    }

    return cfg;
}

void callPollAndResetWatchdog()
{
    board::resetWatchdog();

    /*
     * Status LED update
     */
    const auto ts = board::clock::getMonotonic();

    static board::MonotonicTime led_update_deadline = ts;
    static bool led_status = false;
    static bool boot = true;
    static bool first_time_led_update = true;

    /*
     * LED DIP test after boot
     */
    if (boot)
    {
        boot = !boot;
        if(board::readDipSwitch() == 0)
        {
            board::setStatusLed(true);
            board::setCanLed(true);
            led_update_deadline += board::MonotonicDuration::fromMSec(500);
        }
    }

    if (ts >= led_update_deadline)
    {
        if(first_time_led_update)           // Turn off CAN status
        {
            board::setCanLed(false);
            first_time_led_update = !first_time_led_update;
        }
        led_status = !led_status;
        board::setStatusLed(led_status);

        if (led_status)
        {
            led_update_deadline += board::MonotonicDuration::fromMSec(50);
        }
        else
        {
            led_update_deadline += board::MonotonicDuration::fromMSec(
                (magnet::getHealth() == magnet::Health::Ok)      ? 950 :
                (magnet::getHealth() == magnet::Health::Warning) ? 500 : 100);
        }
        /* Debug
         * board::syslog("Vin = ", board::getSupplyVoltageInMillivolts(), " mV\r\n");
         * board::syslog("Vout = ", board::getOutVoltageInVolts(), " V\r\n");
         *
         */
        board::syslog("Mag = ", board::getMagInMilliTeslas(), " V\r\n");
    }

    /*
     * PWM control update
     */
    const auto pwm = board::getPwmInput();
    if (pwm != board::PwmInput::NoSignal &&
        pwm != board::PwmInput::Neutral)
    {
        if (pwm == board::PwmInput::High)
        {
            magnet::turnOn(magnet::MinTurnOnCycles);
        }
        if (pwm == board::PwmInput::Low)
        {
            magnet::turnOff();
        }
    }

    /*
     * Button update
     */
    if (board::hadButtonPressEvent())
    {
        if (magnet::isTurnedOn())
        {
            magnet::turnOff();
        }
        else
        {
            magnet::turnOn(magnet::MinTurnOnCycles);
        }
    }

    /*
     * Magnet update
     */
    magnet::poll();
}

uavcan::NodeID performDynamicNodeIDAllocation()
{
    uavcan::DynamicNodeIDClient client(getNode());

    const int client_start_res = client.start(getNode().getHardwareVersion().unique_id);
    if (client_start_res < 0)
    {
        board::die();
    }

    while (!client.isAllocationComplete())
    {
        (void)getNode().spinOnce();
        callPollAndResetWatchdog();
    }

    return client.getAllocatedNodeID();
}

void fillNodeInfo()
{
    getNode().setName("com.zubax.opengrab_epm_v3");

    {
        uavcan::protocol::SoftwareVersion swver;

        swver.major = FW_VERSION_MAJOR;
        swver.minor = FW_VERSION_MINOR;
        swver.vcs_commit = GIT_HASH;
        swver.optional_field_flags = swver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;

        getNode().setSoftwareVersion(swver);
    }

    {
        uavcan::protocol::HardwareVersion hwver;

        hwver.major = HW_VERSION_MAJOR;

        {
            board::UniqueID uid;
            board::readUniqueID(uid);
            std::copy(std::begin(uid), std::end(uid), std::begin(hwver.unique_id));
        }

        {
            board::DeviceSignature coa;
            if (board::tryReadDeviceSignature(coa))
            {
                std::copy(std::begin(coa), std::end(coa), std::back_inserter(hwver.certificate_of_authenticity));
            }
        }

        getNode().setHardwareVersion(hwver);
    }
}

void configureAcceptanceFilters()
{
    // These masks are specific for UAVCAN - we're using only extended data frames and nothing else.
    static constexpr auto CommonIDBits   = uavcan::CanFrame::FlagEFF;
    static constexpr auto CommonMaskBits = uavcan::CanFrame::FlagEFF |
                                           uavcan::CanFrame::FlagRTR |
                                           uavcan::CanFrame::FlagERR;

    static constexpr auto NodeIDShift     = 8;
    static constexpr auto MessageMaskBits = 0xFFFF80U;
    static constexpr auto ServiceIDBits   = 0x80U;
    static constexpr auto ServiceMaskBits = 0x7F80U;

    // Allocating a buffer large enough to fit all filters the application may need.
    static constexpr unsigned MaxFilterConfigs = 32;
    uavcan::CanFilterConfig filter_configs[MaxFilterConfigs];
    std::uint16_t filter_config_index = 0;

    // Building message filters.
    auto p = getNode().getDispatcher().getListOfMessageListeners().get();
    while (p != NULL)
    {
        filter_configs[filter_config_index].id =
            (static_cast<unsigned>(p->getDataTypeDescriptor().getID().get()) << NodeIDShift) | CommonIDBits;

        filter_configs[filter_config_index].mask = MessageMaskBits | CommonMaskBits;

        p = p->getNextListNode();

        filter_config_index++;
        if (filter_config_index >= std::min<unsigned>(MaxFilterConfigs,
                                                      uavcan_lpc11c24::CanDriver::instance().getNumFilters()))
        {
            board::die(); // Filter compaction algorithm defined in libuavcan is not used because of memory constraints
        }
    }

    // Adding one filter for unicast transfers - note that it's filtering on our Node ID.
    filter_configs[filter_config_index].id =
        ServiceIDBits | (static_cast<unsigned>(getNode().getNodeID().get()) << NodeIDShift) | CommonIDBits;
    filter_configs[filter_config_index].mask = ServiceMaskBits | CommonMaskBits;

    filter_config_index++;

    // Sending the configuration to the CAN driver.
    if (uavcan_lpc11c24::CanDriver::instance().configureFilters(filter_configs, filter_config_index) < 0)
    {
        board::die();
    }
}

void handleHardpointCommand(const uavcan::equipment::hardpoint::Command& msg)
{
    if (msg.hardpoint_id != getHwConfig().hardpoint_id)
    {
        return;
    }

    /*
     * The last command field is initialized at an impossible value in order to force a switch once
     * the first command is received. This will force the magnet into a known state.
     */
    static unsigned last_command = std::numeric_limits<unsigned>::max();

    if ((bool(msg.command) != magnet::isTurnedOn()) || (msg.command != last_command))
    {
        if (msg.command == 0)
        {
            magnet::turnOff();
        }
        else
        {
            magnet::turnOn(msg.command);
        }
    }

    // Oi moroz moroz ne moroz' mena
    last_command = msg.command; // Ne moroz' mena moigo kona
}

void publishHardpointStatus()
{
    static const auto Priority = uavcan::TransferPriority::MiddleLower;

    static uavcan::Publisher<uavcan::equipment::hardpoint::Status> pub(getNode());

    static bool initialized = false;
    if (!initialized)
    {
        initialized = true;
        pub.setPriority(Priority);
    }

    uavcan::equipment::hardpoint::Status msg;

    msg.hardpoint_id = getHwConfig().hardpoint_id;

    msg.payload_weight = std::numeric_limits<float>::quiet_NaN();
    msg.payload_weight_variance = std::numeric_limits<float>::infinity();

    msg.status = magnet::isTurnedOn() ? 1 : 0;

    (void)pub.broadcast(msg);
}

void updateUavcanStatus(const uavcan::TimerEvent&)
{
    publishHardpointStatus();

    switch (magnet::getHealth())
    {
    case magnet::Health::Ok:
    {
        getNode().setHealthOk();
        break;
    }
    case magnet::Health::Warning:
    {
        getNode().setHealthWarning();
        break;
    }
    default:
    {
        getNode().setHealthError();
        break;
    }
    }

    getNode().setVendorSpecificStatusCode(
        static_cast<std::uint16_t>(magnet::getStatusFlags() | ((board::getOutVoltageInVolts() >> 1) << 8)));
}

void updateCanLed(const uavcan::TimerEvent&)
{
    board::setCanLed(uavcan_lpc11c24::CanDriver::instance().hadActivity());
}

#if __GNUC__
__attribute__((noinline))
#endif
void init()
{
    board::syslog("Boot\r\n");
    board::resetWatchdog();

    callPollAndResetWatchdog();

    /*
     * Configuring the CAN controller
     */
    std::uint32_t bit_rate = 0;
    while (bit_rate == 0)
    {
        bit_rate = uavcan_lpc11c24::CanDriver::detectBitRate(&callPollAndResetWatchdog);
    }
    board::syslog("Bitrate: ", bit_rate, "\r\n");

    if (uavcan_lpc11c24::CanDriver::instance().init(bit_rate) < 0)
    {
        board::die();
    }

    board::syslog("CAN init ok\r\n");

    callPollAndResetWatchdog();

    /*
     * Starting the node
     */
    fillNodeInfo();

    if (getNode().start() < 0)
    {
        board::die();
    }

    callPollAndResetWatchdog();

    /*
     * Starting the node and performing dynamic node ID allocation
     */
    if (getNode().start() < 0)
    {
        board::die();
    }

    /*
     * Initializing other libuavcan-related objects
     * Why reinterpret_cast<>() on function pointers? Try to remove it, or replace with static_cast. GCC is fun.   D:
     */
    static uavcan::TimerEventForwarder<void (*)(const uavcan::TimerEvent&)> can_led_timer(getNode());   // CAN LED tmr
    can_led_timer.setCallback(reinterpret_cast<decltype(can_led_timer)::Callback>(&updateCanLed));
    can_led_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(25));

    if (getHwConfig().use_hardpoint_id_as_node_id)
    {
        board::syslog("Node ID is fixed\r\n");
        getNode().setNodeID(static_cast<std::uint8_t>(getHwConfig().hardpoint_id + HwConfig::NodeIDOffset));
    }
    else
    {
        board::syslog("Node ID allocation...\r\n");
        getNode().setNodeID(performDynamicNodeIDAllocation());
    }

    board::syslog("Node ID ", getNode().getNodeID().get(), "\r\n");

    callPollAndResetWatchdog();

    /*
     * Initializing other libuavcan-related objects
     */
    static uavcan::TimerEventForwarder<void (*)(const uavcan::TimerEvent&)> update_timer(getNode());    // Status pub
    update_timer.setCallback(reinterpret_cast<decltype(update_timer)::Callback>(&updateUavcanStatus));
    update_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(500));

    static uavcan::Subscriber<uavcan::equipment::hardpoint::Command,                                    // Command sub
                              void (*)(const uavcan::equipment::hardpoint::Command&)> command_sub(getNode());
    if (command_sub.start(reinterpret_cast<decltype(command_sub)::Callback>(&handleHardpointCommand)) < 0)
    {
        board::die();
    }

    /*
     * Configuring the filters in the last order, when all subscribers are initialized.
     */
    configureAcceptanceFilters();
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
            board::syslog("Spin error ", res, "\r\n");
        }

        callPollAndResetWatchdog();
    }
}
