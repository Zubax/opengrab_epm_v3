/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 * Author: Andreas Jochum <Andreas@Nicadrone.com>
 */

#include <cstdio>
#include <algorithm>
#include <sys/board.hpp>
#include <uavcan_lpc11c24/uavcan_lpc11c24.hpp>
#include <uavcan/equipment/hardpoint/Command.hpp>
#include <uavcan/equipment/hardpoint/Status.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include <magnet/magnet.hpp>

#if __GNUC__
# pragma GCC optimize 1
#endif

namespace
{

static constexpr unsigned NodeMemoryPoolSize = 2800;

uavcan::Node<NodeMemoryPoolSize>& getNode()
{
    static uavcan::Node<NodeMemoryPoolSize> node(uavcan_lpc11c24::CanDriver::instance(),
                                                 uavcan_lpc11c24::SystemClock::instance());
    return node;
}

std::uint8_t getHardpointID()
{
    static int cached = -1;
    if (cached < 0)
    {
        cached = board::readDipSwitch();
    }
    return std::uint8_t(cached);
}

void callPollAndResetWatchdog()
{
    board::resetWatchdog();
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
    getNode().setName("com.zubax.opengrab_epm");

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

        board::UniqueID uid;
        board::readUniqueID(uid);
        std::copy(std::begin(uid), std::end(uid), std::begin(hwver.unique_id));

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

    board::syslog("Installed ", filter_config_index, " HW filters\r\n");
}

void handleHardpointCommand(const uavcan::equipment::hardpoint::Command& msg)
{
    if (msg.hardpoint_id != getHardpointID())
    {
        return;
    }

    board::syslog("Cmd ", msg.command, "\r\n");

    // XXX TODO handle
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

    msg.hardpoint_id = getHardpointID();
    msg.status = 0;                                             // XXX TODO

    (void)pub.broadcast(msg);
}

void updateUavcanStatus(const uavcan::TimerEvent&)
{
    publishHardpointStatus();

    // TODO: node health update
}

#if __GNUC__
__attribute__((noinline))
#endif
void init()
{
    board::syslog("Boot\r\n");
    board::resetWatchdog();

    /*
     * Configuring the clock
     */
    uavcan_lpc11c24::clock::init();

    /*
     * Initializing the magnet before first poll() is called
     */
    magnet::init();

    callPollAndResetWatchdog();

    /*
     * Configuring the CAN controller
     */
    std::uint32_t bit_rate = 0;
    while (bit_rate == 0)
    {
        board::syslog("CAN auto bitrate...\r\n");
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

    board::syslog("Node ID allocation...\r\n");

    getNode().setNodeID(performDynamicNodeIDAllocation());

    board::syslog("Node ID ", getNode().getNodeID().get(), "\r\n");

    callPollAndResetWatchdog();

    /*
     * Initializing other libuavcan-related objects
     * Why reinterpret_cast<>() on function pointers? Try to remove it, or replace with static_cast. GCC is fun.   D:
     */
    static uavcan::TimerEventForwarder<void (*)(const uavcan::TimerEvent&)> update_timer(getNode());
    update_timer.setCallback(reinterpret_cast<decltype(update_timer)::Callback>(&updateUavcanStatus));
    update_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(500));

    static uavcan::Subscriber<uavcan::equipment::hardpoint::Command,
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
