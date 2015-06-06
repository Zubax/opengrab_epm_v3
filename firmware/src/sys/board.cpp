/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 *
 * Board initialization for Olimex LPC11C24
 */

#include "board.hpp"
#include <chip.h>
#include <cstdlib>
#include <cstring>
#include <numeric>
#include <cstdint>
#include <algorithm>

#ifndef BOARD_OLIMEX_LPC_P11C24
#define BOARD_OLIMEX_LPC_P11C24 0
#endif

#define PDRUNCFGUSEMASK 0x0000ED00
#define PDRUNCFGMASKTMP 0x000000FF

constexpr std::uint32_t OscRateIn = 12000000; ///< External crystal
constexpr std::uint32_t ExtRateIn = 0;

std::uint32_t SystemCoreClock = 12000000; ///< Initialized to default clock value, will be changed on init

namespace board
{
namespace
{

constexpr std::uint32_t TargetSystemCoreClock = 48000000;

struct PortPin
{
    const std::uint8_t port : 3;
    const std::uint8_t pin  : 5;

    constexpr PortPin(std::uint8_t arg_port, std::uint8_t arg_pin) :
        port(arg_port & 0b111),
        pin(arg_pin & 0b11111)
    { }

    void set(bool state) const
    {
        LPC_GPIO[port].DATA[1 << pin] = static_cast<unsigned long>(state) << pin;
    }

    bool get() const
    {
        return LPC_GPIO[port].DATA[1 << pin] != 0;
    }

    void makeOutputAndSet(bool initial_state) const
    {
        LPC_GPIO[port].DIR |= 1U << pin;
        set(initial_state);
    }

    void makeInput() const
    {
        LPC_GPIO[port].DIR &= ~static_cast<std::uint32_t>(1U << pin);
    }

    bool isOutput() const
    {
        return (LPC_GPIO[port].DIR & (1U << pin)) != 0;
    }
};

namespace gpio
{
#if BOARD_OLIMEX_LPC_P11C24
constexpr PortPin CanLed(1, 11);
#else
constexpr PortPin CanLed(2, 6);
#endif
constexpr PortPin StatusLed(2, 0);

constexpr PortPin PumpSwitchLow(3, 0);
constexpr PortPin PumpSwitchHigh(3, 1);

constexpr PortPin MagnetCtrl[4]
{
    PortPin(1, 0),
    PortPin(2, 7),
    PortPin(1, 1),
    PortPin(2, 8)
};

constexpr PortPin DipSwitch[4]
{
    PortPin(1, 5),
    PortPin(1, 6),
    PortPin(1, 7),      // TODO: This one will have to be changed
    PortPin(3, 3)
};
}

struct PinMuxGroup
{
    const unsigned pin      : 8;
    const unsigned modefunc : 24;
};

/**
 * Note that output pins that must be initialized with low output level are configured with PULLDOWN.
 * This is a work around - the GPIO controller does not provide a deterministic way to set initial output value;
 * instead, once the DIR bit goes up, the output state will be determined by the input state at the moment when the
 * DIR bit was changed.
 * Refer to the chapter "12.3.1 GPIO data register" of the user manual for details.
 */
constexpr PinMuxGroup pinmux[] =
{
    // PIO0
    { IOCON_PIO0_11, IOCON_FUNC2 },                                                             // Vin_ADC
    // PIO1
    { IOCON_PIO1_0,  IOCON_FUNC1 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN | IOCON_DIGMODE_EN },     // CTRL_1
    { IOCON_PIO1_1,  IOCON_FUNC1 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN | IOCON_DIGMODE_EN },     // CTRL_3
    { IOCON_PIO1_5,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLUP },                          // DIP_1
    { IOCON_PIO1_6,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLUP },                          // DIP_2
    { IOCON_PIO1_7,  IOCON_FUNC1 | IOCON_HYS_EN | IOCON_MODE_PULLUP },                          // UART_TXD
#if BOARD_OLIMEX_LPC_P11C24
    { IOCON_PIO1_10, IOCON_FUNC0 | IOCON_HYS_EN | IOCON_DIGMODE_EN },                           // LED2
    { IOCON_PIO1_11, IOCON_FUNC0 | IOCON_HYS_EN | IOCON_DIGMODE_EN },                           // CAN LED
#endif
    // PIO2
    { IOCON_PIO2_0,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN},                         // Status LED
#if !BOARD_OLIMEX_LPC_P11C24
    { IOCON_PIO2_6,  IOCON_FUNC0 },                                                             // CAN LED
#endif
    { IOCON_PIO2_7,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN },                        // CTRL_2
    { IOCON_PIO2_8,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN },                        // CTRL_4
    { IOCON_PIO2_10, IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN },                        // PWM
    // PIO3
    { IOCON_PIO3_0,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN },                        // SW_L
    { IOCON_PIO3_1,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN },                        // SW_H
    { IOCON_PIO3_3,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLUP }                           // DIP_4
    // TODO: DIP_3 is unassigned
};


void sysctlPowerDown(unsigned long powerdownmask)
{
    unsigned long pdrun = LPC_SYSCTL->PDRUNCFG & PDRUNCFGMASKTMP;
    pdrun |= (powerdownmask & PDRUNCFGMASKTMP);
    LPC_SYSCTL->PDRUNCFG = pdrun | PDRUNCFGUSEMASK;
}

void sysctlPowerUp(unsigned long powerupmask)
{
    unsigned long pdrun = LPC_SYSCTL->PDRUNCFG & PDRUNCFGMASKTMP;
    pdrun &= ~(powerupmask & PDRUNCFGMASKTMP);
    LPC_SYSCTL->PDRUNCFG = pdrun | PDRUNCFGUSEMASK;
}

void initWatchdog()
{
    Chip_WWDT_Init(LPC_WWDT);                                   // Initialize watchdog
    sysctlPowerUp(SYSCTL_POWERDOWN_WDTOSC_PD);                  // Enable watchdog oscillator
    Chip_Clock_SetWDTOSC(WDTLFO_OSC_0_60, 4);                   // WDT osc rate 0.6 MHz / 4 = 150 kHz
    Chip_Clock_SetWDTClockSource(SYSCTL_WDTCLKSRC_WDTOSC, 1);   // Clocking watchdog from its osc, div rate 1
    Chip_WWDT_SetTimeOut(LPC_WWDT, 37500);                      // 1 sec (hardcoded to reduce code size)
    Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);          // Mode: reset on timeout
    Chip_WWDT_Start(LPC_WWDT);                                  // Go
}

void initClock()
{
    sysctlPowerUp(SYSCTL_POWERDOWN_SYSOSC_PD);   // Enable system oscillator
    for (volatile int i = 0; i < 1000; i++) { }

    Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_MAINOSC);
    sysctlPowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);

    /*
     * Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 4 = 48MHz
     * MSEL = 3 (this is pre-decremented), PSEL = 1 (for P = 2)
     * FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 4 = 48MHz
     * FCCO = FCLKOUT * 2 * P = 48MHz * 2 * 2 = 192MHz (within FCCO range)
     */
    Chip_Clock_SetupSystemPLL(3, 1);
    sysctlPowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);
    while (!Chip_Clock_IsSystemPLLLocked()) { }

    Chip_Clock_SetSysClockDiv(1);

    Chip_FMC_SetFLASHAccess(FLASHTIM_50MHZ_CPU);

    Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);

    SystemCoreClock = Chip_Clock_GetSystemClockRate();

    while (SystemCoreClock != TargetSystemCoreClock) { }  // Loop forever if the clock failed to initialize properly
}

void initGpio()
{
    LPC_SYSCTL->SYSAHBCLKCTRL |= 1 << SYSCTL_CLOCK_IOCON;
    LPC_SYSCTL->SYSAHBCLKCTRL |= 1 << SYSCTL_CLOCK_GPIO;

    for (auto x : pinmux)
    {
        LPC_IOCON->REG[x.pin] = x.modefunc;
    }

    // Waiting for the pull-down resistors to drive outputs to the low level - see the work-around above
    for (volatile int i = 0; i < 10000; i++) { }

    gpio::CanLed.makeOutputAndSet(false);

    gpio::PumpSwitchHigh.makeOutputAndSet(false);
    gpio::PumpSwitchLow.makeOutputAndSet(false);

    for (auto x : gpio::MagnetCtrl)
    {
        x.makeOutputAndSet(false);
    }
}

void initUart()
{
    Chip_UART_Init(LPC_USART);
    Chip_UART_SetBaud(LPC_USART, 115200);
    Chip_UART_TXEnable(LPC_USART);
}

void init()
{
    Chip_SYSCTL_SetBODLevels(SYSCTL_BODRSTLVL_2_06V, SYSCTL_BODINTVAL_RESERVED1);
    Chip_SYSCTL_EnableBODReset();

    initWatchdog();
    initClock();
    initGpio();
    // TODO: ADC initalization
    initUart();

    resetWatchdog();
}

} // namespace

#if __GNUC__
__attribute__((optimize(0)))     // Optimization must be disabled lest it hardfaults in the IAP call
#endif
void readUniqueID(std::uint8_t out_uid[UniqueIDSize])
{
    unsigned aligned_array[4] = {};  // out_uid may be unaligned, so we need to use temp array
    unsigned iap_command = 58;
    reinterpret_cast<void(*)(void*, void*)>(0x1FFF1FF1)(&iap_command, aligned_array);
    std::memcpy(out_uid, aligned_array, 16);
}

void resetWatchdog()
{
    Chip_WWDT_Feed(LPC_WWDT);
}

void setStatusLed(bool state)
{
    if (state)
    {
        gpio::StatusLed.makeOutputAndSet(state);
    }
    else
    {
        gpio::StatusLed.makeInput();
    }
}

void setCanLed(bool state)
{
#if BOARD_OLIMEX_LPC_P11C24
    state = !state;
#endif
    gpio::CanLed.set(state);
}

void setChargePumpSwitch(bool state)
{
    gpio::PumpSwitchLow.set(state);
    gpio::PumpSwitchHigh.set(state);
}

void setMagnetBridgeState(const MagnetBridgeState state)
{
    /// This enum reflects the net names on the schematics, for clarity
    enum { CTRL_1, CTRL_2, CTRL_3, CTRL_4 };

    if (state == MagnetBridgeState::RightHighLeftLow)
    {
        // Off
        gpio::MagnetCtrl[CTRL_1].set(false);
        gpio::MagnetCtrl[CTRL_4].set(false);
        // On
        gpio::MagnetCtrl[CTRL_2].set(true);
        gpio::MagnetCtrl[CTRL_3].set(true);
    }
    else if (state == MagnetBridgeState::RightLowLeftHigh)
    {
        // Off
        gpio::MagnetCtrl[CTRL_2].set(false);
        gpio::MagnetCtrl[CTRL_3].set(false);
        // On
        gpio::MagnetCtrl[CTRL_1].set(true);
        gpio::MagnetCtrl[CTRL_4].set(true);
    }
    else
    {
        for (auto x : gpio::MagnetCtrl)
        {
            x.set(false);
        }
    }
}

std::uint8_t readDipSwitch()
{
    std::uint8_t out = 0;
    std::uint8_t shift = 0;
    for (auto x : gpio::DipSwitch)
    {
        out = static_cast<std::uint8_t>(out | (static_cast<unsigned>(!x.get()) << (shift++))); // Inversion
    }
    return out;
}

bool hadButtonPressEvent()
{
    constexpr std::uint8_t PressCounterThreshold = 10;
    static std::uint8_t press_counter = 0;

    if (gpio::StatusLed.get() && !gpio::StatusLed.isOutput())
    {
        press_counter = std::min(static_cast<std::uint8_t>(press_counter + 1), PressCounterThreshold);
        return false;
    }
    else
    {
        const bool had_press = press_counter >= PressCounterThreshold;
        press_counter = 0;
        return had_press;
    }
}

void delayUSec(std::uint8_t usec)
{
    /*
     * This function assumes that systick reload value is greater than or equal 0xFFFF.
     * Otherwise delays will be inconsistent.
     */
    if (usec > 0)
    {
        usec--; // Poor man's calibration

        const std::uint16_t delay_ticks = static_cast<std::uint16_t>(usec * (TargetSystemCoreClock / 1000000U));
        const std::uint16_t started_at = SysTick->VAL & 0xFFFFU;

        // Systick counts downwards
        while ((started_at - (SysTick->VAL & 0xFFFFU)) < delay_ticks)
        {
            ; // Doing nothing, it's a busyloop
        }
    }
}

void syslog(const char* msg)
{
    Chip_UART_SendBlocking(LPC_USART, msg, static_cast<int>(std::strlen(msg)));
}

} // namespace board

extern "C"
{

void SystemInit();

void SystemInit()
{
    board::init();
}

}
