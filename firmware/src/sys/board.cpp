/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 * Author: Andreas Jochum <Andreas@Nicadrone.com>
 *
 * Board initialization for Olimex LPC11C24 and EPM hard
 */

#include "board.hpp"
#include <chip.h>
#include <cstdlib>
#include <cstring>
#include <numeric>
#include <cstdint>
#include <algorithm>
#include <array>

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

constexpr unsigned AdcReferenceMillivolts = 3300;
constexpr unsigned AdcResolutionBits = 10;

#if BOARD_OLIMEX_LPC_P11C24
constexpr unsigned CanLedPortNum = 1;
constexpr unsigned CanLedPinMask = 1U << 11;
#else
constexpr unsigned CanLedPortNum = 2;
constexpr unsigned CanLedPinMask = 1U << 6;
#endif

constexpr unsigned StatusLedPortNum = 2;
constexpr unsigned StatusLedPinMask = 1U << 0;

constexpr unsigned PumpSwitchPortNum = 1;
constexpr unsigned PumpSwitchPinMask = (1U << 0) | (1U << 1) | (1U << 2) | (1U << 4);

constexpr unsigned MagnetCtrlPortNum = 2;
constexpr unsigned MagnetCtrlPinMask23 = (1U << 1) | (1U << 7);
constexpr unsigned MagnetCtrlPinMask14 = (1U << 2) | (1U << 8);

constexpr unsigned DipSwitchPortNum = 1;
constexpr unsigned DipSwitchPinMask = 0b1111;

constexpr std::uint16_t PwmInputPeriodMinUSec = 500;
constexpr std::uint16_t PwmInputPeriodMaxUSec = 2500;
static std::uint16_t pwm_input_pulse_usec;

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
    { IOCON_PIO0_11, IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_ADMODE_EN |IOCON_OPENDRAIN_EN },    // Vout_ADC

    // PIO1
    { IOCON_PIO1_10, IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_ADMODE_EN |IOCON_OPENDRAIN_EN },    // Vin_ADC

    { IOCON_PIO1_7,  IOCON_FUNC1 | IOCON_HYS_EN | IOCON_MODE_PULLUP },                          // UART_TXD
#if BOARD_OLIMEX_LPC_P11C24
    { IOCON_PIO1_10, IOCON_FUNC0 | IOCON_HYS_EN | IOCON_DIGMODE_EN },                           // LED2
    { IOCON_PIO1_11, IOCON_FUNC0 | IOCON_HYS_EN | IOCON_DIGMODE_EN },                           // CAN LED
#endif

    { IOCON_PIO1_0,  IOCON_FUNC1 | IOCON_MODE_PULLDOWN | IOCON_HYS_EN | IOCON_DIGMODE_EN },     // PUMP SW0
    { IOCON_PIO1_1,  IOCON_FUNC1 | IOCON_MODE_PULLDOWN | IOCON_HYS_EN | IOCON_DIGMODE_EN },     // PUMP SW1
    { IOCON_PIO1_2,  IOCON_FUNC1 | IOCON_MODE_PULLDOWN | IOCON_HYS_EN | IOCON_DIGMODE_EN },     // PUMP SW2
    { IOCON_PIO1_4,  IOCON_FUNC0 | IOCON_MODE_PULLDOWN | IOCON_HYS_EN | IOCON_DIGMODE_EN },     // PUMP SW4

    { IOCON_PIO1_8,  IOCON_FUNC1 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN },                        // PWM

    // PIO2
    { IOCON_PIO2_0,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN },                        // Status LED
#if !BOARD_OLIMEX_LPC_P11C24
    { IOCON_PIO2_6,  IOCON_FUNC0 },                                                             // CAN LED
#endif

    { IOCON_PIO2_1,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN | IOCON_DIGMODE_EN },     // CTRL3
    { IOCON_PIO2_7,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN | IOCON_DIGMODE_EN },     // CTRL2

    { IOCON_PIO2_2,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN | IOCON_DIGMODE_EN },     // CTRL1
    { IOCON_PIO2_8,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN | IOCON_DIGMODE_EN },     // CTRL4

    // PIO3
    { IOCON_PIO3_0,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLUP },                          // DIP_1
    { IOCON_PIO3_1,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLUP },                          // DIP_2
    { IOCON_PIO3_2,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLUP },                          // DIP_3
    { IOCON_PIO3_3,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLUP },                          // DIP_4
};

namespace gpio
{

inline unsigned get(const unsigned port, const unsigned mask)
{
    return LPC_GPIO[port].DATA[mask];
}

inline void set(const unsigned port, const unsigned mask, const unsigned value)
{
    LPC_GPIO[port].DATA[mask] = value;
}

inline void makeOutputsAndSet(const unsigned port, const unsigned mask, const unsigned value)
{
    LPC_GPIO[port].DIR |= mask;
    set(port, mask, value);
}

inline void makeInputs(const unsigned port, const unsigned mask)
{
    LPC_GPIO[port].DIR &= ~mask;
}

inline unsigned markOutputs(const unsigned port, const unsigned mask)
{
    return LPC_GPIO[port].DIR & mask;
}

}

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

    gpio::makeOutputsAndSet(CanLedPortNum, CanLedPinMask, 0);

    gpio::makeOutputsAndSet(PumpSwitchPortNum, PumpSwitchPinMask, 0);

    gpio::makeOutputsAndSet(MagnetCtrlPortNum, MagnetCtrlPinMask23 | MagnetCtrlPinMask14, 0);
}

void initAdc()
{
    constexpr unsigned SamplesPerSecond = 1000;

    auto clock = ::ADC_CLOCK_SETUP_T();
    Chip_ADC_Init(LPC_ADC, &clock);

    Chip_ADC_SetSampleRate(LPC_ADC, &clock, SamplesPerSecond);
    Chip_ADC_EnableChannel(LPC_ADC, ADC_CH6, ENABLE);       // Vin
    Chip_ADC_EnableChannel(LPC_ADC, ADC_CH0, ENABLE);       // Vout
    Chip_ADC_SetBurstCmd(LPC_ADC, ENABLE);
}

void initUart()
{
    Chip_UART_Init(LPC_USART);
    Chip_UART_SetBaud(LPC_USART, 115200);
    Chip_UART_TXEnable(LPC_USART);
}

void initPwmCapture()
{
    Chip_TIMER_Init(LPC_TIMER16_1);
    // 1 usec per tick
    Chip_TIMER_PrescaleSet(LPC_TIMER16_1, (TargetSystemCoreClock / 1000000U) - 1U);
    // Hardware PWM capture (edges will be initialized in the IRQ handler)
    Chip_TIMER_CaptureEnableInt(LPC_TIMER16_1, 0);
    // PWM timeout detection
    Chip_TIMER_MatchEnableInt(LPC_TIMER16_1, 0);
    // Start
    Chip_TIMER_Enable(LPC_TIMER16_1);
    // Enabling the IRQ
    NVIC_EnableIRQ(TIMER_16_1_IRQn);
    NVIC_SetPriority(TIMER_16_1_IRQn, 1);
}

void init()
{
    Chip_SYSCTL_SetBODLevels(SYSCTL_BODRSTLVL_2_06V, SYSCTL_BODINTVAL_RESERVED1);
    Chip_SYSCTL_EnableBODReset();

    initWatchdog();
    initClock();
    initGpio();
    initAdc();
    initUart();
    initPwmCapture();

    resetWatchdog();
}

} // namespace

void die()
{
    while (true) { }
}

#if __GNUC__
__attribute__((optimize(0)))     // Optimization must be disabled lest it hardfaults in the IAP call
#endif
void readUniqueID(UniqueID& out_uid)
{
    unsigned aligned_array[4] = {};  // out_uid may be unaligned, so we need to use temp array
    unsigned iap_command = 58;
    reinterpret_cast<void(*)(void*, void*)>(0x1FFF1FF1)(&iap_command, aligned_array);
    std::memcpy(out_uid.data(), aligned_array, 16);
}

void resetWatchdog()
{
    Chip_WWDT_Feed(LPC_WWDT);
}

void setStatusLed(bool state)
{
    if (state)
    {
        gpio::makeOutputsAndSet(StatusLedPortNum, StatusLedPinMask, state ? StatusLedPinMask : 0);
    }
    else
    {
        gpio::makeInputs(StatusLedPortNum, StatusLedPinMask);
    }
}

void setCanLed(bool state)
{
#if BOARD_OLIMEX_LPC_P11C24
    state = !state;
#endif
    gpio::set(CanLedPortNum, CanLedPinMask, state ? CanLedPinMask : 0);
}

void setPumpSwitch(bool state)
{
    gpio::set(PumpSwitchPortNum, PumpSwitchPinMask, state ? PumpSwitchPinMask : 0);
}

void setMagnetPos()
{
    gpio::set(MagnetCtrlPortNum, MagnetCtrlPinMask23, MagnetCtrlPinMask23);
    board::delayUSec(20);
    gpio::set(MagnetCtrlPortNum, MagnetCtrlPinMask23, 0);
}

void setMagnetNeg()
{
    gpio::set(MagnetCtrlPortNum, MagnetCtrlPinMask14, MagnetCtrlPinMask14);
    board::delayUSec(20);
    gpio::set(MagnetCtrlPortNum, MagnetCtrlPinMask14, 0);
}

std::uint8_t readDipSwitch()
{
    return gpio::get(DipSwitchPortNum, DipSwitchPinMask) & DipSwitchPinMask;
}

bool hadButtonPressEvent()
{
    constexpr std::uint8_t PressCounterThreshold = 10;
    static std::uint8_t press_counter = 0;

    if ((gpio::get(StatusLedPortNum, StatusLedPinMask) != 0) &&
        (gpio::markOutputs(StatusLedPortNum, StatusLedPinMask) == 0))
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

unsigned getSupplyVoltageInMillivolts()
{
    static std::uint16_t old_value = 0;

    std::uint16_t new_value = 0;
    (void)Chip_ADC_ReadValue(LPC_ADC, ADC_CH6, &new_value);

    // Division and multiplication by 2 are reduced
    unsigned x = static_cast<unsigned>((static_cast<unsigned>(new_value + old_value) * AdcReferenceMillivolts) >>
                                       AdcResolutionBits);

    old_value = new_value;

    x *= 5;                             //should be x*=5.5
    x += 650;                           //Poor man's optimizatin to not unintegerify the math, it's within 100mV -
    if (x < 4500)                       //Under 4500mV Vref drops, mesurements useless
    {
        x = 0;
    }
    return x;                           // Voltage divider on board, shoud not go here
}

unsigned getOutVoltageInVolts()
{
    static std::uint16_t old_value = 0;

    std::uint16_t new_value = 0;
    (void)Chip_ADC_ReadValue(LPC_ADC, ADC_CH0, &new_value);

    // Division and multiplication by 2 are reduced
    const unsigned x =
        static_cast<unsigned>((static_cast<unsigned>(new_value + old_value) * AdcReferenceMillivolts) >>
                              AdcResolutionBits);

    old_value = new_value;
    return x / 10;
}

unsigned getPwmInputPulseLengthInMicroseconds()
{
    return pwm_input_pulse_usec;
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

void delayMSec(unsigned msec)
{
    while (msec --> 0)
    {
        for (std::uint8_t i = 0; i < 4; i++)
        {
            board::delayUSec(246); // 246 * 4 = 984 usec, 16 usec calibration
        }
    }
}

void syslog(const char* msg)
{
    Chip_UART_SendBlocking(LPC_USART, msg, static_cast<int>(std::strlen(msg)));
}

void syslog(const char* prefix, long long integer_value, const char* suffix)
{
    static const auto lltoa = [](long long n, char buf[24])
    {
        const short sign = (n < 0) ? -1 : 1;
        if (sign < 0)
        {
            n = -n;
        }
        unsigned pos = 0;
        do
        {
            buf[pos++] = char(n % 10 + '0');
        }
        while ((n /= 10) > 0);
        if (sign < 0)
        {
            buf[pos++] = '-';
        }
        buf[pos] = '\0';
        for (unsigned i = 0, j = pos - 1U; i < j; i++, j--)
        {
            std::swap(buf[i], buf[j]);
        }
    };

    syslog(prefix);

    char buf[24];
    lltoa(integer_value, buf);
    syslog(&buf[0]);

    syslog(suffix);
}

} // namespace board

extern "C"
{

void TIMER16_1_IRQHandler();
void TIMER16_1_IRQHandler()
{
    using namespace board;

    static std::int32_t rising_edge_timestamp = -1;

    static constexpr unsigned CCR_RISING_EDGE  = 0b101;
    static constexpr unsigned CCR_FALLING_EDGE = 0b110;

    /*
     * PWM edge capture interrupt
     */
    if (Chip_TIMER_CapturePending(LPC_TIMER16_1, 0))
    {
        // Clearing the flag
        Chip_TIMER_ClearCapture(LPC_TIMER16_1, 0);

        // Swapping polarity and processing the event
        if (rising_edge_timestamp < 0)
        {
            LPC_TIMER16_1->CCR = CCR_FALLING_EDGE;
            rising_edge_timestamp = static_cast<std::int32_t>(LPC_TIMER16_1->CR[0]);
        }
        else
        {
            LPC_TIMER16_1->CCR = CCR_RISING_EDGE;

            const std::uint16_t duration =
                static_cast<std::uint16_t>(LPC_TIMER16_1->CR[0] - static_cast<std::uint16_t>(rising_edge_timestamp));

            if ((duration >= PwmInputPeriodMinUSec) && (duration <= PwmInputPeriodMaxUSec))
            {
                if (pwm_input_pulse_usec > 0)
                {
                    pwm_input_pulse_usec =
                        static_cast<std::uint16_t>((duration + pwm_input_pulse_usec) / 2);
                }
                else
                {
                    pwm_input_pulse_usec = duration;
                }
            }
            else
            {
                pwm_input_pulse_usec = 0;
            }

            rising_edge_timestamp = -1;
        }

        // Resetting the timeout interrupt
        LPC_TIMER16_1->MR[0] = LPC_TIMER16_1->CR[0] + 0xFFFFU;   // 65.535 ms timeout (~15 Hz)
        Chip_TIMER_ClearMatch(LPC_TIMER16_1, 0);
    }

    /*
     * PWM edge timeout interrupt (will be invoked periodically as long as the PWM signal is not present)
     */
    if (Chip_TIMER_MatchPending(LPC_TIMER16_1, 0))
    {
        Chip_TIMER_ClearMatch(LPC_TIMER16_1, 0);

        LPC_TIMER16_1->CCR = CCR_RISING_EDGE;

        rising_edge_timestamp = -1;
        pwm_input_pulse_usec = 0;
    }
}

void Chip_SYSCTL_PowerUp(std::uint32_t powerupmask)
{
    board::sysctlPowerUp(powerupmask);
}

void SystemInit();

void SystemInit()
{
    board::init();
}

}
