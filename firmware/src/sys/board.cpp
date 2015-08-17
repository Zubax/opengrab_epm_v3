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
#include <uavcan_lpc11c24/clock.hpp>

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

constexpr unsigned PwmPortNum = 2;
constexpr unsigned PwmInputPinMask = 1U << 10;

template <unsigned NumPins>
struct PinGroup
{
    const std::uint8_t port;
    const std::array<std::uint8_t, NumPins> pins;

    constexpr PinGroup(std::uint8_t arg_port, const std::array<std::uint8_t, NumPins>& arg_pins) :
        port(arg_port & 0b111),
        pins(arg_pins)
    { }

    std::uint32_t getMask() const
    {
        std::uint32_t mask = 0;
        for (auto x : pins) { mask |= 1U << x; }
        return mask;
    }

    void set(const unsigned pin_states) const
    {
        std::uint32_t out_mask = 0;
        for (unsigned i = 0; i < NumPins; i++)
        {
            out_mask |= (static_cast<unsigned>((pin_states & (1U << i)) != 0) << pins[i]);
        }
        LPC_GPIO[port].DATA[getMask()] = out_mask;
    }

    unsigned get() const
    {
        unsigned out = 0;
        const auto sample = LPC_GPIO[port].DATA[getMask()];
        for (unsigned i = 0; i < NumPins; i++)
        {
            if ((sample & (1U << pins[i])) != 0)
            {
                out |= 1U << i;
            }
        }
        return out;
    }

    void makeOutputsAndSet(const unsigned pin_states) const
    {
        LPC_GPIO[port].DIR |= getMask();
        set(pin_states);
    }

    void makeInputs() const
    {
        LPC_GPIO[port].DIR &= ~getMask();
    }

    bool areAllOutputs() const
    {
        const auto mask = getMask();
        return (LPC_GPIO[port].DIR & mask) == mask;
    }
};

struct Pin : private PinGroup<1>
{
    constexpr Pin(std::uint8_t arg_port, std::uint8_t arg_pin) : PinGroup<1>(arg_port, {arg_pin}) { }

    void set(bool state) const { PinGroup<1>::set(state); }
    bool get()           const { return PinGroup<1>::get() != 0; }

    void makeOutputAndSet(bool state) const { PinGroup<1>::makeOutputsAndSet(state); }
    void makeInput()                  const { PinGroup<1>::makeInputs(); }
    bool isOutput()                   const { return PinGroup<1>::areAllOutputs(); }
};

namespace gpio
{
#if BOARD_OLIMEX_LPC_P11C24
constexpr Pin CanLed(1, 11);
#else
constexpr Pin CanLed(2, 6);
#endif
constexpr Pin StatusLed(2, 0);
constexpr PinGroup<4> PumpSwitch(1, {0, 1, 2, 4});
constexpr PinGroup<2> MagnetCtrl23(2, {1, 7});
constexpr PinGroup<2> MagnetCtrl14(2, {2, 8});

// TODO: DIP switch pins are not yet defined
}

constexpr std::uint32_t PwmInputPeriodMinUSec = 500;
constexpr std::uint32_t PwmInputPeriodMaxUSec = 2500;
constexpr std::uint32_t PwmInputTimeoutUSec   = 100000;
static std::uint32_t pwm_input_period_usec;
static uavcan::MonotonicTime last_pwm_input_update_ts;

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
    // PIO1 ADC
    { IOCON_PIO1_10, IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_ADMODE_EN |IOCON_OPENDRAIN_EN },    // Vin_ADC
    
    // PIO0 ADC 
    { IOCON_PIO0_11, IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_ADMODE_EN |IOCON_OPENDRAIN_EN },    // Vout_ADC

    { IOCON_PIO1_7,  IOCON_FUNC1 | IOCON_HYS_EN | IOCON_MODE_PULLUP },                          // UART_TXD
#if BOARD_OLIMEX_LPC_P11C24
    { IOCON_PIO1_10, IOCON_FUNC0 | IOCON_HYS_EN | IOCON_DIGMODE_EN },                           // LED2
    { IOCON_PIO1_11, IOCON_FUNC0 | IOCON_HYS_EN | IOCON_DIGMODE_EN },                           // CAN LED
#endif
    // PIO2
    { IOCON_PIO2_0,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN },                        // Status LED
#if !BOARD_OLIMEX_LPC_P11C24
    { IOCON_PIO2_6,  IOCON_FUNC0 },                                                             // CAN LED
#endif
    
    { IOCON_PIO2_10, IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN },                        // PWM

    // PIO1 PUMP
    { IOCON_PIO1_0,  IOCON_FUNC1 | IOCON_MODE_PULLDOWN | IOCON_HYS_EN | IOCON_DIGMODE_EN },     //PUMP SW0
    { IOCON_PIO1_1,  IOCON_FUNC1 | IOCON_MODE_PULLDOWN | IOCON_HYS_EN | IOCON_DIGMODE_EN },     //PUMP SW1
    { IOCON_PIO1_2,  IOCON_FUNC1 | IOCON_MODE_PULLDOWN | IOCON_HYS_EN | IOCON_DIGMODE_EN },     //PUMP SW2
    { IOCON_PIO1_4,  IOCON_FUNC0 | IOCON_MODE_PULLDOWN | IOCON_HYS_EN | IOCON_DIGMODE_EN },     //PUMP SW4

    // PIO2 CTRL
    { IOCON_PIO2_1,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN | IOCON_DIGMODE_EN },     //CTRL3 
    { IOCON_PIO2_7,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN | IOCON_DIGMODE_EN },     //CTRL2

    { IOCON_PIO2_8,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN | IOCON_DIGMODE_EN },     //CTRL4
    { IOCON_PIO2_2,  IOCON_FUNC0 | IOCON_HYS_EN | IOCON_MODE_PULLDOWN | IOCON_DIGMODE_EN },     //CTRL1

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

    gpio::PumpSwitch.makeOutputsAndSet(0b0000);

    gpio::MagnetCtrl23.makeOutputsAndSet(0);
    gpio::MagnetCtrl14.makeOutputsAndSet(0);

    // PWM input config
    LPC_GPIO[PwmPortNum].IBE |= PwmInputPinMask;
    LPC_GPIO[PwmPortNum].IE  |= PwmInputPinMask;

    NVIC_EnableIRQ(EINT2_IRQn);
    NVIC_SetPriority(EINT2_IRQn, 0);    // Highest
}

void initAdc()
{
    constexpr unsigned SamplesPerSecond = 1000;

    auto clock = ::ADC_CLOCK_SETUP_T();
    Chip_ADC_Init(LPC_ADC, &clock);

    Chip_ADC_SetSampleRate(LPC_ADC, &clock, SamplesPerSecond);
    Chip_ADC_EnableChannel(LPC_ADC, ADC_CH6, ENABLE);       //Vin
    Chip_ADC_EnableChannel(LPC_ADC, ADC_CH0, ENABLE);       //Vout
    Chip_ADC_SetBurstCmd(LPC_ADC, ENABLE);
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
    initAdc();
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

void setMagnetPos()
{
    gpio::MagnetCtrl23.set(0b11);
    board::delayUSec(20);
    gpio::MagnetCtrl23.set(0b00);
}

void setMagnetNeg()
{
    gpio::MagnetCtrl14.set(0b11);
    board::delayUSec(20);
    gpio::MagnetCtrl14.set(0b00);
}

std::uint8_t readDipSwitch()
{
    return 0;   // TODO implement
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

unsigned getSupplyVoltageInMillivolts()
{
    static std::uint16_t old_value = 0;

    std::uint16_t new_value = 0;
    (void)Chip_ADC_ReadValue(LPC_ADC, ADC_CH6, &new_value);

    // Division and multiplication by 2 are reduced
    unsigned x = static_cast<unsigned>((static_cast<unsigned>(new_value + old_value) * AdcReferenceMillivolts) >>
                                             AdcResolutionBits);

    old_value = new_value;

    x*=5;
    x+=700;                             //poor mans calibration, it's within 100mV 
    if(x < 4500)                        //under 4500mV Vref drops, mesurements useless 
    {   
        x = 0;
    }
    return x;                     //Voltage divider on board, shoud not go here
}

unsigned getOutVoltageInVolts()
{
    static std::uint16_t old_value = 0;

    std::uint16_t new_value = 0;
    (void)Chip_ADC_ReadValue(LPC_ADC, ADC_CH0, &new_value);

    // Division and multiplication by 2 are reduced
    const unsigned x = static_cast<unsigned>((static_cast<unsigned>(new_value + old_value) * AdcReferenceMillivolts) >>
                                             AdcResolutionBits);

    old_value = new_value;
    return x/10;                 //    
}

#if __GNUC__
__attribute__((optimize(1)))    // Fails
#endif
unsigned getPwmInputPeriodInMicroseconds()
{
    if ((uavcan_lpc11c24::clock::getMonotonic() - last_pwm_input_update_ts).toUSec() > PwmInputTimeoutUSec)
    {
        pwm_input_period_usec = 0;
    }
    return pwm_input_period_usec;
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

void PIOINT2_IRQHandler();
void PIOINT2_IRQHandler()
{
    using namespace board;

    if ((LPC_GPIO[PwmPortNum].MIS & PwmInputPinMask) != 0)
    {
        LPC_GPIO[PwmPortNum].IC = PwmInputPinMask;

        static uavcan::MonotonicTime prev_ts;
        const auto ts = uavcan_lpc11c24::clock::getMonotonic();         // TODO: Is it safe to call it from here?
        const auto diff_usec = static_cast<std::uint64_t>((ts - prev_ts).toUSec());
        prev_ts = ts;

        const bool input_state = (LPC_GPIO[PwmPortNum].DATA[PwmInputPinMask] & PwmInputPinMask) != 0;

        if (!input_state)       // Updating only on trailing edge
        {
            last_pwm_input_update_ts = ts;

            if (diff_usec >= PwmInputPeriodMinUSec && diff_usec <= PwmInputPeriodMaxUSec)
            {
                if (pwm_input_period_usec == 0)
                {
                    pwm_input_period_usec = static_cast<std::uint32_t>(diff_usec);
                }
                else
                {
                    pwm_input_period_usec = static_cast<std::uint32_t>((pwm_input_period_usec + diff_usec) / 2);
                }
            }
            else
            {
                pwm_input_period_usec = 0;      // Invalid value
            }
        }
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
