/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

/*
 * TODO
 * VDD CAN is Vin, generate critical error when Vin >6.5V, abs max 7V, CAN transiver limit
 * BOR detect
 */

#include "magnet.hpp"
#include "charger.hpp"
#include <sys/board.hpp>

namespace magnet
{
namespace
{

static charger::Charger chrg;
static bool magnet_state = false;

/// TODO: This will be removed
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
    // Turning manget on, charge capacitor to 450V and dishcarge, twice
    for (unsigned i = 0; i != 2; i++)
    {
        chrg.set(450);

        if (chrg.run())
        {
            board::setMagnetPos();
            board::delayMSec(5);      //Thyristors switch of delay, can probably be reduced to 1ms
        }
        else
            board::syslog("Charger error");             // TODO report bad health
    }

    magnet_state = true;
    //limit duty cycle
    board::delayMSec(250);
    board::delayMSec(250);
}

void magnetOff()
{
    static constexpr unsigned ThyristorTurnOffDelayUSec = 5;

    static constexpr unsigned CycleArray[][2] =
    {
        {450,0},
        {450,0},
        {300,1},
        {180,0},
        {162,1},
        {146,0},
        {131,1},
        {118,0},
        {106,1},
        {96,0},
        {86,1},
        {77,0},
        {70,1},
        {63,0},
        {56,1},
        {51,0},
        {46,1},
        {41,0},
        {37,1},
        {33,0},
        {30,1},
        {27,0},
        {24,1},
        {22,0},
        {20,1},
        {18,0},
        {16,1},
        {14,0},
        {13,1},
        {12,0},
        {10,1}
    };

    for (unsigned i = 0; i != 30; i++)
    {
        if (i == 0 && magnet_state == false)     // Manget was not on, we can skip the first 3 cycles
            i = 2;

        chrg.set(CycleArray[i][0]);

        if (!chrg.run())                         // If error,
        {
            board::syslog("Charger error");      // TODO report bad health
        }                                        // ignore and keep going
        if (CycleArray[i][1])                    // 1 = pos
        {
            board::setMagnetPos();
        }
        if (!CycleArray[i][1])                   // 0 = neg
        {
            board::setMagnetNeg();
        }

        board::delayMSec(ThyristorTurnOffDelayUSec);

        magnet_state = false;
    }

    //limit duty cycle

    board::delayMSec(250);
    board::delayMSec(250);
    board::delayMSec(250);
}

}

void init()
{

}

void poll()
{
    const auto supply_voltage_mV = board::getSupplyVoltageInMillivolts();

    const auto pwm_input = board::getPwmInputPulseLengthInMicroseconds();

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

            board::syslog("Calling mangetOff \r\n");
            magnetOff();
        }

        if (magnet_state == false)
        {

            board::syslog("Calling magnetOn \r\n");
            magnetOn();
        }

        /*debug*/
        board::syslog("Magnet state:   ", int(magnet_state), "\r\n");
        board::syslog("Supply voltage: ", supply_voltage_mV, "mV\r\n");
        board::syslog("PWM width:      ", pwm_input, " usec\r\n");

    }

    if (pwm_input > 1000 && pwm_input < 1250)
    {
        blinkStatusMs(30, 3);
        board::delayUSec(5);

        //Turn magnet off
        magnetOff();

        //magnet_state = false; //the state is now set in magnetOn/Off()
    }

    if (pwm_input > 1750 && pwm_input < 2000)
    {
        blinkStatusMs(30, 3);
        board::delayUSec(5);

        //Turn magnet on
        magnetOn();

        //magnet_state = true; //the state is now set in magnetOn/Off()
    }
}

Health getHealth()
{
    return Health::Ok; // TODO
}

}


