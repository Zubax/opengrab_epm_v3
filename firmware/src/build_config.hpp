/*
 * OpenGrab EPM - Electropermanent Magnet
 * Copyright (C) 2015-2016  Zubax Robotics <info@zubax.com>
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
 * Author: Andreas Jochum <andreas@nicadrone.com>
 *         Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

namespace build_config
{

#if defined(PRODROPPER) && PRODROPPER

static constexpr unsigned ChargerTimeout_ms             = 1000;
static constexpr unsigned CommandRateLimit_ms           = 3000;
static constexpr unsigned VinMin_mV                     = 10000;
static constexpr unsigned VinMax_mV                     = 15000;
static constexpr unsigned PRInductance_pH               = 11000000;
static constexpr unsigned ReducedCurrentVoltage_mV      = 8000;
static constexpr unsigned TurnOffSecondCycleVoltage     = 200;
static constexpr int      cycles_to_skip                = 3;

#else                           //OpenGrab EPM V3

static constexpr unsigned ChargerTimeout_ms             = 1000;
static constexpr unsigned CommandRateLimit_ms           = 1500;
static constexpr unsigned VinMin_mV                     = 4300;
static constexpr unsigned VinMax_mV                     = 6700;
static constexpr unsigned PRInductance_pH               = 10500000;
static constexpr unsigned ReducedCurrentVoltage_mV      = 4800;
static constexpr int      cycles_to_skip                = 5;

#endif

}
