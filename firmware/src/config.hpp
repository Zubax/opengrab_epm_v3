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

#if defined(PRODROPPER) && PRODROPPER

# define TIME_OUT_MS            1000
# define RATE_LIMIT_MS          3000
# define VIN_MIN_MV             6700
# define VIN_MAX_MV             15000
# define PR_INDUCTANCE_PH       10000000
# define BLAH                   200

#else                           //OpenGrab EPM V3

# define TIME_OUT_MS            1000
# define RATE_LIMIT_MS          2500
# define VIN_MIN_MV             4300
# define VIN_MAX_MV             6700
# define PR_INDUCTANCE_PH       11000000
# define BLAH                   450

#endif
