/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 * Author: Andreas Jochum <Andreas@Nicadrone.com>
 */

#include <cstdio>
#include "util.hpp"
#include <algorithm>
#include <board.hpp>
#include <charger.hpp>
#include <chip.h>
#include <uavcan_lpc11c24/uavcan_lpc11c24.hpp>
#include <uavcan/protocol/logger.hpp>

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

