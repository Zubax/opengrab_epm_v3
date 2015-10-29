/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 * Author: Andreas Jochum <Andreas@Nicadrone.com>
 */

#pragma once

namespace charger
{

class Charger
{
    void cycle1000_7000();                      ///1000ns on, 7000ns off, total run time ~1ms

    void cycle1500_5000();              
    
    void cycle1500_3000();

    void cycle1500_1500();
      
    void cycle2000_500();

    void cycle(unsigned on, unsigned off);
public:
    bool run(unsigned U_);

};

}
