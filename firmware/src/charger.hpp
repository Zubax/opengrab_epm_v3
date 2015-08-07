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
    unsigned ton=0;                     //bussy loop units
    
    unsigned toff=0;        
    
    
public:

    void run();
    
    unsigned U=0;                       //[V] 0-500
    
    bool done=true;                     //charger done, true false
};

}
