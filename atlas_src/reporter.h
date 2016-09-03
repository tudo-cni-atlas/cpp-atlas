//
//  reporter.h
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__reporter__
#define __atlas__reporter__

#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <chrono>

#include "atlas_types.h"
#include "serial.h"


class Reporter
{
private:
    Serial m_port;

public:
    Reporter (const std::string port);
    ~Reporter();

    void reportPosition(const position_t &p);
};


#endif /* defined(__atlas__reporter__) */