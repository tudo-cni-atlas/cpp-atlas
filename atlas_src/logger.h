//
//  logger.h
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__logger__
#define __atlas__logger__

#include <stdio.h>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <chrono>

#include "atlas_types.h"

class Logger
{
private:
    std::string m_dir;
    std::ofstream m_positionLog;
    std::ofstream m_sampleLog;

public:
    Logger ();
    ~Logger();

    void logSample(const sample_t &s);
    void logPosition(const position_t &p);
};

#endif /* defined(__atlas__logger__) */
