//
//  config.h
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__config__
#define __atlas__config__

#include <stdio.h>
#include <string>

#include "atlas_types.h"


class Config
{
private:

public:
    Config (const std::string file);
    void list();
    ~Config ();

    configAnchors_t    m_anchors;
    configSync_t       m_sync;
    configCal_t        m_cal;
    configParser_t     m_parser;
    configEKF_t        m_ekf;
    configLRF_t        m_lrf;
    configOutput_t     m_output;
    configVerbosity_t  m_verbosity;
};

#endif /* defined(__atlas__config__) */