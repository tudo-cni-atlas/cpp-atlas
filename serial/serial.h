//
//  serial.h
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __SERIAL_INTERFACE_H_
#define __SERIAL_INTERFACE_H_

#include <string>

class Serial
{
public:
    Serial( std::string port, unsigned int baud );
    ~Serial();

    void send( const uint8_t *buffer, size_t length );
    int receive( uint8_t *c );

private:
    int m_portfd;
};

#endif