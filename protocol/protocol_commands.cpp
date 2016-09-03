//
//  protocol_commands.cpp
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include "protocol.h"
#include "protocol_structures.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

using namespace protocol;

const uint16_t msg_id_config = 0x0103;
const uint16_t msg_id_period = 0x0104;

const uint16_t msg_id_toad = 0x0102;
const uint16_t msg_id_toa = 0x0101;

template<typename T> inline uint16_t getMsgId(T t)
{
    return 0;
}
template<> inline uint16_t getMsgId<msgConfig_t>(msgConfig_t t)
{
    return msg_id_config;
}
template<> inline uint16_t getMsgId<msgPeriod_t>(msgPeriod_t t)
{
    return msg_id_period;
}
template<> inline uint16_t getMsgId<msgToaDiag_t>(msgToaDiag_t t)
{
    return msg_id_toad;
}
template<> inline uint16_t getMsgId<msgToa_t>(msgToa_t t)
{
    return msg_id_toa;
}

template<typename T>
bool Protocol::sendMsg(T msg)
{
    uint16_t msg_id = getMsgId(msg);

    packet_t packet;
    populateHeader(&packet, msg_id, sizeof(T));
    memcpy(packet.payload, &msg, sizeof(T));
    packet.checksum = calculateCheckSum(&packet);
    sendMessage(&packet);

    if(!getPacket(&packet, 1000))
    {
        std::cout << "TIMEOUT!!!!" <<  std::endl;
        return false;
    }

    if (calculateCheckSum(&packet) != packet.checksum)
    {
        std::cout << "CHECKSUM ERROR!!!!" << std::endl;
        return false;
    }

    return true;
}

template<typename T>
bool Protocol::sendMsgPoll(T *msg)
{
    uint16_t msg_id = getMsgId(*msg);

    packet_t packet;
    populateHeader(&packet, msg_id, 0);
    packet.checksum = calculateCheckSum(&packet);
    sendMessage(&packet);

    if(!getPacket(&packet, 1000))
    {
        std::cout << "TIMEOUT!!!!" <<  std::endl;
        return false;
    }

    if (calculateCheckSum(&packet) != packet.checksum)
    {
        std::cout << "CHECKSUM ERROR!!!!" << std::endl;
        return false;
    }

    memcpy(msg, packet.payload, sizeof(T));
    return true;
}

template bool Protocol::sendMsg<msgConfig_t>(msgConfig_t msg);
template bool Protocol::sendMsg<msgPeriod_t>(msgPeriod_t msg);
template bool Protocol::sendMsg<msgToaDiag_t>(msgToaDiag_t msg);
template bool Protocol::sendMsg<msgToa_t>(msgToa_t msg);

template bool Protocol::sendMsgPoll<msgConfig_t>(msgConfig_t *msg);
template bool Protocol::sendMsgPoll<msgPeriod_t>(msgPeriod_t *msg);
template bool Protocol::sendMsgPoll<msgToaDiag_t>(msgToaDiag_t *msg);
template bool Protocol::sendMsgPoll<msgToa_t>(msgToa_t *msg);