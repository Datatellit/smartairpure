#ifndef __PROTOCOL_PARSER_H
#define __PROTOCOL_PARSER_H

#include "_global.h"
#include "ProtocolBus.h"

extern uint16_t delaySendTick;
extern uint8_t bDelaySend;

uint8_t ParseProtocol();
//void build(uint8_t _destination, uint8_t _sensor, uint8_t _command, uint8_t _type, bool _enableAck, bool _isAck);

void Msg_RequestNodeID();
void Msg_Presentation();
void Msg_DevStatus(uint8_t _to);

#endif /* __PROTOCOL_PARSER_H */