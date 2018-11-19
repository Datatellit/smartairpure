#ifndef __PROTOCOL_PARSER_H
#define __PROTOCOL_PARSER_H

#include "_global.h"

extern uint8_t bMsgReady;
extern uint16_t delaySendTick;
extern uint8_t delaySend;

uint8_t ParseProtocol();
void build(uint8_t _destination, uint8_t _sensor, uint8_t _command, uint8_t _type, bool _enableAck, bool _isAck);
void Msg_NodeConfigAck(uint8_t _to, uint8_t _ncf);
void Msg_NodeConfigData(uint8_t _to);
void Msg_RequestNodeID();
void Msg_Presentation();
void Msg_DevStatus(uint8_t _to);

void MsgScanner_ProbeAck();
void MsgScanner_ConfigAck(uint8_t offset,uint8_t cfglen,bool _isStart); 
bool ProcessOutputCfgMsg();

#endif /* __PROTOCOL_PARSER_H */