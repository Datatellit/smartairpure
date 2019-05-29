#include "ProtocolParser.h"
#include "_global.h"
#include "MyMessage.h"
#include "xliNodeConfig.h"
#include "rf24l01.h"
#ifdef MIA
#include "airpure_mia.h"
#endif
#ifdef THENOW
#include "airpure_thenow.h"
#endif

uint16_t delaySendTick = 0;
uint8_t bDelaySend = FALSE;

bool NeedProcess(uint8_t *arrType,uint8_t num)
{
  bool bNeedProcess = TRUE;
  for(uint8_t tidx = 0; tidx < num; tidx++ )
  {
    if(*(arrType+tidx) == gConfig.type) 
    {
      break;
    }
    if(tidx == num-1)
    {
      bNeedProcess = FALSE;
    }
  }
  return bNeedProcess;
}

uint8_t ParseProtocol(){
  if( rcvMsg.header.destination != gConfig.nodeID && rcvMsg.header.destination != BROADCAST_ADDRESS ) return 0;
  
  uint8_t ret = ParseCommonProtocol();
  if(ret) return 1;
    
  uint8_t _cmd = miGetCommand();
  uint8_t _sender = rcvMsg.header.sender;  // The original sender
  uint8_t _type = rcvMsg.header.type;
  uint8_t _sensor = rcvMsg.header.sensor;
  uint8_t _lenPayl = miGetLength();
  bool _needAck = (bool)miGetRequestAck();
  bool _isAck = (bool)miGetAck();
  bool _OnOff;
  bDelaySend = FALSE;
  delaySendTick = 0;
  
  switch( _cmd ) {
  case C_PRESENTATION:  
    if( _isAck ) {
      // Device/client got Response to Presentation message, ready to work
      gConfig.token = rcvMsg.payload.uiValue;
      gConfig.present = (gConfig.token >  0);
      GotPresented();
      gIsStatusChanged = TRUE;
    }
    break;
    
  case C_REQ:
    if( _needAck ) {
      if( IS_MINE_SUBID(_sensor) ) {
        if( _type == V_STATUS ) {
/*
//typedef struct
//{
//    //uint8_t devNum;
//    uint8_t devType1;
//    uint8_t devType2;
//    uint8_t devType3;
      ...
//    uint8_t devType5;
//}MyMsgPayload_t  
*/ 
          bool bNeedProcess = TRUE;
          if(rcvMsg.header.destination == 0xFF)
          {
            uint8_t devTypeNum = _lenPayl;
            if(devTypeNum > 0)
            {
              bNeedProcess = NeedProcess(&rcvMsg.payload.data[0],devTypeNum);
            }
          }
          if(bNeedProcess)
          {
            Msg_DevStatus(_sender);
            bDelaySend = (rcvMsg.header.destination == BROADCAST_ADDRESS);
            delaySendTick = GetDelayTick(rcvMsg.header.destination);
          }
          return 1;          
        }
      }
    }    
    break;  
  case C_SET:
    if( IS_MINE_SUBID(_sensor) && !_isAck ) {
      bool bValidCmd = FALSE;
      bool bNeedProcess = TRUE;
       if(_type == V_WIND)
      {
        bValidCmd = TRUE;
        uint8_t airpurestatus[CMD_LEN];
        memset(airpurestatus,0x00,sizeof(airpurestatus));
        memcpy(airpurestatus,&rcvMsg.payload.data[0],_lenPayl);
        /*if(rcvMsg.payload.data[0] == 2 && rcvMsg.payload.data[1] >1)
        { // on
          memset(gConfig.airpureLastOn,0x00,sizeof(gConfig.airpureLastOn));
          memcpy(gConfig.airpureLastOn,&rcvMsg.payload.data[0],_lenPayl);
        }*/
        AddCmd(airpurestatus,_lenPayl);
      }
      else if(_type == V_STATUS) {
        bValidCmd = TRUE;
/*
//typedef struct
//{
//    uint8_t onoff;
//    uint8_t delayopUnit;
//    uint8_t delayopTime;
//    //uint8_t devTypenum;
//    uint8_t devType1;
//    uint8_t devType2;
//    uint8_t devType3;
      ...
//    uint8_t devType5;
//}MyMsgPayload_t    
*/   
        if(rcvMsg.header.destination == 0xFF)
        {
          if(_lenPayl > 3)
          {
            uint8_t devTypeNum = _lenPayl - 3;
            if(devTypeNum > 0)
            {
              bNeedProcess = NeedProcess(&rcvMsg.payload.data[3],devTypeNum);
            }
          }
        }
        if(bNeedProcess)
        {
          if(rcvMsg.payload.data[0] == 1)
          {
            AddCmd(gConfig.airpureLaston,CMD_LEN);
          }
          else if(rcvMsg.payload.data[0] == 0)
          {
            AddCmd(&rcvMsg.payload.data[0],1);
          }
        }
      }
      else if(_type == V_RELAY_MAP) {
        bValidCmd = TRUE;
/*
//typedef struct
//{
//    uint8_t relayCmdLen;
//    char relaycmd[relayCmdLen];
//    uint8_t devType1;
//    uint8_t devType2;
//    uint8_t devType3;
      ...
//    uint8_t devType5;
      ...
//}MyMsgPayload_t  
no dev type default all
*/  
        uint8_t relaycmdLen = rcvMsg.payload.data[0];
        if(rcvMsg.header.destination == 0xFF)
        {
          if(_lenPayl > relaycmdLen+1)
          {
            bNeedProcess = NeedProcess(&rcvMsg.payload.data[relaycmdLen+1],_lenPayl-relaycmdLen-1);
          }
        }
        if(bNeedProcess)
        {
          if(_type == V_RELAY_MAP)
          {
            uint8_t relaynum = (relaycmdLen<=4)?relaycmdLen:4;
            uint8_t windspeed = 0;
            for( uint8_t idx = 0; idx < relaynum; idx++ ) {
              if( rcvMsg.payload.data[idx+1] == '1' ) {
                windspeed = idx+1;
                break;
              }
            }
            AddCmd(&windspeed,1);        
          }
        }
      }
      if(bValidCmd && bNeedProcess && _needAck)
      {
          //Msg_DevStatus(_sender);
          bDelaySend = TRUE;
          delaySendTick = GetDelayTick(rcvMsg.header.destination);
      }
    }
    break;
  }
  
  return 0;
}

void Msg_RequestNodeID() {
  // Request NodeID for device
  build(BASESERVICE_ADDRESS, NODE_TYP_SYSTEM, C_INTERNAL, I_ID_REQUEST, 1, 0);
  moSetPayloadType(P_ULONG32);
  moSetLength(UNIQUE_ID_LEN);
  memcpy(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Prepare device presentation message
void Msg_Presentation() {
  build(NODEID_GATEWAY, S_ZENREMOTE, C_PRESENTATION, gConfig.type, 1, 0);
  moSetPayloadType(P_ULONG32);
  moSetLength(UNIQUE_ID_LEN);
  memcpy(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Prepare device status message
void Msg_DevStatus(uint8_t _to) {
  build(_to, gConfig.subID, C_REQ, V_WIND, 0, 1);
  moSetLength(2);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.data[0] = gConfig.airpureStatus[0];
  sndMsg.payload.data[1] = gConfig.airpureStatus[1];
  bMsgReady = 1;
}