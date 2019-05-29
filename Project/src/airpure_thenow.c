#include <stm8s.h>
#include "airpure_thenow.h"
#include "_global.h"
#include "Uart2Dev.h"
#include "mbcrc.h"
#include "timer_2.h"
#include "ProtocolParser.h"

#define ON_OFF_REG_ADDR                    40000
#define MODE_REG_ADDR                      40001
#define WIND_REG_ADDR                      40005


#define MSG_LEN          20
#define MODBUS_NOR_LEN   7

#define MAX_UART_BUF_SIZE 21
#define MSG_NUM 1
uint8_t uartReceiveDataBuf[MSG_NUM][MAX_UART_BUF_SIZE];
uint8_t rcvMsgLen[MSG_NUM];
uint8_t uartDataPtr = 0;
uint8_t msgWPtr = 0;
uint8_t msgRPtr = 0;
uint8_t msgCount = 0;

uint16_t uartcheck = 0;

uint8_t uartSndDataBuf[MAX_UART_BUF_SIZE];

uint16_t lastSendCMD = ON_OFF_REG_ADDR;
uint16_t alive_tick = 0;
airpureOp_t lastOp;
uint8_t needAck=0;
uint16_t lastsnd_tick = 0;
uint16_t lastrcv_tick = 0;
uint16_t delayquery_tick = 0;

void thenow_init()
{ 
  TIM2_Init();
  uartDataPtr = 0;
  memset(uartReceiveDataBuf,0,sizeof(uartReceiveDataBuf));
  memset(rcvMsgLen,0,sizeof(rcvMsgLen));
}

// when statusLen is 1£¬data as brief mode£¨0-off£¬1-1level£¬2-2level£¬3-3level£©
bool AddCmd(uint8_t *arrAirpurestatus,uint8_t statusLen)
{
    memcpy(lastOp.airpureStatus,arrAirpurestatus,statusLen);
    lastOp.status_size = statusLen;
    return TRUE;
}
void ClearRxData()
{
  memset(uartReceiveDataBuf,0,sizeof(uartReceiveDataBuf));
  memset(rcvMsgLen,0,sizeof(rcvMsgLen));
  uartDataPtr = 0;
  msgWPtr = 0;
  msgRPtr = 0;
  msgCount = 0;
  uartDataPtr = 0;
}

void ExecuteCmd(uint16_t regaddr,uint16_t value,uint8_t iswrite)
{
    lastSendCMD = regaddr;
    memset(uartSndDataBuf,0x00,sizeof(uartSndDataBuf));
    uint8_t len = 0;
    uartSndDataBuf[len++] = gConfig.nodeID - NODEID_MIN_AIRPURE+1;
    if(iswrite == 1) uartSndDataBuf[len++] = 6;
    else uartSndDataBuf[len++] = 3;
    uartSndDataBuf[len++] = regaddr>>8;
    uartSndDataBuf[len++] = regaddr&0xFF;
    uartSndDataBuf[len++] = value>>8;
    uartSndDataBuf[len++] = value&0xFF;
    uint16_t crc = usMBCRC16(uartSndDataBuf,len);
    uartSndDataBuf[len++] = crc & 0xFF ;
    uartSndDataBuf[len++] = crc >> 8 ;
    if(lastsnd_tick <= MSGTIMEOUT)
    {
       Delayms((MSGTIMEOUT-lastsnd_tick)*10);
    }
    disableInterrupts();
    Uart2SendByteByLen(uartSndDataBuf,len);
    enableInterrupts();
    lastsnd_tick = 0;
}

void QueryStatus()
{
    UART2_ITConfig(UART2_IT_RXNE, ENABLE);
    ClearRxData();
    ExecuteCmd(ON_OFF_REG_ADDR,1,0);
    Delayms(200);
    PraseMsg();
    ExecuteCmd(WIND_REG_ADDR,1,0);
    //UART2_ITConfig(UART2_IT_RXNE, DISABLE);
}

// 13byte£¬onff,level...
bool SendCmd()
{
  if(lastOp.status_size == 0)
  {
    return TRUE;
  }
  UART2_ITConfig(UART2_IT_RXNE, DISABLE);
  uint8_t onoff = 0;
  uint8_t windspeed = 0;
  if(lastOp.status_size == 1)
  {
     if(lastOp.airpureStatus[0] == 0 )
     { //power off
       onoff = 0;
     }
     else
     {
       onoff = 1;
       windspeed = lastOp.airpureStatus[0];
     }
  }
  else
  {
    if(lastOp.airpureStatus[0] >= 1)
    {
      onoff = lastOp.airpureStatus[0] - 1;
      windspeed = lastOp.airpureStatus[1] - 1;
    }
    else
    {
      return FALSE;
    }
  }
  ExecuteCmd(ON_OFF_REG_ADDR,onoff,1);
  if(onoff == 1)
  {
    ExecuteCmd(MODE_REG_ADDR,1,1);
    ExecuteCmd(WIND_REG_ADDR,windspeed,1);
  }
  lastOp.status_size = 0;
  /////////////////////update status////////////////////////////
  if(onoff == 1)
  {
    gConfig.airpureLaston[0] = onoff+1;
    gConfig.airpureLaston[1] = windspeed+1;
  }
  gConfig.airpureStatus[0] = onoff+1;
  gConfig.airpureStatus[1] = windspeed+1;
  gIsStatusChanged = TRUE;
  /////////////////////update status////////////////////////////
  delayquery_tick = 0;
  return TRUE;
}

void PraseMsg()
{
  while(msgCount >0 || msgRPtr != msgWPtr)
  {
#ifdef TEST
  PB2_High;
#endif
      uint8_t msglen = rcvMsgLen[msgRPtr];
      if(msglen < MODBUS_NOR_LEN)
      {
        msgRPtr = (msgRPtr+1)%MSG_NUM;
        msgCount = 0;      
        return;
      }
      else
      {
        unsigned int crcCheckRet = usMBCRC16(uartReceiveDataBuf[msgRPtr],msglen-2);
        uint8_t crchigh = crcCheckRet&0xFF;
        uint8_t crclow = crcCheckRet>>8;
        if(crchigh == uartReceiveDataBuf[msgRPtr][msglen-2] && crclow == uartReceiveDataBuf[msgRPtr][msglen-1])
        { // msg process
          uartcheck = 0;
          //uint16_t regaddr = (uartReceiveDataBuf[msgRPtr][2]<<8) | (uartReceiveDataBuf[msgRPtr][3]);
          if(uartReceiveDataBuf[msgRPtr][1] == 3)
          {
            uint8_t index = AIRPURESTATUSLEN;
            if(lastSendCMD == ON_OFF_REG_ADDR) index = 0;
            else if(lastSendCMD == WIND_REG_ADDR) index = 1;
            if(index < 3 && gConfig.airpureStatus[index] != uartReceiveDataBuf[msgRPtr][4]+1)
            {
              gConfig.airpureStatus[index] = uartReceiveDataBuf[msgRPtr][4]+1;
              Msg_DevStatus(NODEID_GATEWAY);
              gIsStatusChanged = TRUE;
            }
          }
        } 
      }
      msgRPtr = (msgRPtr+1)%MSG_NUM;
      msgCount = 0;      

#ifdef TEST
  PB2_Low;
#endif
  } 
  return;
}

void RtuMsgReady()
{
  if(uartDataPtr>0)
  {
    rcvMsgLen[msgWPtr] = uartDataPtr;
    msgWPtr = (msgWPtr+1)%MSG_NUM;
    msgCount++;
    uartDataPtr = 0;
  }
}

INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21)
{
  /* In order to detect unexpected events during development,
  it is recommended to set a breakpoint on the following instruction.
  */
  u8 data;
  if( UART2_GetITStatus(UART2_IT_RXNE) == SET ) {
    data = UART2_ReceiveData8();
    /*if(lastrcv_tick >= CHECKINTERVAL)
    { // new msg
      if(uartDataPtr > 0)
      {    // msg end   
        RtuMsgReady();
      }
    }*/
    lastrcv_tick = 0;
    uartReceiveDataBuf[msgWPtr][uartDataPtr++] = data;
    /*if(uartDataPtr >= MODBUS_NOR_LEN)
    { // msg end
      RtuMsgReady();
    }*/
    UART2_ClearITPendingBit(UART2_IT_RXNE);
  }
}
