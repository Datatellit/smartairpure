#include <stm8s.h>
#include "airpure_thenow.h"
#include "_global.h"
#include "Uart2Dev.h"
#include "mbcrc.h"
#include "timer_2.h"

#define ON_OFF_REG_ADDR                    40000
#define MODE_REG_ADDR                      40001
#define WIND_REG_ADDR                      40005

#define CHECKINTERVAL                      5        //50ms send interval


#define MSG_LEN          20
#define MODBUS_NOR_LEN   8

#define MAX_UART_BUF_SIZE 21
#define MSG_NUM 1
uint8_t uartReceiveDataBuf[MSG_NUM][MAX_UART_BUF_SIZE];
uint8_t rcvMsgLen[MSG_NUM];
uint8_t uartDataPtr = 0;
uint8_t msgWPtr = 0;
uint8_t msgRPtr = 0;
uint8_t msgCount = 0;

uint8_t uartSndDataBuf[MAX_UART_BUF_SIZE];


uint16_t alive_tick = 0;
airpureOp_t lastOp;
uint8_t needAck=0;
uint16_t lastsnd_tick = 0;
uint16_t lastrcv_tick = 0;

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

void ExecuteCmd(uint16_t regaddr,uint16_t value)
{
    memset(uartSndDataBuf,0x00,sizeof(uartSndDataBuf));
    uint8_t len = 0;
    uartSndDataBuf[len++] = gConfig.nodeID - NODEID_MIN_AIRPURE+1;
    uartSndDataBuf[len++] = 6;
    uartSndDataBuf[len++] = regaddr>>8;
    uartSndDataBuf[len++] = regaddr&0xFF;
    uartSndDataBuf[len++] = value>>8;
    uartSndDataBuf[len++] = value&0xFF;
    uint16_t crc = usMBCRC16(uartSndDataBuf,len);
    uartSndDataBuf[len++] = crc & 0xFF ;
    uartSndDataBuf[len++] = crc >> 8 ;
    if(lastsnd_tick <= CHECKINTERVAL)
    {
       Delayms((CHECKINTERVAL-lastsnd_tick)*10);

    }
    disableInterrupts();
    Uart2SendByteByLen(uartSndDataBuf,len);
    enableInterrupts();
    lastsnd_tick = 0;
}

// 13byte£¬onff,level...
bool SendCmd()
{
  if(lastOp.status_size == 0)
  {
    return TRUE;
  }
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
  ExecuteCmd(ON_OFF_REG_ADDR,onoff);
  if(onoff == 1)
  {
    ExecuteCmd(MODE_REG_ADDR,1);
    ExecuteCmd(WIND_REG_ADDR,windspeed);
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
      unsigned int crcCheckRet = usMBCRC16(uartReceiveDataBuf[msgRPtr],msglen-2);
      uint8_t crchigh = crcCheckRet&0xFF;
      uint8_t crclow = crcCheckRet>>8;
      if(crchigh == uartReceiveDataBuf[msgRPtr][msglen-2] && crclow == uartReceiveDataBuf[msgRPtr][msglen-1])
      { // msg process
        uint16_t regaddr = (uartReceiveDataBuf[msgRPtr][2]<<8) | (uartReceiveDataBuf[msgRPtr][3]);
        //uint16_t value = (uartReceiveDataBuf[msgRPtr][4]<<8) | (uartReceiveDataBuf[msgRPtr][5]);
        if(regaddr == ON_OFF_REG_ADDR)
        {
          gConfig.airpureStatus[0] = uartReceiveDataBuf[msgRPtr][5]+1;
        }
        else if(regaddr == MODE_REG_ADDR)
        {
        }
        else if(regaddr == WIND_REG_ADDR)
        {
          gConfig.airpureStatus[1] = uartReceiveDataBuf[msgRPtr][5]+1;
        }
        gIsStatusChanged = TRUE;
      } 
      msgRPtr = (msgRPtr+1)%MSG_NUM;
      msgCount = 0;      

#ifdef TEST
  PB2_Low;
#endif
  } 
  return;
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
        rcvMsgLen[msgWPtr] = uartDataPtr;
        msgWPtr = (msgWPtr+1)%MSG_NUM;
        msgCount++;
        uartDataPtr = 0;
      }
    }*/
    lastrcv_tick = 0;
    uartReceiveDataBuf[msgWPtr][uartDataPtr++] = data;
    if(uartDataPtr >= MODBUS_NOR_LEN)
    { // msg end
      rcvMsgLen[msgWPtr] = uartDataPtr;
      msgWPtr = (msgWPtr+1)%MSG_NUM;
      msgCount++;
      uartDataPtr = 0;
    }
    UART2_ClearITPendingBit(UART2_IT_RXNE);
  }
}
