#include <stm8s.h>
#include "airpure_mia.h"
#include "_global.h"
#include "Uart2Dev.h"

#define MESSAGE_CTL_HEAD                    0xC7
#define MESSAGE_CNF_HEAD                    0xAD
#define UART_STEP_WAIT_HEAD             0
#define UART_STEP_WAIT_LEN              1
#define UART_STEP_WAIT_PAYL             2
#define UART_STEP_WAIT_CHECKSUM0        3
#define UART_STEP_WAIT_CHECKSUM1        4
// C7-1 AD-2
uint8_t msgtype = 0;

#define MSG_LEN          20

#define MAX_UART_BUF_SIZE 21
#define MSG_NUM 1
uint8_t uart_step = UART_STEP_WAIT_HEAD;
uint8_t uartReceiveDataBuf[MSG_NUM][MAX_UART_BUF_SIZE];
uint8_t uartDataPtr = 0;
uint8_t msgWPtr = 0;
uint8_t msgRPtr = 0;
uint8_t msgLen = 0;


uint16_t alive_tick = 0;
airpureOp_t lastOp;
uint8_t needAck=0;

/*CRC 校验函数，生成 CRC*/ 
unsigned int CRC_Check(u8 * msg,u8 len) 
{ 
  u8 i,j;
  unsigned int crc=0;
  unsigned int current=0;
  for(i = 0;i<len;i++)
  {
    current=msg[i]<<8;
    for(j=0;j<8;j++)
    {
      if((crc^current)&0x8000)
        crc=(crc<<1)^0x1021;
      else
        crc<<=1;
      current<<=1;
    }
  }
  return crc;
}  

void mia_init()
{ 
  uartDataPtr = 0;
  uart_step = UART_STEP_WAIT_HEAD;
  memset(uartReceiveDataBuf,0,sizeof(uartReceiveDataBuf));
  // Init serial ports
  uart2_config(9600);
}
// when statusLen is 1，data as brief mode（0-off，1-1level，2-2level，3-3level）
bool AddCmd(uint8_t *arrAirpurestatus,uint8_t statusLen)
{
    memcpy(lastOp.airpureStatus,arrAirpurestatus,statusLen);
    lastOp.status_size = statusLen;
    return TRUE;
}

// 13byte，onff,level...
bool SendCmd()
{
  if(lastOp.status_size == 0)
  {
    return TRUE;
  }
  uint8_t data[MSG_LEN];
  memset(data,0x00,sizeof(data));
  data[0] = 0xC7;
  data[1] = 0x12;
  // index2--addr index4--addr
  // index3--cmdtype:1-query 2-set
  data[3] = 0x02;
  uint16_t crc = 0;

  if(lastOp.status_size == 1)
  {
     if(lastOp.airpureStatus[0] == 0 )
     { //power off
       data[5] = 0x01;
       data[6] = 0x01;
     }
     else
     {//power on
        // index5--onoff: 1-off 2-on
        data[5] = 0x02;
        // index6--windlevel: 1-0level 2-1level 3- 2level 4-3level
        data[6] = lastOp.airpureStatus[0]+1;
     }
  }
  else if(lastOp.status_size <= CMD_LEN)
  {
    memcpy(&data[5],lastOp.airpureStatus,lastOp.status_size);
  }
  else
  {
    return FALSE;
  }
  crc = CRC_Check(data,MSG_LEN-2);
  data[MSG_LEN-2] = crc>>8;
  data[MSG_LEN-1] = crc&0x00FF;
  Uart2SendByteByLen(data,MSG_LEN);
  lastOp.status_size = 0;
  /////////////////////update status////////////////////////////
  //memcpy(&gConfig.airpureStatus,&data[5],CMD_LEN);
  if(data[6] >1 && data[5]==2)
  {
    memcpy(&gConfig.airpureLaston,&data[5],CMD_LEN);
  }
  memcpy(&gConfig.airpureStatus,&data[5],CMD_LEN);
  /////////////////////update status////////////////////////////
  return TRUE;
}

void SendQureyCmd()
{
  uint8_t data[MSG_LEN];
  memset(data,0x00,sizeof(data));
  data[0] = 0xC7;
  data[1] = 0x12;
  // index2--addr index4--addr
  // index3--cmdtype:1-query 2-set
  data[3] = 0x01;
  uint16_t crc = 0;
  crc = CRC_Check(data,MSG_LEN-2);
  data[MSG_LEN-2] = crc>>8;
  data[MSG_LEN-1] = crc&0x00FF;
  Uart2SendByteByLen(data,MSG_LEN);
}

void PraseMsg()
{
  while(msgLen >0 || msgRPtr != msgWPtr)
  {
#ifdef TEST
  PB2_High;
#endif
    if(uartReceiveDataBuf[msgRPtr][0] == MESSAGE_CNF_HEAD)
    {
    }
    else if(uartReceiveDataBuf[msgRPtr][0] == MESSAGE_CTL_HEAD)
    {
      unsigned int crcCheckRet = CRC_Check(uartReceiveDataBuf[msgRPtr],MSG_LEN-2);
      uint8_t crchigh = crcCheckRet>>8;
      uint8_t crclow = crcCheckRet&0x00FF;
      if(crchigh == uartReceiveDataBuf[msgRPtr][MSG_LEN-2] && crclow == uartReceiveDataBuf[msgRPtr][MSG_LEN-1])
      { // msg process
        memcpy(&gConfig.airpureStatus,&uartReceiveDataBuf[msgRPtr][5],CMD_LEN);
        if(uartReceiveDataBuf[msgRPtr][6] >1 && uartReceiveDataBuf[msgRPtr][5]==2)
        {
          memcpy(&gConfig.airpureLaston,&uartReceiveDataBuf[msgRPtr][5],CMD_LEN);
        }
        gIsStatusChanged = TRUE;
      } 
      msgRPtr = (msgRPtr+1)%MSG_NUM;
      msgLen = 0;      
    }
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
    switch( uart_step ) {
    case UART_STEP_WAIT_HEAD:
      if( data == MESSAGE_CTL_HEAD || data == MESSAGE_CNF_HEAD) 
      {
        uart_step = UART_STEP_WAIT_LEN;
        uartReceiveDataBuf[msgWPtr][uartDataPtr++] = data;
        if(data == MESSAGE_CTL_HEAD) msgtype =1;
        else if(data == MESSAGE_CNF_HEAD) msgtype=2;
      }
      break;
    case UART_STEP_WAIT_LEN:
      if( data > 1 && data < MAX_UART_BUF_SIZE ) {
        //uartDataPtr = 0;
        uartReceiveDataBuf[msgWPtr][uartDataPtr++] = data;
        uart_step = UART_STEP_WAIT_PAYL;
      } else {
        uartDataPtr = 0;
        uart_step = UART_STEP_WAIT_HEAD;
      }
      break;
    case UART_STEP_WAIT_PAYL:   
      uartReceiveDataBuf[msgWPtr][uartDataPtr++] = data; 
      if( uartDataPtr == uartReceiveDataBuf[msgWPtr][1])
      {
        if(msgtype == 1)
        {
          uart_step = UART_STEP_WAIT_CHECKSUM0;
        }
        else if(msgtype == 2)
        {
          uart_step = UART_STEP_WAIT_HEAD;
          msgWPtr = (msgWPtr+1)%MSG_NUM;
          msgLen++;
          uartDataPtr = 0;
          msgtype = 0;
        }
      }   
      break;
    case UART_STEP_WAIT_CHECKSUM0:
      uartReceiveDataBuf[msgWPtr][uartDataPtr++] = data;
      uart_step = UART_STEP_WAIT_CHECKSUM1;
      break;
    case UART_STEP_WAIT_CHECKSUM1:
      uartReceiveDataBuf[msgWPtr][uartDataPtr++] = data;
      uart_step = UART_STEP_WAIT_CHECKSUM1;
      msgWPtr = (msgWPtr+1)%MSG_NUM;
      msgLen++;
      uartDataPtr = 0;
      uart_step = UART_STEP_WAIT_HEAD;
      msgtype = 0;
      break;
    default:
      uartDataPtr = 0;
      msgtype = 0;
      uart_step = UART_STEP_WAIT_HEAD;
      break;
    }
    UART2_ClearITPendingBit(UART2_IT_RXNE);
  }
}
