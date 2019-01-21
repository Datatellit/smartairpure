#ifndef __THENOW_AIRPURE_H
#define __THENOW_AIRPURE_H

typedef struct
{
  uint8_t airpureStatus[14];
  uint8_t status_size;
}airpureOp_t;

#define CMD_LEN          13

extern airpureOp_t lastOp;
extern uint16_t alive_tick;
extern uint16_t lastsnd_tick;
extern uint16_t lastrcv_tick;
bool AddCmd(uint8_t *arrAirpurestatus,uint8_t statusLen);
bool SendCmd();
void PraseMsg();
void thenow_init();

#endif /* __THENOW_AIRPURE_H */