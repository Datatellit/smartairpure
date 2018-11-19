#ifndef __MIA_AIRPURE_H
#define __MIA_AIRPURE_H

typedef struct
{
  uint8_t airpureStatus[14];
  uint8_t status_size;
}airpureOp_t;

#define CMD_LEN          13

extern airpureOp_t lastOp;
extern uint16_t alive_tick;
void mia_init(void);
bool AddCmd(uint8_t *arrAirpurestatus,uint8_t statusLen);
bool SendCmd();
void PraseMsg();

#endif /* __MIA_AIRPURE_H */