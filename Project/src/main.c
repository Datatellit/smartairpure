#include "_global.h"
#include "rf24l01.h"
#include "MyMessage.h"
#include "xliNodeConfig.h"
#include "ProtocolParser.h"
#include "timer_4.h"
#include "FlashDataStorage.h"
#include "wwdg.h"
#include "Uart2Dev.h"
#include "airpure_mia.h"
/*
License: MIT

Auther: Baoshi Sun
Email: bs.sun@datatellit.com, bs.sun@uwaterloo.ca
Github: https://github.com/sunbaoshi1975
Please visit xlight.ca for product details

RF24L01 connector pinout:
GND    VCC
CE     CSN
SCK    MOSI
MISO   IRQ

Connections:
  PC3 -> CE
  PC4 -> CSN
  PC7 -> MISO
  PC6 -> MOSI
  PC5 -> SCK
  PC2 -> IRQ

*/

// Unique ID
#if defined(STM8S105) || defined(STM8S005) || defined(STM8AF626x)
  #define     UNIQUE_ID_ADDRESS         (0x48CD)
#endif
#if defined(STM8S103) || defined(STM8S003) ||  defined(STM8S903)
  #define     UNIQUE_ID_ADDRESS         (0x4865)
#endif

#define STATUS_REPORT_INTERVAL          3000   // 30s

const UC RF24_BASE_RADIO_ID[ADDRESS_WIDTH] = {0x00,0x54,0x49,0x54,0x44};

// Public variables
Config_t gConfig;
MyMessage_t sndMsg, rcvMsg;
uint8_t *psndMsg = (uint8_t *)&sndMsg;
uint8_t *prcvMsg = (uint8_t *)&rcvMsg;
bool gIsConfigChanged = FALSE;
bool gNeedSaveBackup = FALSE;
bool gIsStatusChanged = FALSE;
bool gRelayKeyChanged = FALSE;
bool gResetRF = FALSE;
bool gResetNode = FALSE;
uint8_t _uniqueID[UNIQUE_ID_LEN];

// Moudle variables
uint8_t mStatus = SYS_INIT;
bool mGotNodeID = FALSE;
uint8_t mutex = 0;

// Keep Alive Timer
uint16_t mTimerKeepAlive = 0;
uint8_t m_cntRFReset = 0;
uint8_t m_cntRFSendFailed = 0;

uint16_t m_StatusReportTick = 0;

uint8_t *Read_UniqueID(uint8_t *UniqueID, uint16_t Length)  
{
  Flash_ReadBuf(UNIQUE_ID_ADDRESS, UniqueID, Length);
  return UniqueID;
}

bool isIdentityEmpty(const UC *pId, UC nLen)
{
  for( int i = 0; i < nLen; i++ ) { if(pId[i] > 0) return FALSE; }
  return TRUE;
}

bool isIdentityEqual(const UC *pId1, const UC *pId2, UC nLen)
{
  for( int i = 0; i < nLen; i++ ) { if(pId1[i] != pId2[i]) return FALSE; }
  return TRUE;
}

bool isNodeIdRequired()
{
  return( isIdentityEmpty(gConfig.NetworkID, ADDRESS_WIDTH) || isIdentityEqual(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH) );
}

void UpdateNodeAddress(uint8_t _tx) {
  memcpy(rx_addr, gConfig.NetworkID, ADDRESS_WIDTH);
  rx_addr[0] = gConfig.nodeID;
  memcpy(tx_addr, gConfig.NetworkID, ADDRESS_WIDTH);
  
  if( _tx == NODEID_RF_SCANNER ) {
    tx_addr[0] = NODEID_RF_SCANNER;
  } else {  
    tx_addr[0] = NODEID_GATEWAY;
  }
  RF24L01_setup(gConfig.rfChannel, gConfig.rfDataRate, gConfig.rfPowerLevel, BROADCAST_ADDRESS);     // With openning the boardcast pipe
}

// reset rf
void ResetRFModule()
{
  if(gResetRF)
  {
    RF24L01_init();
    NRF2401_EnableIRQ();
    UpdateNodeAddress(NODEID_GATEWAY);
    gResetRF=FALSE;
  }
  if(gResetNode)
  {
    mStatus = SYS_RESET;
    gResetNode=FALSE;
  }
}

// Save config to Flash
void SaveBackupConfig()
{
  if( gNeedSaveBackup ) {
    // back config FLASH
    if(Flash_WriteDataBlock(BACKUP_CONFIG_BLOCK_NUM, (uint8_t *)&gConfig, sizeof(gConfig)))
    {
      gNeedSaveBackup = FALSE;
    }
  }
}

// Save status to Flash
void SaveStatusData()
{
  // status data contain nodeid subid and networkid
  if(gIsStatusChanged)
  {
    gNeedSaveBackup = TRUE;
    uint8_t pData[50] = {0};
    uint16_t nLen = (uint8_t *)(&gConfig.rfChannel) - (uint8_t *)(&gConfig);
    memcpy(pData, (uint8_t *)&gConfig, nLen);
    if(Flash_WriteDataBlock(STATUS_DATA_NUM, pData, nLen))
    {
      gIsStatusChanged = FALSE;
    }
  }
}

// Save config data to Flash(can't be called at working time)
void SaveConfig()
{
  if( gIsConfigChanged ) {
    gNeedSaveBackup = TRUE;
    // Overwrite entire config FLASH 
    uint8_t Attmpts = 0;
    while(++Attmpts <= 3) {
      if(Flash_WriteDataBlock(0, (uint8_t *)&gConfig, sizeof(gConfig)))
      {
        gIsConfigChanged = FALSE;
        break;
      }
    }
  }
  if(!gIsConfigChanged)
  { // ensure rf info is up to date
    ResetRFModule();
  }
  SaveStatusData();
}

// Initialize Node Address and look forward to being assigned with a valid NodeID by the SmartController
void InitNodeAddress() {

  if(isNodeIdInvalid(gConfig.nodeID))
  {
    gConfig.nodeID = XLA_PRODUCT_NODEID;
  }
  
  memcpy(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH);
}

bool IsStatusInvalid() {
  // gConfig.aircondStatus[2] > 32 temp more than 32
  return( gConfig.version > XLA_VERSION || gConfig.version < XLA_MIN_VER_REQUIREMENT);
}

bool IsConfigInvalid() {
  return( gConfig.version > XLA_VERSION || gConfig.version < XLA_MIN_VER_REQUIREMENT 
       || gConfig.nodeID == 0 || gConfig.type != XLA_PRODUCT_Type
       || gConfig.rfPowerLevel > RF24_PA_MAX || gConfig.rfChannel > 127 || gConfig.rfDataRate > RF24_250KBPS );
}

bool isNodeIdInvalid(uint8_t nodeid)
{

  return( !IS_TRANSMODULE_NODEID(nodeid) );

}
// Load config from Flash
void LoadConfig()
{
   // Load the config area
  Flash_ReadBuf(FLASH_DATA_START_PHYSICAL_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
  uint16_t nStatusLen = (uint8_t *)(&gConfig.nodeID) - (uint8_t *)(&gConfig);
  if( IsConfigInvalid() ) {
    // If config isn't OK, then try to load config from backup area
    Flash_ReadBuf(BACKUP_CONFIG_ADDRESS+nStatusLen, (uint8_t *)&gConfig.nodeID, sizeof(gConfig)-nStatusLen);
    bool backupInvalid = IsConfigInvalid();
    InitNodeAddress();
    if( backupInvalid ) {
      // If neither valid, then initialize config with default settings
        memset(&gConfig, 0x00, sizeof(gConfig));
        gConfig.version = XLA_VERSION;
        InitNodeAddress();
        gConfig.subID = 0;
        gConfig.nodeID = XLA_PRODUCT_NODEID;
        gConfig.type = XLA_PRODUCT_Type;
        gConfig.rptTimes = 1;
        gConfig.rfChannel = RF24_CHANNEL;
        gConfig.rfPowerLevel = RF24_PA_MAX;
        gConfig.rfDataRate = RF24_250KBPS;
    }
    gIsConfigChanged = TRUE;
    SaveConfig();
  } else {
    uint8_t bytVersion;
    Flash_ReadBuf(BACKUP_CONFIG_ADDRESS, (uint8_t *)&bytVersion, sizeof(bytVersion));
    if( bytVersion != gConfig.version ) gNeedSaveBackup = TRUE;
  }
  // Load the most recent status from FLASH
  uint8_t pData[50];
  memset(pData,0x00,sizeof(pData));
  uint16_t nLen = (uint8_t *)(&gConfig.rfChannel) - (uint8_t *)(&gConfig);
  Flash_ReadBuf(STATUS_DATA_ADDRESS, pData, nLen);
  if(pData[0] >= XLA_MIN_VER_REQUIREMENT && pData[0] <= XLA_VERSION)
  { // status data valid    
    memcpy(&gConfig,pData,nStatusLen);
    if(isIdentityEqual(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH) && !isNodeIdInvalid(gConfig.nodeID) )
    { // valid nodeid but with default network config,can covered by status data or back data if they are valid
      uint16_t networkOffset = (uint8_t *)(&gConfig.NetworkID) - (uint8_t *)(&gConfig);
      if( !isIdentityEmpty(pData+networkOffset,sizeof(gConfig.NetworkID)) )
      {
        memcpy(gConfig.NetworkID,pData+networkOffset,sizeof(gConfig.NetworkID));
      } 
    } 
  }
  else
  { // load backup area for status data
    Flash_ReadBuf(BACKUP_CONFIG_ADDRESS, pData, nLen);
    if(pData[0] >= XLA_MIN_VER_REQUIREMENT && pData[0] <= XLA_VERSION)
    { // status data valid 
      memcpy(&gConfig,pData,nStatusLen);
      if(isIdentityEqual(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH) && !isNodeIdInvalid(gConfig.nodeID))
      { // valid nodeid but with default network config,can covered by status data or back data if they are valid
        uint16_t networkOffset = (uint8_t *)(&gConfig.NetworkID) - (uint8_t *)(&gConfig);
        if( !isIdentityEmpty(pData+networkOffset,sizeof(gConfig.NetworkID)) )
        {
          memcpy(gConfig.NetworkID,pData+networkOffset,sizeof(gConfig.NetworkID));
        }        
      }
    }
  }
 
  if(IsStatusInvalid())
  {
    // default status value
    gConfig.version = XLA_VERSION;
    gConfig.state = 1;
  }
#ifdef EN_PANEL_BUTTONS
    if( gConfig.btnAction[0][0].action > 0x0F || gConfig.btnAction[1][0].action > 0x0F ) {
      memset(gConfig.btnAction, 0x00, sizeof(Button_Action_t) * MAX_NUM_BUTTONS);
    }
#endif   
    gConfig.rfChannel = 88;
}

bool WaitMutex(uint32_t _timeout) {
  while(_timeout--) {
    if( mutex > 0 ) return TRUE;
    feed_wwdg();
  }
  return FALSE;
}

bool NeedUpdateRFAddress(uint8_t _dest) {
  bool rc = FALSE;
  if( sndMsg.header.destination == NODEID_RF_SCANNER && tx_addr[0] != NODEID_RF_SCANNER ) {
    UpdateNodeAddress(NODEID_RF_SCANNER);
    rc = TRUE;
  } else if( sndMsg.header.destination != NODEID_RF_SCANNER && tx_addr[0] != NODEID_GATEWAY ) {
    UpdateNodeAddress(NODEID_GATEWAY);
    rc = TRUE;
  }
  return rc;
}

uint16_t GetDelayTick(uint8_t ds)
{
  uint16_t delaytick = 0;
  if(ds == BROADCAST_ADDRESS)
  {  
    delaytick = ((gConfig.nodeID - XLA_PRODUCT_NODEID+1)%32)*40;
  }
  else
  {
    delaytick = 10;   //airpure delay 100ms for airpure state ack
  }
  return delaytick;
}

// Send message and switch back to receive mode
bool SendMyMessage() {
  if( bMsgReady ) {
    
    // Change tx destination if necessary
    NeedUpdateRFAddress(sndMsg.header.destination);
      
    uint8_t lv_tried = 0;
    uint16_t delay;

      while (lv_tried++ <= gConfig.rptTimes ) {
        
        mutex = 0;
        if(RF24L01_set_mode_TX_timeout() == -1) 
          break;
        if(RF24L01_write_payload_timeout(psndMsg, PLOAD_WIDTH) == -1) 
          break;
        WaitMutex(0x1FFFF);
        if (mutex == 1) {
          m_cntRFSendFailed = 0;
          m_cntRFReset = 0;
          break; // sent sccessfully
        } else {
          m_cntRFSendFailed++;
          if( m_cntRFSendFailed >= MAX_RF_FAILED_TIME ) {
            m_cntRFSendFailed = 0;
            if( m_cntRFReset < 10 ) m_cntRFReset++;
            if( m_cntRFReset >= 3 ) {
              // Cold Reset
                SaveConfig();
                WWDG->CR = 0x80;
              break;
            } else if( m_cntRFReset >= 2 ) {
              // Reset whole node
              mStatus = SYS_RESET;
              break;
            }

            // Reset RF module
            //RF24L01_DeInit();
            delay = 0x1FFF;
            while(delay--)feed_wwdg();
            RF24L01_init();
            NRF2401_EnableIRQ();
            UpdateNodeAddress(NODEID_GATEWAY);
            continue;
          }
        }
        
        //The transmission failed, Notes: mutex == 2 doesn't mean failed
        //It happens when rx address defers from tx address
        //asm("nop"); //Place a breakpoint here to see memory
        // Repeat the message if necessary
        delay = 0xFFF;
        while(delay--)feed_wwdg();
      }
      
      // Switch back to receive mode
      bMsgReady = 0;
      RF24L01_set_mode_RX();
      
      // Reset Keep Alive Timer
      mTimerKeepAlive = 0;
  }

  return(mutex > 0);
}

void GotNodeID() {
  mGotNodeID = TRUE;
  UpdateNodeAddress(NODEID_GATEWAY);
  gNeedSaveBackup = TRUE;
}

void GotPresented() {
  mStatus = SYS_RUNNING;
  gConfig.swTimes = 0;
  gIsStatusChanged = TRUE;
}
u16 timeoutRFcheck = 0;
int main( void ) {
   static uint8_t bRestart = 1;
  //After reset, the device restarts by default with the HSI clock divided by 8.
  //CLK_DeInit();
  /* High speed internal clock prescaler: 1 */
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);  // now: HSI=16M prescale = 1; sysclk = 16M
  
  // Load config from Flash
  FLASH_DeInit();
  Read_UniqueID(_uniqueID, UNIQUE_ID_LEN);
  LoadConfig();
  SaveConfig();
  
  // Init Watchdog
  wwdg_init();
  
  uart2_config(9600);
  
  // Init timer
  TIM4_10ms_handler = tmrProcess;
  Time4_Init();

#ifdef TEST
   testio();
#endif
  while(1) {
    // Go on only if NRF chip is presented
    gConfig.present = 0;
    RF24L01_init();
    timeoutRFcheck = 0;
    while(!NRF24L01_Check()) {
      if( timeoutRFcheck > 50 ) {
        break;
      }
      feed_wwdg();
      timeoutRFcheck++;
    }
    // IRQ
    NRF2401_EnableIRQ();
    UpdateNodeAddress(NODEID_GATEWAY);
    
    // Send Presentation Message
    if( bRestart == 1 )
    {
      Msg_Presentation();
      bRestart = 0;
    } 
    SendMyMessage();
    RF24L01_set_mode_RX();
    mStatus = SYS_RUNNING;
    while (mStatus == SYS_RUNNING) {
      
      // Feed the Watchdog
      feed_wwdg();
      SendCmd();
      PraseMsg();
      if(delaySendTick == 0 && bDelaySend)
      {
        Msg_DevStatus(NODEID_GATEWAY);
        bDelaySend = FALSE;
      }
      if(!bMsgReady && m_StatusReportTick >= STATUS_REPORT_INTERVAL)
      {
        m_StatusReportTick = 0;
        Msg_DevStatus(NODEID_GATEWAY);
      }
      // Send message if ready
      SendMyMessage();
      // Save Config if Changed
      SaveConfig();

      ResetRFModule();
    }
  }
}

// Execute timer operations
void tmrProcess() {
  if(delaySendTick > 0)
  {
    delaySendTick--;
  }
  m_StatusReportTick++;
  // Tick
  mTimerKeepAlive++;
    // Save config into backup area
   SaveBackupConfig(); 
}

INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5) {

  if(RF24L01_is_data_available()) {
#ifdef TEST
  PB3_High;
#endif
    //Packet was received
    RF24L01_clear_interrupts();
    RF24L01_read_payload(prcvMsg, PLOAD_WIDTH);
    bMsgReady = ParseProtocol();
#ifdef TEST
    PB3_Low;
#endif
    return;
  }
 
  uint8_t sent_info;
  if (sent_info = RF24L01_was_data_sent()) {
#ifdef TEST
  PB4_High;
#endif
    //Packet was sent or max retries reached
    RF24L01_clear_interrupts();
    mutex = sent_info;
#ifdef TEST
    PB4_Low;
#endif    
    return;
  }
   RF24L01_clear_interrupts();
}
