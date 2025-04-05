/******************************************************************************

 @file  simpleBLECentral.c

 @brief This file contains the Simple BLE Central sample application for use
        with the CC2540 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2010-2016, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: ble_sdk_1.4.2.2
 Release Date: 2016-06-09 06:57:10
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "hal_uart.h"//Ìí¼ÓµÄ
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "simpleBLECentral.h"
#include "string.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE//TRUE//

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      20//8//400//16//

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      32//800//32//

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  FALSE//TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           200//1000 06.17   ????200ms????????

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
static uint8 simpleBLETaskId;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8 simpleBLEScanRes;
static uint8 simpleBLEScanIdx;

// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static uint8 simpleBLEScanning = FALSE;

// RSSI polling state
static uint8 simpleBLERssi = FALSE;

// Connection handle of current connection 
static uint16 simpleBLEConnHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8 simpleBLEState = BLE_STATE_IDLE;

// Discovery state
static uint8 simpleBLEDiscState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16 simpleBLESvcStartHdl = 0;
static uint16 simpleBLESvcEndHdl = 0;

// Discovered characteristic handle
static uint16 simpleBLECharHdl = 0;

// Value to write
//static uint8 simpleBLECharVal = 0;

// Value read/write toggle
//static bool simpleBLEDoWrite = FALSE;

// GATT read/write procedure state
//static bool simpleBLEProcedureInProgress = FALSE;

static uint8 uuid128_flag = FALSE;
static uint8 Service_UUID[16] = {0};

#define RCV_MAX                   200
/*********************************************************************
* GLOBAL VARIABLES
*/
uint8 send_to_air_buf[RCV_MAX];
uint8 Msp_Buffer[RCV_MAX];
uint16 Fi_Ptr = 0x00,La_Ptr = 0x00,send_to_air_index_in = 0,send_to_air_index_out = 0;

devInfo_t dev = 
{
  .hasDataFlag = FALSE,
  
};
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );
static void simpleBLECentralRssiCB( uint16 connHandle, int8  rssi );
static uint8 simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
//static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys );
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
static void simpleBLECentralStartDiscovery( void );
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );
char *bdAddr2Str ( uint8 *pAddr );

void SerialPacketPort0(uint8 port, uint8 event);

void packAndSend(void);
static void getNameAndData(uint8 *pAddr, uint8 addrType, uint8 dataLen, uint8 *data);

uint16 Minus_PtrMv(uint16 Current_Ptr,uint16 Minnd);
uint16 Plus_PtrMv(uint16 Current_Ptr,uint16 addnd);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t simpleBLERoleCB =
{
  simpleBLECentralRssiCB,       // RSSI callback
  simpleBLECentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
  simpleBLECentralPasscodeCB,
  simpleBLECentralPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLECentral_Init( uint8 task_id )
{
  simpleBLETaskId = task_id;

  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  
  GAP_SetParamValue( TGAP_CONN_EST_INT_MIN, DEFAULT_UPDATE_MIN_CONN_INTERVAL );
  GAP_SetParamValue( TGAP_CONN_EST_INT_MAX, DEFAULT_UPDATE_MAX_CONN_INTERVAL );

  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( simpleBLETaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

//  // Register for all key events - This app will handle all key events
//  RegisterForKeys( simpleBLETaskId );
//  
//  // makes sure LEDs are off
//  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  
  halUARTCfg_t  config_UART_DMA; 
  config_UART_DMA.configured           = TRUE;
  config_UART_DMA.baudRate             = HAL_UART_BR_115200;//HAL_UART_BR_9600;//
  config_UART_DMA.flowControl          = FALSE;
  config_UART_DMA.flowControlThreshold = 0x00;
  config_UART_DMA.rx.maxBufSize        = 30;
  config_UART_DMA.tx.maxBufSize        = 30;
  config_UART_DMA.idleTimeout          = 200;
  config_UART_DMA.intEnable            = TRUE;
  config_UART_DMA.callBackFunc         = SerialPacketPort0;
  HalUARTOpen(HAL_UART_PORT_0, &config_UART_DMA);//Í¨ÐÅ´®¿Ú UART0£¬alt2
  
  HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_0_DBM);//Ã÷È··¢Éä¹¦ÂÊ 06.29
  HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH); //ÉèÖÃ½ÓÊÕÔöÒæ 10.10 
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      SimpleBLECentral_ProcessEvent
 *
 * @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
    {
      simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );

    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &simpleBLEBondCB );
    
//    osal_start_timerEx( simpleBLETaskId, START_SCAN_EVT, 1000 );
    osal_start_timerEx( simpleBLETaskId, UART_SEND_EVT, 500 );

    return ( events ^ START_DEVICE_EVT );
  }
  
  if(events & START_SCAN_EVT)
  {
    // Start or stop discovery
    if ( simpleBLEState != BLE_STATE_CONNECTED )  //¿ªÊ¼É¨ÃèÉè±¸
    {
      if ( !simpleBLEScanning )
      {
        uint8 start[] = {"start to scan...\n"};
        HalUARTWrite(HAL_UART_PORT_0, start, (sizeof(start)/sizeof(uint8)));
    
        simpleBLEScanning = TRUE;
        simpleBLEScanRes = 0;
        
        GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                       DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                       DEFAULT_DISCOVERY_WHITE_LIST );      
      }
      else
      {
        GAPCentralRole_CancelDiscovery();
      }
    }    
    return events ^ START_SCAN_EVT;
  }
  
  if(events & UART_EXTRACT_CMD_EVT)
  {
    static bool find_head = FALSE;
    static bool find_tail = FALSE;
    static uint16 cmd_head_index = 0;
    static uint16 cmd_tail_index = 0;
    uint8 cmd_buf[40] = {0};
    while(La_Ptr != Fi_Ptr)
    {
      // find head
      if(Msp_Buffer[La_Ptr] == 'A')
      {
        find_head = TRUE;
        cmd_head_index = La_Ptr;
        cmd_tail_index = cmd_head_index;
        if(Minus_PtrMv(Fi_Ptr,La_Ptr)<2)
        {
          break;// break while()
        }
        else
        {
          if(Msp_Buffer[Plus_PtrMv(cmd_head_index,1)] == 'T')
          {
            // find tail
            while(cmd_tail_index != Fi_Ptr)
            {
              if(Msp_Buffer[cmd_tail_index] == '\n')
              {
                find_tail = TRUE;
                break;
              }
              else
              {
                cmd_tail_index=Plus_PtrMv(cmd_tail_index,1);
              }
            }
            
            if(find_tail == TRUE)
            {
              //get cmd
              uint8 cmd_len = Minus_PtrMv(cmd_tail_index,cmd_head_index);
              for( uint16 i = 0, j = cmd_head_index; i < cmd_len; i++, j = Plus_PtrMv(j,1) )
              {
                cmd_buf[i] = Msp_Buffer[j];
              }
              // replace '\n' to '\0'
              cmd_buf[cmd_len -1] = '\0';
              //exe cmd
              HalUARTWrite(HAL_UART_PORT_0, cmd_buf, cmd_len);
              HalUARTWrite(HAL_UART_PORT_0, "\n", 1);
              if(strstr((char const*)cmd_buf,"SCAN"))
              {
                uuid128_flag = FALSE;
                Service_UUID[0] = 0xFF;
                Service_UUID[1] = 0xF0;
                osal_start_timerEx( simpleBLETaskId, START_SCAN_EVT, 10 );
              }
              else if(strstr((char const*)cmd_buf,"CONN1"))
              {
                
              }
              else if(strstr((char const*)cmd_buf,"CONN2"))
              {
                
              }
              else if(strstr((char const*)cmd_buf,"CONN"))
              {
                osal_start_timerEx( simpleBLETaskId, START_CONN_EVT, 10 );
              }
              else
              {
                //cmd error
              }
              
              //´¦ÀíÖ¸Õë£¬Ìø¹ýÕâÌõÖ¸Áî
              La_Ptr = cmd_tail_index + 1;
              //clear cmd flag
              find_head = FALSE;
              find_tail = FALSE;
              cmd_head_index = 0;
              cmd_tail_index = 0;
            }
            else
            {
              break; // break while()
            }
          }
          else
          {
            //clear cmd flag
            find_head = FALSE;
            find_tail = FALSE;
            cmd_head_index = 0;
            cmd_tail_index = 0;
            
            send_to_air_buf[send_to_air_index_in] = Msp_Buffer[La_Ptr];
            send_to_air_index_in=Plus_PtrMv(send_to_air_index_in,1);
            La_Ptr=Plus_PtrMv(La_Ptr,1);
          }
        }
        
      }
      else
      {
        send_to_air_buf[send_to_air_index_in] = Msp_Buffer[La_Ptr];
        send_to_air_index_in=Plus_PtrMv(send_to_air_index_in,1);
        La_Ptr=Plus_PtrMv(La_Ptr,1);
      }
      
    }
    
    if(Minus_PtrMv(send_to_air_index_in,send_to_air_index_out))
    {
      osal_start_timerEx(simpleBLETaskId,SEND_TO_AIR_EVT,5);
    }
    
    return ( events ^ UART_EXTRACT_CMD_EVT );
  }
  
  if(events & UART_SEND_EVT)
  {
    HalUARTWrite(HAL_UART_PORT_0, "central init OK \r\n", sizeof("central init OK \r\n"));
//    //½âÎöÊý¾Ý²¢·¢ËÍ
//    packAndSend();
//    dev.hasDataFlag = FALSE;
    
//    osal_start_timerEx( simpleBLETaskId, UART_SEND_EVT, 15000 );
    return ( events ^ UART_SEND_EVT );
  }

  if ( events & START_CONN_EVT )
  {
    if ( simpleBLEScanRes > 0 )
    {
      simpleBLEScanIdx--;
      simpleBLEScanRes--;
      
//      for(uint8 i = 0; i < simpleBLEScanRes; i++)
      {
        GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE, DEFAULT_LINK_WHITE_LIST, simpleBLEDevList[simpleBLEScanRes].addrType, simpleBLEDevList[simpleBLEScanRes].addr );
      }
      HalUARTWrite(HAL_UART_PORT_0, bdAddr2Str(simpleBLEDevList[simpleBLEScanRes].addr), B_ADDR_STR_LEN);
      HalUARTWrite(HAL_UART_PORT_0, "\r\n", 2);
//      simpleBLEScanRes = 0;
//      osal_start_timerEx( simpleBLETaskId, START_CONN_EVT, 2000 );
    }
    
    return ( events ^ START_CONN_EVT );
  }
  
  if ( events & START_DISCOVERY_EVT )
  {
    simpleBLECentralStartDiscovery( );
    
    return ( events ^ START_DISCOVERY_EVT );
  }
  
  if ( events & SEND_TO_AIR_EVT )
  {
    if(send_to_air_index_out != send_to_air_index_in)
    {
      uint8 len = Minus_PtrMv(send_to_air_index_in,send_to_air_index_out);
      if(len > 20)
      {
        len = 20;
      }
      // Do a write
      attWriteReq_t preq;
      uint16 i = 0,j = 0;
      bool send_to_1st_success = FALSE;
      bool send_to_2ed_success = FALSE;
      // to 1st device
      preq.pValue = GATT_bm_alloc(0, ATT_WRITE_REQ,
                                           len, NULL);
      if(preq.pValue != NULL)
      {
        preq.handle = simpleBLECharHdl;
        preq.len = len;
        preq.sig = 0;
        preq.cmd = 1;
        for( i = 0,j = send_to_air_index_out; i < len; i++, j = Plus_PtrMv(j,1) )
        {
          preq.pValue[i] = send_to_air_buf[j];
        }
        if(GATT_WriteNoRsp( 0, &preq ) == SUCCESS)
        {
          send_to_1st_success = TRUE;
        }
        else
        {
          GATT_bm_free( (gattMsg_t *)&preq, ATT_WRITE_REQ );
        }
      }
      // to 2ed device
      preq.pValue = GATT_bm_alloc( 1, ATT_WRITE_REQ,
                                           len, NULL);//simpleBLEConnHandle
      if(preq.pValue != NULL)
      {
        preq.handle = simpleBLECharHdl;
        preq.len = len;
        preq.sig = 0;
        preq.cmd = 1;
        for( i = 0,j = send_to_air_index_out; i < len; i++, j = Plus_PtrMv(j,1) )
        {
          preq.pValue[i] = send_to_air_buf[j];
        }
        if(GATT_WriteNoRsp( 1, &preq ) == SUCCESS)
        {
          send_to_2ed_success = TRUE;
        }
        else
        {
          GATT_bm_free( (gattMsg_t *)&preq, ATT_WRITE_REQ );
        }
      }
      
      if(send_to_1st_success && send_to_2ed_success)
      {
        send_to_air_index_out = j;
      }
      osal_start_timerEx( simpleBLETaskId, SEND_TO_AIR_EVT, 20 );
    }
    
    return ( events ^ SEND_TO_AIR_EVT );
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLECentral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
//      simpleBLECentral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    case GATT_MSG_EVENT:
      simpleBLECentralProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
  }
}


/*********************************************************************
 * @fn      simpleBLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  if ( simpleBLEState != BLE_STATE_CONNECTED )
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message
    return;
  }
  
  if ( ( pMsg->method == ATT_READ_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
  {
    if ( pMsg->method == ATT_ERROR_RSP )
    {
//      uint8 status = pMsg->msg.errorRsp.errCode;
//      
//      LCD_WRITE_STRING_VALUE( "Read Error", status, 10, HAL_LCD_LINE_1 );
    }
    else
    {
//      // After a successful read, display the read value
//      uint8 valueRead = pMsg->msg.readRsp.pValue[0];
//
//      LCD_WRITE_STRING_VALUE( "Read rsp:", valueRead, 10, HAL_LCD_LINE_1 );
    }
    
//    simpleBLEProcedureInProgress = FALSE;
  }
  else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {
    
    if ( pMsg->method == ATT_ERROR_RSP )
    {
//      uint8 status = pMsg->msg.errorRsp.errCode;
//      
//      LCD_WRITE_STRING_VALUE( "Write Error", status, 10, HAL_LCD_LINE_1 );
    }
    else
    {
//      // After a succesful write, display the value that was written and increment value
//      LCD_WRITE_STRING_VALUE( "Write sent:", simpleBLECharVal++, 10, HAL_LCD_LINE_1 );      
    }
    
//    simpleBLEProcedureInProgress = FALSE;    

  }
  else if ( pMsg->method == ATT_HANDLE_VALUE_NOTI )
  {
    HalUARTWrite(HAL_UART_PORT_0, pMsg->msg.handleValueNoti.pValue, pMsg->msg.handleValueNoti.len);
  }
  else if ( pMsg->method == ATT_HANDLE_VALUE_IND )
  {
    
  }
  else if ( simpleBLEDiscState != BLE_DISC_STATE_IDLE )
  {
    simpleBLEGATTDiscoveryEvent( pMsg );
  }
  
  GATT_bm_free( &pMsg->msg, pMsg->method );
}

/*********************************************************************
 * @fn      simpleBLECentralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void simpleBLECentralRssiCB( uint16 connHandle, int8 rssi )
{
    LCD_WRITE_STRING_VALUE( "RSSI -dB:", (uint8) (-rssi), 10, HAL_LCD_LINE_1 );
}

/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8 simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        LCD_WRITE_STRING( "BLE Central", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( pEvent->initDone.devAddr ),  HAL_LCD_LINE_2 );
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE )
        {
          if ( simpleBLEFindSvcUuid( SIMPLEPROFILE_SERV_UUID,
                                     pEvent->deviceInfo.pEvtData,
                                     pEvent->deviceInfo.dataLen ) )
          {
            simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
          }
        }
//        getNameAndData(pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType, pEvent->deviceInfo.dataLen, pEvent->deviceInfo.pEvtData);
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        simpleBLEScanning = FALSE;

        // if not filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE )
        {
          // Copy results
          simpleBLEScanRes = pEvent->discCmpl.numDevs;
          osal_memcpy( simpleBLEDevList, pEvent->discCmpl.pDevList,
                       (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
        }
        
        LCD_WRITE_STRING_VALUE( "Devices Found", simpleBLEScanRes,
                                10, HAL_LCD_LINE_1 );
        if ( simpleBLEScanRes > 0 )
        {
          LCD_WRITE_STRING( "<- To Select", HAL_LCD_LINE_2 );
          
          for(uint8 i = 0; i < simpleBLEScanRes; i++)
          {
            HalUARTWrite(HAL_UART_PORT_0, bdAddr2Str(simpleBLEDevList[i].addr), B_ADDR_STR_LEN);
            HalUARTWrite(HAL_UART_PORT_0, "\r\n", 2);
          }
        }

        // initialize scan index to last device
        simpleBLEScanIdx = simpleBLEScanRes;
        
//        osal_start_timerEx( simpleBLETaskId, START_SCAN_EVT, 500 );

      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if ( pEvent->gap.hdr.status == SUCCESS )
        {
          simpleBLEState = BLE_STATE_CONNECTED;
          simpleBLEConnHandle = pEvent->linkCmpl.connectionHandle;

//          simpleBLEProcedureInProgress = TRUE;    

          // If service discovery not performed initiate service discovery
          if ( simpleBLECharHdl == 0 )
          {
            osal_start_timerEx( simpleBLETaskId, START_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
          }
                    
          LCD_WRITE_STRING( "Connected", HAL_LCD_LINE_1 );
          LCD_WRITE_STRING( bdAddr2Str( pEvent->linkCmpl.devAddr ), HAL_LCD_LINE_2 );  
          HalUARTWrite(HAL_UART_PORT_0, "conn_OK\r\n", 9);
          // connect next device
        osal_start_timerEx( simpleBLETaskId, START_CONN_EVT, 2000 );
        }
        else
        {
          simpleBLEState = BLE_STATE_IDLE;
          simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
          simpleBLERssi = FALSE;
          simpleBLEDiscState = BLE_DISC_STATE_IDLE;
          
          LCD_WRITE_STRING( "Connect Failed", HAL_LCD_LINE_1 );
          LCD_WRITE_STRING_VALUE( "Reason:", pEvent->gap.hdr.status, 10, HAL_LCD_LINE_2 );
          HalUARTWrite(HAL_UART_PORT_0, "conn_fail\r\n", 11);
        }
        
//        // connect next device
//        osal_start_timerEx( simpleBLETaskId, START_CONN_EVT, 2000 );
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        simpleBLEState = BLE_STATE_IDLE;
        simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
        simpleBLERssi = FALSE;
        simpleBLEDiscState = BLE_DISC_STATE_IDLE;
        simpleBLECharHdl = 0;
//        simpleBLEProcedureInProgress = FALSE;
          
        LCD_WRITE_STRING( "Disconnected", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING_VALUE( "Reason:", pEvent->linkTerminate.reason,
                                10, HAL_LCD_LINE_2 );
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        LCD_WRITE_STRING( "Param Update", HAL_LCD_LINE_1 );
        HalUARTWrite(HAL_UART_PORT_0, "Param Update\r\n", 14);
      }
      break;
      
    default:
      break;
  }
  
  return ( TRUE );
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    LCD_WRITE_STRING( "Pairing started", HAL_LCD_LINE_1 );
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
      LCD_WRITE_STRING( "Pairing success", HAL_LCD_LINE_1 );
    }
    else
    {
      LCD_WRITE_STRING_VALUE( "Pairing fail", status, 10, HAL_LCD_LINE_1 );
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
      LCD_WRITE_STRING( "Bonding success", HAL_LCD_LINE_1 );
    }
  }
}

/*********************************************************************
 * @fn      simpleBLECentralPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
#if (HAL_LCD == TRUE)

  uint32  passcode;
  uint8   str[7];

  // Create random passcode
  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  passcode %= 1000000;
  
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
    LCD_WRITE_STRING( "Passcode:",  HAL_LCD_LINE_1 );
    LCD_WRITE_STRING( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
#endif
}

/*********************************************************************
 * @fn      simpleBLECentralStartDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void simpleBLECentralStartDiscovery( void )
{
  if(uuid128_flag == TRUE)
  {
    uint8 uuid[ATT_UUID_SIZE] = {0};//SIMPLEPROFILE_SERV_UUID
    for(uint8 i = 0; i < 16; i++)
    {
      uuid[i] = Service_UUID[15 - i];
    }
    // Initialize cached handles
    simpleBLESvcStartHdl = simpleBLESvcEndHdl = simpleBLECharHdl = 0;
    
    simpleBLEDiscState = BLE_DISC_STATE_SVC;
    
    // Discovery simple BLE service
    GATT_DiscPrimaryServiceByUUID( simpleBLEConnHandle,
                                  uuid,
                                  ATT_UUID_SIZE,//ATT_BT_UUID_SIZE,
                                  simpleBLETaskId );
  }
  else
  {
    uint8 uuid[ATT_BT_UUID_SIZE] = {0};//SIMPLEPROFILE_SERV_UUID
    for(uint8 i = 0; i < ATT_BT_UUID_SIZE; i++)
    {
      uuid[i] = Service_UUID[ATT_BT_UUID_SIZE -1 - i];
    }
    // Initialize cached handles
    simpleBLESvcStartHdl = simpleBLESvcEndHdl = simpleBLECharHdl = 0;
    
    simpleBLEDiscState = BLE_DISC_STATE_SVC;
    
    // Discovery simple BLE service
    GATT_DiscPrimaryServiceByUUID( simpleBLEConnHandle,
                                  uuid,
                                  ATT_BT_UUID_SIZE,
                                  simpleBLETaskId );
  }
}

/*********************************************************************
 * @fn      simpleBLEGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{
  attReadByTypeReq_t req;
  
  if ( simpleBLEDiscState == BLE_DISC_STATE_SVC )
  {
    // Service found, store handles
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
         pMsg->msg.findByTypeValueRsp.numInfo > 0 )
    {
      simpleBLESvcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      simpleBLESvcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
    }
    
    // If procedure complete
    if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
           pMsg->hdr.status == bleProcedureComplete ) ||
         ( pMsg->method == ATT_ERROR_RSP ) )
    {
      if ( simpleBLESvcStartHdl != 0 )
      {
        // Discover characteristic
        simpleBLEDiscState = BLE_DISC_STATE_CHAR;
        
        req.startHandle = simpleBLESvcStartHdl;
        req.endHandle = simpleBLESvcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        GATT_ReadUsingCharUUID( simpleBLEConnHandle, &req, simpleBLETaskId );
      }
    }
  }
  else if ( simpleBLEDiscState == BLE_DISC_STATE_CHAR )
  {
    // Characteristic found, store handle
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP && 
         pMsg->msg.readByTypeRsp.numPairs > 0 )
    {
      simpleBLECharHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                                      pMsg->msg.readByTypeRsp.pDataList[1]);
      
      LCD_WRITE_STRING( "Simple Svc Found", HAL_LCD_LINE_1 );
//      simpleBLEProcedureInProgress = FALSE;
    }
    
    simpleBLEDiscState = BLE_DISC_STATE_IDLE;

    
  }    
}


/*********************************************************************
 * @fn      simpleBLEFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
{
  uint8 adLen;
  uint8 adType;
  uint8 *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while ( pData < pEnd )
  {
    // Get length of next AD item
    adLen = *pData++;
    if ( adLen > 0 )
    {
      adType = *pData;
      
      // If AD type is for 16-bit service UUID
      if ( adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE )
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while ( adLen >= 2 && pData < pEnd )
        {
          // Check for match
          if ( pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid) )
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if ( adLen == 1 )
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }
  
  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
  uint8 i;
  
  // If result count not at max
  if ( simpleBLEScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < simpleBLEScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, simpleBLEDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }
    
    // Add addr to scan result list
    osal_memcpy( simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN );
    simpleBLEDevList[simpleBLEScanRes].addrType = addrType;
    
    // Increment scan result count
    simpleBLEScanRes++;
  }
}

//devInfo_t dev = 
//{
//  .hasDataFlag = FALSE,
//  
//};
static void getNameAndData(uint8 *pAddr, uint8 addrType, uint8 dataLen, uint8 *data)
{
  uint8 structLen = 0;
  uint8 structType = 0;
  uint8 *structData = NULL;
  for(uint8 i = 0; i < dataLen; )
  {
    structLen = data[i];
    structType = data[i+1];
    structData = &data[i+2];
    if((structLen != 0)&&(structLen < dataLen))
    {
      switch(structType)
      {
      case GAP_ADTYPE_LOCAL_NAME_COMPLETE:
        {
          uint8 *name = "MKR";
          if ( osal_memcmp( name, structData , 3 ) )
          {
            osal_memcpy( dev.addr, pAddr, B_ADDR_LEN );
            osal_memcpy( dev.name, structData, structLen -1 );
          }
        }break;
      case GAP_ADTYPE_MANUFACTURER_SPECIFIC:
        {
          uint8 userID[2] = {0x00,0x01};
          if ( osal_memcmp( userID, &structData[2] , 2 ) )
          {
            osal_memcpy( dev.addr, pAddr, B_ADDR_LEN );
            osal_memcpy( dev.manuData, &structData[2], structLen -3 );
            dev.hasDataFlag = TRUE;
          }
          
        }break;
      default:
        break;
      }
    }
    if(structLen == 0)
    {
      i++;
    }
    else
    {
      i += structLen +1;
    }
    
  }
}

#define FRAME_LEN               14
#define FRAME_HEAD_OFF          0
#define FRAME_LEN_OFF           2
#define FRAME_PROTOCOL_NUM_OFF  3
#define FRAME_TEMPER_OFF        4
#define FRAME_RH_OFF            6
#define FRAME_BATTERY_OFF       8
#define FRAME_LOW_POWER_OFF     9
#define FRAME_ERROR_OFF         10
#define FRAME_CRC_OFF           11
#define FRAME_END_OFF           12
void packAndSend(void)
{
  uint8 data[FRAME_LEN] = {0x18,0x18,0x00,0x01};
  data[FRAME_LEN_OFF] = 0x08;
  data[FRAME_TEMPER_OFF] = dev.manuData[0];
  data[FRAME_TEMPER_OFF +1] = dev.manuData[0];
  data[FRAME_RH_OFF] = dev.manuData[0];
  data[FRAME_RH_OFF +1] = dev.manuData[0];
  data[FRAME_BATTERY_OFF] = dev.manuData[0];
  if(data[FRAME_BATTERY_OFF] > 20)
    data[FRAME_LOW_POWER_OFF] = 0;//Õý³£
  else
    data[FRAME_LOW_POWER_OFF] = 1;//Ç·µç
  if(dev.hasDataFlag)
    data[FRAME_ERROR_OFF] = 0;//Õý³£
  else
    data[FRAME_ERROR_OFF] = 1;//±ùÏäÄÚÍâÄ£¿éÍ¨ÐÅÒì³£
  data[FRAME_CRC_OFF] = data[FRAME_PROTOCOL_NUM_OFF];
  for(uint8 i = 0; i < data[FRAME_LEN_OFF]; i++)
  {
    data[FRAME_CRC_OFF] ^= data[FRAME_TEMPER_OFF + i];
  }
  data[FRAME_END_OFF] = 0x0D;
  data[FRAME_END_OFF +1] = 0x0A;
  
  HalUARTWrite(HAL_UART_PORT_0, data, FRAME_LEN);
}
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}


void SerialPacketPort0(uint8 port, uint8 event)
{
  uint8 buffer[310];
  uint16 numBytes;
  (void)event;
  // check if there's any serial port data to process
  if (Hal_UART_RxBufLen(port) > 0 )
  {
    // Note: Assumes we'll get the data indicated by Hal_UART_RxBufLen.
    numBytes = HalUARTRead (port, buffer, 310);
    for(uint16 Cnt = 0; Cnt < numBytes; Cnt ++)
    {
      Msp_Buffer[Fi_Ptr] = buffer[Cnt];
      Fi_Ptr ++;
      if(Fi_Ptr >= RCV_MAX)
        Fi_Ptr = 0;
    }
//    Extract_Command();
//    /**************2013.01.14,19:30,ÐÞ¸Ä£¬Ô­Òò£¬¸Ä³ÉÈ«¾Ö±äÁ¿******************/
//    //uint8 Record_tmp = Command_Record_Cnt;
//    Record_tmp = Command_Record_Cnt;
//    /*ÅÐ¶ÏÊÇ·ñÓÐÁ½ÌõÖ¸Áî£¬²¢Æô¶¯Ö´ÐÐ
//    */
//    if(Command_Record[Record_tmp][1] == 1)//ÅÐ¶Ï×îºóÒ»¸öÀïÓÐÃüÁî
//    {
//      if(Record_tmp == 0)//½«Record_tmp·­×ª
//        Record_tmp = 1;
//      else
//        Record_tmp = 0;
//
//      if(Command_Record[Record_tmp][1] == 1)//ÅÐ¶ÏÇ°Ò»¸öbufÀïÒ²ÓÐÃüÁî
//      {
//        Twice_Flag = 1;//¸æËßUART_CMD_EXE_EVTÒªÖ´ÐÐÁ½´Î
//      }
//      else//Ç°Ò»¸öbufÀïÃ»ÓÐÃüÁî£¬¹ÊÓ¦¸Ã½«Record_tmp·­×ª»ØÈ¥
//      {
//        Twice_Flag = 0;//¸æËßUART_CMD_EXE_EVT½öÖ´ÐÐÒ»´Î¼´¿É
//
//        if(Record_tmp == 0)//½«Record_tmp·­×ª»ØÈ¥
//          Record_tmp = 1;
//        else
//          Record_tmp = 0;
//      }
//      osal_set_event(activityMonitor_TaskID, UART_EXTRACT_CMD_EVT);
//    
//    }
    osal_start_timerEx(simpleBLETaskId,UART_EXTRACT_CMD_EVT,5);
  }
}

/*********************************************************************
* @fn      Minus_PtrMv
*
* @brief   caculate D-value current point with Minnd
*
* @param   uint8 Current_Ptr,uint8 Minnd
*
* @return  void
*/
uint16 Minus_PtrMv(uint16 Current_Ptr,uint16 Minnd)
{
  uint16 Stp_Mv;
  if (Current_Ptr >= Minnd)
    Stp_Mv = Current_Ptr - Minnd;
  else
    Stp_Mv = Current_Ptr + RCV_MAX - Minnd;
  return Stp_Mv;
}

/*********************************************************************
* @fn      Plus_PtrMv
*
* @brief   caculate SUM current point with Minnd
*
* @param   uint8 Current_Ptr,uint8 addnd
*
* @return  void
*/
uint16 Plus_PtrMv(uint16 Current_Ptr,uint16 addnd)
{
  uint16 Stp_Mv;
  Stp_Mv = (Current_Ptr + addnd) % RCV_MAX;
  return Stp_Mv;
}

/*********************************************************************
*********************************************************************/
