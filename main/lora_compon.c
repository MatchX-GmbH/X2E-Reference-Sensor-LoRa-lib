//==========================================================================
// LoRa Application Layer
//==========================================================================
//  Copyright (c) MatchX GmbH.  All rights reserved.
//==========================================================================
// Naming conventions
// ~~~~~~~~~~~~~~~~~~
//                Class : Leading C
//               Struct : Leading T
//       typedef Struct : tailing _t
//             Constant : Leading k
//      Global Variable : Leading g
//    Function argument : Leading a
//       Local Variable : All lower case
//==========================================================================

#include "lora_compon.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "LoRaCompon_debug.h"
#include "LoRaMac.h"
#include "board.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lora_crc.h"
#include "lora_mutex_helper.h"
#include "radio.h"
#include "timer.h"

//==========================================================================
//==========================================================================
#define TASK_PRIO_GENERAL 8
#define DelayMs(x) vTaskDelay(x / portTICK_PERIOD_MS)

//==========================================================================
//==========================================================================
#define LORAWAN_PUBLIC_NETWORK true

// Active LoRa Region
#if defined(REGION_CN470)
#define SUB_GHZ_REGION LORAMAC_REGION_CN470
#define SUB_GHZ_REGION_NAME "CN470"
#elif defined(REGION_EU868)
#define SUB_GHZ_REGION LORAMAC_REGION_EU868
#define SUB_GHZ_REGION_NAME "EU868"
#elif defined(REGION_US915)
#define SUB_GHZ_REGION LORAMAC_REGION_US915
#define SUB_GHZ_REGION_NAME "US915"
#elif defined(REGION_KR920)
#define SUB_GHZ_REGION LORAMAC_REGION_KR920
#define SUB_GHZ_REGION_NAME "KR920"
#elif defined(REGION_AS923)
#define SUB_GHZ_REGION LORAMAC_REGION_AS923
#define SUB_GHZ_REGION_NAME "AS923"
#elif defined(REGION_AU915)
#define SUB_GHZ_REGION LORAMAC_REGION_AU915
#define SUB_GHZ_REGION_NAME "AU915"
#elif defined(REGION_IN865)
#define SUB_GHZ_REGION LORAMAC_REGION_IN865
#define SUB_GHZ_REGION_NAME "IN865"
#else
#error "Please define the target region."
#define SUB_GHZ_REGION_NAME ""
#endif

// Kconfig
#if defined(CONFIG_LORAWAN_PREFERRED_ISM2400)
#define LORAWAN_PREFERRED_ISM2400 1
#define LORAWAN_USING_ISM2400 true
#else
#define LORAWAN_PREFERRED_ISM2400 0
#define LORAWAN_USING_ISM2400 false
#endif

#if defined(CONDIF_LORAWAN_PREFERRED_SUBGHZ)
#define LORAWAN_PREFERRED_SUBGHZ 1
#else
#define LORAWAN_PREFERRED_SUBGHZ 0
#endif

#if defined(CONFIG_LORAWAN_DEFAULT_DATARATE)
#define LORAWAN_DEFAULT_DATARATE CONFIG_LORAWAN_DEFAULT_DATARATE
#else
#define LORAWAN_DEFAULT_DATARATE DR_3
#endif

#if defined(CONFIG_LORAWAN_SW_RADIO_COUNT)
#define LORAWAN_SW_RADIO_COUNT CONFIG_LORAWAN_SW_RADIO_COUNT
#else
#define LORAWAN_SW_RADIO_COUNT 2
#endif

#if defined(CONFIG_LORAWAN_MAX_NOACK_RETRY)
#define LORAWAN_MAX_NOACK_RETRY CONFIG_LORAWAN_MAX_NOACK_RETRY
#else
#define LORAWAN_MAX_NOACK_RETRY 2
#endif

#if defined(CONFIG_LORAWAN_NOACK_RETRY_INTERVAL)
#define LORAWAN_NOACK_RETRY_INTERVAL (CONFIG_LORAWAN_NOACK_RETRY_INTERVAL * 1000)
#else
#define LORAWAN_NOACK_RETRY_INTERVAL (20 * 1000)
#endif

#if defined(CONFIG_LORAWAN_LINK_FAIL_COUNT)
#define LORAWAN_LINK_FAIL_COUNT CONFIG_LORAWAN_LINK_FAIL_COUNT
#else
#define LORAWAN_LINK_FAIL_COUNT 8
#endif

#if defined(CONFIG_LORAWAN_UNCONFIRMED_COUNT)
#define LORAWAN_UNCONFIRMED_COUNT CONFIG_LORAWAN_UNCONFIRMED_COUNT
#else
#define LORAWAN_UNCONFIRMED_COUNT 0
#endif

//
#if defined(REGION_EU868) || defined(REGION_RU864) || defined(REGION_CN779) || defined(REGION_EU433)
#include "LoRaMacTest.h"
#define LORAWAN_DUTYCYCLE_ON true
#endif

// Default datarate
#define LORAWAN_HELLO_DATARATE DR_3
#define LORAWAN_JOIN_DR_MIN DR_0
#define LORAWAN_JOIN_DR_MAX LORAWAN_DEFAULT_DATARATE
#define LORAWAN_ISM2400_DATARATE DR_3

// LoRaWAN Adaptive Data Rate
//  remark Please note that when ADR is enabled the end-device should be static
#if defined(CONFIG_LORAWAN_ADR_ON)
#define LORAWAN_ADR_ON true
#else
#define LORAWAN_ADR_ON false
#endif

#define LORAWAN_FPORT_DATA 2

// ms
#define TIME_JOIN_INTERVAL_MIN 90000
#define TIME_JOIN_INTERVAL_MAX 120000

#define TIME_TXCHK_INTERVAL 10000

#define TIME_PROV_INTERVAL_MIN 1    // 60000
#define RAND_RANGE_PROV_INTERVAL 1  // 30000

#define TIME_PROVISIONING_TIMEOUT 10000
#define TIME_PROVISIONING_AUTHWAIT 5000
#define TIMEOUT_SEND_WAITING 17500

/*!
 * Device states
 */
typedef enum {
  S_LORALINK_INIT,
  S_LORALINK_PROVISIONING_START,
  S_LORALINK_PROVISIONING_HELLO,
  S_LORALINK_PROVISIONING_AUTH,
  S_LORALINK_PROVISIONING_WAIT,
  S_LORALINK_JOIN,
  S_LORALINK_JOIN_WAIT,
  S_LORALINK_JOINED,
  S_LORALINK_SEND,
  S_LORALINK_SEND_MAC,
  S_LORALINK_SEND_WAITING,
  S_LORALINK_SEND_SUCCESS,
  S_LORALINK_SEND_FAILURE,
  S_LORALINK_RETRY_WAITING,
  S_LORALINK_WAITING,
  S_LORALINK_SLEEP,
  S_LORALINK_WAKEUP,
} LoraDevicState_t;
//
static LoraDevicState_t gLoraLinkState;
static volatile uint32_t gTickLoraLink;

// LoRaWAN compliance tests support data
typedef struct {
  bool Running;
  uint8_t State;
  bool IsTxConfirmed;
  uint8_t AppPort;
  uint8_t AppDataSize;
  uint8_t *AppDataBuffer;
  uint16_t DownLinkCounter;
  bool LinkCheck;
  uint8_t DemodMargin;
  uint8_t NbGateways;
} ComplianceTest_t;

// typedef void(loramac_cb_t)(void);
// static loramac_cb_t *gp_loramac_cb;
static TaskHandle_t gLoRaTaskHandle = NULL;
static bool gLoRaTaskAbort;

//
typedef struct TLoraAppData {
  uint8_t *data;
  int16_t dataSize;
  uint8_t port;
  uint8_t retry;
} LoraAppData_t;

// Link status bits
static volatile uint32_t gLinkStatus;
#define BIT_LORASTATUS_ERROR 0x8000
#define BIT_LORASTATUS_JOIN_PASS 0x0001
#define BIT_LORASTATUS_JOIN_FAIL 0x0100
#define BIT_LORASTATUS_SEND_PASS 0x0002
#define BIT_LORASTATUS_SEND_FAIL 0x0200
#define BIT_LORASTATUS_TX_RDY 0x0004
#define BIT_LORASTATUS_RX_RDY 0x0008
#define BIT_LORASTATUS_DEV_PROV 0x0080

// Provisioning status bits
static uint32_t gProvisionStatus;
#define BIT_PROV_HELLO_OK 0x01
#define BIT_PROV_AUTH_OK 0x02

//
static uint8_t gTxBuf[LORAWAN_MAX_PAYLOAD_LEN];
static LoraAppData_t gTxData = {.data = gTxBuf, .dataSize = -1, .port = LORAWAN_FPORT_DATA, .retry = 0};

static uint8_t gRxBuf[LORAWAN_MAX_PAYLOAD_LEN];
static LoraAppData_t gRxData = {.data = gRxBuf, .dataSize = -1, .port = 0, .retry = 0};

static LoRaMacPrimitives_t gLoRaMacPrimitives;
static LoRaMacCallback_t gLoRaMacCallbacks;

static int16_t gLastRxRssi;
static uint8_t gLastRxDatarate;

typedef struct {
  uint32_t ackCount;
  uint32_t nakCount;
  int32_t failCount;
  uint32_t joinInterval;
  uint8_t joinRetryTimes;
  uint8_t batteryValue;
  int8_t dateRate;
  bool usingIsm2400;
  bool txConfirmed;
  uint16_t unconfigmedCount;
} LoRaLinkVar_t;
static LoRaLinkVar_t gLoRaLinkVar;

static LoRaSetting_t gLoRaSetting;

static bool gWakeFromSleep;

// Preserved data when sleep
#define VALUE_PRESERVED_DATA_CRC_IV 0x1234
#define VALUE_PRESERVED_DATA_MAGIC_CODE 0x48ad3f56

typedef struct {
  uint32_t magicCode;
  LoRaMacNvmData_t contexts;
  LoRaLinkVar_t linkVar;
} LoRaPreservedData_t;

static RTC_DATA_ATTR LoRaPreservedData_t gLoRaPreservedData;
static RTC_DATA_ATTR uint16_t gLoRaPreservedDataCrc;

//==========================================================================
// MAC status strings
//==========================================================================
static const char *kMacStatusStrings[] = {
    "OK",                             // LORAMAC_STATUS_OK
    "Busy",                           // LORAMAC_STATUS_BUSY
    "Service unknown",                // LORAMAC_STATUS_SERVICE_UNKNOWN
    "Parameter invalid",              // LORAMAC_STATUS_PARAMETER_INVALID
    "Frequency invalid",              // LORAMAC_STATUS_FREQUENCY_INVALID
    "Datarate invalid",               // LORAMAC_STATUS_DATARATE_INVALID
    "Frequency or datarate invalid",  // LORAMAC_STATUS_FREQ_AND_DR_INVALID
    "No network joined",              // LORAMAC_STATUS_NO_NETWORK_JOINED
    "Length error",                   // LORAMAC_STATUS_LENGTH_ERROR
    "Region not supported",           // LORAMAC_STATUS_REGION_NOT_SUPPORTED
    "Skipped APP data",               // LORAMAC_STATUS_SKIPPED_APP_DATA
    "Duty-cycle restricted",          // LORAMAC_STATUS_DUTYCYCLE_RESTRICTED
    "No channel found",               // LORAMAC_STATUS_NO_CHANNEL_FOUND
    "No free channel found",          // LORAMAC_STATUS_NO_FREE_CHANNEL_FOUND
    "Busy beacon reserved time",      // LORAMAC_STATUS_BUSY_BEACON_RESERVED_TIME
    "Busy ping-slot window time",     // LORAMAC_STATUS_BUSY_PING_SLOT_WINDOW_TIME
    "Busy uplink collision",          // LORAMAC_STATUS_BUSY_UPLINK_COLLISION
    "Crypto error",                   // LORAMAC_STATUS_CRYPTO_ERROR
    "FCnt handler error",             // LORAMAC_STATUS_FCNT_HANDLER_ERROR
    "MAC command error",              // LORAMAC_STATUS_MAC_COMMAD_ERROR
    "ClassB error",                   // LORAMAC_STATUS_CLASS_B_ERROR
    "Confirm queue error",            // LORAMAC_STATUS_CONFIRM_QUEUE_ERROR
    "Multicast group undefined",      // LORAMAC_STATUS_MC_GROUP_UNDEFINED
    "Unknown error",                  // LORAMAC_STATUS_ERROR
};
static const char *getMacStatusString(uint8_t aStatus) {
  if (aStatus >= LORAMAC_STATUS_ERROR) {
    return kMacStatusStrings[LORAMAC_STATUS_ERROR];
  } else {
    return kMacStatusStrings[aStatus];
  }
}

//==========================================================================
// MAC event status strings
//==========================================================================
static const char *kMacEventStatusStrings[] = {
    "OK",                   // LORAMAC_EVENT_INFO_STATUS_OK
    "Status Error",         // LORAMAC_EVENT_INFO_STATUS_ERROR,
    "TX timeout",           // LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT,
    "RX1 timeout",          // LORAMAC_EVENT_INFO_STATUS_RX1_TIMEOUT,
    "RX2 timeout",          // LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT,
    "RX1 error",            // LORAMAC_EVENT_INFO_STATUS_RX1_ERROR,
    "RX2 error",            // LORAMAC_EVENT_INFO_STATUS_RX2_ERROR,
    "Join fail",            // LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL,
    "Downlink repeated",    // LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED,
    "TX size error as DR",  // LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR,
    "Address error",        // LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL,
    "MIC fail",             // LORAMAC_EVENT_INFO_STATUS_MIC_FAIL,
    "Multicast faile",      // LORAMAC_EVENT_INFO_STATUS_MULTICAST_FAIL,
    "Beacon locked",        // LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED,
    "Beacon lost",          // LORAMAC_EVENT_INFO_STATUS_BEACON_LOST,
    "Beacon not found",     // LORAMAC_EVENT_INFO_STATUS_BEACON_NOT_FOUND,
};
static const char *getMacEventStatusString(uint8_t aStatus) {
  if (aStatus > LORAMAC_EVENT_INFO_STATUS_BEACON_NOT_FOUND) {
    return kMacEventStatusStrings[LORAMAC_EVENT_INFO_STATUS_ERROR];
  } else {
    return kMacEventStatusStrings[aStatus];
  }
}

//==========================================================================
// Battery value for MAC DevStatusReq/DevStatusAns
//==========================================================================
static uint8_t GetBatteryLevel(void) { return gLoRaLinkVar.batteryValue; }

//==========================================================================
// ProvisioningHello
//==========================================================================
static void ProvisioningHello(void) {
  LORACOMPON_PRINTLINE("ProvisioningHello()");

  // //
  // LoRaMacStatus_t status;
  // MlmeReq_t mlmeReq;
  // mlmeReq.Type = MLME_PROPRIETARY;
  // mlmeReq.Req.Proprietary.Datarate = LORAWAN_HELLO_DATARATE;

  // uint8_t txlen = DevProvisionPrepareHello(gTxBuf, sizeof(gTxBuf));
  // if (txlen == 0) {
  //   printf("ERROR. ProvisioningHello(): TX buf too small.\n");
  // } else {
  //   mlmeReq.Req.Proprietary.Payload = gTxBuf;
  //   mlmeReq.Req.Proprietary.PayloadLen = txlen;

  //   // Starts the Proprietary procedure
  //   status = LoRaMacMlmeRequest(&mlmeReq);
  //   LORACOMPON_PRINTLINE("MLME-Request - MLME_PROPRIETARY");
  //   LORACOMPON_PRINTLINE("  STATUS: %s", getMacStatusString(status));

  //   if (status == LORAMAC_STATUS_OK) {
  //     LORACOMPON_PRINTLINE("  SUCCESS");
  //   } else {
  //     if (status == LORAMAC_STATUS_DUTYCYCLE_RESTRICTED) {
  //       LORACOMPON_PRINTLINE("  Next Tx in: %u [ms]", (unsigned int)mlmeReq.ReqReturn.DutyCycleWaitTime);
  //     }
  //   }
  // }
}

//==========================================================================
// ProvisioningAuth
//==========================================================================
static void ProvisioningAuth(void) {
  LORACOMPON_PRINTLINE("ProvisioningAuth()");

  //
  // LoRaMacStatus_t status;
  // MlmeReq_t mlmeReq;
  // mlmeReq.Type = MLME_PROPRIETARY;
  // mlmeReq.Req.Proprietary.Datarate = LORAWAN_HELLO_DATARATE;

  // uint8_t txlen = DevProvisionPrepareAuth(gTxBuf, sizeof(gTxBuf));

  // mlmeReq.Req.Proprietary.Payload = gTxBuf;
  // mlmeReq.Req.Proprietary.PayloadLen = txlen;

  // // Starts the Proprietary procedure
  // status = LoRaMacMlmeRequest(&mlmeReq);

  // LORACOMPON_PRINTLINE("MLME-Request - MLME_PROPRIETARY");
  // LORACOMPON_PRINTLINE("  STATUS: %s", getMacStatusString(status));

  // if (status == LORAMAC_STATUS_OK) {
  //   LORACOMPON_PRINTLINE("  SUCCESS");
  // } else {
  //   if (status == LORAMAC_STATUS_DUTYCYCLE_RESTRICTED) {
  //     LORACOMPON_PRINTLINE("  Next Tx in: %u [ms]", (unsigned int)mlmeReq.ReqReturn.DutyCycleWaitTime);
  //   }
  // }
}

//==========================================================================
// Return: true - send failed
//==========================================================================
static int8_t sendFrame(void) {
  MibRequestConfirm_t mibReq;
  McpsReq_t mcpsReq;
  LoRaMacStatus_t ret_mac;
  LoRaMacTxInfo_t txInfo;

  // Set Datarate
  if ((!LORAWAN_ADR_ON) || (gLoRaLinkVar.usingIsm2400)) {
    mibReq.Type = MIB_CHANNELS_DATARATE;
    mibReq.Param.ChannelsDatarate = gLoRaLinkVar.dateRate;
    LoRaMacMibSetRequestConfirm(&mibReq);
  }

  // Check frame size
  ret_mac = LoRaMacQueryTxPossible(gTxData.dataSize, &txInfo);
  if (ret_mac != LORAMAC_STATUS_OK) {
    // LORACOMPON_PRINTLINE("CurrentPossiblePayloadSize=%d", txInfo.CurrentPossiblePayloadSize);
    // LORACOMPON_PRINTLINE("MaxPossibleApplicationDataSize=%d", txInfo.MaxPossibleApplicationDataSize);
    if (ret_mac == LORAMAC_STATUS_LENGTH_ERROR) {
      uint8_t max_size = txInfo.CurrentPossiblePayloadSize;
      // Try set datarate
      LORACOMPON_PRINTLINE("Payload too large, try default DR%d", gLoRaLinkVar.dateRate);
      mibReq.Type = MIB_CHANNELS_DATARATE;
      mibReq.Param.ChannelsDatarate = gLoRaLinkVar.dateRate;
      LoRaMacMibSetRequestConfirm(&mibReq);
      if (LoRaMacQueryTxPossible(gTxData.dataSize, &txInfo) != LORAMAC_STATUS_OK) {
        printf("ERROR. Payload is too large at current datarate. Max is %d.\n", txInfo.CurrentPossiblePayloadSize);
        return -1;
      }
      printf("WARNING. Changed to default DR%d due to data size>%d\n", gLoRaLinkVar.dateRate, max_size);

    } else {
      printf("ERROR. LoRaMacQueryTxPossible() failed, %s\n", getMacStatusString(ret_mac));
      return -1;
    }
  }

  if ((gLoRaLinkVar.txConfirmed) && (gTxData.dataSize > 0)) {
    mcpsReq.Type = MCPS_CONFIRMED;
    mcpsReq.Req.Confirmed.fPort = gTxData.port;
    mcpsReq.Req.Confirmed.fBuffer = gTxData.data;
    mcpsReq.Req.Confirmed.fBufferSize = gTxData.dataSize;
    //    mcpsReq.Req.Confirmed.NbTrials = g_data_send_nbtrials ? g_data_send_nbtrials : gMacConfig->nbtrials.conf + 1;
    mcpsReq.Req.Confirmed.Datarate = gLoRaLinkVar.dateRate;
  } else {
    // MibRequestConfirm_t mibReq;
    // mibReq.Type = MIB_CHANNELS_NB_TRANS;
    // mibReq.Param.ChannelsNbTrans = g_data_send_nbtrials ? g_data_send_nbtrials : gMacConfig->nbtrials.unconf + 1;
    // LoRaMacMibSetRequestConfirm(&mibReq);

    mcpsReq.Type = MCPS_UNCONFIRMED;
    mcpsReq.Req.Unconfirmed.fPort = gTxData.port;
    mcpsReq.Req.Unconfirmed.fBuffer = gTxData.data;
    mcpsReq.Req.Unconfirmed.fBufferSize = gTxData.dataSize;
    mcpsReq.Req.Unconfirmed.Datarate = gLoRaLinkVar.dateRate;
  }

  //
  ret_mac = LoRaMacMcpsRequest(&mcpsReq);
  if (ret_mac == LORAMAC_STATUS_OK) {
    return 0;
  } else {
    LORACOMPON_PRINTLINE("LoRaMacMcpsRequest() failed, %s", getMacStatusString(ret_mac));
    return -1;
  }
}

//==========================================================================
// MCPS-Confirm event function
//==========================================================================
static void McpsConfirm(McpsConfirm_t *mcpsConfirm) {
  if (mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
    LORACOMPON_PRINTLINE("McpsConfirm() OK");
    if (!gLoRaLinkVar.txConfirmed) {
      TakeMutex();
      gLinkStatus |= BIT_LORASTATUS_SEND_PASS;
      FreeMutex();
    }
  } else {
    printf("ERROR. McpsConfirm() failed. %s\n", getMacEventStatusString(mcpsConfirm->Status));
    TakeMutex();
    gLinkStatus |= BIT_LORASTATUS_SEND_FAIL;
    FreeMutex();
  }
  // LoRaComponNotify(EVENT_NOTIF_LORAMAC, NULL);
}

//==========================================================================
// MCPS-Indication event function
//==========================================================================
static void McpsIndication(McpsIndication_t *mcpsIndication) {
  if (mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
    printf("ERROR. McpsIndication() failed. %s\n", getMacEventStatusString(mcpsIndication->Status));
    return;
  }
  // Check Multicast
  // Check Port
  // Check Datarate
  // Check FramePending
  // Check Buffer
  // Check BufferSize
  // Check Rssi
  // Check Snr
  // Check RxSlot
  LORACOMPON_PRINTLINE("rx frame: rssi=%d, snr=%d, dr=%d", mcpsIndication->Rssi, mcpsIndication->Snr, mcpsIndication->RxDatarate);
  gLastRxRssi = mcpsIndication->Rssi;
  gLastRxDatarate = mcpsIndication->RxDatarate;
  if (mcpsIndication->RxData == true) {
    switch (mcpsIndication->Port) {
      case 224:
        break;
      default: {
        TakeMutex();
        gRxData.port = mcpsIndication->Port;
        gRxData.dataSize = mcpsIndication->BufferSize;
        memcpy1(gRxData.data, mcpsIndication->Buffer, gRxData.dataSize);
        FreeMutex();
        LORACOMPON_PRINTLINE("Rxed %d bytes.", gRxData.dataSize);
        break;
      }
    }
  }
  if (mcpsIndication->AckReceived) {
    // ACK
    gLoRaLinkVar.ackCount++;
    LORACOMPON_PRINTLINE("AckReceived, ackCount=%u", gLoRaLinkVar.ackCount);
    if (gLoRaLinkVar.txConfirmed) {
      TakeMutex();
      gLinkStatus |= BIT_LORASTATUS_SEND_PASS;
      FreeMutex();
    }
  }

  // LoRaComponNotify(EVENT_NOTIF_LORAMAC, NULL);
}

//==========================================================================
// MLME-Confirm event function
//==========================================================================
static void MlmeConfirm(MlmeConfirm_t *mlmeConfirm) {
  LORACOMPON_PRINTLINE("MLME-Confirm");
  LORACOMPON_PRINTLINE("  STATUS: %s", getMacEventStatusString(mlmeConfirm->Status));
  switch (mlmeConfirm->MlmeRequest) {
    case MLME_JOIN: {
      if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
        // Status is OK, node has joined the network
        TakeMutex();
        gLinkStatus |= BIT_LORASTATUS_JOIN_PASS;
        FreeMutex();
      } else {
        LORACOMPON_PRINTLINE("Join failed.");
        TakeMutex();
        gLinkStatus |= BIT_LORASTATUS_JOIN_FAIL;
        FreeMutex();
      }
      break;
    }
    // case MLME_PROPRIETARY: {
    //   if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
    //     MibRequestConfirm_t mibGet;
    //     LORACOMPON_PRINTLINE("  PROPRIETARY RXED");
    //     LORACOMPON_PRINTLINE("  RSSI: %d", mlmeConfirm->ProprietaryRssi);
    //     LORACOMPON_PRINTLINE("  MIC=%08X (%d)", (unsigned int)mlmeConfirm->ProprietaryMic, mlmeConfirm->ProprietaryMicCorrect);
    //     LORACOMPON_PRINTLINE("  PayloadLen=%d", mlmeConfirm->ProprietaryPayloadLen);
    //     if (mlmeConfirm->ProprietaryPayloadLen > 0) {
    //       LORACOMPON_HEX2STRING("Payload:", mlmeConfirm->ProprietaryPayload, mlmeConfirm->ProprietaryPayloadLen);
    //     }
    //     mibGet.Type = MIB_CHANNELS_DATARATE;
    //     LoRaMacMibGetRequestConfirm(&mibGet);
    //     LORACOMPON_PRINTLINE("  DATA RATE: DR_%d", mibGet.Param.ChannelsDatarate);

    //     // Check resp
    //     const uint8_t *resp = mlmeConfirm->ProprietaryPayload;
    //     if (!mlmeConfirm->ProprietaryMicCorrect) {
    //       LORACOMPON_PRINTLINE("  Wrong MIC, response dropped.");
    //     } else if ((mlmeConfirm->ProprietaryPayloadLen == SIZE_DOWN_RESP_HELLO) && (resp[0] == DOWN_RESP_HELLO)) {
    //       // Hello Response
    //       if (DevProvisionHelloResp(resp, mlmeConfirm->ProprietaryPayloadLen) == 0) {
    //         LORACOMPON_PRINTLINE("  Hello response got.");
    //         gProvisionStatus |= BIT_PROV_HELLO_OK;
    //       }
    //     } else if ((mlmeConfirm->ProprietaryPayloadLen == SIZE_DOWN_RESP_AUTH_ACCEPT) && (resp[0] == DOWN_RESP_AUTH_ACCEPT)) {
    //       // Auth Response
    //       if (DevProvisionAuthResp(resp, mlmeConfirm->ProprietaryPayloadLen) == 0) {
    //         LORACOMPON_PRINTLINE("  Auth accepted.");
    //         gProvisionStatus |= BIT_PROV_AUTH_OK;
    //       }
    //     } else if ((mlmeConfirm->ProprietaryPayloadLen == SIZE_DOWN_RESP_AUTH_REJECT) && (resp[0] == DOWN_RESP_AUTH_REJECT)) {
    //       if (DevProvisionCmpDevEui(&resp[1]) != 0) {
    //         // Mismatch devEUI
    //       } else {
    //         LORACOMPON_PRINTLINE("  Auth rejected.");
    //       }
    //     } else {
    //       LORACOMPON_PRINTLINE("Unknown proprietary frame.");
    //     }
    //   }
    //   break;
    // }
    case MLME_LINK_CHECK: {
      break;
    }
    case MLME_DEVICE_TIME: {
      break;
    }
    case MLME_BEACON_ACQUISITION: {
      break;
    }
    case MLME_PING_SLOT_INFO: {
      break;
    }
    default:
      break;
  }
  // LoRaComponNotify(EVENT_NOTIF_LORAMAC, NULL);
}

//==========================================================================
// MLME-Indication event function
//==========================================================================
static void MlmeIndication(MlmeIndication_t *mlmeIndication) {
  LORACOMPON_PRINTLINE("MLME-Indication");
  switch (mlmeIndication->MlmeIndication) {
    case MLME_BEACON_LOST: {
      break;
    }
    case MLME_BEACON: {
      break;
    }
    default:
      break;
  }
}

//==========================================================================
//==========================================================================
static void OnMacProcessNotify(void) {
  // LoRaComponNotify(EVENT_NOTIF_LORAMAC, NULL);
}

//==========================================================================
// Setup for OTAA
//==========================================================================
static void InitOtaa(void) {
  MibRequestConfirm_t mibReq;

  mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
  mibReq.Param.ChannelsDefaultDatarate = gLoRaLinkVar.dateRate;
  LoRaMacMibSetRequestConfirm(&mibReq);

  mibReq.Type = MIB_CHANNELS_DATARATE;
  mibReq.Param.ChannelsDatarate = gLoRaLinkVar.dateRate;
  LoRaMacMibSetRequestConfirm(&mibReq);

  // Setup Dev EUI
  mibReq.Type = MIB_DEV_EUI;
  mibReq.Param.DevEui = gLoRaSetting.devEui;
  LoRaMacMibSetRequestConfirm(&mibReq);

  mibReq.Type = MIB_JOIN_EUI;
  mibReq.Param.JoinEui = gLoRaSetting.joinEui;
  LoRaMacMibSetRequestConfirm(&mibReq);

  mibReq.Type = MIB_NWK_KEY;
  mibReq.Param.NwkKey = gLoRaSetting.nwkKey;
  LoRaMacMibSetRequestConfirm(&mibReq);

  mibReq.Type = MIB_APP_KEY;
  mibReq.Param.NwkKey = gLoRaSetting.appKey;
  LoRaMacMibSetRequestConfirm(&mibReq);

  LORACOMPON_PRINTLINE("InitOtaa()");
  LORACOMPON_HEX2STRING("DevEui:", gLoRaSetting.devEui, LORA_EUI_LENGTH);
  LORACOMPON_HEX2STRING("JoinEui:", gLoRaSetting.joinEui, LORA_EUI_LENGTH);
  LORACOMPON_HEX2STRING("NwkKey:", gLoRaSetting.nwkKey, LORA_KEY_LENGTH);
  LORACOMPON_HEX2STRING("AppKey:", gLoRaSetting.appKey, LORA_KEY_LENGTH);
}

//==========================================================================
// Init for Device Provisioning
//==========================================================================
// void CalVerifyCode(uint8_t *aDest, uint32_t aDestSize, const char *aProvisionId, const uint8_t *aNonce);

// static void InitDevProvision(void) {
//   LORACOMPON_PRINTLINE("InitDevProvision()");
//   LORACOMPON_PRINTLINE("PID=%s", gSystemSetting.provisionId);
//   LORACOMPON_HEX2STRING("Hash=", gSystemSetting.provisionIdHash, sizeof(gSystemSetting.provisionIdHash));

//   DevProvisionInit(gSystemSetting.provisionId, gSystemSetting.provisionIdHash);

//   // Setup key for MIC calculation
//   MibRequestConfirm_t mibReq;
//   uint8_t app_key[16];
//   DevProvisionGetFixedKey(app_key, sizeof(app_key));
//   mibReq.Type = MIB_NWK_KEY;
//   mibReq.Param.AppKey = app_key;
//   LoRaMacMibSetRequestConfirm(&mibReq);
// }

//==========================================================================
//==========================================================================
static void ProcessJoinRetry(void) {
  if (LORAWAN_SW_RADIO_COUNT != 0) {
    LORACOMPON_PRINTLINE("joinRetryTimes=%d", gLoRaLinkVar.joinRetryTimes);
    gLoRaLinkVar.joinRetryTimes++;
    if (gLoRaLinkVar.joinRetryTimes >= LORAWAN_SW_RADIO_COUNT) {
      gLoRaLinkVar.usingIsm2400 = !gLoRaLinkVar.usingIsm2400;
      gLoRaLinkVar.joinRetryTimes = 0;
    }
  }
}

//==========================================================================
//==========================================================================
void loraTask(void *param) {
  gLoraLinkState = S_LORALINK_INIT;
  gLastRxRssi = -999;
  gLoRaLinkVar.ackCount = 0;
  gLoRaLinkVar.nakCount = 0;
  gLoRaLinkVar.joinRetryTimes = 0;
  gLoRaLinkVar.joinInterval = 0;
  gLoRaLinkVar.batteryValue = BAT_LEVEL_NO_MEASURE;
  gLoRaTaskAbort = false;
  gLoRaLinkVar.txConfirmed = true;
  gLoRaLinkVar.unconfigmedCount = 0;
  gLoRaLinkVar.usingIsm2400 = LORAWAN_USING_ISM2400;
  gLoRaLinkVar.dateRate = LORAWAN_DEFAULT_DATARATE;

  //
  // ExtPowerInit();
  // ExtPowerEnable(true);
  // BoardInitMcu();

  // Check wake up
  if (gWakeFromSleep) {
    uint16_t cal_crc =
        LoRaCrc16Ccitt(VALUE_PRESERVED_DATA_CRC_IV, (const uint8_t *)&gLoRaPreservedData, sizeof(gLoRaPreservedData));
    if ((gLoRaPreservedData.magicCode == VALUE_PRESERVED_DATA_MAGIC_CODE) && (cal_crc == gLoRaPreservedDataCrc)) {
      LORACOMPON_PRINTLINE("Preserved Data CRC correct.");

      // Restore Link variables
      memcpy(&gLoRaLinkVar, &gLoRaPreservedData.linkVar, sizeof(LoRaLinkVar_t));
    } else {
      // Clear wake up flag
      gWakeFromSleep = false;
    }
  }

  // Startup Delay
  if (!gWakeFromSleep) {
    DelayMs(5000);
  }

  // start main loop of lora task.
  for (;;) {
    //    uint32_t notif = 0;

    // Abort this task
    if (gLoRaTaskAbort) break;

    // Process Radio IRQ
    //    if (gLoraLinkState != S_LORALINK_SLEEP) {
    // if (Radio.IrqProcess != NULL) {
    //   Radio.IrqProcess();
    // }
    // Processes the LoRaMac events
    LoRaMacProcess();
    //    }

    // State machine
    switch (gLoraLinkState) {
      case S_LORALINK_INIT: {
        int ret_mac;
        MibRequestConfirm_t mibReq;

        // LORACOMPON_PRINTLINE("S_LORALINK_INIT");
        LoRaMacDeInitialization();

        gLoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
        gLoRaMacPrimitives.MacMcpsIndication = McpsIndication;
        gLoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
        gLoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
        gLoRaMacCallbacks.GetBatteryLevel = GetBatteryLevel;
        gLoRaMacCallbacks.GetTemperatureLevel = NULL;
        gLoRaMacCallbacks.NvmDataChange = NULL;
        gLoRaMacCallbacks.MacProcessNotify = OnMacProcessNotify;
        if (gLoRaLinkVar.usingIsm2400) {
          ret_mac = LoRaMacInitialization(&gLoRaMacPrimitives, &gLoRaMacCallbacks, LORAMAC_REGION_ISM2400);
        } else {
          ret_mac = LoRaMacInitialization(&gLoRaMacPrimitives, &gLoRaMacCallbacks, SUB_GHZ_REGION);
        }
        if (ret_mac != LORAMAC_STATUS_OK) {
          printf("ERROR. LoRaMac wasn't properly initialized, error: %s\n", getMacStatusString(ret_mac));
          // Fatal error, endless loop.
          while (1) {
          }
        }

        // LoRa settings
        bool using_adr = LORAWAN_ADR_ON;
        if (gLoRaLinkVar.usingIsm2400) {
          gLoRaLinkVar.dateRate = LORAWAN_ISM2400_DATARATE;
          using_adr = false;
        } else {
          gLoRaLinkVar.dateRate = LORAWAN_DEFAULT_DATARATE;
        }

        //
        bool device_activated = false;
        if (gWakeFromSleep) {
          // Clear the wake up flag
          gWakeFromSleep = false;

          //
          MibRequestConfirm_t mibReq;
          mibReq.Type = MIB_NVM_CTXS;
          if (LoRaMacMibGetRequestConfirm(&mibReq) == LORAMAC_STATUS_OK) {
            memcpy(mibReq.Param.Contexts, &gLoRaPreservedData.contexts, sizeof(LoRaMacNvmData_t));
            mibReq.Type = MIB_NETWORK_ACTIVATION;
            if (LoRaMacMibGetRequestConfirm(&mibReq) == LORAMAC_STATUS_OK) {
              if (mibReq.Param.NetworkActivation != ACTIVATION_TYPE_NONE) {
                LORACOMPON_PRINTLINE("LoRa Activation done.");
                device_activated = true;
              } else {
                ProcessJoinRetry();
                break;
              }
            }
          }
        }

        //
        if (!device_activated) {
          mibReq.Type = MIB_PUBLIC_NETWORK;
          mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
          LoRaMacMibSetRequestConfirm(&mibReq);

          mibReq.Type = MIB_ADR;
          mibReq.Param.AdrEnable = using_adr;
          LoRaMacMibSetRequestConfirm(&mibReq);
          if (using_adr) {
            LORACOMPON_PRINTLINE("ADR is ON.");
          } else {
            LORACOMPON_PRINTLINE("ADR is OFF.");
          }

#if defined(REGION_EU868) || defined(REGION_RU864) || defined(REGION_CN779) || defined(REGION_EU433)
          LoRaMacTestSetDutyCycleOn(LORAWAN_DUTYCYCLE_ON);
#endif

          // Set rx time error range
          mibReq.Type = MIB_SYSTEM_MAX_RX_ERROR;
          mibReq.Param.SystemMaxRxError = 50;  // Increase this could make the RX window open earlier.
                                               // Warning: Since there is a limitation on the RX widnow,
                                               //          a too large value will cause the window shifted
                                               //          to early and missed the signal.
          LoRaMacMibSetRequestConfirm(&mibReq);

          if (gLoRaLinkVar.usingIsm2400) {
            // Set Channels mask, forced to using CH0
            uint16_t ch_mask[6] = {0x0001, 0x0000, 0x0000, 0x0000, 0x0001, 0x0000};
            mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
            mibReq.Param.ChannelsDefaultMask = ch_mask;
            LoRaMacMibSetRequestConfirm(&mibReq);
          } else {
#if defined(REGION_US915)
            // Set Channels mask
            uint16_t ch_mask[6] = {0xff00, 0x0000, 0x0000, 0x0000, 0x0001, 0x0000};
            mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
            mibReq.Param.ChannelsDefaultMask = ch_mask;
            LoRaMacMibSetRequestConfirm(&mibReq);
#endif
          }

          // Init variables
          gLoRaLinkVar.txConfirmed = true;
          gLoRaLinkVar.unconfigmedCount = 0;
          TakeMutex();
          gLinkStatus = 0;
          gLoRaLinkVar.failCount = 0;
          gTxData.dataSize = -1;  // Ready for TX
          gRxData.dataSize = -1;  // Ready for RX
          FreeMutex();
        }

        LoRaMacStart();

#if mx_configUSE_DEVICE_PROVISIONING
        if (gLoRaSetting.provisionDone) {
          LORACOMPON_PRINTLINE("Device is provisioned.");
          gLoraLinkState = S_LORALINK_JOIN;
        } else {
          gLinkStatus |= BIT_LORASTATUS_DEV_PROV;
          gLoraLinkState = S_LORALINK_PROVISIONING_START;
          gTickLoraLink = 0;
          gLoRaLinkVar.joinInterval = TIME_PROV_INTERVAL_MIN;
        }
#else
        if (device_activated) {
          gLinkStatus |= BIT_LORASTATUS_JOIN_PASS;
          gLoraLinkState = S_LORALINK_JOINED;
        } else {
          gLoraLinkState = S_LORALINK_JOIN;
        }
#endif
        break;
      }
      case S_LORALINK_PROVISIONING_START:
        // if ((gTickLoraLink == 0) || (LoRaTickElapsed(gTickLoraLink) >= gLoRaLinkVar.joinInterval)) {
        //   gProvisionStatus = 0;
        //   InitDevProvision();
        //   ProvisioningHello();
        //   gTickLoraLink = LoRaGetTick();
        //   gLoraLinkState = S_LORALINK_PROVISIONING_HELLO;
        // }
        break;

      case S_LORALINK_PROVISIONING_HELLO:
        // if (LoRaTickElapsed(gTickLoraLink) >= TIME_PROVISIONING_TIMEOUT) {
        //   gLoraLinkState = S_LORALINK_PROVISIONING_START;
        //   gLoRaLinkVar.joinInterval = TIME_PROV_INTERVAL_MIN + randr(0, RAND_RANGE_PROV_INTERVAL);
        //   gTickLoraLink = LoRaGetTick();
        // } else if ((gProvisionStatus & BIT_PROV_HELLO_OK) != 0) {
        //   DevProvisionGenKeys();
        //   gTickLoraLink = LoRaGetTick();
        //   gLoraLinkState = S_LORALINK_PROVISIONING_AUTH;
        // }
        break;

      case S_LORALINK_PROVISIONING_AUTH:
        // if (LoRaTickElapsed(gTickLoraLink) >= TIME_PROVISIONING_AUTHWAIT) {
        //   ProvisioningAuth();
        //   gTickLoraLink = LoRaGetTick();
        //   gLoraLinkState = S_LORALINK_PROVISIONING_WAIT;
        // }
        break;

      case S_LORALINK_PROVISIONING_WAIT:
        // if (LoRaTickElapsed(gTickLoraLink) >= TIME_PROVISIONING_TIMEOUT) {
        //   gLoraLinkState = S_LORALINK_PROVISIONING_START;
        //   gLoRaLinkVar.joinInterval = TIME_PROV_INTERVAL_MIN + randr(0, RAND_RANGE_PROV_INTERVAL);
        //   gTickLoraLink = LoRaGetTick();
        // } else if ((gProvisionStatus & BIT_PROV_AUTH_OK) != 0) {
        //   LORACOMPON_PRINTLINE("  Provisioning the device.");
        //   memcpy(gLoRaSetting.devEui, DevProvisioningGetAssignedDevEui(), LORA_EUI_LENGTH);
        //   memcpy(gLoRaSetting.joinEui, DevProvisioningGetAssignedJoinEui(), LORA_EUI_LENGTH);
        //   memcpy(gLoRaSetting.nwkKey, DevProvisioningGetAssignedNwkKey(), LORA_KEY_LENGTH);
        //   memcpy(gLoRaSetting.appKey, DevProvisioningGetAssignedAppKey(), LORA_KEY_LENGTH);
        //   gLoRaSetting.provisionDone = true;
        //   AppSettingSave();

        //   LORACOMPON_HEX2STRING("  devEui:", gLoRaSetting.devEui, LORA_EUI_LENGTH);
        //   LORACOMPON_HEX2STRING("  joinEui:", gLoRaSetting.joinEui, LORA_EUI_LENGTH);
        //   LORACOMPON_HEX2STRING("  appKey:", gLoRaSetting.appKey, LORA_KEY_LENGTH);
        //   LORACOMPON_HEX2STRING("  nwkKey:", gLoRaSetting.nwkKey, LORA_KEY_LENGTH);

        //   gLoraLinkState = S_LORALINK_INIT;
        // }
        break;

      case S_LORALINK_JOIN: {
        InitOtaa();

        //
        MlmeReq_t mlmeReq;
        mlmeReq.Type = MLME_JOIN;
        if (gLoRaLinkVar.usingIsm2400) {
          mlmeReq.Req.Join.Datarate = LORAWAN_ISM2400_DATARATE;
        } else {
          mlmeReq.Req.Join.Datarate = randr(LORAWAN_JOIN_DR_MIN, LORAWAN_JOIN_DR_MAX);
        }
        mlmeReq.Req.Join.NetworkActivation = ACTIVATION_TYPE_OTAA;

        LORACOMPON_PRINTLINE("Start to Join, dr=%d", mlmeReq.Req.Join.Datarate);
        int ret_mac = LoRaMacMlmeRequest(&mlmeReq);
        if (ret_mac != LORAMAC_STATUS_OK) {
          LORACOMPON_PRINTLINE("LoRaMacMlmeRequest() failed, %s", getMacStatusString(ret_mac));
        }

        gLoraLinkState = S_LORALINK_JOIN_WAIT;
        gLoRaLinkVar.joinInterval = randr(TIME_JOIN_INTERVAL_MIN, TIME_JOIN_INTERVAL_MAX);
        gTickLoraLink = LoRaGetTick();
        break;
      }

      case S_LORALINK_JOIN_WAIT: {
        TakeMutex();
        uint32_t status = gLinkStatus;
        FreeMutex();
        if ((status & BIT_LORASTATUS_JOIN_PASS) != 0) {
          gLoRaLinkVar.joinRetryTimes = 0;
          gLoraLinkState = S_LORALINK_JOINED;
        } else if (LoRaTickElapsed(gTickLoraLink) >= gLoRaLinkVar.joinInterval) {
          ProcessJoinRetry();
          gLoraLinkState = S_LORALINK_INIT;
        }
        break;
      }

      case S_LORALINK_JOINED: {
        LORACOMPON_PRINTLINE("Joined");
        gLoraLinkState = S_LORALINK_WAITING;
        gTickLoraLink = LoRaGetTick();
        break;
      }

      case S_LORALINK_SEND: {
        if (sendFrame() < 0) {
          LORACOMPON_PRINTLINE("sendFrame() failed.");
          TakeMutex();
          gLinkStatus |= BIT_LORASTATUS_SEND_FAIL;
          FreeMutex();
        }
        gLoraLinkState = S_LORALINK_SEND_WAITING;
        gTickLoraLink = LoRaGetTick();
        break;
      }

      case S_LORALINK_SEND_MAC: {
        TakeMutex();
        gTxData.dataSize = 0;
        FreeMutex();
        if (sendFrame() < 0) {
          TakeMutex();
          gLinkStatus |= BIT_LORASTATUS_SEND_FAIL;
          FreeMutex();
        }
        gLoraLinkState = S_LORALINK_WAITING;
        gTickLoraLink = LoRaGetTick();
        break;
      }

      case S_LORALINK_SEND_WAITING: {
        TakeMutex();
        uint32_t status = gLinkStatus;
        FreeMutex();
        if ((status & BIT_LORASTATUS_SEND_PASS) != 0) {
          gLoraLinkState = S_LORALINK_SEND_SUCCESS;
        } else if ((status & BIT_LORASTATUS_SEND_FAIL) != 0) {
          gLoraLinkState = S_LORALINK_SEND_FAILURE;
        } else if (LoRaTickElapsed(gTickLoraLink) >= TIMEOUT_SEND_WAITING) {
          printf("ERROR. SEND_WAITING timeout.\n");
          TakeMutex();
          gLinkStatus |= BIT_LORASTATUS_SEND_FAIL;
          FreeMutex();
          gLoraLinkState = S_LORALINK_SEND_FAILURE;
        }
        break;
      }

      case S_LORALINK_SEND_FAILURE:
        gLoRaLinkVar.nakCount++;
        if (LORAWAN_LINK_FAIL_COUNT) {
          gLoRaLinkVar.failCount++;
          LORACOMPON_PRINTLINE("failCount=%d", gLoRaLinkVar.failCount);
        }
        else {
          gLoRaLinkVar.failCount = -1;
        }
        TakeMutex();
        gTxData.retry++;
        FreeMutex();
        if ((gTxData.retry <= LORAWAN_MAX_NOACK_RETRY) && (gLoRaLinkVar.failCount < LORAWAN_LINK_FAIL_COUNT)) {
          // Do retry
          TakeMutex();
          gLinkStatus &= ~(BIT_LORASTATUS_SEND_PASS | BIT_LORASTATUS_SEND_FAIL);
          FreeMutex();
          gTickLoraLink = LoRaGetTick();
          gLoraLinkState = S_LORALINK_RETRY_WAITING;

        } else {
          TakeMutex();
          gTxData.dataSize = -1;  // End of TX
          FreeMutex();
          gTickLoraLink = 0;  // Instant check on S_LORALINK_WAITING
          gLoraLinkState = S_LORALINK_WAITING;
        }
        break;

      case S_LORALINK_SEND_SUCCESS:
        TakeMutex();
        gTxData.dataSize = -1;  // End of TX
        FreeMutex();
        gLoRaLinkVar.failCount = 0;
        if (gLoRaLinkVar.unconfigmedCount >= LORAWAN_UNCONFIRMED_COUNT) {
          gLoRaLinkVar.unconfigmedCount = 0;
          gLoRaLinkVar.txConfirmed = true;
        } else {
          gLoRaLinkVar.unconfigmedCount++;
          gLoRaLinkVar.txConfirmed = false;
        }
        gTickLoraLink = LoRaGetTick();
        gLoraLinkState = S_LORALINK_WAITING;
        break;

      case S_LORALINK_RETRY_WAITING:
        if (LoRaTickElapsed(gTickLoraLink) >= LORAWAN_NOACK_RETRY_INTERVAL) {
          LORACOMPON_PRINTLINE("Retry %d, nakCount=%d", gTxData.retry, gLoRaLinkVar.nakCount);
          gLoraLinkState = S_LORALINK_SEND;
        }
        break;

      case S_LORALINK_WAITING: {
        TakeMutex();
        uint32_t status = gLinkStatus;
        int16_t tx_len = gTxData.dataSize;
        FreeMutex();
        if (status & BIT_LORASTATUS_JOIN_PASS) {
          if ((gTickLoraLink == 0) || (LoRaTickElapsed(gTickLoraLink) >= TIME_TXCHK_INTERVAL)) {
            gTickLoraLink = LoRaGetTick();
            if (gLoRaLinkVar.failCount >= LORAWAN_LINK_FAIL_COUNT) {
              printf("ERROR. Too many link fail. Disconnect.\n");
              gLoraLinkState = S_LORALINK_INIT;
            } else if ((tx_len >= 0) && (!LoRaMacIsBusy())) {
              gLoraLinkState = S_LORALINK_SEND;
            }
          }
        }
        break;
      }

      case S_LORALINK_SLEEP: {
        break;
      }

      case S_LORALINK_WAKEUP: {
        TakeMutex();
        uint32_t status = gLinkStatus;
        FreeMutex();
        if (status & BIT_LORASTATUS_JOIN_PASS) {
          gLoraLinkState = S_LORALINK_WAITING;
        } else {
          gLoraLinkState = S_LORALINK_JOIN_WAIT;
        }
        break;
      }

      default: {
        gLoraLinkState = S_LORALINK_INIT;
        break;
      }
    }

    DelayMs(10);

    // if (gLoraLinkState == S_LORALINK_SLEEP) {
    //   int ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notif, OS_TASK_NOTIFY_FOREVER);
    //   OS_ASSERT(ret == OS_OK);
    // } else {
    //   int ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notif, OS_MS_2_TICKS(10));
    //   OS_ASSERT(ret == OS_OK);
    // }

    // if ((notif & EVENT_NOTIF_LORA_DIO1) || (hw_gpio_get_pin_status(HW_LORA_DIO1_PORT, HW_LORA_DIO1_PIN))) {
    //   gRadioIrqFunc(NULL);
    // }

    // if (notif & EVENT_NOTIF_LORAMAC) {
    //   if (gp_loramac_cb != NULL) {
    //     gp_loramac_cb();
    //     gp_loramac_cb = NULL;
    //   }
    // }
  }
  printf("LoRaTask ended.\n");
  LoRaMacDeInitialization();
  gLoRaTaskHandle = NULL;
  vTaskDelete(NULL);
}

//==========================================================================
// Get Raw Status
//==========================================================================
static uint32_t GetStatus(void) {
  TakeMutex();
  uint32_t status = gLinkStatus;
  int16_t tx_len = gTxData.dataSize;
  int16_t rx_len = gRxData.dataSize;
  FreeMutex();
  if (status & BIT_LORASTATUS_JOIN_PASS) {
    if (rx_len >= 0) {
      status |= BIT_LORASTATUS_RX_RDY;
    }
    if (tx_len < 0) {
      status |= BIT_LORASTATUS_TX_RDY;
    }
  }
  return status;
}

//==========================================================================
// Hardware related init. Please call once at power up sequence
//==========================================================================
void LoRaComponHwInit(void) { LoRaBoardInitMcu(); }

//==========================================================================
// Start the LoRa
//==========================================================================
int8_t LoRaComponStart(bool aWakeFromSleep) {
  //
  InitMutex();

  // Get MAC address
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);

  // Default setting
  gLoRaSetting.devEui[0] = mac[0];
  gLoRaSetting.devEui[1] = mac[1];
  gLoRaSetting.devEui[2] = mac[2];
  gLoRaSetting.devEui[3] = 0xff;
  gLoRaSetting.devEui[4] = 0xfe;
  gLoRaSetting.devEui[5] = mac[3];
  gLoRaSetting.devEui[6] = mac[4];
  gLoRaSetting.devEui[7] = mac[5];

  memset(gLoRaSetting.joinEui, 0x00, LORA_EUI_LENGTH);
  memset(gLoRaSetting.nwkKey, 0x01, LORA_KEY_LENGTH);
  memset(gLoRaSetting.appKey, 0x02, LORA_KEY_LENGTH);

  //
  gWakeFromSleep = aWakeFromSleep;

  // Create task
  if (xTaskCreate(loraTask, "LoRaTask", 4096, NULL, TASK_PRIO_GENERAL, &gLoRaTaskHandle) != pdPASS) {
    printf("ERROR. Failed to create LoRa component task.\n");
    return -1;
  } else {
    return 0;
  }
}

//==========================================================================
// End of LoRa task
//==========================================================================
void LoRaComponStop(void) { gLoRaTaskAbort = true; }

//==========================================================================
// Get the sub-GHz region name
//==========================================================================
const char *LoRaComponRegionName(void) {
  if (LORAWAN_SW_RADIO_COUNT) {
    return SUB_GHZ_REGION_NAME " + ISM2400";
  } else {
    if (LORAWAN_PREFERRED_ISM2400) {
      return "ISM2400";
    } else {
      return SUB_GHZ_REGION_NAME;
    }
  }
}

//==========================================================================
// Send notification bit to task
//==========================================================================
// void LoRaComponNotify(uint32_t aEvent, void *aCallback) {
//   OS_TASK_NOTIFY_FROM_ISR(gLoRaTaskHandle, aEvent, eSetBits);
//   if (aCallback) {
//     gp_loramac_cb = aCallback;
//   }
// }

//==========================================================================
// Is busy
//==========================================================================
bool LoRaComponIsBusy(void) {
  bool ret_busy = true;
  LoraDevicState_t state;
  TakeMutex();
  state = gLoraLinkState;
  FreeMutex();

  if (!LoRaMacIsBusy()) {
    if ((state == S_LORALINK_WAITING) || (state == S_LORALINK_JOIN_WAIT) || (state == S_LORALINK_PROVISIONING_START)) {
      ret_busy = false;
    }
  }
  return ret_busy;
}

//==========================================================================
// Get time for next actions
//==========================================================================
uint32_t LoRaComponGetWaitingTime(void) {
  uint32_t waiting_time = 0;
  TakeMutex();
  LoraDevicState_t state = gLoraLinkState;
  int16_t tx_len = gTxData.dataSize;
  FreeMutex();
  if (!LoRaMacIsBusy()) {
    if (state == S_LORALINK_WAITING) {
      if ((gTickLoraLink != 0) && (tx_len < 0)) {
        waiting_time = UINT32_MAX;
      }
    } else if ((state == S_LORALINK_JOIN_WAIT) || (state == S_LORALINK_PROVISIONING_START)) {
      if (gLoRaLinkVar.joinInterval > 0) {
        uint32_t elapsed = LoRaTickElapsed(gTickLoraLink);
        if (elapsed < gLoRaLinkVar.joinInterval) {
          waiting_time = gLoRaLinkVar.joinInterval - elapsed;
        }
      }
    } else if (state == S_LORALINK_RETRY_WAITING) {
      uint32_t elapsed = LoRaTickElapsed(gTickLoraLink);
      if (elapsed < LORAWAN_NOACK_RETRY_INTERVAL) {
        waiting_time = LORAWAN_NOACK_RETRY_INTERVAL - elapsed;
      }
    }
  }
  return waiting_time;
}

//==========================================================================
// Call before enter of sleep
//==========================================================================
void LoRaComponPrepareForSleep(bool aDeepSleep) {
  TakeMutex();
  gLoraLinkState = S_LORALINK_SLEEP;
  FreeMutex();

  if (aDeepSleep) {
    if (LoRaMacQueryMacCommandsSize() > 0) {
      // Send a blank frame if some MAC command is waiting to send
      gLinkStatus &= ~(BIT_LORASTATUS_SEND_PASS | BIT_LORASTATUS_SEND_FAIL);
      gTxData.data[0] = 0;
      gTxData.dataSize = 1;
      gTxData.port = LORAWAN_FPORT_DATA;
      gTxData.retry = 0;
      gTickLoraLink = 0;
      LORACOMPON_PRINTLINE("Send a blank frame.");
      if (sendFrame() >= 0) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
      }
    }

    // Save data to preserve area
    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_NVM_CTXS;
    if (LoRaMacMibGetRequestConfirm(&mibReq) == LORAMAC_STATUS_OK) {
      gLoRaPreservedData.magicCode = VALUE_PRESERVED_DATA_MAGIC_CODE;
      memcpy(&gLoRaPreservedData.contexts, mibReq.Param.Contexts, sizeof(LoRaMacNvmData_t));
      memcpy(&gLoRaPreservedData.linkVar, &gLoRaLinkVar, sizeof(LoRaLinkVar_t));
      gLoRaPreservedDataCrc =
          LoRaCrc16Ccitt(VALUE_PRESERVED_DATA_CRC_IV, (const uint8_t *)&gLoRaPreservedData, sizeof(gLoRaPreservedData));
      LORACOMPON_PRINTLINE("Preserved Data CRC=%04X", gLoRaPreservedDataCrc);
    }
  }

  //
  RadioSx126x.Sleep();
  RadioSx1280.Sleep();
  LoRaBoardPrepareForSleep();
}

//==========================================================================
// Call after exit of sleep
//==========================================================================
void LoRaComponResumeFromSleep(void) {
  LoRaBoardResumeFromSleep();
  TakeMutex();
  gLoraLinkState = S_LORALINK_WAKEUP;
  FreeMutex();
}

//==========================================================================
// Check ready for send data
//==========================================================================
bool LoRaComponIsTxReady(void) {
  uint32_t status = GetStatus();
  if ((status & BIT_LORASTATUS_JOIN_PASS) == 0) {
    LORACOMPON_PRINTLINE("Not join");
    return false;
  } else if ((status & BIT_LORASTATUS_TX_RDY) == 0) {
    LORACOMPON_PRINTLINE("Last TX in progress");
    return false;
  } else {
    return true;
  }
}

//==========================================================================
// Send Data
//==========================================================================
int8_t LoRaComponSendData(const uint8_t *aData, uint16_t aLen) {
  // MibRequestConfirm_t mibReq;
  // mibReq.Type = MIB_CHANNELS_MASK;
  // LoRaMacMibGetRequestConfirm(&mibReq);
  // LORACOMPON_PRINTLINE("  Mask: %04X %04X %04X %04X %04X %04X", mibReq.Param.ChannelsDefaultMask[0],
  //                     mibReq.Param.ChannelsDefaultMask[1], mibReq.Param.ChannelsDefaultMask[2],
  //                     mibReq.Param.ChannelsDefaultMask[3], mibReq.Param.ChannelsDefaultMask[4],
  //                     mibReq.Param.ChannelsDefaultMask[5]);

  if (!LoRaComponIsTxReady()) {
    return -1;
  } else if (aLen > LORAWAN_MAX_PAYLOAD_LEN) {
    // Tx data too large
    return -1;
  } else {
    TakeMutex();
    gLinkStatus &= ~(BIT_LORASTATUS_SEND_PASS | BIT_LORASTATUS_SEND_FAIL);
    memcpy(gTxData.data, aData, aLen);
    gTxData.dataSize = aLen;
    gTxData.port = LORAWAN_FPORT_DATA;
    gTxData.retry = 0;
    gTickLoraLink = 0;
    FreeMutex();
    return 0;
  }
}

//==========================================================================
// Check any received data
//==========================================================================
bool LoRaComponIsRxReady(void) {
  uint32_t status = GetStatus();
  if ((status & BIT_LORASTATUS_JOIN_PASS) == 0) {
    LORACOMPON_PRINTLINE("Not join");
    return false;
  } else if ((status & BIT_LORASTATUS_RX_RDY) == 0) {
    return false;
  } else {
    return true;
  }
}

//==========================================================================
// Get received Data
//==========================================================================
int32_t LoRaComponGetData(uint8_t *aData, uint16_t aDataSize, LoRaRxInfo_t *aInfo) {
  if (!LoRaComponIsRxReady()) {
    return -1;
  } else {
    int32_t len;
    TakeMutex();
    if (aDataSize < gRxData.dataSize) {
      len = aDataSize;
    } else {
      len = gRxData.dataSize;
    }
    memcpy(aData, gRxData.data, len);
    if (aInfo != NULL) {
      aInfo->fport = gRxData.port;
      aInfo->rssi = gLastRxRssi;
      aInfo->datarate = gLastRxDatarate;
    }
    gRxData.dataSize = -1;
    FreeMutex();

    return len;
  }
}

//==========================================================================
// Set datarate
//==========================================================================
void LoRaComponSetDatarate(int8_t aValue) {
  TakeMutex();
  gLoRaLinkVar.dateRate = aValue;
  FreeMutex();
}

//==========================================================================
// Set battery info
//==========================================================================
void LoRaComponSetBatteryPercent(float aValue) {
  if (isnan(aValue)) {
    gLoRaLinkVar.batteryValue = BAT_LEVEL_NO_MEASURE;
  } else if (aValue >= 100) {
    gLoRaLinkVar.batteryValue = BAT_LEVEL_FULL;
  } else if (aValue <= 0) {
    gLoRaLinkVar.batteryValue = BAT_LEVEL_EMPTY;
  } else {
    aValue = (aValue / 100) * (BAT_LEVEL_FULL - BAT_LEVEL_EMPTY);
    aValue += 0.5;
    gLoRaLinkVar.batteryValue = (uint8_t)aValue;
  }
}

void LoRaComponSetExtPower(void) { gLoRaLinkVar.batteryValue = BAT_LEVEL_EXT_SRC; }

//==========================================================================
// Check status
//==========================================================================
bool LoRaComponIsProvisioned(void) { return ((GetStatus() & BIT_LORASTATUS_DEV_PROV) != 0); }

bool LoRaComponIsJoined(void) { return ((GetStatus() & BIT_LORASTATUS_JOIN_PASS) != 0); }

bool LoRaComponIsSendSuccess(void) { return ((GetStatus() & BIT_LORASTATUS_SEND_PASS) != 0); }

bool LoRaComponIsSendDone(void) { return ((GetStatus() & BIT_LORASTATUS_TX_RDY) != 0); }

bool LoRaComponIsIsm2400(void) { return gLoRaLinkVar.usingIsm2400; }
