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

#include "LoRaMac.h"
#include "LoRa_debug.h"
#include "board.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lora_mutex_helper.h"
#include "timer.h"

//==========================================================================
//==========================================================================
#define TASK_PRIO_GENERAL 8
#define DelayMs(x) vTaskDelay(x / portTICK_PERIOD_MS)

//==========================================================================
//==========================================================================
// Active LoRa Region
#if defined(REGION_CN470)
#define SUB_GHZ_REGION LORAMAC_REGION_CN470
static const char *kSubGHzRegionName = "CN470";
#elif defined(REGION_EU868)
#define SUB_GHZ_REGION LORAMAC_REGION_EU868
static const char *kSubGHzRegionName = "EU868";
#elif defined(REGION_US915)
#define SUB_GHZ_REGION LORAMAC_REGION_US915
static const char *kSubGHzRegionName = "US915";
#elif defined(REGION_KR920)
#define SUB_GHZ_REGION LORAMAC_REGION_KR920
static const char *kSubGHzRegionName = "KR920";
#elif defined(REGION_AS923)
#define SUB_GHZ_REGION LORAMAC_REGION_AS923
static const char *kSubGHzRegionName = "AS923";
#elif defined(REGION_AU915)
#define SUB_GHZ_REGION LORAMAC_REGION_AU915
static const char *kSubGHzRegionName = "AU915";
#elif defined(REGION_IN865)
#define SUB_GHZ_REGION LORAMAC_REGION_IN865
static const char *kSubGHzRegionName = "IN865";
#else
#error "Please define the target region."
static const char *kSubGHzRegionName = "";
#endif

//
#if defined(REGION_EU868) || defined(REGION_RU864) || defined(REGION_CN779) || defined(REGION_EU433)
#include "LoRaMacTest.h"
#define LORAWAN_DUTYCYCLE_ON true
#endif

// Default datarate
#if defined(CONFIG_LORAWAN_DEFAULT_DATARATE)
#define LORAWAN_DEFAULT_DATARATE CONFIG_LORAWAN_DEFAULT_DATARATE
#else
#define LORAWAN_DEFAULT_DATARATE DR_3
#endif

#define LORAWAN_HELLO_DATARATE DR_3
#define LORAWAN_JOIN_DR_MIN DR_0
#define LORAWAN_JOIN_DR_MAX LORAWAN_DEFAULT_DATARATE

// LoRaWAN confirmed messages
#define LORAWAN_CONFIRMED_MSG_ON true
#define MAX_NUM_OF_UNCONFIRMED 5

// LoRaWAN Adaptive Data Rate
//  remark Please note that when ADR is enabled the end-device should be static
#if defined(CONFIG_LORAWAN_ADR_ON)
#define LORAWAN_ADR_ON true
#else
#define LORAWAN_ADR_ON false
#endif

// Retries
#define NUM_OF_RETRY_TX 2
#define MAX_LINK_FAIL_COUNT 8

// ms
#define TIME_JOIN_INTERVAL_MIN 60000
#define RAND_RANGE_JOIN_INTERVAL 30000

#define TIME_TXCHK_INTERVAL 10000

#define TIME_PROV_INTERVAL_MIN 1    // 60000
#define RAND_RANGE_PROV_INTERVAL 1  // 30000

#define TIME_PROVISIONING_TIMEOUT 10000
#define TIME_PROVISIONING_AUTHWAIT 5000
#define TIMEOUT_SEND_WAITING 17500

// Provisioning status bits
#define BIT_PROV_HELLO_OK 0x01
#define BIT_PROV_AUTH_OK 0x02

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

typedef void(loramac_cb_t)(void);
static loramac_cb_t *gp_loramac_cb;
static TaskHandle_t gLoRaTaskHandle = NULL;
static bool gLoRaTaskAbort;

//
typedef struct TLoraAppData {
  uint8_t *data;
  uint8_t dataSize;
  uint8_t port;
  uint8_t retry;
} LoraAppData_t;

//
static uint8_t gTxBuf[LORA_MAX_PAYLOAD_LEN];
static LoraAppData_t gTxData = {gTxBuf, 1, 10, 0};
static uint8_t gRxBuf[LORA_MAX_PAYLOAD_LEN];
static LoraAppData_t gRxData = {gRxBuf, 0, 0, 0};
static bool gSendingBlankFrame;
static int8_t gUnconfirmedCount;

static uint32_t gAckIndex;
static uint8_t gJoinRetryTimes;
static volatile uint32_t gLinkStatus;
static int16_t gLastRxRssi;
static uint8_t gLastRxDatarate;
static uint32_t gProvisionStatus;
static uint32_t gLinkFailCount;

static LoRaMacPrimitives_t gLoRaMacPrimitives;
static LoRaMacCallback_t gLoRaMacCallbacks;

static uint32_t gJoinInterval;

static uint8_t gBatteryValue;

static LoRaSetting_t gLoRaSetting;

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
static uint8_t GetBatteryLevel(void) { return gBatteryValue; }

void LoRaComponSetBatteryPercent(float aValue) {
  if (isnan(aValue)) {
    gBatteryValue = BAT_LEVEL_NO_MEASURE;
  } else if (aValue >= 100) {
    gBatteryValue = BAT_LEVEL_FULL;
  } else if (aValue <= 0) {
    gBatteryValue = BAT_LEVEL_EMPTY;
  } else {
    aValue = (aValue / 100) * (BAT_LEVEL_FULL - BAT_LEVEL_EMPTY);
    aValue += 0.5;
    gBatteryValue = (uint8_t)aValue;
  }
}

void LoRaComponSetExtPower(void) { gBatteryValue = BAT_LEVEL_EXT_SRC; }

//==========================================================================
// ProvisioningHello
//==========================================================================
static void ProvisioningHello(void) {
  LORA_PRINTLINE("ProvisioningHello()");

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
  //   LORA_PRINTLINE("MLME-Request - MLME_PROPRIETARY");
  //   LORA_PRINTLINE("  STATUS: %s", getMacStatusString(status));

  //   if (status == LORAMAC_STATUS_OK) {
  //     LORA_PRINTLINE("  SUCCESS");
  //   } else {
  //     if (status == LORAMAC_STATUS_DUTYCYCLE_RESTRICTED) {
  //       LORA_PRINTLINE("  Next Tx in: %u [ms]", (unsigned int)mlmeReq.ReqReturn.DutyCycleWaitTime);
  //     }
  //   }
  // }
}

//==========================================================================
// ProvisioningAuth
//==========================================================================
static void ProvisioningAuth(void) {
  LORA_PRINTLINE("ProvisioningAuth()");

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

  // LORA_PRINTLINE("MLME-Request - MLME_PROPRIETARY");
  // LORA_PRINTLINE("  STATUS: %s", getMacStatusString(status));

  // if (status == LORAMAC_STATUS_OK) {
  //   LORA_PRINTLINE("  SUCCESS");
  // } else {
  //   if (status == LORAMAC_STATUS_DUTYCYCLE_RESTRICTED) {
  //     LORA_PRINTLINE("  Next Tx in: %u [ms]", (unsigned int)mlmeReq.ReqReturn.DutyCycleWaitTime);
  //   }
  // }
}

//==========================================================================
// Send a blank frame
//==========================================================================
static int8_t sendBlankFrame(void) {
  MibRequestConfirm_t mibReq;
  McpsReq_t mcpsReq;
  LoRaMacStatus_t ret_mac;

  if (!LORAWAN_ADR_ON) {
    mibReq.Type = MIB_CHANNELS_DATARATE;
    mibReq.Param.ChannelsDatarate = LORAWAN_DEFAULT_DATARATE;
    LoRaMacMibSetRequestConfirm(&mibReq);
  }

  if (LORAWAN_CONFIRMED_MSG_ON) {
    mcpsReq.Type = MCPS_CONFIRMED;
    mcpsReq.Req.Confirmed.fPort = gTxData.port;
    mcpsReq.Req.Confirmed.fBuffer = gTxData.data;
    mcpsReq.Req.Confirmed.fBufferSize = 0;
    mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
  } else {
    mcpsReq.Type = MCPS_UNCONFIRMED;
    mcpsReq.Req.Unconfirmed.fPort = gTxData.port;
    mcpsReq.Req.Unconfirmed.fBuffer = gTxData.data;
    mcpsReq.Req.Unconfirmed.fBufferSize = 0;
    mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
  }

  gSendingBlankFrame = true;
  ret_mac = LoRaMacMcpsRequest(&mcpsReq);
  if (ret_mac == LORAMAC_STATUS_OK) {
    return 0;
  } else {
    LORA_PRINTLINE("LoRaMacMcpsRequest() failed, %s", getMacStatusString(ret_mac));
    return -1;
  }
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
  if (!LORAWAN_ADR_ON) {
    mibReq.Type = MIB_CHANNELS_DATARATE;
    mibReq.Param.ChannelsDatarate = LORAWAN_DEFAULT_DATARATE;
    LoRaMacMibSetRequestConfirm(&mibReq);
  }

  // Check frame size
  ret_mac = LoRaMacQueryTxPossible(gTxData.dataSize, &txInfo);
  if (ret_mac != LORAMAC_STATUS_OK) {
    // LORA_PRINTLINE("CurrentPossiblePayloadSize=%d", txInfo.CurrentPossiblePayloadSize);
    // LORA_PRINTLINE("MaxPossibleApplicationDataSize=%d", txInfo.MaxPossibleApplicationDataSize);
    if (ret_mac == LORAMAC_STATUS_LENGTH_ERROR) {
      printf("ERROR. Payload is too large at current datarate. Max is %d.\n", txInfo.CurrentPossiblePayloadSize);
      if (LORAWAN_ADR_ON) {
        LORA_PRINTLINE("ADR is on. Sending a blank frame.");
        sendBlankFrame();
        return -1;
      } else {
        gTxData.retry = 0;  // No more retry
        return -1;
      }
    } else {
      printf("ERROR. LoRaMacQueryTxPossible() failed, %s\n", getMacStatusString(ret_mac));
      return -1;
    }
  }

  if ((LORAWAN_CONFIRMED_MSG_ON) || (gUnconfirmedCount >= MAX_NUM_OF_UNCONFIRMED)) {
    mcpsReq.Type = MCPS_CONFIRMED;
    mcpsReq.Req.Confirmed.fPort = gTxData.port;
    mcpsReq.Req.Confirmed.fBuffer = gTxData.data;
    mcpsReq.Req.Confirmed.fBufferSize = gTxData.dataSize;
    //    mcpsReq.Req.Confirmed.NbTrials = g_data_send_nbtrials ? g_data_send_nbtrials : gMacConfig->nbtrials.conf + 1;
    mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
  } else {
    // MibRequestConfirm_t mibReq;
    // mibReq.Type = MIB_CHANNELS_NB_TRANS;
    // mibReq.Param.ChannelsNbTrans = g_data_send_nbtrials ? g_data_send_nbtrials : gMacConfig->nbtrials.unconf + 1;
    // LoRaMacMibSetRequestConfirm(&mibReq);

    mcpsReq.Type = MCPS_UNCONFIRMED;
    mcpsReq.Req.Unconfirmed.fPort = gTxData.port;
    mcpsReq.Req.Unconfirmed.fBuffer = gTxData.data;
    mcpsReq.Req.Unconfirmed.fBufferSize = gTxData.dataSize;
    mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
  }

  //
  gSendingBlankFrame = false;
  ret_mac = LoRaMacMcpsRequest(&mcpsReq);
  if (ret_mac == LORAMAC_STATUS_OK) {
    return 0;
  } else {
    LORA_PRINTLINE("LoRaMacMcpsRequest() failed, %s", getMacStatusString(ret_mac));
    return -1;
  }
}

//==========================================================================
// MCPS-Confirm event function
//==========================================================================
static void McpsConfirm(McpsConfirm_t *mcpsConfirm) {
  if (!gSendingBlankFrame) {
    gTxData.dataSize = 0;
  }
  if (mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
    LORA_PRINTLINE("McpsConfirm() OK");
    if (!LORAWAN_CONFIRMED_MSG_ON) {
      TakeMutex();
      gLinkStatus |= BIT_LORASTATUS_SEND_PASS;
      gLinkFailCount = 0;
      FreeMutex();
      if (gUnconfirmedCount < MAX_NUM_OF_UNCONFIRMED) {
        gUnconfirmedCount++;
      }
    }
  } else {
    printf("ERROR. McpsConfirm() failed. %s\n", getMacEventStatusString(mcpsConfirm->Status));
    TakeMutex();
    gLinkStatus |= BIT_LORASTATUS_SEND_FAIL;
    gLinkFailCount++;
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
  LORA_PRINTLINE("rx frame: rssi=%d, snr=%d, dr=%d", mcpsIndication->Rssi, mcpsIndication->Snr, mcpsIndication->RxDatarate);
  gLastRxRssi = mcpsIndication->Rssi;
  gLastRxDatarate = mcpsIndication->RxDatarate;
  if (mcpsIndication->RxData == true) {
    switch (mcpsIndication->Port) {
      case 224:
        break;
      default: {
        gRxData.port = mcpsIndication->Port;
        gRxData.dataSize = mcpsIndication->BufferSize;
        memcpy1(gRxData.data, mcpsIndication->Buffer, gRxData.dataSize);
        LORA_PRINTLINE("Rxed %d bytes.", gRxData.dataSize);
        break;
      }
    }
  }
  if (mcpsIndication->AckReceived) {
    // ACK
    LORA_PRINTLINE("rx, ACK, index %u", (unsigned int)gAckIndex++);
    if (LORAWAN_CONFIRMED_MSG_ON) {
      TakeMutex();
      gLinkStatus |= BIT_LORASTATUS_SEND_PASS;
      gLinkFailCount = 0;
      FreeMutex();
    } else {
      gUnconfirmedCount = 0;
    }
  }

  // LoRaComponNotify(EVENT_NOTIF_LORAMAC, NULL);
}

//==========================================================================
// MLME-Confirm event function
//==========================================================================
static void MlmeConfirm(MlmeConfirm_t *mlmeConfirm) {
  LORA_PRINTLINE("MLME-Confirm");
  LORA_PRINTLINE("  STATUS: %s", getMacEventStatusString(mlmeConfirm->Status));
  switch (mlmeConfirm->MlmeRequest) {
    case MLME_JOIN: {
      if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
        // Status is OK, node has joined the network
        gJoinRetryTimes = 0;
        TakeMutex();
        gLinkStatus |= BIT_LORASTATUS_JOIN_PASS;
        FreeMutex();
      } else {
        LORA_PRINTLINE("Join failed.");
        TakeMutex();
        gLinkStatus |= BIT_LORASTATUS_JOIN_FAIL;
        FreeMutex();
        gJoinRetryTimes++;
      }
      break;
    }
    // case MLME_PROPRIETARY: {
    //   if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
    //     MibRequestConfirm_t mibGet;
    //     LORA_PRINTLINE("  PROPRIETARY RXED");
    //     LORA_PRINTLINE("  RSSI: %d", mlmeConfirm->ProprietaryRssi);
    //     LORA_PRINTLINE("  MIC=%08X (%d)", (unsigned int)mlmeConfirm->ProprietaryMic, mlmeConfirm->ProprietaryMicCorrect);
    //     LORA_PRINTLINE("  PayloadLen=%d", mlmeConfirm->ProprietaryPayloadLen);
    //     if (mlmeConfirm->ProprietaryPayloadLen > 0) {
    //       LORA_HEX2STRING("Payload:", mlmeConfirm->ProprietaryPayload, mlmeConfirm->ProprietaryPayloadLen);
    //     }
    //     mibGet.Type = MIB_CHANNELS_DATARATE;
    //     LoRaMacMibGetRequestConfirm(&mibGet);
    //     LORA_PRINTLINE("  DATA RATE: DR_%d", mibGet.Param.ChannelsDatarate);

    //     // Check resp
    //     const uint8_t *resp = mlmeConfirm->ProprietaryPayload;
    //     if (!mlmeConfirm->ProprietaryMicCorrect) {
    //       LORA_PRINTLINE("  Wrong MIC, response dropped.");
    //     } else if ((mlmeConfirm->ProprietaryPayloadLen == SIZE_DOWN_RESP_HELLO) && (resp[0] == DOWN_RESP_HELLO)) {
    //       // Hello Response
    //       if (DevProvisionHelloResp(resp, mlmeConfirm->ProprietaryPayloadLen) == 0) {
    //         LORA_PRINTLINE("  Hello response got.");
    //         gProvisionStatus |= BIT_PROV_HELLO_OK;
    //       }
    //     } else if ((mlmeConfirm->ProprietaryPayloadLen == SIZE_DOWN_RESP_AUTH_ACCEPT) && (resp[0] == DOWN_RESP_AUTH_ACCEPT)) {
    //       // Auth Response
    //       if (DevProvisionAuthResp(resp, mlmeConfirm->ProprietaryPayloadLen) == 0) {
    //         LORA_PRINTLINE("  Auth accepted.");
    //         gProvisionStatus |= BIT_PROV_AUTH_OK;
    //       }
    //     } else if ((mlmeConfirm->ProprietaryPayloadLen == SIZE_DOWN_RESP_AUTH_REJECT) && (resp[0] == DOWN_RESP_AUTH_REJECT)) {
    //       if (DevProvisionCmpDevEui(&resp[1]) != 0) {
    //         // Mismatch devEUI
    //       } else {
    //         LORA_PRINTLINE("  Auth rejected.");
    //       }
    //     } else {
    //       LORA_PRINTLINE("Unknown proprietary frame.");
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
  LORA_PRINTLINE("MLME-Indication");
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
  mibReq.Param.ChannelsDefaultDatarate = LORAWAN_DEFAULT_DATARATE;
  LoRaMacMibSetRequestConfirm(&mibReq);

  mibReq.Type = MIB_CHANNELS_DATARATE;
  mibReq.Param.ChannelsDatarate = LORAWAN_DEFAULT_DATARATE;
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

  LORA_PRINTLINE("InitOtaa()");
  LORA_HEX2STRING("DevEui:", gLoRaSetting.devEui, LORA_EUI_LENGTH);
  LORA_HEX2STRING("JoinEui:", gLoRaSetting.joinEui, LORA_EUI_LENGTH);
  LORA_HEX2STRING("NwkKey:", gLoRaSetting.nwkKey, LORA_KEY_LENGTH);
  LORA_HEX2STRING("AppKey:", gLoRaSetting.appKey, LORA_KEY_LENGTH);
}

//==========================================================================
// Init for Device Provisioning
//==========================================================================
// void CalVerifyCode(uint8_t *aDest, uint32_t aDestSize, const char *aProvisionId, const uint8_t *aNonce);

// static void InitDevProvision(void) {
//   LORA_PRINTLINE("InitDevProvision()");
//   LORA_PRINTLINE("PID=%s", gSystemSetting.provisionId);
//   LORA_HEX2STRING("Hash=", gSystemSetting.provisionIdHash, sizeof(gSystemSetting.provisionIdHash));

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
void loraTask(void *param) {
  gLoraLinkState = S_LORALINK_INIT;
  gLastRxRssi = -999;
  gAckIndex = 0;
  gJoinRetryTimes = 0;
  gJoinInterval = 0;
  gBatteryValue = BAT_LEVEL_NO_MEASURE;
  gLoRaTaskAbort = false;

  //
  // ExtPowerInit();
  // ExtPowerEnable(true);
  // BoardInitMcu();

  // Startup Delay
  DelayMs(5000);

  // start main loop of lora task.
  for (;;) {
    uint32_t notif = 0;

    // Abort this task
    if (gLoRaTaskAbort) break;

    // Process Radio IRQ
    if (gLoraLinkState != S_LORALINK_SLEEP) {
      // if (Radio.IrqProcess != NULL) {
      //   Radio.IrqProcess();
      // }
      // Processes the LoRaMac events
      LoRaMacProcess();
    }

    // State machine
    switch (gLoraLinkState) {
      case S_LORALINK_INIT: {
        MibRequestConfirm_t mibReq;

        // LORA_PRINTLINE("S_LORALINK_INIT");
        LoRaMacDeInitialization();

        gLoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
        gLoRaMacPrimitives.MacMcpsIndication = McpsIndication;
        gLoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
        gLoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
        gLoRaMacCallbacks.GetBatteryLevel = GetBatteryLevel;
        gLoRaMacCallbacks.GetTemperatureLevel = NULL;
        gLoRaMacCallbacks.NvmDataChange = NULL;
        gLoRaMacCallbacks.MacProcessNotify = OnMacProcessNotify;
        int ret_mac = LoRaMacInitialization(&gLoRaMacPrimitives, &gLoRaMacCallbacks, SUB_GHZ_REGION);
        if (ret_mac != LORAMAC_STATUS_OK) {
          printf("ERROR. LoRaMac wasn't properly initialized, error: %s\n", getMacStatusString(ret_mac));
          // Fatal error, endless loop.
          while (1) {
          }
        }

        // LoRa settings
        mibReq.Type = MIB_PUBLIC_NETWORK;
        mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
        LoRaMacMibSetRequestConfirm(&mibReq);

        mibReq.Type = MIB_ADR;
        mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
        LoRaMacMibSetRequestConfirm(&mibReq);
        if (LORAWAN_ADR_ON) {
          LORA_PRINTLINE("ADR is ON.");
        }
        else {
          LORA_PRINTLINE("ADR is OFF.");
        }

#if defined(REGION_EU868) || defined(REGION_RU864) || defined(REGION_CN779) || defined(REGION_EU433)
        LoRaMacTestSetDutyCycleOn(LORAWAN_DUTYCYCLE_ON);
#endif

        // Set rx time error range
        mibReq.Type = MIB_SYSTEM_MAX_RX_ERROR;
        mibReq.Param.SystemMaxRxError = 60;  // Increase this could make the RX window open earlier.
                                             // Warning: Since there is a limitation on the RX widnow,
                                             //          a too large value will cause the window shifted
                                             //          to early and missed the signal.
        LoRaMacMibSetRequestConfirm(&mibReq);

#if defined(REGION_US915)
        // Set Channels mask
        uint16_t ch_mask[6] = {0xff00, 0x0000, 0x0000, 0x0000, 0x0001, 0x0000};
        mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
        mibReq.Param.ChannelsDefaultMask = ch_mask;
        LoRaMacMibSetRequestConfirm(&mibReq);
#endif

        LoRaMacStart();

        gTxData.dataSize = 0;
        gRxData.dataSize = 0;
        gSendingBlankFrame = false;
        gUnconfirmedCount = MAX_NUM_OF_UNCONFIRMED;
        TakeMutex();
        gLinkStatus = 0;
        gLinkFailCount = 0;
        FreeMutex();
#if mx_configUSE_DEVICE_PROVISIONING
        if (gLoRaSetting.provisionDone) {
          LORA_PRINTLINE("Device is provisioned.");
          gLoraLinkState = S_LORALINK_JOIN;
        } else {
          gLinkStatus |= BIT_LORASTATUS_DEV_PROV;
          gLoraLinkState = S_LORALINK_PROVISIONING_START;
          gTickLoraLink = 0;
          gJoinInterval = TIME_PROV_INTERVAL_MIN;
        }
#else
        gLoraLinkState = S_LORALINK_JOIN;
#endif
        break;
      }
      case S_LORALINK_PROVISIONING_START:
        // if ((gTickLoraLink == 0) || (LoRaTickElapsed(gTickLoraLink) >= gJoinInterval)) {
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
        //   gJoinInterval = TIME_PROV_INTERVAL_MIN + randr(0, RAND_RANGE_PROV_INTERVAL);
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
        //   gJoinInterval = TIME_PROV_INTERVAL_MIN + randr(0, RAND_RANGE_PROV_INTERVAL);
        //   gTickLoraLink = LoRaGetTick();
        // } else if ((gProvisionStatus & BIT_PROV_AUTH_OK) != 0) {
        //   LORA_PRINTLINE("  Provisioning the device.");
        //   memcpy(gLoRaSetting.devEui, DevProvisioningGetAssignedDevEui(), LORA_EUI_LENGTH);
        //   memcpy(gLoRaSetting.joinEui, DevProvisioningGetAssignedJoinEui(), LORA_EUI_LENGTH);
        //   memcpy(gLoRaSetting.nwkKey, DevProvisioningGetAssignedNwkKey(), LORA_KEY_LENGTH);
        //   memcpy(gLoRaSetting.appKey, DevProvisioningGetAssignedAppKey(), LORA_KEY_LENGTH);
        //   gLoRaSetting.provisionDone = true;
        //   AppSettingSave();

        //   LORA_HEX2STRING("  devEui:", gLoRaSetting.devEui, LORA_EUI_LENGTH);
        //   LORA_HEX2STRING("  joinEui:", gLoRaSetting.joinEui, LORA_EUI_LENGTH);
        //   LORA_HEX2STRING("  appKey:", gLoRaSetting.appKey, LORA_KEY_LENGTH);
        //   LORA_HEX2STRING("  nwkKey:", gLoRaSetting.nwkKey, LORA_KEY_LENGTH);

        //   gLoraLinkState = S_LORALINK_INIT;
        // }
        break;

      case S_LORALINK_JOIN: {
        InitOtaa();

        //
        MlmeReq_t mlmeReq;
        mlmeReq.Type = MLME_JOIN;
        mlmeReq.Req.Join.Datarate = randr(LORAWAN_JOIN_DR_MIN, LORAWAN_JOIN_DR_MAX);
        mlmeReq.Req.Join.NetworkActivation = ACTIVATION_TYPE_OTAA;

        LORA_PRINTLINE("Start to Join, dr=%d", mlmeReq.Req.Join.Datarate);
        int ret_mac = LoRaMacMlmeRequest(&mlmeReq);
        if (ret_mac != LORAMAC_STATUS_OK) {
          LORA_PRINTLINE("LoRaMacMlmeRequest() failed, %s", getMacStatusString(ret_mac));
        }

        gLoraLinkState = S_LORALINK_JOIN_WAIT;
        gJoinInterval = TIME_JOIN_INTERVAL_MIN + randr(0, RAND_RANGE_JOIN_INTERVAL);
        gTickLoraLink = LoRaGetTick();
        break;
      }

      case S_LORALINK_JOIN_WAIT: {
        TakeMutex();
        uint32_t status = gLinkStatus;
        FreeMutex();
        if (status & BIT_LORASTATUS_JOIN_PASS) {
          gLoraLinkState = S_LORALINK_JOINED;
        } else if (LoRaTickElapsed(gTickLoraLink) >= gJoinInterval) {
          gLoraLinkState = S_LORALINK_INIT;
        }
        break;
      }

      case S_LORALINK_JOINED: {
        LORA_PRINTLINE("Joined");
        gLoraLinkState = S_LORALINK_WAITING;
        gTickLoraLink = LoRaGetTick();
        break;
      }

      case S_LORALINK_SEND: {
        if (sendFrame() < 0) {
          if (gTxData.retry > 0) {
            LORA_PRINTLINE("TX failed. retry=%d", gTxData.retry);
            gTxData.retry--;
          } else {
            // TX dropped
            LORA_PRINTLINE("TX dropped.");
            gTxData.dataSize = 0;
            TakeMutex();
            gLinkStatus |= BIT_LORASTATUS_SEND_FAIL;
            gLinkFailCount++;
            FreeMutex();
          }
          gLoraLinkState = S_LORALINK_WAITING;
        } else {
          gLoraLinkState = S_LORALINK_SEND_WAITING;
        }
        gTickLoraLink = LoRaGetTick();
        break;
      }

      case S_LORALINK_SEND_MAC: {
        if (sendBlankFrame() < 0) {
        }
        gLoraLinkState = S_LORALINK_WAITING;
        gTickLoraLink = LoRaGetTick();
        break;
      }

      case S_LORALINK_SEND_WAITING: {
        TakeMutex();
        uint32_t status = gLinkStatus;
        FreeMutex();
        if (((status & BIT_LORASTATUS_SEND_FAIL) != 0) || ((status & BIT_LORASTATUS_SEND_PASS) != 0)) {
          gLoraLinkState = S_LORALINK_WAITING;
          gTickLoraLink = LoRaGetTick();
        } else if (LoRaTickElapsed(gTickLoraLink) >= TIMEOUT_SEND_WAITING) {
          printf("ERROR. SEND_WAITING timeout.\n");
          gTxData.dataSize = 0;
          TakeMutex();
          gLinkStatus |= BIT_LORASTATUS_SEND_FAIL;
          gLinkFailCount++;
          FreeMutex();
          gLoraLinkState = S_LORALINK_WAITING;
          gTickLoraLink = LoRaGetTick();
        }
        break;
      }

      case S_LORALINK_WAITING: {
        TakeMutex();
        uint32_t status = gLinkStatus;
        uint32_t fail_count = gLinkFailCount;
        FreeMutex();
        if (status & BIT_LORASTATUS_JOIN_PASS) {
          if ((gTickLoraLink == 0) || (LoRaTickElapsed(gTickLoraLink) >= TIME_TXCHK_INTERVAL)) {
            gTickLoraLink = LoRaGetTick();
            if (gLinkFailCount >= MAX_LINK_FAIL_COUNT) {
              printf("ERROR. Too many link fail. Init again.\n");
              gLoraLinkState = S_LORALINK_INIT;
            } else if ((gTxData.dataSize > 0) && (!LoRaMacIsBusy())) {
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
// Hardware related init. Please call once at power up sequence
//==========================================================================
void LoRaComponHwInit(void) { LoRaBoardInitMcu(); }

//==========================================================================
// Start the LoRa
//==========================================================================
int8_t LoRaComponStart(void) {
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
const char *LoRaComponSubGHzRegionName(void) { return kSubGHzRegionName; }

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
  LoraDevicState_t state;
  TakeMutex();
  state = gLoraLinkState;
  FreeMutex();
  if (!LoRaMacIsBusy()) {
    waiting_time = UINT32_MAX;
    if (gTickLoraLink == 0) {
      waiting_time = 0;
    } else if ((state == S_LORALINK_WAITING) && (gTxData.dataSize > 0)) {
      uint32_t elapsed = LoRaTickElapsed(gTickLoraLink);
      if (elapsed < TIME_TXCHK_INTERVAL) {
        waiting_time = TIME_TXCHK_INTERVAL - elapsed;
      }
    } else if ((state == S_LORALINK_JOIN_WAIT) || (state == S_LORALINK_PROVISIONING_START)) {
      if (gJoinInterval > 0) {
        uint32_t elapsed = LoRaTickElapsed(gTickLoraLink);
        if (elapsed < gJoinInterval) {
          waiting_time = gJoinInterval - elapsed;
        }
      }
    }
  }
  return waiting_time;
}

//==========================================================================
// Call before enter of sleep
//==========================================================================
void LoRaComponSleepEnter(void) {
  TakeMutex();
  gLoraLinkState = S_LORALINK_SLEEP;
  FreeMutex();
  // LoRaComponNotify(0, NULL);
  // OS_DELAY_MS(10);
  // BoardDeInitMcu();
  // SX126xIoDeInit();  // Comment this if Vdd_rfs is using internal 3.3V LDO
  // ExtPowerEnable(false);
}

//==========================================================================
// Call after exit of sleep
//==========================================================================
void LoRaComponSleepExit(void) {
  // LORA_PRINTLINE("LoRaComponSleepExit()");
  // ExtPowerInit();
  // ExtPowerEnable(true);
  // BoardInitMcu();
  // OS_DELAY_MS(10);
  // Radio.Init(NULL);  // Reinit the radio, without change the callbacks
  // srand1(Radio.Random());
  // Radio.SetPublicNetwork(true);
  // Radio.Sleep();
  TakeMutex();
  gLoraLinkState = S_LORALINK_WAKEUP;
  FreeMutex();
  //  LoRaComponNotify(0, NULL);
}

//==========================================================================
// Check ready for send data
//==========================================================================
bool LoRaComponIsTxReady(void) {
  TakeMutex();
  uint32_t status = gLinkStatus;
  FreeMutex();
  if ((status & BIT_LORASTATUS_JOIN_PASS) == 0) {
    LORA_PRINTLINE("Not join");
    return false;
  } else if (gTxData.dataSize != 0) {
    LORA_PRINTLINE("Last TX in progress");
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
  // LORA_PRINTLINE("  Mask: %04X %04X %04X %04X %04X %04X", mibReq.Param.ChannelsDefaultMask[0],
  //                     mibReq.Param.ChannelsDefaultMask[1], mibReq.Param.ChannelsDefaultMask[2],
  //                     mibReq.Param.ChannelsDefaultMask[3], mibReq.Param.ChannelsDefaultMask[4],
  //                     mibReq.Param.ChannelsDefaultMask[5]);

  TakeMutex();
  gLinkStatus &= ~(BIT_LORASTATUS_SEND_PASS | BIT_LORASTATUS_SEND_FAIL);
  FreeMutex();
  if (!LoRaComponIsTxReady()) {
    return -1;
  } else if (aLen > LORA_MAX_PAYLOAD_LEN) {
    // Tx data too large
    return -1;
  } else {
    memcpy(gTxData.data, aData, aLen);
    gTxData.dataSize = aLen;
    gTxData.port = LORA_UPLINK_PORT;
    gTxData.retry = NUM_OF_RETRY_TX;
    gTickLoraLink = 0;
    return 0;
  }
}

//==========================================================================
// Check any received data
//==========================================================================
bool LoRaComponIsRxReady(void) {
  TakeMutex();
  uint32_t status = gLinkStatus;
  FreeMutex();
  if ((status & BIT_LORASTATUS_JOIN_PASS) == 0) {
    LORA_PRINTLINE("Not join");
    return false;
  } else if (gRxData.dataSize == 0) {
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
    gRxData.dataSize = 0;

    return len;
  }
}

//==========================================================================
// Check status
//==========================================================================
bool LoRaComponIsProvisioned(void) { return ((LoRaComponGetStatus() & BIT_LORASTATUS_DEV_PROV) != 0); }

bool LoRaComponIsJoined(void) { return ((LoRaComponGetStatus() & BIT_LORASTATUS_JOIN_PASS) != 0); }

bool LoRaComponIsSendSuccess(void) { return ((LoRaComponGetStatus() & BIT_LORASTATUS_SEND_PASS) != 0); }

bool LoRaComponIsSendFailure(void) { return ((LoRaComponGetStatus() & BIT_LORASTATUS_SEND_FAIL) != 0); }

//==========================================================================
// Get Raw Status
//==========================================================================
uint32_t LoRaComponGetStatus(void) {
  TakeMutex();
  uint32_t status = gLinkStatus;
  FreeMutex();
  if (status & BIT_LORASTATUS_JOIN_PASS) {
    if (gRxData.dataSize != 0) {
      status |= BIT_LORASTATUS_RX_RDY;
    }
    if (gTxData.dataSize == 0) {
      status |= BIT_LORASTATUS_TX_RDY;
    }
  }
  return status;
}
