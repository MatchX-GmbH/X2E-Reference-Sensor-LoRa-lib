//==========================================================================
//==========================================================================
#ifndef INC_LORA_COMPON_H
#define INC_LORA_COMPON_H
//==========================================================================
//==========================================================================
#include <stdint.h>
#include <stdbool.h>

//==========================================================================
//==========================================================================
// Config
#define LORAWAN_PUBLIC_NETWORK true
#define LORA_UPLINK_PORT 2
#define LORA_MAX_PAYLOAD_LEN 128

// Link status bits
#define BIT_LORASTATUS_ERROR 0x8000
#define BIT_LORASTATUS_JOIN_PASS 0x0001
#define BIT_LORASTATUS_JOIN_FAIL 0x0100
#define BIT_LORASTATUS_SEND_PASS 0x0002
#define BIT_LORASTATUS_SEND_FAIL 0x0200
#define BIT_LORASTATUS_TX_RDY 0x0004
#define BIT_LORASTATUS_RX_RDY 0x0008
#define BIT_LORASTATUS_DEV_PROV 0x0080

//
#define LORA_KEY_LENGTH 16
#define LORA_EUI_LENGTH 8

typedef struct {
    uint8_t devEui[LORA_EUI_LENGTH];
    uint8_t joinEui[LORA_EUI_LENGTH];
    uint8_t nwkKey[LORA_KEY_LENGTH];
    uint8_t appKey[LORA_KEY_LENGTH];
}LoRaSetting_t;

typedef struct {
    uint8_t fport;
    int16_t rssi;
    int16_t datarate;
}LoRaRxInfo_t;

//==========================================================================
//==========================================================================
void LoRaComponHwInit(void);

int8_t LoRaComponStart(void);
void LoRaComponStop(void);
const char *LoRaComponSubGHzRegionName(void);
//void LoRaComponNotify(uint32_t aEvent, void *aCallback);

bool LoRaComponIsBusy(void);
uint32_t LoRaComponGetWaitingTime(void);
void LoRaComponSleepEnter(void);
void LoRaComponSleepExit(void);

bool LoRaComponIsTxReady(void);
int8_t LoRaComponSendData(const uint8_t *aData, uint16_t aLen);

bool LoRaComponIsRxReady(void);
int32_t LoRaComponGetData(uint8_t *aData, uint16_t aDataSize, LoRaRxInfo_t *aInfo);

void LoRaComponSetBatteryPercent(float aValue);
void LoRaComponSetExtPower(void);

bool LoRaComponIsProvisioned(void);
bool LoRaComponIsJoined(void);
bool LoRaComponIsSendSuccess(void);
bool LoRaComponIsSendFailure(void);

uint32_t LoRaComponGetStatus(void);

//==========================================================================
//==========================================================================
#endif // INC_LORA_COMPON_H
