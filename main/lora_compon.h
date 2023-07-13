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
//
#define LORAWAN_MAX_PAYLOAD_LEN 240
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

void LoRaComponSetDatarate(int8_t aValue);

void LoRaComponSetBatteryPercent(float aValue);
void LoRaComponSetExtPower(void);

bool LoRaComponIsProvisioned(void);
bool LoRaComponIsJoined(void);
bool LoRaComponIsSendDone(void);
bool LoRaComponIsSendSuccess(void);
bool LoRaComponIsIsm2400(void);

//==========================================================================
//==========================================================================
#endif // INC_LORA_COMPON_H
