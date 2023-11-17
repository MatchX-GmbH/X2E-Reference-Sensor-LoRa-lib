//==========================================================================
//==========================================================================
#ifndef INC_DEV_PROVISION_H
#define INC_DEV_PROVISION_H
//==========================================================================
//==========================================================================
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "ecdh.h"

//==========================================================================
//==========================================================================
// Provisioning message type
#define UP_MESSAGE_HELLO 0x01
#define UP_MESSAGE_AUTH 0x11
#define DOWN_RESP_HELLO 0x81
#define DOWN_RESP_AUTH_ACCEPT 0x91
#define DOWN_RESP_AUTH_REJECT 0x92

//
#define SIZE_UP_MSG_HELLO 74
#define SiZE_UP_MSG_AUTH 61
#define SIZE_DOWN_RESP_HELLO 77
#define SIZE_DOWN_RESP_AUTH_ACCEPT 41
#define SIZE_DOWN_RESP_AUTH_REJECT 9

//==========================================================================
//==========================================================================

//==========================================================================
//==========================================================================
void DevProvisionInit(const char *aProvisionId, const uint8_t *aProvisionIdHash);
void DevProvisionGetFixedKey(uint8_t *aDest, uint16_t aDestSize);
uint16_t DevProvisionPrepareHello(uint8_t *aDest, uint16_t aDestSize);
int8_t DevProvisionHelloResp(const uint8_t *aResp, uint16_t aRespLen);
uint16_t DevProvisionPrepareAuth(uint8_t *aDest, uint16_t aDestSize);
int8_t DevProvisionAuthResp(const uint8_t *aResp, uint16_t aRespLen);
void DevProvisionGenKeys(void);
int8_t DevProvisionCmpDevEui(const uint8_t *aEui);
const uint8_t *DevProvisioningGetAssignedDevEui(void);
const uint8_t *DevProvisioningGetAssignedJoinEui(void);
const uint8_t *DevProvisioningGetAssignedNwkKey(void);
const uint8_t *DevProvisioningGetAssignedAppKey(void);

//==========================================================================
//==========================================================================
#endif  // INC_DEV_PROVISION_H
