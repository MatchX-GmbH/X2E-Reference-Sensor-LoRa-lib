//==========================================================================
//  Copyright (c) MatchX GmbH.  All rights reserved.
//==========================================================================
// Naming conventions
// ~~~~~~~~~~~~~~~~~~
//                Class : Leading C
//               Struct : Leading T
//             Constant : Leading k
//      Global Variable : Leading g
//    Function argument : Leading a
//       Local Variable : All lower case
//==========================================================================
#include <string.h>

#include "LoRaCompon_debug.h"
#include "ecdh.h"
#include "secure-element.h"
#include "aes.h"
#include "cmac.h"

#include "dev_provision.h"

//==========================================================================
// Defines
//==========================================================================
#ifndef mx_configAPP_MAINNET
#define mx_configAPP_MAINNET 0
#endif

#if mx_configAPP_MAINNET
#define VALUE_MAINNET 1
#else
#define VALUE_MAINNET 0
#endif

#define LORA_KEY_LENGTH 16
#define LORA_EUI_LENGTH 8

typedef struct TEcdhData {
  uint8_t devEui[LORA_EUI_LENGTH];
  uint8_t pubKey[ECC_PUB_KEY_SIZE];
  uint8_t privateKey[ECC_PRV_KEY_SIZE];
  uint8_t sharedKey[ECC_PUB_KEY_SIZE];
  uint8_t serverPubKey[ECC_PUB_KEY_SIZE];
  uint8_t serverNonce[4];
  uint8_t devNonce[4];
  uint8_t appKey[LORA_KEY_LENGTH];
  uint8_t nwkKey[LORA_KEY_LENGTH];
  uint8_t provKey[LORA_KEY_LENGTH];
  uint8_t assignedDevEui[LORA_EUI_LENGTH];
  uint8_t assignedJoinEui[LORA_EUI_LENGTH];
} EcdhData_t;

// sec/soft-se.c
SecureElementStatus_t SecureElementRandomNumber( uint32_t* randomNum );


//==========================================================================
//==========================================================================
// Device Provisioning
static const uint8_t kEncFixedKey[16] = {0x75, 0x12, 0xe0, 0x38, 0x53, 0x98, 0xbc, 0x95,
                                         0xe5, 0xa2, 0xf1, 0x7e, 0xb6, 0xbe, 0x28, 0x50};
static const uint8_t kEpromKey[16] = {0xea, 0xc8, 0x55, 0x65, 0x25, 0xe4, 0x3c, 0x88,
                                      0xaf, 0x5d, 0x1a, 0xcf, 0x56, 0x0f, 0x48, 0xaa};

static const char *gProvisionId = "";
static const uint8_t *gProvisionIdHash = NULL;

static uint8_t gPayloadBuf[32];

static EcdhData_t gEcdhData;

//==========================================================================
// Derive Keys
//==========================================================================
static void DeriveKey(uint8_t *aAppKey, uint8_t *aNwkKey, uint8_t *aProvKey, const uint8_t *aDevEui, const uint8_t *aSharedKey) {
  aes_context aes_ctx;
  uint8_t key[16];
  uint8_t in_buf[16];
  uint8_t out_buf[16];

  // App Key
  memcpy(key, &aSharedKey[0], sizeof(key));
  memset(in_buf, 0x01, sizeof(in_buf));
  memset(out_buf, 0, sizeof(out_buf));
  memcpy(in_buf, aDevEui, 8);

  memset(aes_ctx.ksch, '\0', 240);
  aes_set_key(key, 16, &aes_ctx);
  laes_encrypt(in_buf, out_buf, &aes_ctx);
  memset(aes_ctx.ksch, '\0', 240);

  memcpy(aAppKey, out_buf, sizeof(out_buf));

  // Nwk Key
  memcpy(key, &aSharedKey[32], sizeof(key));
  memset(in_buf, 0x02, sizeof(in_buf));
  memset(out_buf, 0, sizeof(out_buf));
  memcpy(in_buf, aDevEui, 8);

  memset(aes_ctx.ksch, '\0', 240);
  aes_set_key(key, 16, &aes_ctx);
  laes_encrypt(in_buf, out_buf, &aes_ctx);
  memset(aes_ctx.ksch, '\0', 240);

  memcpy(aNwkKey, out_buf, sizeof(out_buf));

  // Prov Key
  memcpy(&key[0], &aSharedKey[16], 8);
  memcpy(&key[8], &aSharedKey[48], 8);
  memset(in_buf, 0x03, sizeof(in_buf));
  memset(out_buf, 0, sizeof(out_buf));
  memcpy(in_buf, aDevEui, 8);

  memset(aes_ctx.ksch, '\0', 240);
  aes_set_key(key, 16, &aes_ctx);
  laes_encrypt(in_buf, out_buf, &aes_ctx);
  memset(aes_ctx.ksch, '\0', 240);

  memcpy(aProvKey, out_buf, sizeof(out_buf));
}

//==========================================================================
// Init
//==========================================================================
void DevProvisionInit(const char *aProvisionId, const uint8_t *aProvisionIdHash) {
  uint32_t rand_num;

  gProvisionId = aProvisionId;
  gProvisionIdHash = aProvisionIdHash;

  memset(&gEcdhData, 0, sizeof(gEcdhData));
  for (uint8_t i = 0; i < sizeof(gEcdhData.devEui); i++) {
    SecureElementRandomNumber(&rand_num);
    gEcdhData.devEui[i] = rand_num;
  }
  for (uint8_t i = 0; i < sizeof(gEcdhData.privateKey); i++) {
    SecureElementRandomNumber(&rand_num);
    gEcdhData.privateKey[i] = rand_num;
  }
  ecdh_generate_keys(gEcdhData.pubKey, gEcdhData.privateKey);

  for (uint8_t i = 0; i < sizeof(gEcdhData.devNonce); i++) {
    SecureElementRandomNumber(&rand_num);
    gEcdhData.devNonce[i] = rand_num;
  }
}

//==========================================================================
// Get the fixed key
//==========================================================================
void DevProvisionGetFixedKey(uint8_t *aDest, uint16_t aDestSize) {
  aes_context aes_ctx;
  uint8_t out_buf[16];
  memset(aDest, 0, sizeof(aDestSize));

  memset(aes_ctx.ksch, '\0', 240);
  aes_set_key(kEpromKey, 16, &aes_ctx);
  laes_encrypt(kEncFixedKey, out_buf, &aes_ctx);
  memset(aes_ctx.ksch, '\0', 240);

  if (aDestSize > sizeof(out_buf)) {
    memcpy(aDest, out_buf, sizeof(out_buf));
  } else {
    memcpy(aDest, out_buf, aDestSize);
  }
}

//==========================================================================
// Calculate Provisioning Verify Code
//   aProvisionId 20 bytes, aNonce 4 bytes
//==========================================================================
void CalVerifyCode(uint8_t *aDest, uint32_t aDestSize, const char *aProvisionId, const uint8_t *aNonce) {
  uint8_t cal_buf[32];
  uint32_t cal_buf_len;
  uint32_t sn_len = strlen(aProvisionId);

  memset(aDest, 0, aDestSize);
  if (sn_len > sizeof (cal_buf)) {
    sn_len = sizeof (cal_buf);
  }
  memcpy(&cal_buf[0], aProvisionId, sn_len);
  memcpy(&cal_buf[sn_len], aNonce, 4);
  cal_buf_len = sn_len + 4;

  //
  uint8_t fixed_key[16];
  DevProvisionGetFixedKey(fixed_key, sizeof(fixed_key));

  //
  uint8_t cal_cmac[AES_CMAC_DIGEST_LENGTH];
  AES_CMAC_CTX aes_ctx;

  AES_CMAC_Init(&aes_ctx);
  AES_CMAC_SetKey(&aes_ctx, fixed_key);
  AES_CMAC_Update(&aes_ctx, cal_buf, cal_buf_len);
  AES_CMAC_Final(cal_cmac, &aes_ctx);

  if (aDestSize > AES_CMAC_DIGEST_LENGTH) {
    memcpy(aDest, cal_cmac, AES_CMAC_DIGEST_LENGTH);
  } else {
    memcpy(aDest, cal_cmac, aDestSize);
  }
}

//==========================================================================
// Encrypt / Decrypt payload
//==========================================================================
static void EncryptPayload(uint8_t *aPayload, uint32_t aPayloadLen, const uint8_t *aKey, const uint8_t *aDevEui, uint8_t aDir) {
  uint8_t a_block[16];
  uint8_t s_block[16];
  uint8_t block_counter = 1;
  aes_context aes_ctx;

  memset(a_block, 0, sizeof(a_block));
  memset(s_block, 0, sizeof(s_block));

  a_block[0] = 0x02;
  a_block[5] = aDir;
  memcpy(&a_block[6], aDevEui, 8);

  while (aPayloadLen > 0) {
    a_block[15] = block_counter;

    memset(aes_ctx.ksch, '\0', 240);
    aes_set_key(aKey, 16, &aes_ctx);
    laes_encrypt(a_block, s_block, &aes_ctx);

    uint32_t xor_len = aPayloadLen;
    if (xor_len > 16) {
      xor_len = 16;
    }

    for (uint32_t i = 0; i < xor_len; i++) {
      *aPayload ^= s_block[i];
      aPayload++;
    }
    aPayloadLen -= xor_len;
    block_counter ++;
  }
  memset(aes_ctx.ksch, '\0', 240);
}

//==========================================================================
// Prepare Hello
//==========================================================================
uint16_t DevProvisionPrepareHello(uint8_t *aDest, uint16_t aDestSize) {
  memset(aDest, 0, aDestSize);
  uint8_t *packing_ptr = aDest;

  if (aDestSize > SIZE_UP_MSG_HELLO) {
    *packing_ptr = UP_MESSAGE_HELLO;  // Message Type
    packing_ptr++;

    memcpy(packing_ptr, gEcdhData.devEui, sizeof(gEcdhData.devEui));
    packing_ptr += sizeof(gEcdhData.devEui);

    memcpy(packing_ptr, gEcdhData.pubKey, sizeof(gEcdhData.pubKey));
    packing_ptr += sizeof(gEcdhData.pubKey);

    *packing_ptr = 0x01;  // Version
    packing_ptr++;

    *packing_ptr = VALUE_MAINNET;
    packing_ptr++;

    LORACOMPON_HEX2STRING("ECDH DevEUI:", gEcdhData.devEui, sizeof(gEcdhData.devEui));
  }
  return (packing_ptr - aDest);
}

//==========================================================================
// Process the Hello response
//==========================================================================
int8_t DevProvisionHelloResp(const uint8_t *aResp, uint16_t aRespLen) {
  int8_t ret = -1;

  if (aRespLen == SIZE_DOWN_RESP_HELLO) {
    if (memcmp(&aResp[1], gEcdhData.devEui, LORA_EUI_LENGTH) == 0) {
      memcpy(gEcdhData.serverPubKey, &aResp[9], ECC_PUB_KEY_SIZE);
      memcpy(gEcdhData.serverNonce, &aResp[73], sizeof(gEcdhData.serverNonce));
      ret = 0;
    }
  }

  return ret;
}

//==========================================================================
// Prepare Auth
//==========================================================================
uint16_t DevProvisionPrepareAuth(uint8_t *aDest, uint16_t aDestSize) {
  memset(aDest, 0, aDestSize);
  uint8_t *packing_ptr = aDest;

  if (aDestSize > SiZE_UP_MSG_AUTH) {
    *packing_ptr = UP_MESSAGE_AUTH;  // Message Type
    packing_ptr++;

    memcpy(packing_ptr, gEcdhData.devEui, sizeof(gEcdhData.devEui));
    packing_ptr += sizeof(gEcdhData.devEui);

    //
    uint8_t enc_payload[52];
    uint8_t *enc_ptr = enc_payload;
    uint8_t verify_code[16];

    for (uint8_t i = 0; i < sizeof(enc_payload); i++) {
      uint32_t rand_num;
      SecureElementRandomNumber(&rand_num);
      enc_payload[i] = rand_num;
    }
    CalVerifyCode(verify_code, sizeof(verify_code), gProvisionId, gEcdhData.serverNonce);
    LORACOMPON_HEX2STRING("devNonce: ", gEcdhData.devNonce, sizeof (gEcdhData.devNonce));
    LORACOMPON_HEX2STRING("serverNonce: ", gEcdhData.serverNonce, sizeof (gEcdhData.serverNonce));
    LORACOMPON_HEX2STRING("gProvisionId: ", (const uint8_t *)gProvisionId, strlen (gProvisionId));
    LORACOMPON_HEX2STRING("verify_code: ", verify_code, sizeof (verify_code));

    memcpy(enc_ptr, gProvisionIdHash, 32);
    enc_ptr += 32;

    memcpy(enc_ptr, verify_code, sizeof(verify_code));
    enc_ptr += sizeof(verify_code);

    memcpy(enc_ptr, gEcdhData.devNonce, sizeof(gEcdhData.devNonce));
    enc_ptr += sizeof(gEcdhData.devNonce);

    EncryptPayload(enc_payload, sizeof(enc_payload), gEcdhData.provKey, gEcdhData.devEui, 0);

    // LORACOMPON_HEX2STRING("enc_payload: ", enc_payload, sizeof (enc_payload));

    //
    memcpy(packing_ptr, enc_payload, sizeof(enc_payload));
    packing_ptr += sizeof(enc_payload);
  }
  return (packing_ptr - aDest);
}

//==========================================================================
// Process the Auth response
//==========================================================================
int8_t DevProvisionAuthResp(const uint8_t *aResp, uint16_t aRespLen) {
  int8_t ret = -1;

  if (aRespLen == SIZE_DOWN_RESP_AUTH_ACCEPT) {
    if (memcmp(&aResp[1], gEcdhData.devEui, LORA_EUI_LENGTH) == 0) {
      memcpy(gPayloadBuf, &aResp[9], sizeof(gPayloadBuf));
      EncryptPayload(gPayloadBuf, sizeof(gPayloadBuf), gEcdhData.provKey, gEcdhData.devEui, 1);

      uint8_t *assigned_dev_eui = &gPayloadBuf[0];
      uint8_t *assigned_join_eui = &gPayloadBuf[8];
      uint8_t *auth_verify_code = &gPayloadBuf[16];
      // LORACOMPON_HEX2STRING("  devEUI:", assigned_dev_eui, LORA_EUI_LENGTH);
      // LORACOMPON_HEX2STRING("  joinEUI:", assigned_join_eui, LORA_EUI_LENGTH);
      // LORACOMPON_HEX2STRING("  verifyCode:", auth_verify_code, 16);

      uint8_t verify_code[16];
      CalVerifyCode(verify_code, sizeof(verify_code), gProvisionId, gEcdhData.devNonce);
      LORACOMPON_HEX2STRING("  Cal verifyCode:", verify_code, sizeof(verify_code));

      if (memcmp(verify_code, auth_verify_code, sizeof(verify_code)) != 0) {
        // Incorrest Verify Code
        LORACOMPON_PRINTLINE("  Incorrect verify code.");
      } else {
        // Verify code is good
        memcpy(gEcdhData.assignedDevEui, assigned_dev_eui, LORA_EUI_LENGTH);
        memcpy(gEcdhData.assignedJoinEui, assigned_join_eui, LORA_EUI_LENGTH);
        LORACOMPON_PRINTLINE("  Provision done.");
        ret = 0;
      }
    }
  }

  return ret;
}

//==========================================================================
// Generate keys
//==========================================================================
void DevProvisionGenKeys(void) {
  ecdh_shared_secret(gEcdhData.privateKey, gEcdhData.serverPubKey, gEcdhData.sharedKey);
  DeriveKey(gEcdhData.appKey, gEcdhData.nwkKey, gEcdhData.provKey, gEcdhData.devEui, gEcdhData.sharedKey);

  LORACOMPON_HEX2STRING("  Server PubKey:", gEcdhData.serverPubKey, ECC_PUB_KEY_SIZE);
  LORACOMPON_HEX2STRING("  ECDH PrivateKey:", gEcdhData.privateKey, ECC_PRV_KEY_SIZE);
  LORACOMPON_HEX2STRING("  ECDH PubKey:", gEcdhData.pubKey, ECC_PUB_KEY_SIZE);
  LORACOMPON_HEX2STRING("  sharedKey:", gEcdhData.sharedKey, ECC_PUB_KEY_SIZE);
  LORACOMPON_HEX2STRING("  provKey:", gEcdhData.provKey, sizeof(gEcdhData.provKey));
}

//==========================================================================
// Compare with Dev EUI that using for device provisioning
//==========================================================================
int8_t DevProvisionCmpDevEui(const uint8_t *aEui) {
  return memcmp(aEui, gEcdhData.devEui, sizeof(gEcdhData.devEui));
}

//==========================================================================
// Get Result
//==========================================================================
const uint8_t *DevProvisioningGetAssignedDevEui(void) { return gEcdhData.assignedDevEui; }

const uint8_t *DevProvisioningGetAssignedJoinEui(void) { return gEcdhData.assignedJoinEui; }

const uint8_t *DevProvisioningGetAssignedNwkKey(void) { return gEcdhData.nwkKey; }

const uint8_t *DevProvisioningGetAssignedAppKey(void) { return gEcdhData.appKey; }