//==========================================================================
// Application Data
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
#include "lora_data.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "LoRaCompon_debug.h"
#include "lora_mutex_helper.h"
#include "nvs_flash.h"

//==========================================================================
// Defines
//==========================================================================
// Key list, max 15 character
#define KEY_LORA_DATA "lora_data"

// Magic code
#define MAGIC_LORA_DATA    0xa38d72f1

//==========================================================================
// Variables
//==========================================================================
static bool gNvsReady;
static LoRaData_t gLoRaData;

//==========================================================================
// Constants
//==========================================================================
static const char *kStorageNamespace = "MatchX";

//==========================================================================
// Calculate XOR value
//==========================================================================
static uint8_t CalXorValue (const void *aData, uint32_t aLen) {
  const uint8_t *ptr = (const uint8_t *)aData;
  uint8_t cal_xor = 0xAA; // Initial value
  while (aLen > 0) {
    cal_xor ^= *ptr;
    ptr ++;
    aLen --;
  }
  return cal_xor;
}

//==========================================================================
// Init
//   nvs_flash_init() must called before this.
//==========================================================================
int8_t LoRaDataInit(void) {
  esp_err_t esp_ret;

  //
  gNvsReady = false;

  // Init variables
  InitMutex();

  //
  nvs_handle h_matchx;

  esp_ret = nvs_open(kStorageNamespace, NVS_READONLY, &h_matchx);
  if (esp_ret == ESP_ERR_NVS_NOT_FOUND) {
    printf("WARNING. LoRaDataInit namespace not found. Create.\n");
    // No namespace, create one
    gNvsReady = true;
    if (LoRaDataResetToDefault() < 0) {
      gNvsReady = false;
      return -1;
    } else {
      return 0;
    }
  } else if (esp_ret != ESP_OK) {
    printf("ERROR. LoRaDataInit nvs_open() failed. %s.\n", esp_err_to_name(esp_ret));
    return -1;
  } else {
    esp_err_t esp_ret;
    size_t len;

    gNvsReady = true;

    // gLoRaData
    len = sizeof(gLoRaData);
    esp_ret = nvs_get_blob(h_matchx, KEY_LORA_DATA, &gLoRaData, &len);
    if (esp_ret != ESP_OK) {
      printf("ERROR. LoRaDataInit get %s failed. %s.\n", KEY_LORA_DATA, esp_err_to_name(esp_ret));
      LoRaDataResetToDefault();
    } else {
      printf("INFO. Read LoRaData success.\n");

      // Check XOR
      uint8_t cal_xor = CalXorValue(&gLoRaData, sizeof (gLoRaData));
      if (cal_xor != 0) {
        printf ("WARNING. Invalid LoRaData, set to default.\n");
        LoRaDataResetToDefault();
      }
    }

    //
    nvs_close(h_matchx);
    return 0;
  }
}

//==========================================================================
// Read LoRa setting
//==========================================================================
int8_t LoRaDataReadSettings (LoRaSettings_t *aSettings) {
  if (TakeMutex()) {
    memcpy (aSettings, &gLoRaData.settings, sizeof (LoRaSettings_t));
    FreeMutex();
    return 0;
  }
  else {
    return -1;
  }
}

//==========================================================================
// Save LoRa setting
//==========================================================================
int8_t LoRaDataSaveSettings (const LoRaSettings_t *aSettings){
  esp_err_t esp_ret;
  nvs_handle h_matchx;

  // Check NVS is ready
  if (!gNvsReady) {
    printf("ERROR. LoRaDataSaveToStorage failed. NVS not ready.\n");
    return -1;
  }

  // Open storage
  esp_ret = nvs_open(kStorageNamespace, NVS_READWRITE, &h_matchx);
  if (esp_ret != ESP_OK) {
    printf("ERROR. LoRaDataSaveToStorage nvs_open(), %s.\n", esp_err_to_name(esp_ret));
    return -1;
  } else {
    int ret = -1;
    if (TakeMutex()) {
      // Copy the settings
      memcpy (&gLoRaData.settings, aSettings, sizeof (LoRaSettings_t));

      // Set magic code anyway
      gLoRaData.magicCode = MAGIC_LORA_DATA;

      // Update XOR
      gLoRaData.xorValue = 0;
      uint8_t cal_xor = CalXorValue(&gLoRaData, sizeof (gLoRaData));
      gLoRaData.xorValue = cal_xor;

      //
      esp_err_t esp_ret = nvs_set_blob(h_matchx, KEY_LORA_DATA, &gLoRaData, sizeof(gLoRaData));
      if (esp_ret != ESP_OK) {
        printf("ERROR. LoRaDataSaveToStorage set %s failed. %s.\n", KEY_LORA_DATA, esp_err_to_name(esp_ret));
      } else {
        // All done
        ret = 0;
      }
      FreeMutex();
    }

    // Commit and close
    if (nvs_commit(h_matchx) != ESP_OK) {
      printf("ERROR. LoRaDataSaveToStorage nvs commit failed. %s.\n", esp_err_to_name(ret));
    }
    nvs_close(h_matchx);
    return ret;
  }
}

//==========================================================================
//==========================================================================
int8_t LoRaDataResetToDefault(void) {
  LoRaSettings_t default_settings; 
  memset (&default_settings, 0, sizeof (default_settings));

  // Get MAC address
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);

  // Default setting
  default_settings.provisionDone = false;
  default_settings.devEui[0] = mac[0];
  default_settings.devEui[1] = mac[1];
  default_settings.devEui[2] = mac[2];
  default_settings.devEui[3] = 0xff;
  default_settings.devEui[4] = 0xfe;
  default_settings.devEui[5] = mac[3];
  default_settings.devEui[6] = mac[4];
  default_settings.devEui[7] = mac[5];

  memset(default_settings.joinEui, 0x00, LORA_EUI_LENGTH);
  memset(default_settings.nwkKey, 0x01, LORA_KEY_LENGTH);
  memset(default_settings.appKey, 0x02, LORA_KEY_LENGTH);

  int ret = LoRaDataSaveSettings(&default_settings);
  if (ret == 0) {
    printf("INFO. Set LoRa data to default.");
  }
  return ret;
}
