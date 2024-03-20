//==========================================================================
// Radio additional
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
#include "radio.h"

#include <string.h>

#include "LoRaRadio_debug.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

//==========================================================================
//==========================================================================
#define DelayMs(x) vTaskDelay(x / portTICK_PERIOD_MS)

//==========================================================================
//==========================================================================
// use by LoRa MAC
struct Radio_s Radio;
const struct Radio_s RadioSx126x;
const struct Radio_s RadioSx1280;

//
static RadioChip_t gCurrentChip;

//==========================================================================
// HAL Prototypes
//==========================================================================
bool SX126xIsError(void);
void SX126xIoInit(void);
bool SX1280IsError(void);
void SX1280HalInit(void);

//==========================================================================
// Select active Radio chip
//==========================================================================
void RadioSelectChip(RadioChip_t aRadioType) {
  memset(&Radio, 0, sizeof(struct Radio_s));
  if (aRadioType == RADIO_CHIP_SX1280) {
    LORARADIO_PRINTLINE("RADIO_CHIP_SX1280 Selected");
    memcpy(&Radio, &RadioSx1280, sizeof(struct Radio_s));
    gCurrentChip = RADIO_CHIP_SX1280;
  } else {
    LORARADIO_PRINTLINE("RADIO_CHIP_SX126X Selected");
    memcpy(&Radio, &RadioSx126x, sizeof(struct Radio_s));
    gCurrentChip = RADIO_CHIP_SX126X;
  }
}

//==========================================================================
// Check Radio chip error
//==========================================================================
void RadioHandleChipError(void) {
  if (SX1280IsError()) {
    LORARADIO_PRINTLINE("SX1280 chip error. Try init again.");
    SX1280HalInit();
  }
  if (SX126xIsError()) {
    LORARADIO_PRINTLINE("SX126x chip error. Try init again.");
    SX126xIoInit();
  }
}