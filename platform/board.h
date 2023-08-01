//==========================================================================
//==========================================================================
#ifndef LORA_INC_BOARD_H
#define LORA_INC_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

//==========================================================================
//==========================================================================
#include <stdint.h>

//==========================================================================
//==========================================================================
void LoRaBoardInitMcu(void);
void LoRaBoardGetUniqueId(uint8_t *id);
void LoRaBoardCriticalSectionBegin(void);
void LoRaBoardCriticalSectionEnd(void);

void LoRaBoardPrepareForSleep(void);
void LoRaBoardResumeFromSleep(void);

//==========================================================================
//==========================================================================
#ifdef __cplusplus
}
#endif

#endif  // LORA_INC_BOARD_H
