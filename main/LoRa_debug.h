//==========================================================================
//==========================================================================
#ifndef INC_LORARADIO_DEBUG_H
#define INC_LORARADIO_DEBUG_H

//==========================================================================
//==========================================================================
#define LORA_DEBUG 1

#if LORA_DEBUG
#include <stdint.h>
#include <stdio.h>

void LoraDebugPrintLine(const char *szFormat, ...);
void LoraDebugHex2String(const char *aPrefix, const uint8_t *aSrc, int aLen);

#define LORA_PRINTLINE(x...) LoraDebugPrintLine(x)
#define LORA_HEX2STRING(x, y, z) LoraDebugHex2String(x, y, z)

#else  // LORA_DEBUG

#define LORARADIO_PRINTLINE(...) \
  {}
#define LORARADIO_HEX2STRING(...) \
  {}

#endif  // LORA_DEBUG

//==========================================================================
//==========================================================================

//==========================================================================
//==========================================================================
#endif
