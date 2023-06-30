//==========================================================================
//==========================================================================
#ifndef INC_LORARADIO_DEBUG_H
#define INC_LORARADIO_DEBUG_H

//==========================================================================
//==========================================================================
#define LORARADIO_DEBUG 1

#if LORARADIO_DEBUG
#include <stdint.h>
#include <stdio.h>

void LoraRadioDebugPrintLine(const char *szFormat, ...);
void LoraRadioDebugHex2String(const char *aPrefix, const uint8_t *aSrc, int aLen);

#define LORARADIO_PRINTLINE(x...) LoraRadioDebugPrintLine(x)
#define LORARADIO_HEX2STRING(x, y, z) LoraRadioDebugHex2String(x, y, z)

#else  // LORARADIO_DEBUG

#define LORARADIO_PRINTLINE(...) \
  {}
#define LORARADIO_HEX2STRING(...) \
  {}

#endif  // LORARADIO_DEBUG

//==========================================================================
//==========================================================================

//==========================================================================
//==========================================================================
#endif
