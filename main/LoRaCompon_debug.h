//==========================================================================
//==========================================================================
#ifndef INC_LORACOMPON_DEBUG_H
#define INC_LORACOMPON_DEBUG_H

//==========================================================================
//==========================================================================
#if defined(CONFIG_LORACOMPON_DEBUG)
#define LORACOMPON_DEBUG 1
#else
#define LORACOMPON_DEBUG 0
#endif

#if LORACOMPON_DEBUG
#include <stdint.h>
#include <stdio.h>

void LoraComponDebugPrintLine(const char *szFormat, ...);
void LoraComponDebugHex2String(const char *aPrefix, const uint8_t *aSrc, int aLen);

#define LORACOMPON_PRINTLINE(x...) LoraComponDebugPrintLine(x)
#define LORACOMPON_HEX2STRING(x, y, z) LoraComponDebugHex2String(x, y, z)

#else  // LORACOMPON_DEBUG

#define LORACOMPON_PRINTLINE(...) \
  {}
#define LORACOMPON_HEX2STRING(...) \
  {}

#endif  // LORACOMPON_DEBUG

//==========================================================================
//==========================================================================

//==========================================================================
//==========================================================================
#endif // INC_LORACOMPON_DEBUG_H
