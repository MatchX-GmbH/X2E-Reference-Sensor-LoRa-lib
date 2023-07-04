//==========================================================================
//==========================================================================
#ifndef INC_LORAPLATFORM_DEBUG_H
#define INC_LORAPLATFORM_DEBUG_H

//==========================================================================
//==========================================================================
#if defined(CONFIG_LORAPLATFORM_DEBUG)
#define LORAPLATFORM_DEBUG 1
#else
#define LORAPLATFORM_DEBUG 0
#endif

#if LORAPLATFORM_DEBUG
#include <stdint.h>
#include <stdio.h>

void LoraPlatformDebugPrintLine(const char *szFormat, ...);
void LoraPlatformDebugHex2String(const char *aPrefix, const uint8_t *aSrc, int aLen);

#define LORAPLATFORM_PRINTLINE(x...) LoraPlatformDebugPrintLine(x)
#define LORAPLATFORM_HEX2STRING(x, y, z) LoraPlatformDebugHex2String(x, y, z)

#else  // LORAPLATFORM_DEBUG

#define LORAPLATFORM_PRINTLINE(...) \
  {}
#define LORAPLATFORM_HEX2STRING(...) \
  {}

#endif  // LORAPLATFORM_DEBUG

//==========================================================================
//==========================================================================

//==========================================================================
//==========================================================================
#endif // INC_LORAPLATFORM_DEBUG_H
