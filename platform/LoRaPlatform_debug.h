//==========================================================================
//==========================================================================
#ifndef INC_LORAPLATFORM_DEBUG_H
#define INC_LORAPLATFORM_DEBUG_H

//==========================================================================
//==========================================================================
#define LORAPLATFORM_DEBUG 0

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
