//==========================================================================
//==========================================================================
#ifndef INC_LORAMAC_DEBUG_H
#define INC_LORAMAC_DEBUG_H

//==========================================================================
//==========================================================================
#define LORAMAC_DEBUG 1

#if LORAMAC_DEBUG
#include <stdint.h>
#include <stdio.h>

void LoraMacDebugPrintLine(const char *szFormat, ...);
void LoraMacDebugHex2String(const char *aPrefix, const uint8_t *aSrc, int aLen);

#define LORAMAC_PRINTLINE(x...) LoraMacDebugPrintLine(x)
#define LORAMAC_HEX2STRING(x, y, z) LoraMacDebugHex2String(x, y, z)

#else  // LORAMAC_DEBUG

#define LORAMAC_PRINTLINE(...) \
  {}
#define LORAMAC_HEX2STRING(...) \
  {}

#endif  // LORAMAC_DEBUG

//==========================================================================
//==========================================================================

//==========================================================================
//==========================================================================
#endif
