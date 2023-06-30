
//==========================================================================
//==========================================================================
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

#include "LoRaPlatform_debug.h"

//==========================================================================
//==========================================================================
#if LORAPLATFORM_DEBUG

static const char *kDebugPrefix = "[PLATFORM]";

//==========================================================================
//==========================================================================
void LoraPlatformDebugPrintLine(const char *szFormat, ...) {
  va_list lpStart;
  va_start(lpStart, szFormat);
  printf(kDebugPrefix);
  vprintf(szFormat, lpStart);
  printf("\n");
  va_end(lpStart);
}

void LoraPlatformDebugHex2String(const char *aPrefix, const uint8_t *aSrc, int aLen) {
  printf(kDebugPrefix);
  printf("%s", aPrefix);
  while (aLen) {
    printf(" %02X", *aSrc);
    aLen--;
    aSrc++;
  }
  printf("\n");
}

//==========================================================================
//==========================================================================
#endif  // LORAPLATFORM_DEBUG
