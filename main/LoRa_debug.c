
//==========================================================================
//==========================================================================
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

#include "LoRa_debug.h"

//==========================================================================
//==========================================================================
#if LORA_DEBUG

static const char *kDebugPrefix = "[LORA]";

//==========================================================================
//==========================================================================
void LoraDebugPrintLine(const char *szFormat, ...) {
  va_list lpStart;
  va_start(lpStart, szFormat);
  printf(kDebugPrefix);
  vprintf(szFormat, lpStart);
  printf("\n");
  va_end(lpStart);
}

void LoraDebugHex2String(const char *aPrefix, const uint8_t *aSrc, int aLen) {
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
#endif  // LORARADIO_DEBUG
