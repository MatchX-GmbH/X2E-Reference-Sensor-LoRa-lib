//==========================================================================
//==========================================================================
#ifndef INC_LORA_MUTEX_HELPER_H
#define INC_LORA_MUTEX_HELPER_H
//==========================================================================
//==========================================================================
#include <stddef.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

//==========================================================================
//==========================================================================
#define MUTEX_WAIT_TIME 50
static SemaphoreHandle_t gMutex;

//==========================================================================
// Mutex Lock helper
//==========================================================================
static bool TakeMutex(void) {
  if (gMutex != NULL) {
    if (xSemaphoreTake(gMutex, MUTEX_WAIT_TIME / portTICK_PERIOD_MS) ==
        pdTRUE) {
      return true;
    }
  }
  return false;
}
static void FreeMutex(void) { xSemaphoreGive(gMutex); }

static void InitMutex (void) {
  gMutex = NULL;

  gMutex = xSemaphoreCreateMutex();
  if (gMutex == NULL) {
    printf("ERROR. xSemaphoreCreateMutex() failed.\n");
  }
}

//==========================================================================
//==========================================================================
#endif  // INC_LORA_MUTEX_HELPER_H
