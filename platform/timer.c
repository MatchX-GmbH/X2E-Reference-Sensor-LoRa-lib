//==========================================================================
// Timer functions for LoRa MAC
//==========================================================================
//  Copyright (c) MatchX GmbH.  All rights reserved.
//==========================================================================
// Naming conventions
// ~~~~~~~~~~~~~~~~~~
//                Class : Leading C
//               Struct : Leading T
//       typedef Struct : tailing _t
//             Constant : Leading k
//      Global Variable : Leading g
//    Function argument : Leading a
//       Local Variable : All lower case
//==========================================================================
#include "timer.h"

#include <sys/time.h>
#include <time.h>

#include "board.h"
#include "esp_debug_helpers.h"
#include "esp_timer.h"
#include "rtc-board.h"
#include "utilities.h"
#include "LoRaPlatform_debug.h"

//==========================================================================
//==========================================================================
#define MAX_NUM_OF_TIMER 16
static TimerEvent_t *gTimerList[MAX_NUM_OF_TIMER];

//==========================================================================
// Tick
//==========================================================================
uint32_t LoRaGetTick(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

uint32_t LoRaTickElapsed(uint32_t aTick) {
  uint32_t curr_tick;

  curr_tick = LoRaGetTick();
  if (curr_tick >= aTick) {
    return (curr_tick - aTick);
  } else {
    return (0xffffffff - aTick + curr_tick);
  }
}

//==========================================================================
//==========================================================================
void TimerInsertTimer(TimerEvent_t *obj) {
  // Find a blank slot
  for (int i = 0; i < MAX_NUM_OF_TIMER; i++) {
    if (gTimerList[i] == NULL) {
      gTimerList[i] = obj;
      return;
    }
  }

  // No free slot
  printf("ERROR. TimerInsertTimer no free slot.");
}

//==========================================================================
//==========================================================================
bool TimerExists(TimerEvent_t *obj) {
  TimerEvent_t *cur = NULL;
  for (int i = 0; i < MAX_NUM_OF_TIMER; i++) {
    cur = gTimerList[i];
    if (cur == obj) {
      break;
    }
  }
  if (cur == obj) {
    return true;
  } else {
    return false;
  }
}

//==========================================================================
//==========================================================================
void TimerPowerUpInit(void) {
  for (int i = 0; i < MAX_NUM_OF_TIMER; i++) {
    gTimerList[i] = NULL;
  }
}

//==========================================================================
//==========================================================================
void TimerInit(TimerEvent_t *obj, void (*callback)(void *context)) {
  obj->Timestamp = 0;
  obj->ReloadValue = 0;
  obj->IsStarted = false;
  obj->IsNext2Expire = false;
  obj->Callback = callback;
  obj->Context = NULL;
  obj->Next = NULL;
}

void TimerSetContext(TimerEvent_t *obj, void *context) { obj->Context = context; }

//==========================================================================
//==========================================================================
void TimerStart(TimerEvent_t *obj) {
  CRITICAL_SECTION_BEGIN();

  if (obj != NULL) {
    // esp_backtrace_print(6);
    obj->Timestamp = LoRaGetTick();
    obj->IsStarted = true;
    obj->IsNext2Expire = false;

    // Find obj from timer list
    TimerEvent_t *cur = NULL;
    for (int i = 0; i < MAX_NUM_OF_TIMER; i++) {
      cur = gTimerList[i];
      if (cur == obj) {
        break;
      }
    }
    if (cur != obj) {
      TimerInsertTimer(obj);
    }
  }

  CRITICAL_SECTION_END();
}

//==========================================================================
//==========================================================================
bool TimerIsStarted(TimerEvent_t *obj) { return obj->IsStarted; }

//==========================================================================
//==========================================================================
void TimerIrqHandler(void) {
  for (int i = 0; i < MAX_NUM_OF_TIMER; i++) {
    void (*callback)(void *) = NULL;
    void *callback_context = NULL;

    CRITICAL_SECTION_BEGIN();
    if (gTimerList[i] != NULL) {
      if (gTimerList[i]->IsStarted) {
        if (LoRaTickElapsed(gTimerList[i]->Timestamp) >= gTimerList[i]->ReloadValue) {
          gTimerList[i]->Timestamp = LoRaGetTick();
          gTimerList[i]->IsStarted = false;
          if (gTimerList[i]->Callback == NULL) {
            printf("WARN. TimerIrqHandler missing callback on slot %d.", i);
          } else {
            callback = gTimerList[i]->Callback;
            callback_context = gTimerList[i]->Context;
          }
        }
      }
    }
    CRITICAL_SECTION_END();

    //
    if (callback != NULL) {
      // LORAPLATFORM_PRINTLINE("Timer triggered.");
      callback(callback_context);
    }
  }
}

//==========================================================================
//==========================================================================
void TimerStop(TimerEvent_t *obj) {
  CRITICAL_SECTION_BEGIN();

  // Find obj from timer list
  TimerEvent_t *cur = NULL;
  for (int i = 0; i < MAX_NUM_OF_TIMER; i++) {
    cur = gTimerList[i];
    if (cur == obj) {
      break;
    }
  }
  if (cur == obj) {
    cur->IsStarted = false;
  }
  CRITICAL_SECTION_END();
}

//==========================================================================
//==========================================================================
void TimerReset(TimerEvent_t *obj) {
  TimerStop(obj);
  TimerStart(obj);
}

//==========================================================================
//==========================================================================
void TimerSetValue(TimerEvent_t *obj, uint32_t value) {
  uint32_t minValue = 0;
  TimerStop(obj);

  minValue = 10;
  if (value < minValue) {
    value = minValue;
  }

  obj->Timestamp = LoRaGetTick();
  obj->ReloadValue = value;
}

//==========================================================================
//==========================================================================
TimerTime_t TimerGetCurrentTime(void) { return LoRaGetTick(); }

//==========================================================================
//==========================================================================
TimerTime_t TimerGetElapsedTime(TimerTime_t past) { return LoRaTickElapsed(past); }

//==========================================================================
//==========================================================================
TimerTime_t TimerTempCompensation(TimerTime_t period, float temperature) { return period; }
