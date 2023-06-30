#include "rtc-board.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "board.h"
#include "delay.h"
#include "systime.h"
#include "timer.h"
#include "utilities.h"


//==========================================================================
//==========================================================================
typedef struct {
    uint32_t Time;  // Reference time
    // pthread_t ThreadId;
    bool ThreadValid;
    bool Running;
    uint32_t Period;
    uint32_t StartTick;
} RtcTimerContext_t;
static RtcTimerContext_t RtcTimerContext = {0, 0, false, false, 1, 0};

// static pthread_mutex_t gMutex = PTHREAD_MUTEX_INITIALIZER; /* control access to the concentrator */

//==========================================================================
// Implement this on App level. It will call when timer triggered
//==========================================================================
extern void TimerIrqAppFunc(void);
static void rtc_timer_cb(void)
{
    TimerIrqAppFunc();
}

//==========================================================================
//==========================================================================
// static uint32_t GetTick(void) {
//   uint32_t ret;
//   struct timeval curr_time;

//   gettimeofday(&curr_time, NULL);

//   ret = curr_time.tv_usec / 1000;
//   ret += (curr_time.tv_sec * 1000);

//   return ret;
// }

// static uint32_t TickElapsed(uint32_t aTick) {
//   uint32_t curr_tick;

//   curr_tick = GetTick();
//   if (curr_tick >= aTick)
//     return (curr_tick - aTick);
//   else
//     return (0xffffffff - aTick + curr_tick);
// }

// Emulation of a timer interrupt
void RtcTimerFunc(void)
{
    // for (;;) {
    //   pthread_mutex_lock(&gMutex);
    //   if (RtcTimerContext.Running) {
    //     if (RtcTimerContext.StartTick == 0) {
    //       RtcTimerContext.StartTick = GetTick();
    //     } else if (TickElapsed(RtcTimerContext.StartTick) >= RtcTimerContext.Period) {
    //       RtcTimerContext.Running = false;
    //       rtc_timer_cb();
    //     }
    //   } else {
    //     RtcTimerContext.StartTick = 0;
    //   }
    //   pthread_mutex_unlock(&gMutex);
    //   usleep(1000);
    // }
}

//==========================================================================
// Start/Stop RTC
//==========================================================================
void RtcInit(void)
{
    RtcTimerContext.Time = RtcGetTimerValue();
    // if (!RtcTimerContext.ThreadValid) {
    //   int i = pthread_create(&RtcTimerContext.ThreadId, NULL, (void *(*)(void *))RtcTimerFunc, NULL);
    //   if (i != 0) {
    //     perror("ERROR: fail to create RtcTimer thread");
    //   } else {
    //     RtcTimerContext.ThreadValid = true;
    //   }
    // }
}

void RtcDeInit(void)
{
    // pthread_cancel(RtcTimerContext.ThreadId);
    // RtcTimerContext.ThreadValid = false;
}

/*!
 * \brief Sets the RTC timer reference, sets also the RTC_DateStruct and RTC_TimeStruct
 *
 * \param none
 * \retval timerValue In ticks
 */
uint32_t RtcSetTimerContext(void)
{
    RtcTimerContext.Time = RtcGetTimerValue();
    return (uint32_t)RtcTimerContext.Time;
}

/*!
 * \brief Gets the RTC timer reference
 *
 * \param none
 * \retval timerValue In ticks
 */
uint32_t RtcGetTimerContext(void)
{
    return RtcTimerContext.Time;
}

/*!
 * \brief returns the wake up time in ticks
 *
 * \retval wake up time in ticks
 */
uint32_t RtcGetMinimumTimeout(void)
{
    return 50;
}

/*!
 * \brief converts time in ms to time in ticks
 *
 * \param[IN] milliseconds Time in milliseconds
 * \retval returns time in timer ticks
 */
uint64_t RtcMs2Tick(uint32_t milliseconds)
{
    return milliseconds;
}

/*!
 * \brief converts time in ticks to time in ms
 *
 * \param[IN] time in timer ticks
 * \retval returns time in milliseconds
 */
uint32_t RtcTick2Ms(uint64_t tick)
{
    return tick;
}

/*!
 * \brief a delay of delay ms by polling RTC
 *
 * \param[IN] delay in ms
 */
void RtcDelayMs(uint32_t delay)
{
    uint64_t delayTicks = 0;
    uint64_t refTicks = RtcGetTimerValue();

    delayTicks = RtcMs2Tick(delay);

    // Wait delay ms
    while (((RtcGetTimerValue() - refTicks)) < delayTicks) {
        usleep(10000);
    }
}

void RtcSetAlarm(uint32_t timeout)
{
    RtcStartAlarm(timeout);
}

void RtcStopAlarm(void)
{
    // pthread_mutex_lock(&gMutex);
    // RtcTimerContext.Running = false;
    // pthread_mutex_unlock(&gMutex);
}

void RtcStartAlarm(uint32_t timeout)
{
    // pthread_mutex_lock(&gMutex);
    // RtcTimerContext.Running = true;
    // RtcTimerContext.Period = timeout;
    // pthread_mutex_unlock(&gMutex);
}

uint64_t RtcGetTimerValue(void)
{
    struct timeval curr_time;
    gettimeofday(&curr_time, NULL);

    uint64_t tick_ms = curr_time.tv_usec / 1000;
    tick_ms += ((uint32_t)curr_time.tv_sec * 1000);
    return tick_ms;
}

uint32_t RtcGetTimerElapsedTime(void)
{
    return ((uint32_t)(RtcGetTimerValue() - RtcTimerContext.Time));
}

uint32_t RtcGetCalendarTime(uint16_t *milliseconds)
{
    struct timeval curr_time;
    gettimeofday(&curr_time, NULL);

    uint32_t tick_ms = curr_time.tv_usec / 1000;
    tick_ms += ((uint32_t)curr_time.tv_sec * 1000);

    *milliseconds = tick_ms;
    return curr_time.tv_sec;
}

void RtcBkupWrite(uint32_t seconds, uint32_t miliseconds)
{
    struct timeval curr_time;
    curr_time.tv_sec = seconds;
    curr_time.tv_usec = (miliseconds - (seconds * 1000)) * 1000;
    settimeofday(&curr_time, NULL);
}

void RtcBkupRead(uint32_t *data0, uint32_t *data1)
{
    // Not used on this platform.
}
