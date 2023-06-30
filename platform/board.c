//==========================================================================
// Board functions for LoRa MAC
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
#include "board.h"

#include <stdio.h>
#include <unistd.h>

#include "LoRaPlatform_debug.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "radio.h"
#include "sx-gpio.h"
#include "timer.h"

//==========================================================================
//==========================================================================
#define TASK_PRIO_IRQ 16

//==========================================================================
// LoRa Chip HAL Prototypes
//==========================================================================
void SX126xIoInit(void);
void SX126xIoIrqInit(DioIrqHandler);
uint32_t SX126xGetDio1PinState(void);
void SX126xClearIrqStatus(int16_t);

void SX1280HalInit(void);
void SX1280HalIoIrqInit(DioIrqHandler);
uint8_t SX1280HalGetDioStatus(void);
void SX1280ClearIrqStatus(int16_t);

//==========================================================================
// IRQ handler of Radio.
//==========================================================================
DioIrqHandler* gSx126xDioIrqHandler = NULL;
DioIrqHandler* gSx1280DioIrqHandler = NULL;

//==========================================================================
// Timer interrupt
//==========================================================================
static esp_timer_handle_t gBoardTimer = NULL;
static xQueueHandle gLoRaDioEventQueue = NULL;

static void PeriodicTimerFunc(void* arg) { TimerIrqHandler(); }

//==========================================================================
// DIO interrupts for LoRa chips
//==========================================================================
void LoRaDioIsrHandler(void* arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gLoRaDioEventQueue, &gpio_num, NULL);
}

static void LoRaDioIrqTask(void* arg) {
  uint32_t io_num;
  for (;;) {
    if (xQueueReceive(gLoRaDioEventQueue, &io_num, portMAX_DELAY)) {
      // If DIO level keep high, keep running up to 50ms.
      for (uint8_t i = 0; i < 10; i++) {
        // Process SX1261
        if (SX126xGetDio1PinState() != 0) {
          if (gSx126xDioIrqHandler != NULL) {
            LORAPLATFORM_PRINTLINE("call gSx126xDioIrqHandler()");
            gSx126xDioIrqHandler(NULL);
          }
          // It is not current radio, an unexpected DIO IRQ
          if (Radio.IrqProcess != RadioSx126x.IrqProcess) {
            printf("ERROR. Unexpected DIO irq, SX1261.\n");
            SX126xClearIrqStatus(0xFFFF);
            RadioSx126x.Standby();
          }
        }

        // Process SX1280
        if (SX1280HalGetDioStatus() != 0) {
          if (gSx1280DioIrqHandler != NULL) {
            LORAPLATFORM_PRINTLINE("call gSx1280DioIrqHandler()");
            gSx1280DioIrqHandler(NULL);
          }

          // It is not current radio, an unexpected DIO IRQ
          if (Radio.IrqProcess != RadioSx1280.IrqProcess) {
            printf("ERROR. Unexpected DIO irq, SX1280.\n");
            SX1280ClearIrqStatus(0xFFFF);
            RadioSx1280.Standby();
          }
        }

        // Call radio IRQ
        if (Radio.IrqProcess == NULL) break;
        Radio.IrqProcess();
        vTaskDelay(1 / portTICK_PERIOD_MS);

        // DIO back to low, exit
        if ((SX126xGetDio1PinState() == 0) && (SX1280HalGetDioStatus() == 0)) break;

        // DIO keep high, wait a while and process again
        LORAPLATFORM_PRINTLINE("DIO still high.");
        vTaskDelay(4 / portTICK_PERIOD_MS);
      }
    }
  }
  vTaskDelete(NULL);
}
//==========================================================================
// Critical section
//==========================================================================
void LoRaBoardCriticalSectionBegin(void) { vTaskSuspendAll(); }

void LoRaBoardCriticalSectionEnd(void) { xTaskResumeAll(); }

//==========================================================================
//==========================================================================
void LoRaBoardGetUniqueId(uint8_t* id) {
  id[7] = 0;
  id[6] = 0;
  id[5] = 0;
  id[4] = 0;
  id[3] = 0;
  id[2] = 0;
  id[1] = 0;
  id[0] = 0;
  esp_efuse_mac_get_default(id);  // Fill first 6 bytes

  esp_chip_info_t info;
  esp_chip_info(&info);
  id[6] = info.model;
  id[7] = info.revision;
}

//==========================================================================
//==========================================================================
void LoRaBoardInitMcu(void) {
  // RtcInit();
  TimerPowerUpInit();

  // Create ESP timer
  if (gBoardTimer == NULL) {
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &PeriodicTimerFunc,
        .name = "periodic",
    };
    if (esp_timer_create(&periodic_timer_args, &gBoardTimer) != ESP_OK) {
      printf("ERROR. Failed to create periodic timer.\n");
      gBoardTimer = NULL;
    }
  }
  if (gBoardTimer != NULL) {
    // Kick start a 1ms periodic timer
    esp_timer_start_periodic(gBoardTimer, 1000);
    LORAPLATFORM_PRINTLINE("Periodic timer started.");
  }

  // LoRa Radio
  SX126xIoInit();
  SX126xIoIrqInit(NULL);

  SX1280HalInit();
  SX1280HalIoIrqInit(NULL);

  //
  if (gpio_install_isr_service(0) != ESP_OK) {
    printf("ERROR. Failed to install GPIO IRQ service.\n");
  }

  // Task to handle LoRa chip DIO IRQ
  gLoRaDioEventQueue = xQueueCreate(200, sizeof(uint32_t));
  if (xTaskCreate(LoRaDioIrqTask, "LoRaDioIrqTask", 2048, NULL, TASK_PRIO_IRQ, NULL) != pdPASS) {
    printf("ERROR. Failed to create LoRa DIO IRQ task.\n");
  }

  //
  gpio_set_intr_type(SX1261_DIO1, GPIO_INTR_POSEDGE);
  if (gpio_isr_handler_add(SX1261_DIO1, LoRaDioIsrHandler, (void*)SX1261_DIO1) != ESP_OK) {
    printf("ERROR. Failed to add SX1261 DIO1 IRQ handler.");
  }

  // // Setup LoRa chip DIO interrupt
  gpio_set_intr_type(SX1280_DIO1, GPIO_INTR_POSEDGE);
  if (gpio_isr_handler_add(SX1280_DIO1, LoRaDioIsrHandler, (void*)SX1280_DIO1) != ESP_OK) {
    printf("ERROR. Failed to add SX1280 DIO1 IRQ handler.");
  }
}

//==========================================================================
//==========================================================================
void LoRaBoardDeInitMcu(void) {
  if (gBoardTimer != NULL) {
    esp_timer_stop(gBoardTimer);
    LORAPLATFORM_PRINTLINE("Periodic timer stopped.");
  }

  // Clear all timer
  TimerPowerUpInit();
}