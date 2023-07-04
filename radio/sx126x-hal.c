//==========================================================================
//==========================================================================
#include "sx126x-hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "LoRaRadio_debug.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "radio.h"
#include "sx-gpio.h"

//==========================================================================
// Defines
//==========================================================================
#define MAX_HAL_BUFFER_SIZE 255

#define DelayMs(x) vTaskDelay(x / portTICK_PERIOD_MS)

//==========================================================================
// Variables
//==========================================================================
static bool gChipError = false;
static spi_device_handle_t gDevSx126x = NULL;

static RadioOperatingModes_t gOperatingMode;
static uint8_t halTxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};
static uint8_t halRxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};

extern DioIrqHandler *gSx126xDioIrqHandler;

//==========================================================================
//==========================================================================
void SX126xIoInit(void) {
  gChipError = false;

  // Register device
  if (gDevSx126x == NULL) {
    // Init GPIOs
    gpio_reset_pin(SX1261_SS);
    gpio_set_direction(SX1261_SS, GPIO_MODE_OUTPUT);
    gpio_set_level(SX1261_SS, 1);

    gpio_reset_pin(SX1261_BUSY);
    gpio_set_direction(SX1261_BUSY, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SX1261_BUSY, GPIO_FLOATING);
    gpio_reset_pin(SX1261_DIO1);
    gpio_set_direction(SX1261_DIO1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SX1261_DIO1, GPIO_FLOATING);

    gpio_reset_pin(SX1261_nRES);
    gpio_set_direction(SX1261_nRES, GPIO_MODE_OUTPUT);
    gpio_set_level(SX1261_nRES, 1);

    spi_device_interface_config_t sx1261_cfg;
    memset(&sx1261_cfg, 0, sizeof(sx1261_cfg));
    sx1261_cfg.mode = 0;  // SPI mode 0
    sx1261_cfg.clock_speed_hz = SPI_MASTER_FREQ_13M;
    sx1261_cfg.spics_io_num = SX1261_SS;
    sx1261_cfg.flags = 0;
    sx1261_cfg.queue_size = 20;

    esp_err_t ret = spi_bus_add_device(SPIHOST, &sx1261_cfg, &gDevSx126x);
    if (ret != ESP_OK) {
      printf("ERROR. SPI add SX1261 device failed.\n");
    } else {
      LORARADIO_PRINTLINE("Registered SX1261 to SPI drvice.");
    }
  }
  SX126xReset();
}

//==========================================================================
//==========================================================================
void SX126xIoIrqInit(DioIrqHandler dioIrq) { gSx126xDioIrqHandler = dioIrq; }

//==========================================================================
//==========================================================================
void SX126xIoDeInit(void) {}

//==========================================================================
//==========================================================================
void SX126xIoTcxoInit(void) {
  // No TCXO component available on this board design.
}

//==========================================================================
//==========================================================================
#define BOARD_TCXO_WAKEUP_TIME 6  // [ms]
uint32_t SX126xGetBoardTcxoWakeupTime(void) { return BOARD_TCXO_WAKEUP_TIME; }

//==========================================================================
//==========================================================================
void SX126xIoRfSwitchInit(void) { SX126xSetDio2AsRfSwitchCtrl(true); }

//==========================================================================
//==========================================================================
RadioOperatingModes_t SX126xGetOperatingMode(void) { return gOperatingMode; }

//==========================================================================
//==========================================================================
void SX126xSetOperatingMode(RadioOperatingModes_t mode) { gOperatingMode = mode; }

//==========================================================================
//==========================================================================
void SX126xReset(void) {
  DelayMs(20);
  gpio_set_level(SX1261_nRES, 0);
  DelayMs(50);
  gpio_set_level(SX1261_nRES, 1);
  DelayMs(20);
}

//==========================================================================
//==========================================================================
int8_t SX126xWaitOnBusy(void) {
  // Timeout at about 1s
  for (uint32_t timeout = 1000; timeout > 0; timeout--) {
    if (gpio_get_level(SX1261_BUSY) == 0) {
      return 0;
    }
    DelayMs(1);
  }
  gChipError = true;
  printf("ERROR. SX126xWaitOnBusy Timeout.\n");
  return -1;
}

//==========================================================================
//==========================================================================
void SX126xWakeup(void) {
  spi_transaction_t xfer;

  if (gDevSx126x == NULL) {
    printf("ERROR. SX126xWakeup device not registered.\n");
    return;
  }

  // Don't wait for BUSY here

  // Set up transfer info
  memset(&halTxBuffer, 0, sizeof(halTxBuffer));
  memset(&halRxBuffer, 0, sizeof(halRxBuffer));
  memset(&xfer, 0, sizeof(xfer));
  halTxBuffer[0] = RADIO_GET_STATUS;
  xfer.length = 2 * 8;
  xfer.tx_buffer = halTxBuffer;
  xfer.rx_buffer = halRxBuffer;

  // start
  if (spi_device_acquire_bus(gDevSx126x, portMAX_DELAY) != ESP_OK) {
    printf("ERROR. SX126xWakeup accuire SPI bus failed.\n");
  } else if (spi_device_polling_transmit(gDevSx126x, &xfer) != ESP_OK) {
    printf("ERROR. SX126xWakeup SPI transmit failed.\n");
  } else {
    // All ok
  }
  spi_device_release_bus(gDevSx126x);

  // Wait for chip to be ready.
  SX126xWaitOnBusy();

  // Update operating mode context variable
  SX126xSetOperatingMode(MODE_STDBY_RC);
}

//==========================================================================
//==========================================================================
void SX126xWriteCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size) {
  spi_transaction_t xfer;

  if (gDevSx126x == NULL) {
    printf("ERROR. SX126xWriteCommand device not registered.\n");
    return;
  }

  // Check Device Ready
  SX126xCheckDeviceReady();

  // Set up transfer info
  memset(&halTxBuffer, 0, sizeof(halTxBuffer));
  memset(&halRxBuffer, 0, sizeof(halRxBuffer));
  memset(&xfer, 0, sizeof(xfer));
  halTxBuffer[0] = (uint8_t)command;
  if (size > (MAX_HAL_BUFFER_SIZE - 1)) {
    printf("ERROR. SX126xWriteCommand data size (%u) too larget.\n", size);
    return;
  }
  if ((buffer != NULL) && (size > 0)) {
    memcpy(&halTxBuffer[1], buffer, size);
  }
  xfer.length = (1 + size) * 8;
  xfer.tx_buffer = halTxBuffer;

  // start
  if (spi_device_acquire_bus(gDevSx126x, portMAX_DELAY) != ESP_OK) {
    printf("ERROR. SX126xWriteCommand accuire SPI bus failed.\n");
  } else if (spi_device_polling_transmit(gDevSx126x, &xfer) != ESP_OK) {
    printf("ERROR. SX126xWriteCommand SPI transmit failed.\n");
  } else {
    // All ok
  }
  spi_device_release_bus(gDevSx126x);

  if (command != RADIO_SET_SLEEP) {
    SX126xWaitOnBusy();
  }
}

//==========================================================================
//==========================================================================
uint8_t SX126xReadCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size) {
  spi_transaction_t xfer;
  uint8_t status = 0;

  if (gDevSx126x == NULL) {
    printf("ERROR. SX126xReadCommand device not registered.\n");
    return 0;
  }
  memset(buffer, 0xff, size);

  // Check Device Ready
  SX126xCheckDeviceReady();

  // Set up transfer info
  memset(&halTxBuffer, 0, sizeof(halTxBuffer));
  memset(&halRxBuffer, 0, sizeof(halRxBuffer));
  memset(&xfer, 0, sizeof(xfer));
  halTxBuffer[0] = (uint8_t)command;
  halTxBuffer[1] = 0;
  if (size > (MAX_HAL_BUFFER_SIZE - 2)) {
    printf("ERROR. SX126xReadCommand data size (%u) too larget.\n", size);
    return 0;
  }
  xfer.length = (2 + size) * 8;
  xfer.tx_buffer = halTxBuffer;
  xfer.rx_buffer = halRxBuffer;

  // start
  if (spi_device_acquire_bus(gDevSx126x, portMAX_DELAY) != ESP_OK) {
    printf("ERROR. SX126xReadCommand accuire SPI bus failed.\n");
  } else if (spi_device_polling_transmit(gDevSx126x, &xfer) != ESP_OK) {
    printf("ERROR. SX126xReadCommand SPI transmit failed.\n");
  } else {
    // All ok
    status = halRxBuffer[1];
    memcpy(buffer, &halRxBuffer[2], size);
  }
  spi_device_release_bus(gDevSx126x);

  SX126xWaitOnBusy();

  return status;
}

//==========================================================================
//==========================================================================
void SX126xWriteRegisters(uint16_t address, uint8_t *buffer, uint16_t size) {
  spi_transaction_t xfer;

  if (gDevSx126x == NULL) {
    printf("ERROR. SX126xWriteRegisters device not registered.\n");
    return;
  }

  // Wait end of busy
  SX126xCheckDeviceReady();

  // Set up transfer info
  memset(&halTxBuffer, 0, sizeof(halTxBuffer));
  memset(&halRxBuffer, 0, sizeof(halRxBuffer));
  memset(&xfer, 0, sizeof(xfer));
  halTxBuffer[0] = RADIO_WRITE_REGISTER;
  halTxBuffer[1] = (uint8_t)(address >> 8);
  halTxBuffer[2] = (uint8_t)(address >> 0);
  if (size > (MAX_HAL_BUFFER_SIZE - 3)) {
    printf("ERROR. SX126xWriteRegisters data size (%u) too larget.\n", size);
    return;
  }
  if ((buffer != NULL) && (size > 0)) {
    memcpy(&halTxBuffer[3], buffer, size);
  }
  xfer.length = (3 + size) * 8;
  xfer.tx_buffer = halTxBuffer;

  // start
  if (spi_device_acquire_bus(gDevSx126x, portMAX_DELAY) != ESP_OK) {
    printf("ERROR. SX126xWriteRegisters accuire SPI bus failed.\n");
  } else if (spi_device_polling_transmit(gDevSx126x, &xfer) != ESP_OK) {
    printf("ERROR. SX126xWriteRegisters SPI transmit failed.\n");
  } else {
    // All ok
  }
  spi_device_release_bus(gDevSx126x);

  SX126xWaitOnBusy();
}

//==========================================================================
//==========================================================================
void SX126xWriteRegister(uint16_t address, uint8_t value) { SX126xWriteRegisters(address, &value, 1); }

//==========================================================================
//==========================================================================
void SX126xReadRegisters(uint16_t address, uint8_t *buffer, uint16_t size) {
  spi_transaction_t xfer;

  if (gDevSx126x == NULL) {
    printf("ERROR. SX126xReadRegisters device not registered.\n");
    return;
  }
  memset(buffer, 0xff, size);

  // Wait end of busy
  SX126xCheckDeviceReady();

  // Set up transfer info
  memset(&halTxBuffer, 0, sizeof(halTxBuffer));
  memset(&halRxBuffer, 0, sizeof(halRxBuffer));
  memset(&xfer, 0, sizeof(xfer));
  halTxBuffer[0] = RADIO_READ_REGISTER;
  halTxBuffer[1] = (uint8_t)(address >> 8);
  halTxBuffer[2] = (uint8_t)(address >> 0);
  halTxBuffer[3] = 0;
  if (size > (MAX_HAL_BUFFER_SIZE - 4)) {
    printf("ERROR. SX126xReadRegisters data size (%u) too larget.\n", size);
    return;
  }
  xfer.length = (4 + size) * 8;
  xfer.tx_buffer = halTxBuffer;
  xfer.rx_buffer = halRxBuffer;

  // start
  if (spi_device_acquire_bus(gDevSx126x, portMAX_DELAY) != ESP_OK) {
    printf("ERROR. SX126xReadRegisters accuire SPI bus failed.\n");
  } else if (spi_device_polling_transmit(gDevSx126x, &xfer) != ESP_OK) {
    printf("ERROR. SX126xReadRegisters SPI transmit failed.\n");
  } else {
    // All ok
    memcpy(buffer, &halRxBuffer[4], size);
  }
  spi_device_release_bus(gDevSx126x);

  SX126xWaitOnBusy();
}

//==========================================================================
//==========================================================================
uint8_t SX126xReadRegister(uint16_t address) {
  uint8_t data;
  SX126xReadRegisters(address, &data, 1);
  return data;
}

//==========================================================================
//==========================================================================
void SX126xWriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) {
  spi_transaction_t xfer;

  if (gDevSx126x == NULL) {
    printf("ERROR. SX126xWriteBuffer device not registered.\n");
    return;
  }

  // Wait end of busy
  SX126xCheckDeviceReady();

  // Set up transfer info
  memset(&halTxBuffer, 0, sizeof(halTxBuffer));
  memset(&halRxBuffer, 0, sizeof(halRxBuffer));
  memset(&xfer, 0, sizeof(xfer));
  halTxBuffer[0] = RADIO_WRITE_BUFFER;
  halTxBuffer[1] = offset;
  if (size > (MAX_HAL_BUFFER_SIZE - 2)) {
    printf("ERROR. SX126xWriteBuffer data size (%u) too larget.\n", size);
    return;
  }
  if ((buffer != NULL) && (size > 0)) {
    memcpy(&halTxBuffer[2], buffer, size);
  }
  xfer.length = (2 + size) * 8;
  xfer.tx_buffer = halTxBuffer;

  // start
  if (spi_device_acquire_bus(gDevSx126x, portMAX_DELAY) != ESP_OK) {
    printf("ERROR. SX126xWriteBuffer accuire SPI bus failed.\n");
  } else if (spi_device_polling_transmit(gDevSx126x, &xfer) != ESP_OK) {
    printf("ERROR. SX126xWriteBuffer SPI transmit failed.\n");
  } else {
    // All ok
  }
  spi_device_release_bus(gDevSx126x);

  SX126xWaitOnBusy();
}

//==========================================================================
//==========================================================================
void SX126xReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) {
  spi_transaction_t xfer;

  if (gDevSx126x == NULL) {
    printf("ERROR. SX126xReadBuffer device not registered.\n");
    return;
  }
  memset(buffer, 0xff, size);

  // Wait end of busy
  SX126xCheckDeviceReady();

  // Set up transfer info
  memset(&halTxBuffer, 0, sizeof(halTxBuffer));
  memset(&halRxBuffer, 0, sizeof(halRxBuffer));
  memset(&xfer, 0, sizeof(xfer));
  halTxBuffer[0] = RADIO_READ_BUFFER;
  halTxBuffer[1] = offset;
  halTxBuffer[2] = 0;
  if (size > (MAX_HAL_BUFFER_SIZE - 3)) {
    printf("ERROR. SX126xReadBuffer data size (%u) too larget.\n", size);
    return;
  }
  xfer.length = (3 + size) * 8;
  xfer.tx_buffer = halTxBuffer;
  xfer.rx_buffer = halRxBuffer;

  // start
  if (spi_device_acquire_bus(gDevSx126x, portMAX_DELAY) != ESP_OK) {
    printf("ERROR. SX126xReadBuffer accuire SPI bus failed.\n");
  } else if (spi_device_polling_transmit(gDevSx126x, &xfer) != ESP_OK) {
    printf("ERROR. SX126xReadBuffer SPI transmit failed.\n");
  } else {
    // All ok
    memcpy(buffer, &halRxBuffer[3], size);
  }
  spi_device_release_bus(gDevSx126x);

  SX126xWaitOnBusy();
}

//==========================================================================
//==========================================================================
void SX126xSetRfTxPower(int8_t power) { SX126xSetTxParams(power, RADIO_RAMP_40_US); }

//==========================================================================
//==========================================================================
uint8_t SX126xGetDeviceId(void) { return SX1261; }

//==========================================================================
//==========================================================================
void SX126xAntSwOn(void) {}

void SX126xAntSwOff(void) {}

//==========================================================================
//==========================================================================
bool SX126xCheckRfFrequency(uint32_t frequency) {
  // Implement check. Currently all frequencies are supported
  return true;
}

//==========================================================================
//==========================================================================
uint32_t SX126xGetDio1PinState(void) {
  if (gpio_get_level(SX1261_DIO1) == 1) {
    return 1;
  } else {
    return 0;
  }
}

//==========================================================================
//==========================================================================
bool SX126xIsError(void) { return gChipError; };