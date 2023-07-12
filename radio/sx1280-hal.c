//==========================================================================
//==========================================================================
#include "sx1280-hal.h"

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
//==========================================================================
static bool gChipError = false;
static spi_device_handle_t gDevSx1280 = NULL;

#define DelayMs(x) vTaskDelay(x / portTICK_PERIOD_MS)

#define MAX_HAL_BUFFER_SIZE 256
static RadioOperatingModes_t gOperatingMode;
static uint8_t halTxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};
static uint8_t halRxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};

//
extern DioIrqHandler* gSx1280DioIrqHandler;

//==========================================================================
//==========================================================================
void SX1280HalWaitOnBusy(void) {
  // Timeout at about 1s
  for (uint32_t timeout = 1000; timeout > 0; timeout--) {
    if (gpio_get_level(SX1280_BUSY) == 0) {
      return;
    }
    DelayMs(1);
  }
  gChipError = true;
  printf("ERROR. SX1280HalWaitOnBusy Timeout.\n");
}

//==========================================================================
//==========================================================================
void SX1280HalInit(void) {
  gChipError = false;

  // Register device
  if (gDevSx1280 == NULL) {
    // Init GPIOs
    gpio_reset_pin(SX1280_SS);
    gpio_set_direction(SX1280_SS, GPIO_MODE_OUTPUT);
    gpio_set_level(SX1280_SS, 1);

    gpio_reset_pin(SX1280_BUSY);
    gpio_set_direction(SX1280_BUSY, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SX1280_BUSY, GPIO_FLOATING);
    gpio_reset_pin(SX1280_DIO1);
    gpio_set_direction(SX1280_DIO1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SX1280_DIO1, GPIO_FLOATING);

    gpio_reset_pin(SX1280_nRES);
    gpio_set_direction(SX1280_nRES, GPIO_MODE_OUTPUT);
    gpio_set_level(SX1280_nRES, 1);

    spi_device_interface_config_t sx1280_cfg;
    memset(&sx1280_cfg, 0, sizeof(sx1280_cfg));
    sx1280_cfg.mode = 0;  // SPI mode 0
    sx1280_cfg.clock_speed_hz = SPI_MASTER_FREQ_8M;
    sx1280_cfg.spics_io_num = SX1280_SS;
    sx1280_cfg.flags = 0;
    sx1280_cfg.queue_size = 20;

    esp_err_t ret = spi_bus_add_device(SPIHOST, &sx1280_cfg, &gDevSx1280);
    if (ret != ESP_OK) {
      printf("ERROR. SPI add SX1280 device failed.\n");
    } else {
      LORARADIO_PRINTLINE("Registered SX1280 to SPI drvice.");
    }
  }
  SX1280HalReset();
  
  // Issue a valid SPI command to disable the UART func at SX1280
  uint8_t standby = 0;
  SX1280HalWriteCommand(RADIO_SET_STANDBY, &standby, 1);
}

//==========================================================================
//==========================================================================
void SX1280HalIoIrqInit(DioIrqHandler irqHandlers) { gSx1280DioIrqHandler = irqHandlers; }

//==========================================================================
//==========================================================================
void SX1280HalReset(void) {
  DelayMs(20);
  gpio_set_level(SX1280_nRES, 0);
  DelayMs(50);
  gpio_set_level(SX1280_nRES, 1);
  DelayMs(20);
}

//==========================================================================
//==========================================================================
void SX1280HalClearInstructionRam(void) {
  spi_transaction_t xfer;
  uint16_t size = MAX_HAL_BUFFER_SIZE / 2;

  if (gDevSx1280 == NULL) {
    printf("ERROR. SX1280HalClearInstructionRam device not registered.\n");
    return;
  }

  // Clearing the instruction RAM is writing 0x00s on every bytes of the
  // instruction RAM
  for (uint16_t addr = 0; addr < IRAM_SIZE; addr += size) {
    // Set up transfer info
    memset(&halTxBuffer, 0, sizeof(halTxBuffer));
    memset(&halRxBuffer, 0, sizeof(halRxBuffer));
    memset(&xfer, 0, sizeof(xfer));
    halTxBuffer[0] = RADIO_WRITE_REGISTER;
    halTxBuffer[1] = (uint8_t)((addr >> 8) & 0x00FF);
    halTxBuffer[2] = (uint8_t)(addr & 0x00FF);
    xfer.length = (size + 3) * 8;
    xfer.tx_buffer = halTxBuffer;

    // start
    if (spi_device_acquire_bus(gDevSx1280, portMAX_DELAY) != ESP_OK) {
      printf("ERROR. SX1280HalClearInstructionRam accuire SPI bus failed.\n");
    } else if (spi_device_polling_transmit(gDevSx1280, &xfer) != ESP_OK) {
      printf("ERROR. SX1280HalClearInstructionRam SPI transmit failed.\n");
    } else {
      // All ok
    }
    spi_device_release_bus(gDevSx1280);
  }
  SX1280HalWaitOnBusy();
}

//==========================================================================
//==========================================================================
void SX1280HalWakeup(void) {
  spi_transaction_t xfer;

  if (gDevSx1280 == NULL) {
    printf("ERROR. SX1280HalWakeup device not registered.\n");
    return;
  }

  // Set up transfer info
  memset(&halTxBuffer, 0, sizeof(halTxBuffer));
  memset(&halRxBuffer, 0, sizeof(halRxBuffer));
  memset(&xfer, 0, sizeof(xfer));
  halTxBuffer[0] = RADIO_GET_STATUS;
  xfer.length = 2 * 8;
  xfer.tx_buffer = halTxBuffer;
  xfer.rx_buffer = halRxBuffer;

  // start
  if (spi_device_acquire_bus(gDevSx1280, portMAX_DELAY) != ESP_OK) {
    printf("ERROR. SX1280HalWakeup accuire SPI bus failed.\n");
  } else if (spi_device_polling_transmit(gDevSx1280, &xfer) != ESP_OK) {
    printf("ERROR. SX1280HalWakeup SPI transmit failed.\n");
  } else {
    // All ok
  }
  spi_device_release_bus(gDevSx1280);

  // Wait for chip to be ready.
  SX1280HalWaitOnBusy();

  // Update operating mode context variable
  SX1280SetOperatingMode(MODE_STDBY_RC);
}

//==========================================================================
//==========================================================================
void SX1280HalWriteCommand(RadioCommands_t command, uint8_t* buffer, uint16_t size) {
  spi_transaction_t xfer;

  if (gDevSx1280 == NULL) {
    printf("ERROR. SX1280HalWriteCommand device not registered.\n");
    return;
  }

  // Check Device Ready
  if (command != RADIO_SET_STANDBY) {
    SX1280CheckDeviceReady();
  }

  // Set up transfer info
  memset(&halTxBuffer, 0, sizeof(halTxBuffer));
  memset(&halRxBuffer, 0, sizeof(halRxBuffer));
  memset(&xfer, 0, sizeof(xfer));
  halTxBuffer[0] = (uint8_t)command;
  if (size > (MAX_HAL_BUFFER_SIZE - 1)) {
    printf("ERROR. SX1280HalWriteCommand data size (%u) too larget.\n", size);
    return;
  }
  if ((buffer != NULL) && (size > 0)) {
    memcpy(&halTxBuffer[1], buffer, size);
  }
  xfer.length = (1 + size) * 8;
  xfer.tx_buffer = halTxBuffer;

  // LORARADIO_HEX2STRING("WR CMD:", halTxBuffer, (1 + size));

  // start
  if (spi_device_acquire_bus(gDevSx1280, portMAX_DELAY) != ESP_OK) {
    printf("ERROR. SX1280HalWriteCommand accuire SPI bus failed.\n");
  } else if (spi_device_polling_transmit(gDevSx1280, &xfer) != ESP_OK) {
    printf("ERROR. SX1280HalWriteCommand SPI transmit failed.\n");
  } else {
    // All ok
  }
  spi_device_release_bus(gDevSx1280);

  if (command != RADIO_SET_SLEEP) {
    SX1280HalWaitOnBusy();
  }
}

//==========================================================================
//==========================================================================
void SX1280HalReadCommand(RadioCommands_t command, uint8_t* buffer, uint16_t size) {
  spi_transaction_t xfer;

  if (gDevSx1280 == NULL) {
    printf("ERROR. SX1280HalReadCommand device not registered.\n");
    return;
  }
  memset(buffer, 0xff, size);

  // Check Device Ready
  SX1280CheckDeviceReady();

  // Set up transfer info
  memset(&halTxBuffer, 0, sizeof(halTxBuffer));
  memset(&halRxBuffer, 0, sizeof(halRxBuffer));
  memset(&xfer, 0, sizeof(xfer));
  halTxBuffer[0] = (uint8_t)command;
  halTxBuffer[1] = 0;

  if (command == RADIO_GET_STATUS) {
    if (size > (MAX_HAL_BUFFER_SIZE - 1)) {
      printf("ERROR. SX1280HalReadCommand data size (%u) too larget.\n", size);
      return;
    }
    xfer.length = (1 + size) * 8;
  } else {
    if (size > (MAX_HAL_BUFFER_SIZE - 2)) {
      printf("ERROR. SX1280HalReadCommand data size (%u) too larget.\n", size);
      return;
    }
    xfer.length = (2 + size) * 8;
  }

  xfer.tx_buffer = halTxBuffer;
  xfer.rx_buffer = halRxBuffer;

  // start
  if (spi_device_acquire_bus(gDevSx1280, portMAX_DELAY) != ESP_OK) {
    printf("ERROR. SX1280HalReadCommand accuire SPI bus failed.\n");
  } else if (spi_device_polling_transmit(gDevSx1280, &xfer) != ESP_OK) {
    printf("ERROR. SX1280HalReadCommand SPI transmit failed.\n");
  } else {
    // All ok
    if (command == RADIO_GET_STATUS) {
      memcpy(buffer, &halRxBuffer[1], size);
    } else {
      memcpy(buffer, &halRxBuffer[2], size);
    }
  }
  spi_device_release_bus(gDevSx1280);

  SX1280HalWaitOnBusy();
}

//==========================================================================
//==========================================================================
void SX1280HalWriteRegisters(uint16_t address, uint8_t* buffer, uint16_t size) {
  spi_transaction_t xfer;

  if (gDevSx1280 == NULL) {
    printf("ERROR. SX1280HalWriteRegisters device not registered.\n");
    return;
  }

  // Check Device Ready
  SX1280CheckDeviceReady();

  // Set up transfer info
  memset(&halTxBuffer, 0, sizeof(halTxBuffer));
  memset(&halRxBuffer, 0, sizeof(halRxBuffer));
  memset(&xfer, 0, sizeof(xfer));
  halTxBuffer[0] = RADIO_WRITE_REGISTER;
  halTxBuffer[1] = (uint8_t)(address >> 8);
  halTxBuffer[2] = (uint8_t)(address >> 0);
  if (size > (MAX_HAL_BUFFER_SIZE - 3)) {
    printf("ERROR. SX1280HalWriteRegisters data size (%u) too larget.\n", size);
    return;
  }
  if ((buffer != NULL) && (size > 0)) {
    memcpy(&halTxBuffer[3], buffer, size);
  }
  xfer.length = (3 + size) * 8;
  xfer.tx_buffer = halTxBuffer;

  // LORARADIO_HEX2STRING("WR REG:", halTxBuffer, (3 + size));

  // start
  if (spi_device_acquire_bus(gDevSx1280, portMAX_DELAY) != ESP_OK) {
    printf("ERROR. SX1280HalWriteRegisters accuire SPI bus failed.\n");
  } else if (spi_device_polling_transmit(gDevSx1280, &xfer) != ESP_OK) {
    printf("ERROR. SX1280HalWriteRegisters SPI transmit failed.\n");
  } else {
    // All ok
  }
  spi_device_release_bus(gDevSx1280);

  SX1280HalWaitOnBusy();
}

//==========================================================================
//==========================================================================
void SX1280HalWriteRegister(uint16_t address, uint8_t value) { SX1280HalWriteRegisters(address, &value, 1); }

//==========================================================================
//==========================================================================
void SX1280HalReadRegisters(uint16_t address, uint8_t* buffer, uint16_t size) {
  spi_transaction_t xfer;

  if (gDevSx1280 == NULL) {
    printf("ERROR. SX1280HalReadCommand device not registered.\n");
    return;
  }
  memset(buffer, 0xff, size);

  // Check Device Ready
  SX1280CheckDeviceReady();

  // Set up transfer info
  memset(&halTxBuffer, 0, sizeof(halTxBuffer));
  memset(&halRxBuffer, 0, sizeof(halRxBuffer));
  memset(&xfer, 0, sizeof(xfer));
  halTxBuffer[0] = RADIO_READ_REGISTER;
  halTxBuffer[1] = (uint8_t)(address >> 8);
  halTxBuffer[2] = (uint8_t)(address >> 0);
  halTxBuffer[3] = 0;
  if (size > (MAX_HAL_BUFFER_SIZE - 4)) {
    printf("ERROR. SX1280HalReadCommand data size (%u) too larget.\n", size);
    return;
  }
  xfer.length = (4 + size) * 8;
  xfer.tx_buffer = halTxBuffer;
  xfer.rx_buffer = halRxBuffer;

  // start
  if (spi_device_acquire_bus(gDevSx1280, portMAX_DELAY) != ESP_OK) {
    printf("ERROR. SX1280HalReadCommand accuire SPI bus failed.\n");
  } else if (spi_device_polling_transmit(gDevSx1280, &xfer) != ESP_OK) {
    printf("ERROR. SX1280HalReadCommand SPI transmit failed.\n");
  } else {
    // All ok
    memcpy(buffer, &halRxBuffer[4], size);
  }
  spi_device_release_bus(gDevSx1280);

  SX1280HalWaitOnBusy();
}

//==========================================================================
//==========================================================================
uint8_t SX1280HalReadRegister(uint16_t address) {
  uint8_t data;

  SX1280HalReadRegisters(address, &data, 1);

  return data;
}

//==========================================================================
//==========================================================================
void SX1280HalWriteBuffer(uint8_t offset, uint8_t* buffer, uint8_t size) {
  spi_transaction_t xfer;

  if (gDevSx1280 == NULL) {
    printf("ERROR. SX1280HalWriteBuffer device not registered.\n");
    return;
  }

  // Check Device Ready
  SX1280CheckDeviceReady();

  // Set up transfer info
  memset(&halTxBuffer, 0, sizeof(halTxBuffer));
  memset(&halRxBuffer, 0, sizeof(halRxBuffer));
  memset(&xfer, 0, sizeof(xfer));
  halTxBuffer[0] = RADIO_WRITE_BUFFER;
  halTxBuffer[1] = offset;
  if (size > (MAX_HAL_BUFFER_SIZE - 2)) {
    printf("ERROR. SX1280HalWriteBuffer data size (%u) too larget.\n", size);
    return;
  }
  if ((buffer != NULL) && (size > 0)) {
    memcpy(&halTxBuffer[2], buffer, size);
  }
  xfer.length = (2 + size) * 8;
  xfer.tx_buffer = halTxBuffer;

  // LORARADIO_HEX2STRING("WR BUF:", halTxBuffer, (2 + size));

  // start
  if (spi_device_acquire_bus(gDevSx1280, portMAX_DELAY) != ESP_OK) {
    printf("ERROR. SX1280HalWriteBuffer accuire SPI bus failed.\n");
  } else if (spi_device_polling_transmit(gDevSx1280, &xfer) != ESP_OK) {
    printf("ERROR. SX1280HalWriteBuffer SPI transmit failed.\n");
  } else {
    // All ok
  }
  spi_device_release_bus(gDevSx1280);

  SX1280HalWaitOnBusy();
}

//==========================================================================
//==========================================================================
void SX1280HalReadBuffer(uint8_t offset, uint8_t* buffer, uint8_t size) {
  spi_transaction_t xfer;

  if (gDevSx1280 == NULL) {
    printf("ERROR. SX1280HalReadCommand device not registered.\n");
    return;
  }
  memset(buffer, 0xff, size);

  // Check Device Ready
  SX1280CheckDeviceReady();

  // Set up transfer info
  memset(&halTxBuffer, 0, sizeof(halTxBuffer));
  memset(&halRxBuffer, 0, sizeof(halRxBuffer));
  memset(&xfer, 0, sizeof(xfer));
  halTxBuffer[0] = RADIO_READ_BUFFER;
  halTxBuffer[1] = offset;
  halTxBuffer[2] = 0;
  if (size > (MAX_HAL_BUFFER_SIZE - 3)) {
    printf("ERROR. SX1280HalReadCommand data size (%u) too larget.\n", size);
    return;
  }
  xfer.length = (3 + size) * 8;
  xfer.tx_buffer = halTxBuffer;
  xfer.rx_buffer = halRxBuffer;

  // start
  if (spi_device_acquire_bus(gDevSx1280, portMAX_DELAY) != ESP_OK) {
    printf("ERROR. SX1280HalReadCommand accuire SPI bus failed.\n");
  } else if (spi_device_polling_transmit(gDevSx1280, &xfer) != ESP_OK) {
    printf("ERROR. SX1280HalReadCommand SPI transmit failed.\n");
  } else {
    // All ok
    memcpy(buffer, &halRxBuffer[3], size);
  }
  spi_device_release_bus(gDevSx1280);

  SX1280HalWaitOnBusy();
}

//==========================================================================
//==========================================================================
uint8_t SX1280HalGetDioStatus(void) {
  if (gpio_get_level(SX1280_DIO1) == 1) {
    return 1;
  } else {
    return 0;
  }
}

//==========================================================================
//==========================================================================
RadioOperatingModes_t SX1280GetOperatingMode(void) { return gOperatingMode; }

//==========================================================================
//==========================================================================
void SX1280SetOperatingMode(RadioOperatingModes_t mode) { gOperatingMode = mode; }

//==========================================================================
//==========================================================================
bool SX1280IsError(void) { return gChipError; };