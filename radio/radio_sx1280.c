//==========================================================================
//==========================================================================
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "LoRaRadio_debug.h"
#include "board.h"
#include "delay.h"
#include "radio.h"
#include "sx1280-hal.h"
#include "sx1280.h"
#include "timer.h"
#include "utilities.h"

//==========================================================================
//==========================================================================
#define LORA_MAC_PRIVATE_SYNCWORD 0x1424
#define LORA_MAC_PUBLIC_SYNCWORD 0x3444

#define max(a, b)           \
  ({                        \
    __typeof__(a) _a = (a); \
    __typeof__(b) _b = (b); \
    _a > _b ? _a : _b;      \
  })

//==========================================================================
//==========================================================================
/*!
 * \brief Initializes the radio
 *
 * \param [IN] events Structure containing the driver callback functions
 */
static void RadioInit(RadioEvents_t* events);

/*!
 * Return current radio status
 *
 * \param status Radio status.[RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
static RadioState_t RadioGetStatus(void);

/*!
 * \brief Configures the radio with the given modem
 *
 * \param [IN] modem Modem to be used [0: FSK, 1: LoRa]
 */
static void RadioSetModem(RadioModems_t modem);

/*!
 * \brief Sets the channel frequency
 *
 * \param [IN] freq         Channel RF frequency
 */
static void RadioSetChannel(uint32_t freq);

/*!
 * \brief Checks if the channel is free for the given time
 *
 * \remark The FSK modem is always used for this task as we can select the Rx bandwidth at will.
 *
 * \param [IN] freq                Channel RF frequency in Hertz
 * \param [IN] rxBandwidth         Rx bandwidth in Hertz
 * \param [IN] rssiThresh          RSSI threshold in dBm
 * \param [IN] maxCarrierSenseTime Max time in milliseconds while the RSSI is measured
 *
 * \retval isFree         [true: Channel is free, false: Channel is not free]
 */
static bool RadioIsChannelFree(uint32_t freq, uint32_t rxBandwidth, int16_t rssiThresh, uint32_t maxCarrierSenseTime);

/*!
 * \brief Generates a 32 bits random value based on the RSSI readings
 *
 * \remark This function sets the radio in LoRa modem mode and disables
 *         all interrupts.
 *         After calling this function either Radio.SetRxConfig or
 *         Radio.SetTxConfig functions must be called.
 *
 * \retval randomValue    32 bits random value
 */
static uint32_t RadioRandom(void);

/*!
 * \brief Sets the reception parameters
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] bandwidthAfc Sets the AFC Bandwidth (FSK only)
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: N/A ( set to 0 )
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] symbTimeout  Sets the RxSingle timeout value
 *                          FSK : timeout in number of bytes
 *                          LoRa: timeout in symbols
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed length is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] rxContinuous Sets the reception in continuous mode
 *                          [false: single mode, true: continuous mode]
 */
static void RadioSetRxConfig(RadioModems_t modem, uint32_t bandwidth, uint32_t datarate, uint8_t coderate, uint32_t bandwidthAfc,
                             uint16_t preambleLen, uint16_t symbTimeout, bool fixLen, uint8_t payloadLen, bool crcOn,
                             bool FreqHopOn, uint8_t HopPeriod, bool iqInverted, bool rxContinuous);

/*!
 * \brief Sets the transmission parameters
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] power        Sets the output power [dBm]
 * \param [IN] fdev         Sets the frequency deviation (FSK only)
 *                          FSK : [Hz]
 *                          LoRa: 0
 * \param [IN] bandwidth    Sets the bandwidth (LoRa only)
 *                          FSK : 0
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] crcOn        Enables disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] timeout      Transmission timeout [ms]
 */
static void RadioSetTxConfig(RadioModems_t modem, int8_t power, uint32_t fdev, uint32_t bandwidth, uint32_t datarate,
                             uint8_t coderate, uint16_t preambleLen, bool fixLen, bool crcOn, bool FreqHopOn, uint8_t HopPeriod,
                             bool iqInverted, uint32_t timeout);

/*!
 * \brief Checks if the given RF frequency is supported by the hardware
 *
 * \param [IN] frequency RF frequency to be checked
 * \retval isSupported [true: supported, false: unsupported]
 */
static bool RadioCheckRfFrequency(uint32_t frequency);

/*!
 * \brief Computes the packet time on air in ms for the given payload
 *
 * \Remark Can only be called once SetRxConfig or SetTxConfig have been called
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed length is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 *
 * \retval airTime        Computed airTime (ms) for the given packet payload length
 */
static uint32_t RadioTimeOnAir(RadioModems_t modem, uint32_t bandwidth, uint32_t datarate, uint8_t coderate, uint16_t preambleLen,
                               bool fixLen, uint8_t payloadLen, bool crcOn);

/*!
 * \brief Sends the buffer of size. Prepares the packet to be sent and sets
 *        the radio in transmission
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
static void RadioSend(uint8_t* buffer, uint8_t size);

/*!
 * \brief Sets the radio in sleep mode
 */
static void RadioSleep(void);

/*!
 * \brief Sets the radio in standby mode
 */
static void RadioStandby(void);

/*!
 * \brief Sets the radio in reception mode for the given time
 * \param [IN] timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
static void RadioRx(uint32_t timeout);

/*!
 * \brief Start a Channel Activity Detection
 */
static void RadioStartCad(void);

/*!
 * \brief Sets the radio in continuous wave transmission mode
 *
 * \param [IN]: freq       Channel RF frequency
 * \param [IN]: power      Sets the output power [dBm]
 * \param [IN]: time       Transmission mode timeout [s]
 */
static void RadioSetTxContinuousWave(uint32_t freq, int8_t power, uint16_t time);

/*!
 * \brief Reads the current RSSI value
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
static int16_t RadioRssi(RadioModems_t modem);

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [IN]: data New register value
 */
static void RadioWrite(uint32_t addr, uint8_t data);

/*!
 * \brief Reads the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \retval data Register value
 */
static uint8_t RadioRead(uint32_t addr);

/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
static void RadioWriteBuffer(uint32_t addr, uint8_t* buffer, uint8_t size);

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
static void RadioReadBuffer(uint32_t addr, uint8_t* buffer, uint8_t size);

/*!
 * \brief Sets the maximum payload length.
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] max        Maximum payload length in bytes
 */
static void RadioSetMaxPayloadLength(RadioModems_t modem, uint8_t max);

/*!
 * \brief Sets the network to public or private. Updates the sync byte.
 *
 * \remark Applies to LoRa modem only
 *
 * \param [IN] enable if true, it enables a public network
 */
static void RadioSetPublicNetwork(bool enable);

/*!
 * \brief Gets the time required for the board plus radio to get out of sleep.[ms]
 *
 * \retval time Radio plus board wakeup time in ms.
 */
static uint32_t RadioGetWakeupTime(void);

/*!
 * \brief Process radio irq
 */
static void RadioIrqProcess(void);

/*!
 * \brief Sets the Rx duty cycle management parameters
 *
 * \param [in]  rxTime        Structure describing reception timeout value
 * \param [in]  sleepTime     Structure describing sleep timeout value
 */
static void RadioSetRxDutyCycle(uint32_t rxTime, uint32_t sleepTime);

//==========================================================================
//==========================================================================
/*!
 * Radio driver structure initialization
 */
const struct Radio_s RadioSx1280 = {RadioInit,
                                    RadioGetStatus,
                                    RadioSetModem,
                                    RadioSetChannel,
                                    RadioIsChannelFree,
                                    RadioRandom,
                                    RadioSetRxConfig,
                                    RadioSetTxConfig,
                                    RadioCheckRfFrequency,
                                    RadioTimeOnAir,
                                    RadioSend,
                                    RadioSleep,
                                    RadioStandby,
                                    RadioRx,
                                    RadioStartCad,
                                    RadioSetTxContinuousWave,
                                    RadioRssi,
                                    RadioWrite,
                                    RadioRead,
                                    RadioWriteBuffer,
                                    RadioReadBuffer,
                                    RadioSetMaxPayloadLength,
                                    RadioSetPublicNetwork,
                                    RadioGetWakeupTime,
                                    RadioIrqProcess,
                                    RadioRx,
                                    RadioSetRxDutyCycle};

//==========================================================================
//==========================================================================
/*
 * Local types definition
 */

static const RadioLoRaBandwidths_t Bandwidths[] = {LORA_BW_0200, LORA_BW_0400, LORA_BW_0800, LORA_BW_1600};
static const RadioLoRaSpreadingFactors_t kSfTable[8] = {LORA_SF5, LORA_SF6,  LORA_SF7,  LORA_SF8,
                                                        LORA_SF9, LORA_SF10, LORA_SF11, LORA_SF12};

static uint8_t MaxPayloadLength = 0xFF;

static uint32_t TxTimeout = 0;
static uint32_t RxTimeout = 0;

static bool RxContinuous = false;

static PacketStatus_t RadioPktStatus;
static uint8_t RadioRxPayload[255];

static bool IrqFired = false;

static uint32_t Frequency = 2403000000;

/*
 * SX1280 DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
static void RadioOnDioIrq(void* context);

/*!
 * \brief Tx timeout timer callback
 */
static void RadioOnTxTimeoutIrq(void* context);

/*!
 * \brief Rx timeout timer callback
 */
static void RadioOnRxTimeoutIrq(void* context);

/*
 * Private global variables
 */

/*!
 * Holds the current network type for the radio
 */
typedef struct {
  bool Previous;
  bool Current;
} RadioPublicNetwork_t;

static RadioPublicNetwork_t RadioPublicNetwork = {false};

/*!
 * Radio callbacks variable
 */
static RadioEvents_t* RadioEvents;

/*!
 * Radio hardware and global parameters
 */
SX1280_t SX1280;

/*!
 * Tx and Rx timers
 */
static TimerEvent_t TxTimeoutTimer;
static TimerEvent_t RxTimeoutTimer;

//==========================================================================
//==========================================================================
static uint8_t ConvertPreamble(uint16_t aLen) {
  if (aLen <= 30) {
    return (aLen >> 1) | 0x10;
  }
  else if (aLen <= 60) {
    return (aLen >> 2) | 0x20;
  }
  else if (aLen <= 120) {
    return (aLen >> 3) | 0x30;
  }
  else {
    return (aLen >> 4) | 0x40;
  }
}

//==========================================================================
//==========================================================================
static void RadioInit(RadioEvents_t* events) {
  RadioEvents = events;

  SX1280Init(RadioOnDioIrq);
  SX1280SetStandby(STDBY_RC);
  SX1280SetRegulatorMode(USE_DCDC);

  SX1280SetBufferBaseAddresses(0x00, 0x00);
  SX1280SetTxParams(0, RADIO_RAMP_02_US);
  SX1280SetDioIrqParams(IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE);

  // Initialize driver timeout timers
  TimerInit(&TxTimeoutTimer, RadioOnTxTimeoutIrq);
  TimerInit(&RxTimeoutTimer, RadioOnRxTimeoutIrq);

  IrqFired = false;
  RadioPublicNetwork.Current = false;
}

//==========================================================================
//==========================================================================
static RadioState_t RadioGetStatus(void) {
  switch (SX1280GetOperatingMode()) {
    case MODE_TX:
      return RF_TX_RUNNING;
    case MODE_RX:
      return RF_RX_RUNNING;
    case MODE_CAD:
      return RF_CAD;
    default:
      return RF_IDLE;
  }
}

//==========================================================================
//==========================================================================
static void RadioSetModem(RadioModems_t modem) {
  switch (modem) {
    default:
    case MODEM_FSK:
      SX1280SetPacketType(PACKET_TYPE_GFSK);
      // When switching to GFSK mode the LoRa SyncWord register value is reset
      // Thus, we also reset the RadioPublicNetwork variable
      RadioPublicNetwork.Current = false;
      break;
    case MODEM_LORA:
      SX1280SetPacketType(PACKET_TYPE_LORA);

      if (RadioPublicNetwork.Current == true) {
        SX1280HalWriteRegister(0x944, (LORA_MAC_PUBLIC_SYNCWORD >> 8) & 0xFF);
        SX1280HalWriteRegister(0x944 + 1, LORA_MAC_PUBLIC_SYNCWORD & 0xFF);
      } else {
        SX1280HalWriteRegister(0x944, (LORA_MAC_PRIVATE_SYNCWORD >> 8) & 0xFF);
        SX1280HalWriteRegister(0x944 + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF);
      }
      break;
  }
}

//==========================================================================
//==========================================================================
static void RadioSetChannel(uint32_t freq) { Frequency = freq; }

//==========================================================================
//==========================================================================
static bool RadioIsChannelFree(uint32_t freq, uint32_t rxBandwidth, int16_t rssiThresh, uint32_t maxCarrierSenseTime) {
  bool status = true;
  int16_t rssi = 0;
  uint32_t carrierSenseTime = 0;

  RadioSetModem(MODEM_FSK);

  RadioSetChannel(freq);

  // Set Rx bandwidth. Other parameters are not used.
  RadioSetRxConfig(MODEM_FSK, rxBandwidth, 600, 0, rxBandwidth, 3, 0, false, 0, false, 0, 0, false, true);
  RadioRx(0);

  DelayMs(1);

  carrierSenseTime = TimerGetCurrentTime();

  // Perform carrier sense for maxCarrierSenseTime
  while (TimerGetElapsedTime(carrierSenseTime) < maxCarrierSenseTime) {
    rssi = RadioRssi(MODEM_FSK);

    if (rssi > rssiThresh) {
      status = false;
      break;
    }
  }
  RadioSleep();
  return status;
}

//==========================================================================
//==========================================================================
static uint32_t RadioRandom(void) {
  uint32_t rnd = 0;

  /*
   * Radio setup for random number generation
   */
  // Set LoRa modem ON
  RadioSetModem(MODEM_LORA);

  // Disable LoRa modem interrupts
  SX1280SetDioIrqParams(IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE);

  rnd = rand();

  return rnd;
}

//==========================================================================
//==========================================================================
static RadioGfskBleBitrates_t GetFskBitrateBandwidth(uint32_t bandwidth, uint32_t datarate) {
  if (bandwidth == 300000) {
    if (datarate == 1)
      return GFSK_BLE_BR_0_250_BW_0_3;
    else
      return GFSK_BLE_BR_0_125_BW_0_3;
  } else if (bandwidth == 600000) {
    if (datarate == 1)
      return GFSK_BLE_BR_0_400_BW_0_6;
    else if (datarate == 2)
      return GFSK_BLE_BR_0_500_BW_0_6;
    else
      return GFSK_BLE_BR_0_250_BW_0_6;

  } else if (bandwidth == 1200000) {
    if (datarate == 1)
      return GFSK_BLE_BR_0_500_BW_1_2;
    else if (datarate == 2)
      return GFSK_BLE_BR_0_800_BW_1_2;
    else
      return GFSK_BLE_BR_0_400_BW_1_2;
  } else {
    return GFSK_BLE_BR_2_000_BW_2_4;
  }
}

//==========================================================================
//==========================================================================
static void RadioSetRxConfig(RadioModems_t modem, uint32_t bandwidth, uint32_t spreading, uint8_t coderate, uint32_t bandwidthAfc,
                             uint16_t preambleLen, uint16_t symbTimeout, bool fixLen, uint8_t payloadLen, bool crcOn,
                             bool freqHopOn, uint8_t hopPeriod, bool iqInverted, bool rxContinuous) {
  uint8_t fsk_sync_word[8] = {0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00};

  RxContinuous = rxContinuous;
  if (rxContinuous == true) {
    symbTimeout = 0;
  }
  if (fixLen == true) {
    MaxPayloadLength = payloadLen;
  } else {
    MaxPayloadLength = 255;
  }

  switch (modem) {
    case MODEM_FSK:
      //      SX1280SetStopRxTimerOnPreambleDetect(false);

      SX1280.ModulationParams.PacketType = PACKET_TYPE_GFSK;
      SX1280.ModulationParams.Params.Gfsk.BitrateBandwidth = GetFskBitrateBandwidth(bandwidth, spreading);
      SX1280.ModulationParams.Params.Gfsk.ModulationIndex = GFSK_BLE_MOD_IND_0_50;
      SX1280.ModulationParams.Params.Gfsk.ModulationShaping = RADIO_MOD_SHAPING_BT_1_0;

      if (preambleLen < 4) preambleLen = 4;
      if (preambleLen > 32) preambleLen = 32;

      SX1280.PacketParams.PacketType = PACKET_TYPE_GFSK;
      SX1280.PacketParams.Params.Gfsk.PreambleLength = (RadioPreambleLengths_t)(((preambleLen / 4) - 1) << 4);  // convert
      SX1280.PacketParams.Params.Gfsk.SyncWordLength = GFSK_SYNCWORD_LENGTH_3_BYTE;
      SX1280.PacketParams.Params.Gfsk.SyncWordMatch = RADIO_RX_MATCH_SYNCWORD_3;
      SX1280.PacketParams.Params.Gfsk.HeaderType = (fixLen == true) ? RADIO_PACKET_FIXED_LENGTH : RADIO_PACKET_VARIABLE_LENGTH;
      SX1280.PacketParams.Params.Gfsk.PayloadLength = MaxPayloadLength;
      if (crcOn == true) {
        SX1280.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_2_BYTES;
      } else {
        SX1280.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_OFF;
      }
      SX1280.PacketParams.Params.Gfsk.Whitening = RADIO_WHITENING_ON;

      RadioStandby();
      RadioSetModem((SX1280.ModulationParams.PacketType == PACKET_TYPE_GFSK) ? MODEM_FSK : MODEM_LORA);
      SX1280SetModulationParams(&SX1280.ModulationParams);
      SX1280SetPacketParams(&SX1280.PacketParams);
      SX1280SetSyncWord(0, fsk_sync_word);
      SX1280SetWhiteningSeed(0xFF);

      RxTimeout = (uint32_t)symbTimeout * 8000UL / spreading;
      break;

    case MODEM_LORA:
      //      SX1280SetStopRxTimerOnPreambleDetect(false);
      SX1280.ModulationParams.PacketType = PACKET_TYPE_LORA;
      if (spreading < 5) {
        SX1280.ModulationParams.Params.LoRa.SpreadingFactor = LORA_SF5;
      } else if (spreading > 12) {
        SX1280.ModulationParams.Params.LoRa.SpreadingFactor = LORA_SF12;
      } else {
        SX1280.ModulationParams.Params.LoRa.SpreadingFactor = kSfTable[spreading - 5];
      }
      SX1280.ModulationParams.Params.LoRa.Bandwidth = Bandwidths[bandwidth];
      SX1280.ModulationParams.Params.LoRa.CodingRate = (RadioLoRaCodingRates_t)coderate;

      SX1280.PacketParams.PacketType = PACKET_TYPE_LORA;

      if ((spreading == 6) || (spreading == 5)) {
        if (preambleLen < 12) {
          SX1280.PacketParams.Params.LoRa.PreambleLength = PREAMBLE_LENGTH_12_BITS;
        } else {
          SX1280.PacketParams.Params.LoRa.PreambleLength = ConvertPreamble(preambleLen);
        }
      } else {
        SX1280.PacketParams.Params.LoRa.PreambleLength = ConvertPreamble(preambleLen);
      }

      if (fixLen)
        SX1280.PacketParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
      else
        SX1280.PacketParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;

      SX1280.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength;
      if (crcOn)
        SX1280.PacketParams.Params.LoRa.CrcMode = LORA_CRC_ON;
      else
        SX1280.PacketParams.Params.LoRa.CrcMode = LORA_CRC_OFF;

      if (iqInverted)
        SX1280.PacketParams.Params.LoRa.InvertIQ = LORA_IQ_INVERTED;
      else
        SX1280.PacketParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;

      RadioStandby();
      RadioSetModem((SX1280.ModulationParams.PacketType == PACKET_TYPE_GFSK) ? MODEM_FSK : MODEM_LORA);
      SX1280SetModulationParams(&SX1280.ModulationParams);
      SX1280SetPacketParams(&SX1280.PacketParams);
      // SX1280SetLoRaSymbNumTimeout(symbTimeout);

      // Timeout Max, Timeout handled directly in SetRx function
      RxTimeout = 0xFFFF;

      break;
  }
}

//==========================================================================
//==========================================================================
static void RadioSetTxConfig(RadioModems_t modem, int8_t power, uint32_t fdev, uint32_t bandwidth, uint32_t spreading,
                             uint8_t coderate, uint16_t preambleLen, bool fixLen, bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                             bool iqInverted, uint32_t timeout) {
  uint8_t fsk_sync_word[8] = {0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00};

  switch (modem) {
    case MODEM_FSK:
      SX1280.ModulationParams.PacketType = PACKET_TYPE_GFSK;
      SX1280.ModulationParams.Params.Gfsk.BitrateBandwidth = GetFskBitrateBandwidth(bandwidth, spreading);
      SX1280.ModulationParams.Params.Gfsk.ModulationIndex = GFSK_BLE_MOD_IND_0_50;
      SX1280.ModulationParams.Params.Gfsk.ModulationShaping = RADIO_MOD_SHAPING_BT_1_0;

      if (preambleLen < 4) preambleLen = 4;
      if (preambleLen > 32) preambleLen = 32;

      SX1280.PacketParams.PacketType = PACKET_TYPE_GFSK;
      SX1280.PacketParams.Params.Gfsk.PreambleLength = (RadioPreambleLengths_t)(((preambleLen / 4) - 1) << 4);  // convert
      SX1280.PacketParams.Params.Gfsk.SyncWordLength = GFSK_SYNCWORD_LENGTH_3_BYTE;
      SX1280.PacketParams.Params.Gfsk.SyncWordMatch = RADIO_RX_MATCH_SYNCWORD_3;
      SX1280.PacketParams.Params.Gfsk.HeaderType = (fixLen == true) ? RADIO_PACKET_FIXED_LENGTH : RADIO_PACKET_VARIABLE_LENGTH;
      SX1280.PacketParams.Params.Gfsk.PayloadLength = MaxPayloadLength;
      if (crcOn == true) {
        SX1280.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_2_BYTES;
      } else {
        SX1280.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_OFF;
      }
      SX1280.PacketParams.Params.Gfsk.Whitening = RADIO_WHITENING_ON;

      RadioStandby();
      RadioSetModem((SX1280.ModulationParams.PacketType == PACKET_TYPE_GFSK) ? MODEM_FSK : MODEM_LORA);
      SX1280SetModulationParams(&SX1280.ModulationParams);
      SX1280SetPacketParams(&SX1280.PacketParams);
      SX1280SetSyncWord(0, fsk_sync_word);
      SX1280SetWhiteningSeed(0xFF);
      break;

    case MODEM_LORA:
      SX1280.ModulationParams.PacketType = PACKET_TYPE_LORA;
      if (spreading < 5) {
        SX1280.ModulationParams.Params.LoRa.SpreadingFactor = LORA_SF5;
      } else if (spreading > 12) {
        SX1280.ModulationParams.Params.LoRa.SpreadingFactor = LORA_SF12;
      } else {
        SX1280.ModulationParams.Params.LoRa.SpreadingFactor = kSfTable[spreading - 5];
      }

      SX1280.ModulationParams.Params.LoRa.Bandwidth = Bandwidths[bandwidth];
      SX1280.ModulationParams.Params.LoRa.CodingRate = (RadioLoRaCodingRates_t)coderate;

      SX1280.PacketParams.PacketType = PACKET_TYPE_LORA;

      if ((spreading == 6) || (spreading == 5)) {
        if (preambleLen < 12) {
          SX1280.PacketParams.Params.LoRa.PreambleLength = PREAMBLE_LENGTH_12_BITS;
        } else {
          SX1280.PacketParams.Params.LoRa.PreambleLength = ConvertPreamble(preambleLen);
        }
      } else {
        SX1280.PacketParams.Params.LoRa.PreambleLength = ConvertPreamble(preambleLen);
      }

      if (fixLen)
        SX1280.PacketParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
      else
        SX1280.PacketParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
      SX1280.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength;
      if (crcOn)
        SX1280.PacketParams.Params.LoRa.CrcMode = LORA_CRC_ON;
      else
        SX1280.PacketParams.Params.LoRa.CrcMode = LORA_CRC_OFF;
      if (iqInverted)
        SX1280.PacketParams.Params.LoRa.InvertIQ = LORA_IQ_INVERTED;
      else
        SX1280.PacketParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;

      LORARADIO_PRINTLINE("SF=%d", spreading);
      LORARADIO_PRINTLINE("SpreadingFactor=%d, CodingRate=%d", SX1280.ModulationParams.Params.LoRa.SpreadingFactor,
                          SX1280.ModulationParams.Params.LoRa.CodingRate);
      LORARADIO_PRINTLINE("Bandwidth=%d", SX1280.ModulationParams.Params.LoRa.Bandwidth);
      LORARADIO_PRINTLINE("HeaderType=%d, PayloadLength=%d", SX1280.PacketParams.Params.LoRa.HeaderType,
                          SX1280.PacketParams.Params.LoRa.PayloadLength);
      LORARADIO_PRINTLINE("PreambleLength=0x%02X, InvertIQ=%d", SX1280.PacketParams.Params.LoRa.PreambleLength,
                          SX1280.PacketParams.Params.LoRa.InvertIQ);
      LORARADIO_PRINTLINE("Crc=%d", SX1280.PacketParams.Params.LoRa.CrcMode);

      RadioStandby();
      RadioSetModem((SX1280.ModulationParams.PacketType == PACKET_TYPE_GFSK) ? MODEM_FSK : MODEM_LORA);
      SX1280SetModulationParams(&SX1280.ModulationParams);
      SX1280SetPacketParams(&SX1280.PacketParams);
      break;
  }
  SX1280SetTxParams(power, RADIO_RAMP_02_US);
  TxTimeout = timeout;
}

//==========================================================================
//==========================================================================
static bool RadioCheckRfFrequency(uint32_t frequency) { return true; }

//==========================================================================
//==========================================================================
static uint32_t RadioGetLoRaBandwidthInHz(RadioLoRaBandwidths_t bw) {
  uint32_t bandwidthInHz = 0;

  switch (bw) {
    case LORA_BW_0200:
      bandwidthInHz = 203125UL;
      break;
    case LORA_BW_0400:
      bandwidthInHz = 406250UL;
      break;
    case LORA_BW_0800:
      bandwidthInHz = 812500UL;
      break;
    case LORA_BW_1600:
      bandwidthInHz = 1625000UL;
      break;
  }

  return bandwidthInHz;
}

//==========================================================================
//==========================================================================
static uint32_t RadioGetGfskTimeOnAirNumerator(uint32_t bandwidth, uint32_t datarate, uint8_t coderate, uint16_t preambleLen,
                                               bool fixLen, uint8_t payloadLen, bool crcOn) {
  uint16_t BitCount = 0;
  double tPayload = 0.0;
  int8_t sync_word_len = 3;

  BitCount = 4 + (preambleLen >> 4) * 4;               // preamble
  BitCount = BitCount + 8 + (sync_word_len >> 1) * 8;  // sync word
  BitCount = BitCount + ((fixLen) ? 0 : 80);
  BitCount = BitCount + payloadLen * 8;
  BitCount = BitCount + ((crcOn) ? 8 : 0);

  switch (GetFskBitrateBandwidth(bandwidth, datarate)) {
    case GFSK_BLE_BR_2_000_BW_2_4:
      tPayload = (double)BitCount / 2000.0;
      break;

    case GFSK_BLE_BR_1_600_BW_2_4:
      tPayload = (double)BitCount / 1600.0;
      break;

    case GFSK_BLE_BR_1_000_BW_2_4:
    case GFSK_BLE_BR_1_000_BW_1_2:
      tPayload = (double)BitCount / 1000.0;
      break;

    case GFSK_BLE_BR_0_800_BW_2_4:
    case GFSK_BLE_BR_0_800_BW_1_2:
      tPayload = (double)BitCount / 800.0;
      break;

    case GFSK_BLE_BR_0_500_BW_1_2:
    case GFSK_BLE_BR_0_500_BW_0_6:
      tPayload = (double)BitCount / 500.0;
      break;

    case GFSK_BLE_BR_0_400_BW_1_2:
    case GFSK_BLE_BR_0_400_BW_0_6:
      tPayload = (double)BitCount / 400.0;
      break;

    case GFSK_BLE_BR_0_250_BW_0_6:
    case GFSK_BLE_BR_0_250_BW_0_3:
      tPayload = (double)BitCount / 250.0;
      break;

    case GFSK_BLE_BR_0_125_BW_0_3:
      tPayload = (double)BitCount / 125.0;
      break;

    default:
      break;
  }
  return ceil(tPayload);
}

//==========================================================================
//==========================================================================
static uint32_t RadioGetLoRaTimeOnAirNumerator(uint32_t bandwidth, uint32_t datarate, uint8_t coderate, uint16_t preambleLen,
                                               bool fixLen, uint8_t payloadLen, bool crcOn) {
  uint16_t bw = 0.0;
  double nPayload = 0.0;
  double ts = 0.0;
  double tPayload = 0.0;
  uint8_t SF = kSfTable[datarate] >> 4;
  uint8_t crc = (crcOn) ? 16 : 0;
  uint8_t header = (fixLen) ? 0 : 20;

  switch (Bandwidths[bandwidth]) {
    case LORA_BW_0200:
      bw = 203;
      break;

    case LORA_BW_0400:
      bw = 406;
      break;

    case LORA_BW_0800:
      bw = 812;
      break;

    case LORA_BW_1600:
      bw = 1625;
      break;

    default:
      break;
  }

  if (SF < 7) {
    nPayload = max(((double)(payloadLen + crc - (4 * SF) + header)), 0.0);
    nPayload = nPayload / (double)(4 * SF);
    nPayload = ceil(nPayload);
    nPayload = nPayload * (coderate + 4);
    nPayload = nPayload + preambleLen + 6.25 + 8;
  } else if (SF > 10) {
    nPayload = max(((double)(payloadLen + crc - (4 * SF) + 8 + header)), 0.0);
    nPayload = nPayload / (double)(4 * (SF - 2));
    nPayload = ceil(nPayload);
    nPayload = nPayload * (coderate + 4);
    nPayload = nPayload + preambleLen + 4.25 + 8;
  } else {
    nPayload = max(((double)(payloadLen + crc - (4 * SF) + 8 + header)), 0.0);
    nPayload = nPayload / (double)(4 * SF);
    nPayload = ceil(nPayload);
    nPayload = nPayload * (coderate + 4);
    nPayload = nPayload + preambleLen + 4.25 + 8;
  }
  ts = (double)(1 << SF) / (double)(bw);
  tPayload = nPayload * ts;

  return ceil(tPayload);
}

//==========================================================================
//==========================================================================
static uint32_t RadioTimeOnAir(RadioModems_t modem, uint32_t bandwidth, uint32_t datarate, uint8_t coderate, uint16_t preambleLen,
                               bool fixLen, uint8_t payloadLen, bool crcOn) {
  switch (modem) {
    case MODEM_FSK: {
      return RadioGetGfskTimeOnAirNumerator(bandwidth, datarate, coderate, preambleLen, fixLen, payloadLen, crcOn);
    } break;
    case MODEM_LORA: {
      return RadioGetLoRaTimeOnAirNumerator(bandwidth, datarate, coderate, preambleLen, fixLen, payloadLen, crcOn);
    } break;
  }
  return 0;
}

//==========================================================================
//==========================================================================
static void RadioSend(uint8_t* buffer, uint8_t size) {
  SX1280SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RADIO_NONE, IRQ_RADIO_NONE);

  if (SX1280GetPacketType() == PACKET_TYPE_LORA) {
    SX1280.PacketParams.Params.LoRa.PayloadLength = size;
  } else {
    SX1280.PacketParams.Params.Gfsk.PayloadLength = size;
  }
  SX1280SetPacketParams(&SX1280.PacketParams);
  SX1280SetRfFrequency(Frequency);
  SX1280SendPayload(buffer, size, RX_TX_SINGLE);
  TimerSetValue(&TxTimeoutTimer, TxTimeout);
  TimerStart(&TxTimeoutTimer);
}

//==========================================================================
//==========================================================================
static void RadioSleep(void) {
  SleepParams_t params = {0};

  SX1280SetSleep(params);

  DelayMs(2);
}

//==========================================================================
//==========================================================================
static void RadioStandby(void) { SX1280SetStandby(STDBY_RC); }

//==========================================================================
//==========================================================================
static void RadioRx(uint32_t timeout) {
  SX1280SetDioIrqParams(IRQ_RADIO_ALL,  // IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                        IRQ_RADIO_ALL,  // IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                        IRQ_RADIO_NONE, IRQ_RADIO_NONE);

  if (timeout != 0) {
    TimerSetValue(&RxTimeoutTimer, timeout);
    TimerStart(&RxTimeoutTimer);
  }

  if (RxContinuous == true) {
    SX1280SetRfFrequency(Frequency);
    SX1280SetRx(RX_TX_CONTINUOUS);  // Rx Continuous
  } else {
    uint16_t num_of_step = timeout;
    if (timeout > 0xfffe) num_of_step = 0xfffe;
    SX1280SetRfFrequency(Frequency);
    // LORARADIO_PRINTLINE("RadioRx %u, %u", num_of_step, TimerGetCurrentTime());
    // LORARADIO_PRINTLINE("SpreadingFactor=%d, CodingRate=%d", SX1280.ModulationParams.Params.LoRa.SpreadingFactor,
    //                     SX1280.ModulationParams.Params.LoRa.CodingRate);
    // LORARADIO_PRINTLINE("Bandwidth=%d", SX1280.ModulationParams.Params.LoRa.Bandwidth);
    // LORARADIO_PRINTLINE("HeaderType=%d, PayloadLength=%d", SX1280.PacketParams.Params.LoRa.HeaderType,
    //                     SX1280.PacketParams.Params.LoRa.PayloadLength);
    // LORARADIO_PRINTLINE("PreambleLength=%d, InvertIQ=%d", SX1280.PacketParams.Params.LoRa.PreambleLength,
    //                     SX1280.PacketParams.Params.LoRa.InvertIQ);
    // LORARADIO_PRINTLINE("CrcMode=%d", SX1280.PacketParams.Params.LoRa.CrcMode);
    SX1280SetRx((TickTime_t){RADIO_TICK_SIZE_1000_US, num_of_step});
  }
}

//==========================================================================
//==========================================================================
static void RadioSetRxDutyCycle(uint32_t rxTime, uint32_t sleepTime) {
  SX1280SetRxDutyCycle(RADIO_TICK_SIZE_1000_US, rxTime, sleepTime);
}

//==========================================================================
//==========================================================================
static void RadioStartCad(void) {
  SX1280SetDioIrqParams(IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED, IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED, IRQ_RADIO_NONE,
                        IRQ_RADIO_NONE);
  SX1280SetCad();
}

//==========================================================================
//==========================================================================
static void RadioSetTxContinuousWave(uint32_t freq, int8_t power, uint16_t time) {
  uint32_t timeout = (uint32_t)time * 1000;

  SX1280SetRfFrequency(freq);
  SX1280SetTxParams(power, RADIO_RAMP_20_US);
  SX1280SetTxContinuousWave();

  TimerSetValue(&TxTimeoutTimer, timeout);
  TimerStart(&TxTimeoutTimer);
}

//==========================================================================
//==========================================================================
static int16_t RadioRssi(RadioModems_t modem) { return SX1280GetRssiInst(); }

//==========================================================================
//==========================================================================
static void RadioWrite(uint32_t addr, uint8_t data) { SX1280HalWriteRegister(addr, data); }

//==========================================================================
//==========================================================================
static uint8_t RadioRead(uint32_t addr) { return SX1280HalReadRegister(addr); }

//==========================================================================
//==========================================================================
static void RadioWriteBuffer(uint32_t addr, uint8_t* buffer, uint8_t size) { SX1280HalWriteRegisters(addr, buffer, size); }

//==========================================================================
//==========================================================================
static void RadioReadBuffer(uint32_t addr, uint8_t* buffer, uint8_t size) { SX1280HalReadRegisters(addr, buffer, size); }

//==========================================================================
//==========================================================================
static void RadioSetMaxPayloadLength(RadioModems_t modem, uint8_t max) {
  if (modem == MODEM_LORA) {
    SX1280.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength = max;
    SX1280SetPacketParams(&SX1280.PacketParams);
  } else {
    if (SX1280.PacketParams.Params.Gfsk.HeaderType == RADIO_PACKET_VARIABLE_LENGTH) {
      SX1280.PacketParams.Params.Gfsk.PayloadLength = MaxPayloadLength = max;
      SX1280SetPacketParams(&SX1280.PacketParams);
    }
  }
}

//==========================================================================
//==========================================================================
static void RadioSetPublicNetwork(bool enable) {
  RadioPublicNetwork.Current = RadioPublicNetwork.Previous = enable;

  RadioSetModem(MODEM_LORA);
  // if (enable == true) {
  //   // Change LoRa modem SyncWord
  //   SX1280HalWriteRegister(0x944, (LORA_MAC_PUBLIC_SYNCWORD >> 8) & 0xFF);
  //   SX1280HalWriteRegister(0x944 + 1, LORA_MAC_PUBLIC_SYNCWORD & 0xFF);
  // } else {
  //   // Change LoRa modem SyncWord
  //   SX1280HalWriteRegister(0x944, (LORA_MAC_PRIVATE_SYNCWORD >> 8) & 0xFF);
  //   SX1280HalWriteRegister(0x944 + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF);
  // }
}

//==========================================================================
//==========================================================================
static uint32_t RadioGetWakeupTime(void) { return TCXO_WAKEUP_TIME + RADIO_WAKEUP_TIME; }

//==========================================================================
//==========================================================================
static void RadioOnTxTimeoutIrq(void* context) {
  if ((RadioEvents != NULL) && (RadioEvents->TxTimeout != NULL)) {
    RadioEvents->TxTimeout();
  }
}

//==========================================================================
//==========================================================================
static void RadioOnRxTimeoutIrq(void* context) {
  if ((RadioEvents != NULL) && (RadioEvents->RxTimeout != NULL)) {
    RadioEvents->RxTimeout();
  }
}

//==========================================================================
//==========================================================================
static void RadioOnDioIrq(void* context) { IrqFired = true; }

//==========================================================================
//==========================================================================
static void RadioIrqProcess(void) {
  CRITICAL_SECTION_BEGIN();
  // Clear IRQ flag
  const bool isIrqFired = IrqFired;
  IrqFired = false;
  CRITICAL_SECTION_END();

  if (isIrqFired == true) {
    uint16_t irqRegs = SX1280GetIrqStatus();
    SX1280ClearIrqStatus(irqRegs);
    // LORARADIO_PRINTLINE("irqRegs=%02X, %u", irqRegs, TimerGetCurrentTime());

    // Check if DIO1 pin is High. If it is the case revert IrqFired to true
    CRITICAL_SECTION_BEGIN();
    if (SX1280HalGetDioStatus() == 1) {
      IrqFired = true;
    }
    CRITICAL_SECTION_END();

    if ((irqRegs & IRQ_TX_DONE) == IRQ_TX_DONE) {
      TimerStop(&TxTimeoutTimer);
      //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
      SX1280SetStandby(STDBY_RC);
      if ((RadioEvents != NULL) && (RadioEvents->TxDone != NULL)) {
        RadioEvents->TxDone();
      }
    }

    if ((irqRegs & IRQ_RX_DONE) == IRQ_RX_DONE) {
      TimerStop(&RxTimeoutTimer);

      if ((irqRegs & IRQ_CRC_ERROR) == IRQ_CRC_ERROR) {
        if (RxContinuous == false) {
          //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
          SX1280SetStandby(STDBY_RC);
        }
        if ((RadioEvents != NULL) && (RadioEvents->RxError)) {
          RadioEvents->RxError();
        }
      } else {
        uint8_t size;

        if (RxContinuous == false) {
          //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
          SX1280SetStandby(STDBY_RC);
        }
        SX1280GetPayload(RadioRxPayload, &size, 255);
        SX1280GetPacketStatus(&RadioPktStatus);
        if ((RadioEvents != NULL) && (RadioEvents->RxDone != NULL)) {
          RadioEvents->RxDone(RadioRxPayload, size, RadioPktStatus.Params.LoRa.RssiPkt, RadioPktStatus.Params.LoRa.SnrPkt);
        }
      }
    }

    if ((irqRegs & IRQ_CAD_DONE) == IRQ_CAD_DONE) {
      //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
      SX1280SetStandby(STDBY_RC);
      if ((RadioEvents != NULL) && (RadioEvents->CadDone != NULL)) {
        RadioEvents->CadDone(((irqRegs & IRQ_CAD_ACTIVITY_DETECTED) == IRQ_CAD_ACTIVITY_DETECTED));
      }
    }

    if ((irqRegs & IRQ_RX_TX_TIMEOUT) == IRQ_RX_TX_TIMEOUT) {
      if (SX1280GetOperatingMode() == MODE_TX) {
        TimerStop(&TxTimeoutTimer);
        //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
        SX1280SetStandby(STDBY_RC);
        if ((RadioEvents != NULL) && (RadioEvents->TxTimeout != NULL)) {
          RadioEvents->TxTimeout();
        }
      } else if (SX1280GetOperatingMode() == MODE_RX) {
        TimerStop(&RxTimeoutTimer);
        //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
        SX1280SetStandby(STDBY_RC);
        if ((RadioEvents != NULL) && (RadioEvents->RxTimeout != NULL)) {
          RadioEvents->RxTimeout();
        }
      }
    }

    if ((irqRegs & IRQ_PREAMBLE_DETECTED) == IRQ_PREAMBLE_DETECTED) {
      //__NOP( );
    }

    if ((irqRegs & IRQ_SYNCWORD_VALID) == IRQ_SYNCWORD_VALID) {
      //__NOP( );
    }

    if ((irqRegs & IRQ_HEADER_VALID) == IRQ_HEADER_VALID) {
      //__NOP( );
    }

    if ((irqRegs & IRQ_HEADER_ERROR) == IRQ_HEADER_ERROR) {
      TimerStop(&RxTimeoutTimer);
      if (RxContinuous == false) {
        //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
        SX1280SetStandby(STDBY_RC);
      }
      if ((RadioEvents != NULL) && (RadioEvents->RxTimeout != NULL)) {
        RadioEvents->RxTimeout();
      }
    }
  }
}
