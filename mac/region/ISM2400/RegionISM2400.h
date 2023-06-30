/*!
 * \file      RegionISM2400.h
 *
 * \brief     Region definition for ISM2400
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 */
#ifndef __REGION_ISM2400_H__
#define __REGION_ISM2400_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "region/Region.h"

/*!
 * LoRaMac maximum number of channels
 */
#define ISM2400_MAX_NB_CHANNELS                       3

/*!
 * Number of default channels
 */
#define ISM2400_NUMB_DEFAULT_CHANNELS                 3

/*!
 * Number of channels to apply for the CF list
 */
#define ISM2400_NUMB_CHANNELS_CF_LIST                 1

/*!
 * Minimal datarate that can be used by the node
 */
#define ISM2400_TX_MIN_DATARATE                       DR_0

/*!
 * Maximal datarate that can be used by the node
 */
#define ISM2400_TX_MAX_DATARATE                       DR_7

/*!
 * Minimal datarate that can be used by the node
 */
#define ISM2400_RX_MIN_DATARATE                       DR_0

/*!
 * Maximal datarate that can be used by the node
 */
#define ISM2400_RX_MAX_DATARATE                       DR_7

/*!
 * Default datarate used by the node
 */
#define ISM2400_DEFAULT_DATARATE                      DR_0

/*!
 * Minimal Rx1 receive datarate offset
 */
#define ISM2400_MIN_RX1_DR_OFFSET                     0

/*!
 * Maximal Rx1 receive datarate offset
 */
#define ISM2400_MAX_RX1_DR_OFFSET                     5

/*!
 * Minimal Tx output power that can be used by the node
 */
#define ISM2400_MIN_TX_POWER                          TX_POWER_7

/*!
 * Maximal Tx output power that can be used by the node
 */
#define ISM2400_MAX_TX_POWER                          TX_POWER_0

/*!
 * Default Tx output power used by the node
 */
#define ISM2400_DEFAULT_TX_POWER                      TX_POWER_0

/*!
 * Default Max EIRP
 */
#define ISM2400_DEFAULT_MAX_EIRP                      16.0f

/*!
 * Default antenna gain
 */
#define ISM2400_DEFAULT_ANTENNA_GAIN                  2.15f

/*!
 * Enabled or disabled the duty cycle
 */
#define ISM2400_DUTY_CYCLE_ENABLED                    1

/*!
 * Maximum RX window duration
 */
#define ISM2400_MAX_RX_WINDOW                         3000

/*!
 * Second reception window channel frequency definition.
 */
#define ISM2400_RX_WND_2_FREQ                         2423000000

/*!
 * Second reception window channel datarate definition.
 */
#define ISM2400_RX_WND_2_DR                           DR_0

/*!
 * Default uplink dwell time configuration
 */
#define ISM2400_DEFAULT_UPLINK_DWELL_TIME             0

/*
 * CLASS B
 */
/*!
 * Beacon frequency
 */
#define ISM2400_BEACON_CHANNEL_FREQ                   2424000000

/*!
 * Ping slot channel frequency
 */
#define ISM2400_PING_SLOT_CHANNEL_FREQ                2424000000

/*!
 * Payload size of a beacon frame
 */
#define ISM2400_BEACON_SIZE                           23

/*!
 * Size of RFU 1 field
 */
#define ISM2400_RFU1_SIZE                             5

/*!
 * Size of RFU 2 field
 */
#define ISM2400_RFU2_SIZE                             3

/*!
 * Datarate of the beacon channel
 */
#define ISM2400_BEACON_CHANNEL_DR                     DR_0

/*!
 * Bandwith of the beacon channel
 */
#define ISM2400_BEACON_CHANNEL_BW                     0

/*!
 * Ping slot channel datarate
 */
#define ISM2400_PING_SLOT_CHANNEL_DR                  DR_3

/*!
 * Maximum number of bands
 */
#define ISM2400_MAX_NB_BANDS                          1

/*!
 * Band 0 definition
 * Band = { DutyCycle, TxMaxPower, LastBandUpdateTime, LastMaxCreditAssignTime, TimeCredits, MaxTimeCredits, ReadyForTransmission }
 */
#define ISM2400_BAND0                                 { 100 , ISM2400_MAX_TX_POWER, 0, 0, 0, 0, 0 } //  1.0 %

/*!
 * LoRaMac default channel 1
 * Channel = { Frequency [Hz], RX1 Frequency [Hz], { ( ( DrMax << 4 ) | DrMin ) }, Band }
 */
#define ISM2400_LC1                                   { 2403000000, 0, { ( ( DR_7 << 4 ) | DR_0 ) }, 0 }

/*!
 * LoRaMac default channel 2
 * Channel = { Frequency [Hz], RX1 Frequency [Hz], { ( ( DrMax << 4 ) | DrMin ) }, Band }
 */
#define ISM2400_LC2                                   { 2425000000, 0, { ( ( DR_7 << 4 ) | DR_0 ) }, 0 }

/*!
 * LoRaMac default channel 3
 * Channel = { Frequency [Hz], RX1 Frequency [Hz], { ( ( DrMax << 4 ) | DrMin ) }, Band }
 */
#define ISM2400_LC3                                   { 2479000000, 0, { ( ( DR_7 << 4 ) | DR_0 ) }, 0 }

/*!
 * LoRaMac channels which are allowed for the join procedure
 */
#define ISM2400_JOIN_CHANNELS                         ( uint16_t )( LC( 1 ) | LC( 2 ) | LC( 3 ) )

/*!
 * Data rates table definition
 */
static const uint8_t DataratesISM2400[]  = { 12, 11, 10,  9,  8,  7,  6, 5 };

/*!
 * Bandwidths table definition in Hz
 */
static const uint32_t BandwidthsISM2400[] = { 812000, 812000, 812000, 812000, 812000, 812000, 812000, 812000};

/*!
 * Maximum payload with respect to the datarate index.
 */
static const uint8_t MaxPayloadOfDatarateISM2400[] = { 51, 115, 220, 220, 220, 220, 220, 220 };

/*!
 * \brief The function gets a value of a specific phy attribute.
 *
 * \param [IN] getPhy Pointer to the function parameters.
 *
 * \retval Returns a structure containing the PHY parameter.
 */
PhyParam_t RegionISM2400GetPhyParam( GetPhyParams_t* getPhy );

/*!
 * \brief Updates the last TX done parameters of the current channel.
 *
 * \param [IN] txDone Pointer to the function parameters.
 */
void RegionISM2400SetBandTxDone( SetBandTxDoneParams_t* txDone );

/*!
 * \brief Initializes the channels masks and the channels.
 *
 * \param [IN] type Sets the initialization type.
 */
void RegionISM2400InitDefaults( InitDefaultsParams_t* params );

/*!
 * \brief Verifies a parameter.
 *
 * \param [IN] verify Pointer to the function parameters.
 *
 * \param [IN] type Sets the initialization type.
 *
 * \retval Returns true, if the parameter is valid.
 */
bool RegionISM2400Verify( VerifyParams_t* verify, PhyAttribute_t phyAttribute );

/*!
 * \brief The function parses the input buffer and sets up the channels of the
 *        CF list.
 *
 * \param [IN] applyCFList Pointer to the function parameters.
 */
void RegionISM2400ApplyCFList( ApplyCFListParams_t* applyCFList );

/*!
 * \brief Sets a channels mask.
 *
 * \param [IN] chanMaskSet Pointer to the function parameters.
 *
 * \retval Returns true, if the channels mask could be set.
 */
bool RegionISM2400ChanMaskSet( ChanMaskSetParams_t* chanMaskSet );

/*!
 * Computes the Rx window timeout and offset.
 *
 * \param [IN] datarate     Rx window datarate index to be used
 *
 * \param [IN] minRxSymbols Minimum required number of symbols to detect an Rx frame.
 *
 * \param [IN] rxError      System maximum timing error of the receiver. In milliseconds
 *                          The receiver will turn on in a [-rxError : +rxError] ms
 *                          interval around RxOffset
 *
 * \param [OUT]rxConfigParams Returns updated WindowTimeout and WindowOffset fields.
 */
void RegionISM2400ComputeRxWindowParameters( int8_t datarate, uint8_t minRxSymbols, uint32_t rxError, RxConfigParams_t *rxConfigParams );

/*!
 * \brief Configuration of the RX windows.
 *
 * \param [IN] rxConfig Pointer to the function parameters.
 *
 * \param [OUT] datarate The datarate index which was set.
 *
 * \retval Returns true, if the configuration was applied successfully.
 */
bool RegionISM2400RxConfig( RxConfigParams_t* rxConfig, int8_t* datarate );

/*!
 * \brief TX configuration.
 *
 * \param [IN] txConfig Pointer to the function parameters.
 *
 * \param [OUT] txPower The tx power index which was set.
 *
 * \param [OUT] txTimeOnAir The time-on-air of the frame.
 *
 * \retval Returns true, if the configuration was applied successfully.
 */
bool RegionISM2400TxConfig( TxConfigParams_t* txConfig, int8_t* txPower, TimerTime_t* txTimeOnAir );

/*!
 * \brief The function processes a Link ADR Request.
 *
 * \param [IN] linkAdrReq Pointer to the function parameters.
 *
 * \retval Returns the status of the operation, according to the LoRaMAC specification.
 */
uint8_t RegionISM2400LinkAdrReq( LinkAdrReqParams_t* linkAdrReq, int8_t* drOut, int8_t* txPowOut, uint8_t* nbRepOut, uint8_t* nbBytesParsed );

/*!
 * \brief The function processes a RX Parameter Setup Request.
 *
 * \param [IN] rxParamSetupReq Pointer to the function parameters.
 *
 * \retval Returns the status of the operation, according to the LoRaMAC specification.
 */
uint8_t RegionISM2400RxParamSetupReq( RxParamSetupReqParams_t* rxParamSetupReq );

/*!
 * \brief The function processes a Channel Request.
 *
 * \param [IN] newChannelReq Pointer to the function parameters.
 *
 * \retval Returns the status of the operation, according to the LoRaMAC specification.
 */
int8_t RegionISM2400NewChannelReq( NewChannelReqParams_t* newChannelReq );

/*!
 * \brief The function processes a TX ParamSetup Request.
 *
 * \param [IN] txParamSetupReq Pointer to the function parameters.
 *
 * \retval Returns the status of the operation, according to the LoRaMAC specification.
 *         Returns -1, if the functionality is not implemented. In this case, the end node
 *         shall not process the command.
 */
int8_t RegionISM2400TxParamSetupReq( TxParamSetupReqParams_t* txParamSetupReq );

/*!
 * \brief The function processes a DlChannel Request.
 *
 * \param [IN] dlChannelReq Pointer to the function parameters.
 *
 * \retval Returns the status of the operation, according to the LoRaMAC specification.
 */
int8_t RegionISM2400DlChannelReq( DlChannelReqParams_t* dlChannelReq );

/*!
 * \brief Alternates the datarate of the channel for the join request.
 *
 * \param [IN] currentDr Current datarate.
 *
 * \retval Datarate to apply.
 */
int8_t RegionISM2400AlternateDr( int8_t currentDr, AlternateDrType_t type );

/*!
 * \brief Searches and set the next random available channel
 *
 * \param [OUT] channel Next channel to use for TX.
 *
 * \param [OUT] time Time to wait for the next transmission according to the duty
 *              cycle.
 *
 * \param [OUT] aggregatedTimeOff Updates the aggregated time off.
 *
 * \retval Function status [1: OK, 0: Unable to find a channel on the current datarate]
 */
LoRaMacStatus_t RegionISM2400NextChannel( NextChanParams_t* nextChanParams, uint8_t* channel, TimerTime_t* time, TimerTime_t* aggregatedTimeOff );

/*!
 * \brief Adds a channel.
 *
 * \param [IN] channelAdd Pointer to the function parameters.
 *
 * \retval Status of the operation.
 */
LoRaMacStatus_t RegionISM2400ChannelAdd( ChannelAddParams_t* channelAdd );

/*!
 * \brief Removes a channel.
 *
 * \param [IN] channelRemove Pointer to the function parameters.
 *
 * \retval Returns true, if the channel was removed successfully.
 */
bool RegionISM2400ChannelsRemove( ChannelRemoveParams_t* channelRemove  );

/*!
 * \brief Computes new datarate according to the given offset
 *
 * \param [IN] downlinkDwellTime Downlink dwell time configuration. 0: No limit, 1: 400ms
 *
 * \param [IN] dr Current datarate
 *
 * \param [IN] drOffset Offset to be applied
 *
 * \retval newDr Computed datarate.
 */
uint8_t RegionISM2400ApplyDrOffset( uint8_t downlinkDwellTime, int8_t dr, int8_t drOffset );

/*!
 * \brief Sets the radio into beacon reception mode
 *
 * \param [IN] rxBeaconSetup Pointer to the function parameters
 */
void RegionISM2400RxBeaconSetup( RxBeaconSetup_t* rxBeaconSetup, uint8_t* outDr );

/*!
 * \brief Gets the bandwidth.
 *
 * \param [IN] drIndex Datarate index.
 *
 * \param [IN] bandwidths A pointer to the bandwidth table.
 *
 * \retval Bandwidth.
 */
uint32_t RegionISM2400GetBandwidth( uint32_t drIndex, const uint32_t* bandwidths );

/*! \} defgroup REGIONISM2400 */

#ifdef __cplusplus
}
#endif

#endif // __REGION_ISM2400_H__
