//==========================================================================
//==========================================================================
#ifndef INC_LORA_CRC16_H
#define INC_LORA_CRC16_H
//==========================================================================
//==========================================================================
#include <stdint.h>
#include <stddef.h>

//==========================================================================
//==========================================================================
uint16_t LoRaCrc16CcittOne(uint16_t aCrc, uint8_t aData);
uint16_t LoRaCrc16Ccitt(uint16_t aCrc, const uint8_t *aData, size_t aLen);

//==========================================================================
//==========================================================================
#endif // INC_LORA_CRC16_H