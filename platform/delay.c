/*!
 * \file      delay.c
 *
 * \brief     Delay implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <stdlib.h>
#include <unistd.h>

#include "delay.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

void Delay( float s )
{
    uint32_t ms = (uint32_t)(s * 1000);
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void DelayMs( uint32_t ms )
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}
