//==========================================================================
//==========================================================================
#ifndef _INC_SX_GPIO_H
#define _INC_SX_GPIO_H

//==========================================================================
//==========================================================================
#define SPIHOST SPI3_HOST

#if defined(CONFIG_MATCHX_TARGET_X2E_REF)
// GPIO for X2E Reference Sensor V1.0
// #define SX1280_SS GPIO_NUM_41
// #define SX1280_nRES GPIO_NUM_45
// #define SX1280_BUSY GPIO_NUM_46
// #define SX1280_DIO1 GPIO_NUM_42

// #define SX1261_SS GPIO_NUM_47
// #define SX1261_nRES GPIO_NUM_33
// #define SX1261_BUSY GPIO_NUM_48
// #define SX1261_DIO1 GPIO_NUM_40

// GPIO for X2E Reference Sensor V1.1
#define SX1280_SS GPIO_NUM_47
#define SX1280_nRES GPIO_NUM_40
#define SX1280_BUSY GPIO_NUM_48
#define SX1280_DIO1 GPIO_NUM_33

#define SX1261_SS GPIO_NUM_41
#define SX1261_nRES GPIO_NUM_42
#define SX1261_BUSY GPIO_NUM_46
#define SX1261_DIO1 GPIO_NUM_45

#elif defined(CONFIG_MATCHX_TARGET_NEXUS)
// GPIO for Nexus
#define SX1280_SS GPIO_NUM_18
#define SX1280_nRES GPIO_NUM_33
#define SX1280_BUSY GPIO_NUM_47
#define SX1280_DIO1 GPIO_NUM_21

#define SX1261_SS GPIO_NUM_40
#define SX1261_nRES GPIO_NUM_41
#define SX1261_BUSY GPIO_NUM_42
#define SX1261_DIO1 GPIO_NUM_45
#else
#error "Please define the MatchX target board."
#endif

//==========================================================================
//==========================================================================
#endif // _INC_SX_GPIO_H
