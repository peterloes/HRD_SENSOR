/***************************************************************************//**
 * @file
 * @brief	Header file of module SensorMon.c
 * @author	Peter Loes
 * @version	2021-05-03
 ****************************************************************************//*
Revision History:
2015-03-29,rage	Completed documentation.
2014-12-20,rage	Initial version.
*/

#ifndef __INC_SensorMon_h
#define __INC_SensorMon_h

/*=============================== Header Files ===============================*/

#include "em_device.h"
#include "em_gpio.h"
#include "config.h"		// include project configuration parameters

/*=============================== Definitions ================================*/

/*!@brief SBS Commands
 *
 * Here follow the register addresses of the battery controller as used for the
 * SBS commands sent via I2C.  Bit 7:0 of the enum contain the address, while
 * bit 15:8 represent the number of bytes to read for block commands.  A zero
 * means the register is a 16bit word and must be read via function
 * BatteryRegReadWord().  All commands beginn with device address 0x0A to
 * access the battery controller.
 *
 * @see
 * Functions BatteryRegReadWord() and BatteryRegReadBlock() use these enums.
 * A list of all registers can be found in document
 * <a href="../SBS_Commands.pdf">SBS Commands</a>.
 *
 */
typedef enum
{
    SBS_NONE = (-1),		         //!< (-1) No Command / Address
    SBS_REPEATABILITY_CLOCK_HIGH_ENABLED    = 0x2C06,  //!< 0X2C06 enabled
    SBS_REPEATABILITY_CLOCK_MEDIUM_ENABLED  = 0x2C0D,  //!< 0X2C0D enabled
    SBS_REPEATABILITY_CLOCK_LOW_ENABLED     = 0x2C10,  //!< 0X2C10 enabled
    SBS_REPEATABILITY_CLOCK_HIGH_DISABLED   = 0x2400,  //!< 0X2400 disabled
    SBS_REPEATABILITY_CLOCK_MEDIUM_DISABLED = 0x240B,  //!< 0X240B disabled
    SBS_REPEATABILITY_CLOCK_LOW_DISABLED    = 0x2416,  //!< 0X2416 disabled
    SBS_REPEATABILITY_CLOCK_MPS_HIGH        = 0x2032,  //!< 0X2032 0.5mps
    SBS_REPEATABILITY_CLOCK_MPS_MEDIUM      = 0x2024,  //!< 0X2024 0.5mps
    SBS_REPEATABILITY_CLOCK_MPS_LOW         = 0x202F,  //!< 0X202F 0.5mps
    SBS_REPEATABILITY_CLOCK_MPS_1_HIGH      = 0x2130,  //!< 0X2130 1 mps
    SBS_REPEATABILITY_CLOCK_MPS_1_MEDIUM    = 0x2126,  //!< 0X2126 1 mps
    SBS_REPEATABILITY_CLOCK_MPS_1_LOW       = 0x212D,  //!< 0X212D 1 mps
    SBS_REPEATABILITY_CLOCK_MPS_2_HIGH      = 0x2236,  //!< 0X2236 2 mps
    SBS_REPEATABILITY_CLOCK_MPS_2_MEDIUM    = 0x2220,  //!< 0X2220 2 mps
    SBS_REPEATABILITY_CLOCK_MPS_2_LOW       = 0x212B,  //!< 0X212B 2 mps
    SBS_REPEATABILITY_CLOCK_MPS_4_HIGH      = 0x2334,  //!< 0X2334 4 mps
    SBS_REPEATABILITY_CLOCK_MPS_4_MEDIUM    = 0x2322,  //!< 0X2322 4 mps
    SBS_REPEATABILITY_CLOCK_MPS_4_LOW       = 0x2329,  //!< 0X2329 4 mps
    SBS_REPEATABILITY_CLOCK_MPS_10_HIGH     = 0x2737,  //!< 0X2737 10 mps
    SBS_REPEATABILITY_CLOCK_MPS_10_MEDIUM   = 0x2721,  //!< 0X2721 10 mps
    SBS_REPEATABILITY_CLOCK_MPS_10_LOW      = 0x272A,  //!< 0X272A 10 mps
    SBS_FETCH_DATA            = 0xE000,  //!< 0xE000 Fetch Data
    SBS_ART_MEASUREMENT       = 0x2B32,  //!< 0x2B32 ART Periodic Measurement
    SBS_BREAK                 = 0x3093,  //!< 0x3093 Break
    SBS_SOFT_RESET            = 0x30A2,  //!< 0x30A2 Soft Reset 
    SBS_I2C_NRESET            = 0x0006,  //!< 0x0006 I2C_nReset
    SBS_HEATER_ENABLE         = 0x306D,  //!< 0x306D Heater enable
    SBS_HEATER_DISABLE        = 0x3066,  //!< 0x306D Heater disabled
    SBS_READ_STATUS           = 0xF32D,  //!< 0xF32D Word: @see SBS_16_BITS
    SBS_CLEAR_STATUS          = 0x3041,  //!< 0x3041 Clear status register  
    END_SBS_CMD,			//!< End of SBS Command Definitions
} SBS_CMD;

    /*!@brief Macro to extract size from @ref SBS_CMD enum. */
#define SBS_CMD_SIZE(cmd)	((cmd >> 8) & 0xFF)


    /*!@name SBS_16_BITS - Bits of Battery Controller Register 0x16 */
//@{
#define SBS_16_BIT_ALERT_PENDING_STATUS	 15	//!< At least one pending alert '1'
#define SBS_16_BIT_HEATER_STATUS	 13	//!< Heater on '1'
#define SBS_16_BIT_RH_TRACKING_ALERT	 11	//!< RH tracking alert '1' 
#define SBS_16_BIT_T_TRACKING_ALERT	 10	//!< T tracking alert '1'
#define SBS_16_BIT_SYSTEM_RESET_DETECTED  4	//!< System reset detected '1'
#define SBS_16_BIT_COMMAND_STATUS	  1	//!< Last command not processed '1' 
#define SBS_16_BIT_CHECKSUM_STATUS	  0	//!< Checksum of last write failed '1'
//@}



    /*!@brief Error code for I2C timeout, additionally to @ref
     * I2C_TransferReturn_TypeDef
     */
#define i2cTransferTimeout		-10

    /*!@brief Error code for invalid parameter, additionally to @ref
     * I2C_TransferReturn_TypeDef
     */
#define i2cInvalidParameter		-11

/*================================ Prototypes ================================*/

    /* Initialize Sensor Monitor module */
void	 SensorMonInit (void);

    /* Register read functions */
int	 SensorRegReadWord  (SBS_CMD cmd);
int	 SensorRegReadBlock (SBS_CMD cmd, uint8_t *pBuf, size_t bufSize);
uint32_t ReadVdd (void);

#endif /* __INC_BatteryMon_h */
