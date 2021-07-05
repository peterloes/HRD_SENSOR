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

    /*!@brief Sensor Controller Types (bit mask) */
typedef enum
{
    BCT_UNKNOWN    = 0x00,	//!< Sensor Controller Type not known yet
    BCT_SHT3X_DFLT = 0x01,	//!< Sensirion Controller default(2015)
    BCT_SHT3X_ALT  = 0x02,	//!< Sensirion Controller (2015)
} BC_TYPE;

/*!@brief SBS Commands
 *
 * These are the defines for the registers of the sensor controller.  Each
 * define contains the following bit fields:
 * - Bit 26:24 specify the controller type where the define belongs to. Bit 24
 *   (0x1000000) represents the Sensor SHT3X-DFLT controller,
 *   while bit 25 (0x200000) specifies the Sensor SHT3X-ALT controller.
 *   If both bits are set (0x300000), the register exists in both controller
 *   types.  Bit 20 is used if no controller is connected to
 *   identify the remaining valid entries (where SBS_NONE is set).
 * - Bit 23:16 contains the number of bytes to read.  For 8, 16, or 32 bit values
 *   SensorRegReadValue() is called, while for more than 4 bytes function
 *   SensorRegReadBlock() will be used (block command).
 * - Bit 15:0 of the enum contains the address as used for the SBS commands sent
 *   via I2C.
 * All command sequences beginn with I²C device address to access the sensor
 * controller. The device address depends on the type of battery controller:
 * - 0x44 in case of SHT3XL, and
 * - 0x45 for the SHT3XH.
 * The address can be probed via SensorCtrlProbe().
 * @see
 * Functions SensorRegReadWord() and SensorRegReadBlock() use these enums.
 * A list of all registers can be found in document
 * <a href="../SBS_Commands.pdf">SBS Commands</a>.
 *
 */
typedef enum
{
    SBS_NONE = (-1),		         //!< (-1) No Command / Address
    SBS_REPEATABILITY_CLOCK_HIGH_ENABLED    = 0x3022C06,  //!< 0X2C06 enabled
    SBS_REPEATABILITY_CLOCK_MEDIUM_ENABLED  = 0x3002C0D,  //!< 0X2C0D enabled
    SBS_REPEATABILITY_CLOCK_LOW_ENABLED     = 0x3002C10,  //!< 0X2C10 enabled
    SBS_REPEATABILITY_CLOCK_HIGH_DISABLED   = 0x3002400,  //!< 0X2400 disabled
    SBS_REPEATABILITY_CLOCK_MEDIUM_DISABLED = 0x302240B,  //!< 0X240B disabled
    SBS_REPEATABILITY_CLOCK_LOW_DISABLED    = 0x3022416,  //!< 0X2416 disabled
    SBS_REPEATABILITY_CLOCK_MPS_HIGH        = 0x3022032,  //!< 0X2032 0.5mps
    SBS_REPEATABILITY_CLOCK_MPS_MEDIUM      = 0x3022024,  //!< 0X2024 0.5mps
    SBS_REPEATABILITY_CLOCK_MPS_LOW         = 0x302202F,  //!< 0X202F 0.5mps
    SBS_REPEATABILITY_CLOCK_MPS_1_HIGH      = 0x3022130,  //!< 0X2130 1 mps
    SBS_REPEATABILITY_CLOCK_MPS_1_MEDIUM    = 0x3022126,  //!< 0X2126 1 mps
    SBS_REPEATABILITY_CLOCK_MPS_1_LOW       = 0x302212D,  //!< 0X212D 1 mps
    SBS_REPEATABILITY_CLOCK_MPS_2_HIGH      = 0x3022236,  //!< 0X2236 2 mps
    SBS_REPEATABILITY_CLOCK_MPS_2_MEDIUM    = 0x3022220,  //!< 0X2220 2 mps
    SBS_REPEATABILITY_CLOCK_MPS_2_LOW       = 0x302212B,  //!< 0X212B 2 mps
    SBS_REPEATABILITY_CLOCK_MPS_4_HIGH      = 0x3022334,  //!< 0X2334 4 mps
    SBS_REPEATABILITY_CLOCK_MPS_4_MEDIUM    = 0x3022322,  //!< 0X2322 4 mps
    SBS_REPEATABILITY_CLOCK_MPS_4_LOW       = 0x3022329,  //!< 0X2329 4 mps
    SBS_REPEATABILITY_CLOCK_MPS_10_HIGH     = 0x3022737,  //!< 0X2737 10 mps
    SBS_REPEATABILITY_CLOCK_MPS_10_MEDIUM   = 0x3022721,  //!< 0X2721 10 mps
    SBS_REPEATABILITY_CLOCK_MPS_10_LOW      = 0x302272A,  //!< 0X272A 10 mps
    SBS_READ_SERIAL_NUMBER    = 0x3023780,  //!< 0x3780 Serial Number
    SBS_FETCH_DATA            = 0x306E000,  //!< 0xE000 Fetch Data
    SBS_ART_MEASUREMENT       = 0x3022B32,  //!< 0x2B32 ART Periodic Measurement
    SBS_BREAK                 = 0x3023093,  //!< 0x3093 Break
    SBS_SOFT_RESET            = 0x30230A2,  //!< 0x30A2 Soft Reset 
    SBS_I2C_NRESET            = 0x3020006,  //!< 0x0006 I2C_nReset
    SBS_HEATER_ENABLE         = 0x302306D,  //!< 0x306D Heater enable
    SBS_HEATER_DISABLE        = 0x3023066,  //!< 0x306D Heater disabled
    SBS_READ_STATUS           = 0x302F32D,  //!< 0xF32D Hex: @see SBS_16_BITS
    SBS_CLEAR_STATUS          = 0x3023041,  //!< 0x3041 Clear status register  
    END_SBS_CMD,			   //!< End of SBS Command Definitions
} SBS_CMD;


    /*!@brief Macro to extract address from @ref SBS_CMD enum. */
#define SBS_CMD_ADDR(cmd)	((cmd) & 0xFF)

    /*!@brief Macro to extract size from @ref SBS_CMD enum. */
#define SBS_CMD_SIZE(cmd)	((cmd >> 16) & 0xFF)


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


/*================================ Global Data ===============================*/

    /* I2C Device Address of the Sensor Controller */
extern uint8_t g_SensorCtrlAddr;

    /* Sensor Controller Type */
extern BC_TYPE g_SensorCtrlType;

    /* ASCII Name of the Sensor Controller, or "" if no one found */
extern const char *g_SensorCtrlName;

/*================================ Prototypes ================================*/

    /* Initialize Sensor Monitor module */
void	 SensorMonInit (void);

    /* Probe for Controller Type */
void	 SensorCtrlProbe (void);


    /* Register read functions */
int	 SensorRegReadValue (SBS_CMD cmd, uint32_t *pValue);
int	 SensorRegReadBlock (SBS_CMD cmd, uint8_t  *pBuf, size_t bufSize);
uint32_t ReadVdd (void);

#endif /* __INC_SensorMon_h */
