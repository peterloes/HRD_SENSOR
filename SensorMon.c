/***************************************************************************//**
 * @file
 * @brief	Sensor Monitoring
 * @author	Peter Loes
 * @version	2021-05-03
 *
 * This module can be used to read status information from the sensor sht31-d
 * (sensirion) via its SMBus interface. It also provides function 
 * ReadVdd() to read the voltage of the local supply battery.
 *
 * Parts of the code are based on the example code of AN0021
 * "Analog to Digital Converter" from Energy Micro AS / Silicon Labs.
 *
 ***************************************************************************//**
 *
 * Parts are Copyright 2013 Energy Micro AS, http://www.energymicro.com
 *
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ****************************************************************************//*
Revision History:
2020-01013,rage	Implemented probing for a connected sensor controller type.
		Make information available via variables g_SensorCtrlAddr,
		g_SensorCtrlType, and g_SensorCtrlName.
		Reading the voltage of the CR2032 battery is now possible before
		calling BatteryMonInit().  This is used to calculate the LCD
		contrast value depending on the CR2032 voltage.
2015-10-13,rage	BugFix: <SMB_Status> must be declared "volatile".
2015-06-24,rage	Added support for local battery voltage measurement (CR3032).
2015-06-22,rage	Initial version, derived from RFID_MS alias SNB_Heaven.
*/

/*=============================== Header Files ===============================*/

#include "em_cmu.h"
#include "em_i2c.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_adc.h"
#include "AlarmClock.h"		// msDelay()
#include "SensorMon.h"
#include "Display.h"
   

/*=============================== Definitions ================================*/

    /*!@name Hardware Configuration: SMBus controller and pins. */
//@{
#define SMB_GPIOPORT		gpioPortA	//!< Port SMBus interface
#define SMB_SDA_PIN		0		//!< Pin for SDA signal
#define SMB_SCL_PIN		1		//!< Pin for SCL signal
#define SMB_I2C_CTRL		I2C0		//!< I2C controller to use
#define SMB_I2C_CMUCLOCK	cmuClock_I2C0	//!< Enable clock for I2C
#define SMB_LOC		I2C_ROUTE_LOCATION_LOC0 //!< Use location 0
#define SMB_IRQn		I2C0_IRQn	//!< I2C controller interrupt
#define SMB_IRQHandler		I2C0_IRQHandler	//!< SMBus interrupt handler
#define MEASURE_VD_PORT		gpioPortD	//!< Port to enable voltage divider
#define MEASURE_VD_PIN		1		//!< Pin for voltage divider
#define MEASURE_ADC_PORT	gpioPortD	//!< Port to enable voltage divider
#define MEASURE_ADC_PIN		1		//!< Pin for voltage divider
//@}

    /*!@brief I2C Transfer Timeout (500ms) in RTC ticks */
#define I2C_XFER_TIMEOUT	(RTC_COUNTS_PER_SEC / 2)

    /*!@brief I2C Recovery Timeout (5s) in RTC ticks */
#define I2C_RECOVERY_TIMEOUT	(RTC_COUNTS_PER_SEC * 5)

    /*!@brief Structure to hold Information about a Sensor Controller */
typedef struct
{
    uint8_t	 addr;		//!< SMBus address of the sensor controller
    BC_TYPE	 type;		//!< Corresponding controller type
    const char	*name;		//!< ASCII name of the controller
} BC_INFO;


/*================================== Macros ==================================*/

#ifndef LOGGING		// define as UART output, if logging is not enabled
    #define LogError(str)	drvLEUART_puts(str "\n")
#endif

/*================================ Global Data ===============================*/

    /*!@brief I2C Device Address of the Sensor Controller */
uint8_t g_SensorCtrlAddr;

    /*!@brief Battery Controller Type */
BC_TYPE g_SensorCtrlType = BCT_UNKNOWN;

    /*!@brief ASCII Name of the Sensor Controller, or "" if no one found */
const char *g_SensorCtrlName;


/*================================ Local Data ================================*/

    /*!@brief Probe List of supported Sensor Controllers */
static const BC_INFO l_ProbeList[] =
{  //  addr	type		name (maximum 10 characters!)
    {  0x88,	BCT_SHT3X_DFLT,	"SHT3X-DFLT"	},  // 0x44 Address 7 MSBs 0x88
    {  0x8A,	BCT_SHT3X_ALT,	"SHT3X-ALT",	},  // 0x45 Address 7 MSBs 0x8A
    {  0x00,	BCT_UNKNOWN,	""		}   // End of the list};
};
    
    /* Defining the SMBus initialization data */
static I2C_Init_TypeDef smbInit =
{
  .enable   = true,		// Enable controller after initialization
  .master   = true,		// Configure for MASTER mode
  .refFreq  = 0,		// Get clock frequency from clock source
  .freq     = 10000,		// Low frequency because of long SMBus wires
  .clhr     = i2cClockHLRStandard,	// Set to use 4:4 low/high duty cycle
};

    /* Status of the last SMBus transaction */
static volatile I2C_TransferReturn_TypeDef SMB_Status;

/*=========================== Forward Declarations ===========================*/

static void	DisplaySensorType(int userParm);
static void	ADC_Config(void);


/***************************************************************************//**
 *
 * @brief	Initialize the sensor monitoring module
 *
 * This routine initializes the board-specific SMBus (I2C) interface, which
 * is connected to the sensor.
 *
 ******************************************************************************/
void	 SensorMonInit (void)
{
    /* Be sure to enable clock to GPIO (should already be done) */
    CMU_ClockEnable (cmuClock_GPIO, true);

    /* Enable clock for I2C controller*/
    CMU_ClockEnable(SMB_I2C_CMUCLOCK, true);

#if 0	//RAGE: We currently use the internal Vdd/3 channel, see ReadVdd()
    /* Configure GPIO to enable voltage divider for local 3V measurements */
    GPIO_PinModeSet (MEASURE_VD_PORT, MEASURE_VD_PIN, gpioModePushPull, 0);

    /* Configure GPIO as analog input for local 3V measurements */
    GPIO_PinModeSet (MEASURE_ADC_PORT, MEASURE_ADC_PIN, gpioModePushPull, 0);
#endif

    /* Configure GPIOs for SMBus (I2C) functionality with Pull-Ups */
    GPIO_PinModeSet (SMB_GPIOPORT, SMB_SCL_PIN, gpioModeWiredAndPullUp, 1);
    GPIO_PinModeSet (SMB_GPIOPORT, SMB_SDA_PIN, gpioModeWiredAndPullUp, 1);
       
   /* Route SMB signals to the respective pins */
   SMB_I2C_CTRL->ROUTE = I2C_ROUTE_SCLPEN | I2C_ROUTE_SDAPEN | SMB_LOC;

   /* Initialize SMBus (I2C) controller */
   I2C_Init (SMB_I2C_CTRL, &smbInit);
   
    /* Clear and enable SMBus interrupt */
    NVIC_ClearPendingIRQ (SMB_IRQn);
    NVIC_EnableIRQ (SMB_IRQn);
}

    
/***************************************************************************//**
 *
 * @brief	De-Initialize the sensor monitoring module
 *
 * This routine brings the SMBus (I2C) interface, which is connected to the
 * sensor, into a quiescent state.
 *
 ******************************************************************************/
void	 SensorMonDeinit (void)
{
    /* Make sure conversion is not in progress */
    ADC0->CMD = ADC_CMD_SINGLESTOP | ADC_CMD_SCANSTOP;

    /* Disable SMBus interrupt */
    NVIC_DisableIRQ (SMB_IRQn);

    /* Reset SMBus controller */
    I2C_Reset (SMB_I2C_CTRL);

    /* Reset ADC */
    ADC_Reset (ADC0);

    /* Disable clock for I2C controller and ADC */
    CMU_ClockEnable(SMB_I2C_CMUCLOCK, false);
    CMU_ClockEnable(cmuClock_ADC0, false);
    
    /* Reset variables */
    g_SensorCtrlAddr = 0x00;
    g_SensorCtrlName = "";
    g_SensorCtrlType = BCT_UNKNOWN;
}


/***************************************************************************//**
 *
 * @brief	Probe for Controller Type
 *
 * This routine probes the type of sensor controller.  This is done by checking
 * dedicated I2C-bus addresses on the SMBus.  The following addresses and their
 * corresponding controller type are supported:
 * - 0x88 MSBs (0x44) in case of SHT3X-ADDR-DFLT and
 * - 0x8A MSBs (0x45) for the SHT3X-ADDR-ALT.
 * The address is stored in @ref g_SensorCtrlAddr, its ASCII name in @ref
 * g_SensorCtrlName and the controller type is stored as bit definition
 * @ref BC_TYPE in @ref g_SensorCtrlType.
 *
 ******************************************************************************/
void SensorCtrlProbe (void)
{
int	i;
int	status;


    for (i = 0;  l_ProbeList[i].addr != 0x00;  i++)
    {
	g_SensorCtrlAddr = l_ProbeList[i].addr;	// try this address
   	status = SensorRegReadValue(SBS_READ_SERIAL_NUMBER, NULL);
        if (status >= 0)
	{
	    /* Response from controller - sensor found */
	    break;
	}
	else
	{
	    if (status != i2cTransferNack)
		ConsolePrintf ("SensorCtrlProbe: Unexpected error %d\n",
				status);
	}
    }

    g_SensorCtrlAddr = l_ProbeList[i].addr;
    g_SensorCtrlName = l_ProbeList[i].name;
    g_SensorCtrlType = l_ProbeList[i].type;

#if 0
    /*
     * RAGE WORKAROUND: There may be some Sensor Packs with new 
     * controller out in the field, that use I2C-bus address 0x44.  These
     * would be detected as "SHT3X-ADDR-DFLT" devices, which is wrong.
     * Therefore this workaround probes for register SBS_READ_STATUS
     * (0xF32D) which (only) exists in the SHT3X-D controller.
     */
    status = SensorRegReadValue(SBS_READ_STATUS, NULL);
    if (status >= 0)
    {
	/* Register exists - must be Sensor controller */
	g_SensorCtrlName = l_ProbeList[1].name;
	g_SensorCtrlType = l_ProbeList[1].type;
    }
#endif

    DisplayNext(3, DisplaySensorType, (int)g_SensorCtrlType);
}

/***************************************************************************//**
 *
 * @brief	Display Sensor Type
 *
 * This routine displays the sensor type on the LCD and the serial console.
 * It will be installed by the probe routine SensorCtrlProbe() via function
 * DisplayNext() to allow some delay between probing and displaying.
 *
 ******************************************************************************/
static void	DisplaySensorType(int userParm)
{
    (void) userParm;	// suppress compiler warning "unused parameter"

    DisplaySelectItem(1);	// select item 1: "Sensor at SMBus"
}



/***************************************************************************//**
 *
 * @brief	SMBus Interrupt Handler
 *
 * This handler is executed for each byte transferred via the SMBus interface.
 * It calls the driver function I2C_Transfer() to prepare the next data byte,
 * or generate a STOP condition at the end of a transfer.
 *
 ******************************************************************************/
void	 SMB_IRQHandler (void)
{
    /* Update <SMB_Status> */
    SMB_Status = I2C_Transfer (SMB_I2C_CTRL);
}


/***************************************************************************//**
 *
 * @brief	SMBus Reset
 *
 * This internal routine aborts the current I2C-bus transfer and tries to
 * recover from a state where SCL is driven low by the battery controller.
 * It should be called if there occurs a timeout of a transfer.
 *
 ******************************************************************************/
static void  SMB_Reset (void)
{
    LogError("SMB_Reset: Try to recover from invalid state");

    /* abort the current transfer */
    SMB_I2C_CTRL->CMD = I2C_CMD_ABORT;
    msDelay(100);

    /* check if SCL is still low */
    if ((GPIO->P[SMB_GPIOPORT].DIN & (1 << SMB_SCL_PIN)) == 0)
    {
	/* drive SDA low */
	GPIO->P[SMB_GPIOPORT].DOUTCLR = (1 << SMB_SDA_PIN);
	SMB_I2C_CTRL->ROUTE = I2C_ROUTE_SCLPEN | SMB_LOC;

	/* wait until SCL returns to high */
	uint32_t start = RTC->CNT;
	while ((GPIO->P[SMB_GPIOPORT].DIN & (1 << SMB_SCL_PIN)) == 0)
	{
	    /* check for timeout */
	    if (((RTC->CNT - start) & 0x00FFFFFF) > I2C_RECOVERY_TIMEOUT)
	    {
		LogError("SMB_Reset: Recovery failed, giving up");
		break;
	    }
	}

	/* re-configure GPIO as SDA signal */
	SMB_I2C_CTRL->ROUTE = I2C_ROUTE_SCLPEN | I2C_ROUTE_SDAPEN | SMB_LOC;
    }
}
/***************************************************************************//**
 *
 * @brief	Read Register Value from the Sensor Controller
 *
 * This routine reads two bytes from the register address specified by @p cmd,
 * assembles them to a 16bit value, and returns this.  If an error occurred,
 * a negative status code is returned instead.
 *
 * @param[in] cmd
 *	SBS command, i.e. the register address to be read.
 *
 * @param[in] pValue
 *	Address of 32bit variable where to store the value read from the
 *	register.  May be set to NULL, if value is omitted.
 *
 * @return
 *	Status code @ref i2cTransferDone (0), or a negative error code of type
 *	@ref I2C_TransferReturn_TypeDef.  Additionally to those codes, there is
 *	another error code defined, named @ref i2cTransferTimeout.
 *
 * @see
 *	SensorRegReadBlock()
 *
 ******************************************************************************/
int	 SensorRegReadValue (SBS_CMD cmd, uint32_t *pValue)
{
uint8_t  dataBuf[6];			// buffer for data read from register
uint32_t value = 0;
int	 status;
int	 i;


    /* Call block command to transfer data bytes into buffer */
    status = SensorRegReadBlock (cmd, dataBuf, sizeof(dataBuf));

    if (status == i2cTransferDone  &&  pValue != NULL)
    {
	/* build value from data buffer (always little endian) */
	for (i = SBS_CMD_SIZE(cmd) - 1;  i >= 0;  i--)
	    value = (value << 8) | dataBuf[i];

	*pValue = value;
    }
 
    return status;
}


/***************************************************************************//**
 *
 * @brief	Read Data Block from the Sensor Controller
 *
 * This routine reads an amount of bytes from the sensor controller, as
 * specified by parameter cmd.  This contains the register address and number
 * of bytes to read.
 *
 * @param[in] cmd
 *	SBS command, i.e. the register address and number of bytes to read.
 *
 * @param[out] pBuf
 *	Address of a buffer where to store the data.
 *
 * @param[in] rdCnt
 *	Number of bytes to read.
 *
 * @return
 *	Status code @ref i2cTransferDone (0), or a negative error code of type
 *	@ref I2C_TransferReturn_TypeDef.  Additionally to those codes, there is
 *	another error code defined, named @ref i2cTransferTimeout.
 *
 * @see
 *	SensorRegReadValue()
 *
 ******************************************************************************/
int	SensorRegReadBlock (SBS_CMD cmd, uint8_t *pBuf, size_t rdCnt)
{
I2C_TransferSeq_TypeDef smbXfer;	// SMBus transfer data
uint8_t addrBuf[2];			// buffer for command address



    /* Check parameters */
    EFM_ASSERT (SBS_CMD_SIZE(cmd) != 0);// size field must not be 0
    EFM_ASSERT (pBuf != NULL);		// buffer address
    EFM_ASSERT (rdCnt >= SBS_CMD_SIZE(cmd));	// buffer size

   if (rdCnt < SBS_CMD_SIZE(cmd))	// if EFM_ASSERT() is empty
	return i2cInvalidParameter;

    /* Set up SMBus transfer S-Wr-Cmd-Sr-Rd-data1-P */
    smbXfer.addr  = g_SensorCtrlAddr; // I2C address of the Sensor
    smbXfer.flags = I2C_FLAG_WRITE_READ;    // write address, then read data
    smbXfer.buf[0].data = addrBuf;    // first buffer (data to write)
    addrBuf[0] = cmd >> 8;        // register address (high byte)
    addrBuf[1] = cmd & 0xFF;      // register address (low byte)
    smbXfer.buf[0].len  = 2;		 // 2 bytes for command
    smbXfer.buf[1].data = pBuf;		 // second buffer to store bytes read
    smbXfer.buf[1].len  = rdCnt;	 // number of bytes to read

    /* Start I2C Transfer */
    SMB_Status = I2C_TransferInit (SMB_I2C_CTRL, &smbXfer);

    /* Check early status */
    if (SMB_Status < 0)
	return SMB_Status;		// return error code

    /* Wait until data is complete or time out */
    uint32_t start = RTC->CNT;
    while (SMB_Status == i2cTransferInProgress)
    {
	/* Enter EM1 while waiting for I2C interrupt */
	EMU_EnterEM1();

	/* check for timeout */
	if (((RTC->CNT - start) & 0x00FFFFFF) > I2C_XFER_TIMEOUT)
	{
	    SMB_Reset();
	    SMB_Status = (I2C_TransferReturn_TypeDef)i2cTransferTimeout;
	}
    }
    
   
     /* Return final status */
    return SMB_Status;
}



/***************************************************************************//**
 *
 * @brief	ADC Configuration
 *
 * This routine configures the ADC to measure the internal VDD/3 voltage,
 * see AN0021 for more information.
 *
 ******************************************************************************/
static void ADC_Config(void)
{
ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;


    /* Enable clock for ADC */
    CMU_ClockEnable(cmuClock_ADC0, true);

    /* Init common settings for both single conversion and scan mode */
    init.timebase = ADC_TimebaseCalc(0);
    /* Might as well finish conversion as quickly as possibly since polling */
    /* for completion. */
    /* Set ADC clock to 7 MHz, use default HFPERCLK */
    init.prescale = ADC_PrescaleCalc(7000000, 0);

    /* WARMUPMODE must be set to Normal according to ref manual before */
    /* entering EM2. In this example, the warmup time is not a big problem */
    /* due to relatively infrequent polling. Leave at default NORMAL, */

    ADC_Init(ADC0, &init);

    /* Init for single conversion use, measure VDD/3 with 1.25 reference. */
    singleInit.reference  = adcRef1V25;
    singleInit.input      = adcSingleInpVDDDiv3;
    singleInit.resolution = adcRes12Bit;

    /* The datasheet specifies a minimum aquisition time when sampling vdd/3 */
    /* 32 cycles should be safe for all ADC clock frequencies */
    singleInit.acqTime = adcAcqTime32;

    ADC_InitSingle(ADC0, &singleInit);
}


/***************************************************************************//**
 *
 * @brief	Read VDD
 *
 * This routine measures the internal VDD/3 channel via ADC0, to obtain the
 * voltage of the local CR3032 supply battery.  The value is converted to
 * milli volts [mV].
 *
 * @return
 *	VDD value in [mV].
 *
 ******************************************************************************/
uint32_t ReadVdd (void)
{
static bool initADC = true;
uint32_t    value;

    if (initADC)
    {
	/* Routine has been called for the first time - initialize ADC */
	ADC_Config();
	initADC = false;
    }

    ADC_Start(ADC0, adcStartSingle);

    /* Wait while conversion is active */
    while (ADC0->STATUS & ADC_STATUS_SINGLEACT) ;

    /* Get ADC result */
    value = ADC_DataSingleGet(ADC0);

    /* Calculate supply voltage relative to 1.25V reference */
    value = (value * 1250 * 3) / 4096;

    return value;
}
