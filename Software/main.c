/***************************************************************************//**
 * @file
 * @brief	HRD Sensor (SHT3X-DIS)
 * @author	Peter Loes
 * @version	2021-05-03
 *
 * This application consists of the following modules:
 * - ExtInt.c - External interrupt handler.
 * - Keys.c - Key interrupt handling and translation.
 * - AlarmClock.c - Alarm clock and timers facility.
 * - clock.c - An implementation of the POSIX time() function.
 * - LCD_DOGM162.c - Driver for the DOGM162 LC-Display.
 * - LEUART.c - Driver for the Low-Energy UART.
 * - Display.c - Display manager for LCD.
 * - SensorMon.c - Sensor monitor, allows to read the state of the
 *   sensor via the SMBus.
 *
 * Parts of the code are based on the example code of AN0006 "tickless calender"
 * from Energy Micro AS.
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
2020-01-13,rage	Calculate the LCD contrast depending on the CR2032 voltage.
		ConsolePrintf() allows formated output to the serial console.
2016-11-22,rage	Added DMA Channel Assignment for LEUART support.
		Changed l_ExtIntCfg for new version of ExtInt module.
		Initialize LEUART.
		Use separate format types for Overcurrent and Highcurrent
		Reaction Times.
2015-10-12,rage	Removed LED initialization, updated documentation.
2015-06-22,rage	Initial version.
*/

/*!
 * @mainpage
 * <b>Description</b><br>
 * HRD_Sensor is an application to display actual values of a Sensor.
 * For that purpose the HRD_Sensor board must be connected to the Sensor
 * via its SMBus.
 *
 * The system consists of the following components:
 *
 * <b>Microcontroller</b><br>
 * The heart of the board is an EFM32G230 microcontroller.  It provides two
 * different clock domains: All low-energy peripheral is clocked via a
 * 32.768kHz external XTAL.  The MCU and other high performance peripheral
 * uses a high-frequency clock.  The board can be configured to use the
 * internal RC-oscillator, or an external 32MHz XTAL for that purpose,
 * see define @ref USE_EXT_32MHZ_CLOCK.
 *
 * <b>Keys (Push Buttons)</b><br>
 * There exist 3 push buttons on the board (description of the keys is left to
 * right).  When asserted, the following action will be taken:
 * - SW1 is the POWER button.  It is used to switch on the device and
 *   additionally selects the first item, i.e. the firmware version and date.
 * - SW3 is the NEXT key.  It leads to the next menu item.
 *   This key has a auto-repeat functionality when kept asserted.
 * - SW2 is the PREV key.  It returns to the previous menu item.
 *   This key has a auto-repeat functionality when kept asserted.
 *
 * The duration how long the respective information is displayed before
 * switching the LCD off again, can be adjusted by the define @ref
 * LCD_POWER_OFF_TIMEOUT.
 *
 * Another timeout define is the @ref POWER_OFF_TIMEOUT.  It specifies the
 * number of seconds after which the whole device is powered off if no
 * key assertion is detected.  @ref POWER_OFF_TIMEOUT should be greater
 * or eqal @ref LCD_POWER_OFF_TIMEOUT.
 *
 * <b>LC-Display</b><br>
 * The display provides 2 lines á 16 characters.  It is connected to the
 * EFM32 microcontroller via a parallel bus interface.  To save power, the
 * whole display usually is switched off and only activated by asserting a
 * push button.  After a power-up or reset, the LCD displays the firmware
 * version until a push button is asserted.
 *
 * The visual contrast of the LCD can be adjusted via @ref l_Contrast.
 *
 * <b>Sensors</b><br>
 * The sensor has its own controller.  It is connected to the EFM32
 * microcontroller via I2C-bus.  The sensor monitor routines  allow to request
 * status information from the different sensors. This module additionally provides
 * function ReadVdd() to read the voltage of the local supply battery.
 *
 * <b>Firmware</b><br>
 * The firmware consists of an initialization part and a main loop, also called
 * service execution loop.  The initialization part sets up all modules, enables
 * devices and interrupts.  The service execution loop handles all tasks that
 * must not be executed in interrupt context.
 *
 * After power-up or reset the following actions are performed:
 * -# Basic initialization of MCU and clocks
 * -# The Hold-Power pin is activated
 * -# Further hardware initialization (Keys, Interrupts, Alarm Clock)
 * -# The LC-Display is activated and the firmware version is shown
 * -# The Sensors Monitor is initialized
 *
 * The program then enters the Service Execution Loop which takes care of:
 * - Power managemnet for the LC-Display
 * - Sensors monitoring
 * - Entering the right energy mode
 */
/*=============================== Header Files ===============================*/

#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_dma.h"
#include "config.h"		// include project configuration parameters
#include "ExtInt.h"
#include "Keys.h"
#include "AlarmClock.h"
#include "Display.h"
#include "LCD_DOGM162.h"
#include "LEUART.h"

/*================================ Global Data ===============================*/

extern char const prjVersion[];
extern char const prjDate[];


/*! @brief Global DMA Control Block.
 *
 * It contains the configuration for all 8 DMA channels which may be used by
 * various peripheral devices, e.g. ADC, DAC, USART, LEUART, I2C, and others.
 * The entries of this array will be set by the initialization routines of the
 * driver, which was assigned to the respective channel.  Unused entries remain
 * zero.  There is a total of 16 entries in the array.  The first 8 are used
 * for the primary DMA structures, the second 8 for alternate DMA structures
 * as used for DMA scatter-gather mode, where one buffer is still available,
 * while the other can be re-configured.  This application uses only the first
 * 8 entries.
 *
 * @see  DMA Channel Assignment
 *
 * @note This array must be aligned to 256!
 */
#if defined (__ICCARM__)
    #pragma data_alignment=256
    DMA_DESCRIPTOR_TypeDef g_DMA_ControlBlock[DMA_CHAN_COUNT * 2];
#elif defined (__CC_ARM)
    DMA_DESCRIPTOR_TypeDef g_DMA_ControlBlock[DMA_CHAN_COUNT * 2] __attribute__ ((aligned(256)));
#elif defined (__GNUC__)
    DMA_DESCRIPTOR_TypeDef g_DMA_ControlBlock[DMA_CHAN_COUNT * 2] __attribute__ ((aligned(256)));
#else
    #error Undefined toolkit, need to define alignment
#endif


/*! @brief Global DMA Callback Structure.
 *
 * This array contains the addresses of the DMA callback functions, which are
 * executed for a dedicated DMA channel at the end of a DMA transfer.
 * The entries of this array will be set by the initialization routines of the
 * driver, which was assigned to the respective channel.  Unused entries remain
 * zero.
 */
DMA_CB_TypeDef g_DMA_Callback[DMA_CHAN_COUNT];

/*! @brief Flag to indicate that an Interrupt occurred in the meantime.
 *
 * This flag must be set <b>true</b> by any interrupt service routine that
 * requires actions in the service execution loop of main().  This prevents
 * the system from entering sleep mode, so the action can be taken before.
 */
volatile bool		g_flgIRQ;

/*! @brief Modules that require EM1.
 *
 * This global variable is a bit mask for all modules that require EM1.
 * Standard peripherals would stop working in EM2 because clocks, etc. are
 * disabled.  Therefore it is required for software modules that make use
 * of such devices, to set the appropriate bit in this mask, as long as they
 * need EM1.  This prevents the power management of this application to enter
 * EM2.  The enumeration @ref EM1_MODULES lists those modules.
 * Low-Power peripherals, e.g. the LEUART still work in EM1.
 *
 * Examples:
 *
   @code
   // Module RFID requires EM1, set bit in bit mask
   Bit(g_EM1_ModuleMask, EM1_MOD_RFID) = 1;
   ...
   // Module RFID is no longer active, clear bit in bit mask
   Bit(g_EM1_ModuleMask, EM1_MOD_RFID) = 0;
   @endcode
 */
volatile uint16_t	g_EM1_ModuleMask;

/*================================ Local Data ================================*/

    /*! EXTI initialization structure
     *
     * Connect the external interrupts of the push buttons to the key handler.
     */
static const EXTI_INIT  l_ExtIntCfg[] =
{   //	IntBitMask,	IntFct
    {	KEY_EXTI_MASK,	KeyHandler	},	// Keys
    {	0,		NULL		}
};

    /*!
     * Initialization structure to define the timings for the autorepeat (AR)
     * threshold and rate (in milliseconds), and a function to be called for
     * each translated key.
     */
static const KEY_INIT  l_KeyInit =
{
    .AR_Threshold = AUTOREPEAT_THRESHOLD,
    .AR_Rate	= AUTOREPEAT_RATE,
    .KeyFct	= DisplayKeyHandler
};

    /*! LCD field definitions
     *
     * This array specifies the location and size of fields on the LC-Display.
     *
     * @warning	Enum @ref LCD_FIELD_ID is used as index within this array,
     * 		therefore care has to be taken to keep it "in sync"!
     */
static const LCD_FIELD l_LCD_Field[LCD_FIELD_ID_CNT] =
{
    /* X,  Y,	Width	*/
    {  0,  0,	16	},	//!< 0: LCD_LINE1_BLANK
    {  0,  1,	16	},	//!< 1: LCD_LINE2_BLANK
    {  0,  0,	16	},	//!< 2: LCD_LINE1_TEXT
    {  0,  1,	16	},	//!< 3: LCD_LINE2_TEXT
    {  0,  0,	16	},	//!< 4: LCD_ITEM_DESC
    {  0,  1,	16	},	//!< 5: LCD_ITEM_ADDR
    {  7,  1,	 9	},	//!< 6: LCD_ITEM_DATA
    {  0,  0,	16	},	//!< 7: LCD_CLOCK (no RTC, just display the uptime)
};

    /*! List of items which can be displayed
     *
     * This array contains a list of items that can be displayed on the LCD.
     * Element 0 is a special case, it shows the name, version, and date of
     * the firmware image.  Usually up/down push buttons are used to select
     * the item to be displayed.
     *
     * The order of these items may be rearranged by the user.  Single or
     * groups of entries can be de-activated by commenting them out.
     */
static const ITEM l_Item[] =
{  // [1234567890123456]    Cmd				Frmt
    { ">>> SHT3X-D <<<",    SBS_NONE,			FRMT_FW_VERSION	},
    { "Sensor at SMBus",    SBS_NONE,           	FRMT_SENS_CTRL	},
    { "Supply Battery",	    SBS_NONE,			FRMT_CR2032_BAT	},
    { "Serial Number",	    SBS_READ_SERIAL_NUMBER,	FRMT_HEX	},
    { "Soft Reset",         SBS_SOFT_RESET,             FRMT_HEX        },
    { "Status",  	    SBS_READ_STATUS,  	        FRMT_HEX	},
    { "Clock Str. dis H",   SBS_REPEATABILITY_CLOCK_HIGH_DISABLED,   FRMT_HEX},
    { "Clock Str. dis M",   SBS_REPEATABILITY_CLOCK_MEDIUM_DISABLED, FRMT_HEX},
    { "Clock Str. dis L",   SBS_REPEATABILITY_CLOCK_LOW_DISABLED,    FRMT_HEX},
    { "Measure mps2 M",     SBS_REPEATABILITY_CLOCK_MPS_2_MEDIUM,    FRMT_HEX},
    { "Data T",             SBS_FETCH_DATA,             FRMT_TEMP       },
    { "Data RH",            SBS_FETCH_DATA,             FRMT_RH         },
    { "Break ",             SBS_BREAK,                  FRMT_HEX        },
    { "Heater enabled",     SBS_HEATER_ENABLE,          FRMT_HEX        },
    { "Heater disabled",    SBS_HEATER_DISABLE,         FRMT_HEX        },
};

#define ITEM_CNT	ELEM_CNT(l_Item)

/*=========================== Forward Declarations ===========================*/

static void cmuSetup(void);


/******************************************************************************
 * @brief  Main function
 *****************************************************************************/
int main( void )
{
    /* Initialize chip - handle erratas */
    CHIP_Init();

    /* Set up clocks */
    cmuSetup();
    
     /* Enable FET to power the device independent from the Power Button */
    PowerUp();

    /* Init Low Energy UART with 9600bd (this is the maximum) */
    drvLEUART_Init (9600);

#ifdef DEBUG
    dbgInit();

    /* Output version string to SWO or LEUART */
    DBG_PUTS("\n***** SHT3X-D V");
    DBG_PUTS(prjVersion);
    DBG_PUTS(" ");
    DBG_PUTS(prjDate);
    DBG_PUTS(" *****\n\n");
#endif

    /*
     * All modules that make use of external interrupts (EXTI) should be
     * initialized before calling ExtIntInit() because this enables the
     * interrupts, so IRQ handler may be executed immediately!
     */

    /* Initialize key hardware */
    KeyInit (&l_KeyInit);

    /*
     * Initialize (and enable) External Interrupts
     */
    ExtIntInit (l_ExtIntCfg);

    /* Initialize the Alarm Clock module */
    AlarmClockInit();

    /* Verify element count */
    EFM_ASSERT(ELEM_CNT(l_LCD_Field) == LCD_FIELD_ID_CNT);

      /*
     * Set Contrast of LC-Display.  This depends on the CR2032 Battery Voltage:
     * 2.9V -> 30, 2.6V -> 45, i.e. 5 digits per 100mV, 1 digit per 20mV
     * Contrast calculation must be done before initializing the LCD!
     */
    LCD_SetContrast((3500 - ReadVdd()) / 20);
    
    /* Initialize display - show firmware version */
    DisplayInit (l_LCD_Field, l_Item, ITEM_CNT);
    LCD_Printf (LCD_LINE1_TEXT, ">>> SHT3X-D <<<");
    LCD_Printf (LCD_LINE2_TEXT, "V%s %s", prjVersion, prjDate);

    /* Initialize Sensor Monitor */
    SensorMonInit();
    
     /* Enable all other External Interrupts */
    ExtIntEnableAll();


    /* ============================================ *
     * ========== Service Execution Loop ========== *
     * ============================================ */
    while (1)
    {
	/* Update or power-off the LC-Display, update measurements */
	DisplayUpdateCheck();

	/*
	 * Check for current power mode:  If a minimum of one active module
	 * requires EM1, i.e. <g_EM1_ModuleMask> is not 0, this will be
	 * entered.  If no one requires EM1 activity, EM2 is entered.
	 */
	if (! g_flgIRQ)		// enter EM only if no IRQ occured
	{
	    if (g_EM1_ModuleMask)
		EMU_EnterEM1();		// EM1 - Sleep Mode
	    else
		EMU_EnterEM2(true);	// EM2 - Deep Sleep Mode
	}
	else
	{
	    g_flgIRQ = false;	// clear flag to enter EM the next time
	}
    }
}


/******************************************************************************
 * @brief   Configure Clocks
 *
 * This local routine is called once from main() to configure all required
 * clocks of the EFM32 device.
 *
 *****************************************************************************/
static void cmuSetup(void)
{
    /* Start LFXO and wait until it is stable */
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

#if USE_EXT_32MHZ_CLOCK
    /* Start HFXO and wait until it is stable */
    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

    /* Select HFXO as clock source for HFCLK */
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

    /* Disable HFRCO */
    CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);
#endif

    /* Route the LFXO clock to the RTC and set the prescaler */
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);	// RTC
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);	// LEUART0/1
    CMU_ClockEnable(cmuClock_RTC, true);

    /* Prescaler of 1 = 30 us of resolution and overflow each 8 min */
    CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_1);

    /* Enable clock to low energy modules */
    CMU_ClockEnable(cmuClock_CORELE, true);

    /* Enable clock for HF peripherals (ADC, DAC, I2C, TIMER, and USART) */
    CMU_ClockEnable(cmuClock_HFPER, true);

    /* Enable clock to GPIO */
    CMU_ClockEnable(cmuClock_GPIO, true);
}

/***************************************************************************//**
 *
 * @brief	Print string to serial console
 *
 * This routine is used to print text to the serial console, i.e. LEUART.
 *
 * @param[in] frmt
 *	Format string of the text to print - same as for printf().
 *
 * @see		drvLEUART_puts()
 *
 ******************************************************************************/
void ConsolePrintf (const char *frmt, ...)
{
char	 buffer[120];
va_list	 args;


    va_start (args, frmt);
    vsprintf (buffer, frmt, args);
    drvLEUART_puts (buffer);
    va_end (args);
}
