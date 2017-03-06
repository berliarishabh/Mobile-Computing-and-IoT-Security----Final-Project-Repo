/************************************************************************************************************
* Description :	Part of the Final Project for the Course Mobile Computing and IoT Security (ECEN5023)		*
* Author	  :	Omkar Reddy Seelam and Rishabh Berlia                                                       *
* Email		  : omkar.reddy15@gmail.com, omkar.seelam@colorado.edu, rishabh.berlia@colorado.edu				*
* Date		  : 4/28/2016																					*
************************************************************************************************************/

/* Features:
 * Sometimes the alerts are too frequent and redundant so, the alerts can be snoozed for certain time  : snoozeAlerts!
 * Fixed issues like multiple LESENSE sensors DMA_Tx overwrites (Used an if loop to check what sensors have been triggered)
 * Negative temperatures issue has been fixed
 *
 * Different modes:
 * ***** Verbose mode : Where it sends lengthy messages such as "The temp is ....."
 * ***** switchToRPi : Where it waits for the command from the RPi whenever requested and responds accordingly
 * ***** !switchToRPi (default mode) : In which the LG sends a custom 9 Byte data packet every 4 seconds. The data packet has following info:
 * 										Y/N : {IntrusionState, DoorState, LightState, SecurityState, Temperature : +/- XX.X}
 * 										The main idea was that the sensors modify the data packet whenever a change in state is observed using interrupts
 * 										and the current data packet at each 4 seconds interval is sent through bluetooth.
 * 						You can switch between switchToRPi modes using the PusbButton0
 * Monitored using:
 * ***** The program has been structured in a way that it provides flexibility to use the LG with a RPi or
 * 		 any BLE application on a SmartPhone or computer.
 * 		 This can be done using the pushbutton0.
 *		 Care has been taken such that the sensor data stream sent makes sense not only for a RPi but also on the applications.
 *		 For this reason the custom packet of 9 bytes mentioned above has been chosen.
 * */

/* Project Requirements:
 * EEPROM Emulation has been implemented to restore the SecurityState during a reset, but it was not consistent. Did not get enough time to fix the issue.
 * */

/* LC SENSE + LIGHT SENSE + TEMPERATURE SENSE
 * 1. LESENSE AND TEMP SENSE LEUART DMA_TX dont overwrite. If there is a LEUART DMA_Tx in progress, it simply skips any new LEUART DMA_Tx requests
 * 2. In this version IF LIGHT SENSE DMA_Tx requests (if LC AND LIGHT are both on), because LC SENSE is being serviced first in the LESENSE IRQ.
 * 		A way has to be implemented to not skip any of the particular LESENSE DMA_Tx reqyests by using multiple combinations of
 * 3. LCSENSE Snooze has been made global to all the LESENSE sensors; All alerts will be snoozed
 * */

/* LC Sensor + ADC Temperature Sensor (from Assignment 5)
	1. multiple DMA writes dont interrupt the previous write;
	2. Simply skips the new write if an existing DMA LEUART Tx is happening; Used DMA_IDLE flag
*/

/* LC SENSOR FINAL:
 * 1. Turns on LED0 when the window is open (when there is no metal)
 * 	  There is LETIMER0 interrupting every 4 seconds; If the LED0 is on, then it is turned off; If there is no metal
 * 	  	detected in the next scan cycle; The Led0 is immediately turned on again.
 * 	  	So, a On/Off of LED0 can be observed when there is no metal nearby.
 * 2. Notifications sent through BLE when there is no metal nearby i.e, when window is open. Notified every scan cycle
 * 3. Notifications can be snoozed for about LCSENSE_SNOOZECOUNT times the LETIMER0 interrupts => (SnoozeCount * LETIMER0 interrupts)
 * 		After the snooze time, the notifications resume if the window is still open;
 * 		Meanwhile the LED0 performs the same behaviour of On/Off during the entire snooze cycle or untill a metal comes nearby i.e, until the window is closed
 * . The Threshold for LC Sensor is hardcoded; Should implement a way in which the input is taken from the user
 * */

// Include Libraries
#include "mbed.h"       // mbed libraries; includes sleep()
#include "em_device.h"
#include "em_chip.h"    // for chip related libraries
#include "sleepmodes.h" // to use blockSleepMode() and unblockSleepMode()
#include "em_letimer.h"	// for LETIMER functions; LETIMER_CompareSet, etc
#include "em_gpio.h"	// For GPIO setup
#include "em_adc.h"		// For ADC
#include "em_dma.h"		// For the DMA
#include "em_leuart.h"	// For the LEUART
#include "string.h"		// For string manipulation functions; string copy, string concatenation
#include "em_lesense.h"	// For LESENSE setup
#include "em_acmp.h"	// ACMP used for sensor measurements (LESENSE)
#include "em_dac.h"		// DAC used for excitation and reference (LESENSE)
#include "em_int.h"		// For Enabling/Disable Interrupts Globally
#include "em_i2c.h"
#include "i2c_api.h"
#include "eeprom_emulation.h"

/* MACROS DECLARATION */
#define BLE_COMPLETELY_OFF					true

// BLE Connection to either SmartHome System (final project) or on the APP (for debug)
#define BLE_VERBOSE							false
#define LIGHTSENSE_TIMER_RESET				true
#define LCSENSE_TIMER_RESET					true
#define LIGHTSENSE_TIMEOUT					false
#define LCSENSE_TIMEOUT						false
#define SWITCHTORPI_BUTTON_PORT				gpioPortB
#define SWITCHTORPI_BUTTON_PIN				9U
#define SWITCHTORPI_BUTTON					SW0					// or PB9
uint32_t smartHome_temperature = 0;								// Temperature part of the packet
uint32_t smartHome_security = 0;								// Sends the security fields
char smartHomeMessage[] = "NNNN+00.0\r\n";						// Y/N : {Proximity, LC, LIGHT, SECURITY, +/- XX.X}
char command_smartHomeMessage[] = "leopardPacket!";
bool switchToRPi = false;										// Doesnt send message every 4 seconds. Sends data only when requested. Can be done by using a timer on RPi
char switchingToRPi[] = "Switching to RPi\r\n";
char switchingBackFromRPi[] = "Switching back from RPi\r\n";
bool LIGHTSENSE_TIMER = LIGHTSENSE_TIMER_RESET;					// defaulted to true
bool LCSENSE_TIMER = LCSENSE_TIMER_RESET;							// defaulted to true
InterruptIn	switchTORPi_button(SW0);

// Common Variables
#define DEBUG_MODE 					0						// Make this value to 0; To enable Serial print and BCP trace (code-correlation)
#define LOWPOWER_USING_SLEEPONEXIT 	0						// Using SLEEPONEXIT; Even Lower Energy program can be achieved
#define INV_LEUART_DATABITS			0						// Invert LEUART Data bits; While not transmitting

// LETIMER Variables
#define ULFRCO_Freq 				1000					// Frequency of ULFRCO is 1KHz
#define SAMPLEPERIOD 				4						// Total Required period in Seconds

// DMA Setup Variables
#define DMA_CHANNEL_ADC 			2						// DMA channel select - ADC0
#define DMA_CHANNEL_LEUART_RX 		0						// DMA channel select - LEUART RX
#define DMA_CHANNEL_LEUART_TX 		1						// DMA channel select - LEUART TX
#define DMA_ADC_PRIORITY 			false					// DMA Channel Priority for ADC
#define DMA_LEUART_TX_PRIORITY 		false					// DMA Channel Priority for	LEUART TX
#define DMA_LEUART_RX_PRIORITY 		false					// DMA Channel Priority for LEUART RX
#define DMA_ADC_ARBITRATE 			dmaArbitrate1			// ADC Arbitrate rate
#define DMA_LEUARTRX_ARBITRATE 		dmaArbitrate1			// LEUART RX Arbitrate rate
#define DMA_LEUARTTX_ARBITRATE 		dmaArbitrate1			// LEUART TX Arbitrate rate
#define LEUART0_BUFF_MAX 			1023					// Buffer size for LEUART0
#define DMA_LEUARTRX_TRANSFERS 		LEUART0_BUFF_MAX 		// DMA Transfers to be done on LEUART0->RXDATA

// ADC Setup Variables
#define ADC_SAMPLESREQUIRED 		200						// The required number of samples
#define ADC_REFERENCE 				adcRef1V25				// Single Conversion Reference for Temperature sensor
#define ADC_INPUTCHANNEL 			adcSingleInpTemp		// Selecting the input channel; Temperature sensor
#define ADC_RESOLUTION 				adcRes12Bit				// Selecting the resolution
#define ADC_ACQTIMECYCLES 			adcAcqTime2				// Calculated Acquisition time
#define ADC_CLOCKFREQ 				857000					// Required ADC Clock Frequency

// LEUART Setup Variables
#define LEUART0_LOOP_BACK			false
#define LEUART0_BAUDRATE 			9600					// LEUART0 BaudRate
#define LEUART0_DATABITS 			leuartDatabits8			// Number of data bits for LEUART0
#define LEUART0_PARITY 				leuartNoParity			// Parity bit enable/disable
#define LEUART0_STOPBITS 			leuartStopbits1			// Number of Stop bits
#define LEUART0_SIGFRAME			'!'						// The SIG Frame for LEUART0

// Temperature Settings

#define TEMP_UPPERLIMIT_DEFAULT 			30						// Temperature Sensor Upper Limit = 30degreesC [LED1]
#define TEMP_LOWERLIMIT_DEFAULT 			15						// Temperature Sensor Upper Limit = 15degreesC [LED0]

#define TEMPERATURE_SIZE					8						// Size of the Temperature reading appended to the transmitted message
#define CHECK_NEGATIVE_TEMPERATURES			false					// Flag to check if negative temp are being printer; subtracts 25 degrees

// GPIO variables
#define LED_PORT 					gpioPortE				// LED pin port
#define LED0_PIN 					2U						// LED0 Pin
#define LED1_PIN 					3U						// LED1 Pin
#define LED_PORT_DRIVEMODE 			gpioDriveModeLowest		// Drive strength for LED0
#define LED_PORT_CONFIG 			gpioModePushPullDrive	// Drive strength for LED1
#define BLUEFRUIT_CTS_PORT			gpioPortD				// Bluefruit CTS Pin connected to PortD of Leopard Gecko
#define BLUEFRUIT_CTS_PIN			15U						// Bluefruit CTS is connected to PortD Pin 15
#define BLE_SWITCH					false
#define BLE_SWITCH_PORT				gpioPortD
#define BLE_SWITCH_PIN				10U
#define PROXIMITY_SENSOR_PORT		gpioPortA
#define PROXIMITY_PIN_VDD			PA14
#define PROXIMITY_PIN_SIG			PA12

// Bluefruit AT Command Variables
#define BLE_CONFIGURATION			false								// Perform BLE Configuration
#define FACTORYRESET				"AT+FACTORYRESET\n"					// Perform Factory Reset before configuring the BLE
#define SWITCHMODE					"+++\n"
#define TX_POWERLEVEL				"AT+BLEPOWERLEVEL=0\n"				// BLE Radio TX power level (dBm); -40,-20,-16,-12,-8,-4,0,4
#define GAP_INTERVAL				"AT+GAPINTERVALS=50,4000,250,50\n"	// (Min ConnInt,Max ConnInt, AdvInt, AdvTimeout)
#define CONNECTION_STATUS			"AT+GAPGETCONN\n"					// Get the connection status
#define HW_MODE_LED					"AT+HWMODELED=disable\n"			// Manipulate HW Mode led; disable(0), mode(1), hwuart(2), bleuart(3), spi(4), manual(5,on/off/toggle)
#define ECHO						"ATE=0\n"							// Enable/Disable Echo
#define SYS_RESET					"ATZ\n"								// System Reset
#define BLE_TIMEOUT					5									// Number of Retries for each command

// LESENSE
#define LESENSE_ACMP_CHANNEL		acmpChannel6	// Doesnt Matter; Any channel can be given because it is controlled by the LESENSE
#define ALERTS_SNOOZECOUNT				10			// LCSENSE Alerts Snooze time is around SnoozeCount*LETIMER_SAMPLEPERIOD
#define LESENSE_CHANNELMASK_6			0x40		// Channal mask for Channel 6 (0x1UL << 6)
#define LESENSE_CHANNELMASK_7			0x80		// Channel mask for Channel 7 (0x1UL << 7)
#define LESENSE_CHANNELMASK_8			0x100		// Channel mask for Channel 8 (0x1UL << 8)
#define LESENSE_CHANNELMASK_11			0x800		// Channel mask for Channel 11(0x1UL << 11)

// LC Sensor Macros
#define LCSENSE_PORT		   		gpioPortC		// LCSENSE Excitation and Measurement Port
#define LCSENSE_PIN			   		7U				// LCSENSE Excitation Measurement Pin
#define LCSENSE_DAC_DATA	   		800				// DAC input for LC Sensor
#define LCSENSE_DAC_CHANNEL	   		1				// DAC Channel for LC Sensor
#define LCSENSE_ACMP_CHANNEL		acmpChannel7	// ACMP Channel for LC Sensor
#define LCSENSE_LESENSE_CHANNEL		7				// LESENSE channel 7 for LC Sensor
#define LCSENSE_LIGHTSENSE_CHANNEL	6				// LESENSE channel 6 for Light Sensor
#define LCSENSE_SCAN_FREQ_Calb		20				// LC Sense Scan Frequency in Hz for Calibration
#define LCSENSE_SCAN_FREQ			1				// LC Sense Scan Frequency in Hz after Calibration
#define ACMP_NEG_REF           		acmpChannelVDD	// Use VDD as ACMP Negative reference
#define ACMP_VDD_SCALE         		0x0D            // Reference for the LC sensor to be close to the DAC voltage This was calibrated using a scope and probing both the LC sensor and the ACMP output
												// From the Software Example
// Calibration Settings
#define LESENSE_MAX_CHANNELS 			16				// LESENSE Channels
#define BUFFER_INDEX_LAST  				15				// Last Buffer Index
#define LCSENSE_MANUAL_THRESHOLD		0x0B			// LCSENSE threshold
#define LCSENSE_THRESHOLD_HYSTERESIS	0x03			// Hysteresis for LC Sense threshold

// LIGHTSENSE Macros
#define LIGHTSENSE_EXCITE_PORT   	gpioPortD
#define LIGHTSENSE_EXCITE_PIN    	6U
#define LIGHTSENSE_SENSOR_PORT   	gpioPortC
#define LIGHTSENSE_SENSOR_PIN    	6U
#define LIGHTSENSE_ACMP_THRESHOLD 	33				// Lesser value means more light needed to turn the LED ON; LED ON => LIGHTS ON
#define LIGHTSENSE_LESENSE_CHANNEL		6

// Capsense
#define CAPSENSE_LESENSE_CHANNEL	8
#define CAPSENSE_PORT				gpioPortC
#define CAPSENSE_PIN				8U

// Proximity Sensor Macros
#define PROXIMITY_PORT				gpioPortA
#define PROXIMITY_PIN				14U
InterruptIn proximitySensorSIG(PROXIMITY_PIN_SIG);

// EEPROM Macros
#define EEPROM_EMULATION			true

/** Default configuration for alternate excitation channel. */
#define LESENSE_LIGHTSENSE_ALTEX_CH_CONF                                       \
{                                                                              \
  true,                   /* Alternate excitation enabled.*/                  \
  lesenseAltExPinIdleDis, /* Alternate excitation pin is low in idle. */ \
  true                    /* Excite only for corresponding channel. */        \
}

/** Default configuration for alternate excitation channel. */
#define LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF                                   \
{                                                                              \
  false,                   /* Alternate excitation enabled.*/                  \
  lesenseAltExPinIdleDis,  /* Alternate excitation pin is disabled in idle. */ \
  false                    /* Excite only for corresponding channel. */        \
}

/** Default configuration for all alternate excitation channels. */
#define LESENSE_LIGHTSENSE_ALTEX_CONF                                          \
{                                                                              \
  lesenseAltExMapALTEX,                                                         \
  {                                                                            \
    LESENSE_LIGHTSENSE_ALTEX_CH_CONF,     /* Alternate excitation channel 0. */\
    LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF, /* Alternate excitation channel 1. */\
    LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF, /* Alternate excitation channel 2. */\
    LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF, /* Alternate excitation channel 3. */\
    LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF, /* Alternate excitation channel 4. */\
    LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF, /* Alternate excitation channel 5. */\
    LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF, /* Alternate excitation channel 6. */\
    LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF  /* Alternate excitation channel 7. */\
  }                                                                            \
}

/* GLOBAL VARIABLES */
// Common Variables
bool DMA_ADC_transferComplete 	= true;				// Flag to check DMA transfer status of ADC samples
bool DMA_IDLE					= true;
// Enable the Security; Turn the LC Sensor, Light Sensor, Proximity Sensor and Gesture Sensor On
bool SECURITY_ON = false;
DMA_CB_TypeDef cb[DMA_CHAN_COUNT];					// Type Definition for the DMA Call back functions
volatile uint16_t ADC_Buffer[ADC_SAMPLESREQUIRED];	// RAM Buffer for ADC DMA transfer
char Rx_Buffer[LEUART0_BUFF_MAX];		// Receive Buffer for LEUART DMA transfers
float temperature = 0;								// Variable for temperature
uint16_t TEMP_LOWERLIMIT = 0x24;
uint16_t TEMP_UPPERLIMIT = 0x25;
//#if DEBUG_MODE
Serial pc(USBTX, USBRX);    						// To enable pc.printf() statements; un-comment this and use pc.printf("Text\n\r") for printing text on the terminal
//#endif

// LETIMER Variables
uint16_t LETIMER_TOP;								// LETIMER Reset value-COMP0

// LESENSE Global Variables
// LC SENSE
bool CHECK_LCSENSOR_COUNT 		= false;		// To check the count when a metal object is placed/removed
bool MANUAL_THRESHOLD 			= true;
bool NOTIFY_ALERTS 				= true;

// Temperature Sensor Alerts
uint32_t leuartif;													// variable for saving LEUART interrupt flags
char below[] 			= "Temperature BELOW set min : ";			// Lower limit transfer string
char below_negative[] 	= "Temperature BELOW set min: -";			// Lower limit transfer string
char above[] 			= "Temperature ABOVE set max : ";			// Upper limit transfer string
char return_temp[] 		= "\r\nTemperature is : ";					// Return Temperature request string
char return_temp_neg[] 	= "\r\nTemperature is: -";					// Return Temperature request string
char cmdError[] 		= "\r\nERROR: Invalid command\r\n";			// Error message string
char command_RetTemp[] 	= "RetTemp!";								// The command for requesting temperature

// Alerts
char command_alertsSnooze[] 	  = "snoozeAlerts!";							// Command to snooze alerts
char alertsSnoozed[]			  = "\r\nSnoozed alerts for 40s\r\n";				// Response that the alerts have been snoozed
char windowOpenLightOn[]		  = "Window Open, Light On\r\n";				// Alerts
char windowOpenLightOnIntrusion[] = "Window Open, Light On and Intrusion\r\n";
char windowOpenIntrusion[]		  = "Window Open, Intrusion\r\n";
char lightOnIntrusion[] 		  = "Light On, Intrusion\r\n";


// LC Sensor Alerts
char windowOpen[] 		= "Window is open\r\n";					// Alerts
char windowClose[] 		= "Window is closed\r\n";
uint8_t snoozeTime 		= ALERTS_SNOOZECOUNT;					// The default snooze count. It is 10 LETIMER0 interrupt cycles

// Light Sensor Alerts
char lightOn[]			= "Light is On\r\n";					// Alerts

// Cap Sensor Alerts
char command_securityOn[]	= "securityOn!";					// Command to turn on security
char command_securityOff[]	= "securityOff!";
char securityOn[]			= "\r\nSecurity turned On\r\n";
char securityOff[]			= "\r\nSecurity turned Off\r\n";

// Proximity
bool INTRUSION_DETECTED = false;
char intrusion[] = "Intrusion Detected\r\n";

// BLE Configuration Variables
char check_1OK[] 	= "1\r\nOK\r\n";				// Different responses from the Bluefruit when in AT mode
char check_0OK[] = "0\r\nOK\r\n";
char check_OK[] = "OK\r\n";
char check_ERROR[] = "ERROR\r\n";

/********************************************************EEPROM EMULATION RELATED SECTION*******************************************************/

/***************************************** RAM_INTERRUPT.C ********************************************************/
/* Find the correct size of the interrupt vector */
#if defined(_EFM32_GECKO_FAMILY)
#define VECTOR_SIZE (16+30)
#elif defined(_EFM32_TINY_FAMILY)
#define VECTOR_SIZE (16+23)
#elif defined(_EFM32_GIANT_FAMILY)
#define VECTOR_SIZE (16+38)
#else
#error "Unknown Vector size"
#endif

/* Align the RAM vector table */
#if defined (__ICCARM__)
#pragma data_alignment=256
uint32_t vectorTable[VECTOR_SIZE];
#elif defined (__CC_ARM)
uint32_t vectorTable[VECTOR_SIZE] __attribute__ ((aligned(256)));
#elif defined (__GNUC__)
uint32_t vectorTable[VECTOR_SIZE] __attribute__ ((aligned(256)));
#else
#error "Undefined toolkit, need to define alignment"
#endif

/* SysTick interrupt handler. Counts ms ticks */
volatile uint32_t msTicks = 0;

/* Place interrupt handler in RAM */
#if defined (__ICCARM__)               			/* IAR compiler */
__ramfunc
#elif defined(__CC_ARM)
 __attribute__ ((section ("ram_code")))
#elif defined(__GNUC__)                  		/* GCC based compilers */
#if defined (__CROSSWORKS_ARM)         			/* Rowley Crossworks */
__attribute__ ((section(".fast")))
#else                           						/* Other GCC */
__attribute__ ((section(".ram")))
#endif
#endif

void SysTick_Handler(void){
  msTicks++;
}

void moveInterruptVectorToRam(void){
  memcpy(vectorTable, (uint32_t*)SCB->VTOR, sizeof(uint32_t) * VECTOR_SIZE);
  SCB->VTOR = (uint32_t)vectorTable;
}

/*****************EEPROM VARIABLES**********************************/
#define EEPROM_PAGES_NUMBER 2

/* Define the non-volatile variables. */
EE_Variable_TypeDef ee_security_on;

uint16_t             readValue;

/***************************************************************************************************************************************/

LETIMER_Init_TypeDef letimerInit = {
	true,                   // Start counting when init completed.
	false,                   // Counter shall not keep running during debug halt.
	false,                   // Don't start counting on RTC COMP0 match.
	false,                   // Don't start counting on RTC COMP1 match.
	true,                   // Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP
	false,                   // Don't load COMP1 into COMP0 when REP0 reaches 0.
	0,                       // Idle value for output 0.
	0,                       // Idle value for output 1.
	letimerUFOANone,         // No output on underflow on output 0
	letimerUFOANone,         // No output on underflow on output 1
	letimerRepeatFree       // Count until stopped
};

/* FUNCTION PROTOTYPES */
// Interrupt Handling Routines
void LETIMER0_IRQHandler(void);	// LETIMER ISR
void LEUART0_IRQHandler(void);	// LEUART ISR
void LESENSE_IRQHandler(void);
void switchTORPi_ISR(void);
void intruderLeftISR(void);
void intrusionDetectedISR(void);

// Peripheral Initialization Routines
float convertToCelsius(int32_t adcSample);
void DMA_CallBack(unsigned int, bool, void *);

void CMU_setup(void);
void LETIMER_setup(void);						// LETIMER setup routine
void GPIO_setup(void);							// GPIO setup routine
void ADC_setup(void);							// ADC Setup routine
void DMA_setup(void);							// DMA Initialization routine
void LEUART_setup(void);						// LEUART setup routine
void ACMP_setup(void);							// ACMP0 for LC and Ambient Light Sensor
void ACMP1_setup(void);							// ACMP1 for Cap sense
void DMA_ADC_Setup(void);						// DMA setup for ADC
void DMA_LEUART_TX_Setup(void);					// DMA setup for LEUART TX
void DMA_LEUART_RX_Setup(void);					// DMA setup for LEUART RX
void BluetoothManipulation(void);
void BLE_CommandTx(char *, char *);				// Transmit the AT command to the Bluefruit
void DAC_setup(void);
void LESENSE_setup(void);
void CapSensor_Calibration(uint8_t);			// Capacitive Sensor calibration
void writeDataDAC(DAC_TypeDef *dac, unsigned int value, unsigned int ch);
void LCSensor_Calibration(uint8_t chIdx);
void switchTORPi_ISR(void);
void EEPROM_Initialization(void);
void EEPROM_ReadSettings(void);
void PROXIMITY_initSetup(void);
void PROXIMITY_on(void);
void PROXIMITY_off(void);
void switchTORPi_setup(void);

// Common Functions
float convertToCelsius(long int);
#if DEBUG_MODE
void BSP_TraceSwoSetup(void);	// code correlation function0
#endif

// Function to convert ADC sample to Celsius
float convertToCelsius(int32_t adcSample){
	float temp;
	/* Factory calibration temperature from device information page. */
	float cal_temp_0 = (float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK) >> _DEVINFO_CAL_TEMP_SHIFT);
	float cal_value_0 = (float)((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK) >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);
	/* Temperature gradient (from datasheet) */
	float t_grad = -6.27;
	temp = (cal_temp_0 - ((cal_value_0 - adcSample) / t_grad));
	return temp;
}

// DMA_CallBack function
void DMA_CallBack(unsigned int channel, bool primary, void *user){
	// DMA ADC transfers Callback
	if (!DMA_ADC_transferComplete){
		int temp = 0, i = 0;											// Temporary variables; for calculating the average and for index in the for loop
		float temp_temp = 0;
		char temperatureAppend[TEMPERATURE_SIZE];						// Variable used to store the temperature in characters
		char temp_string[strlen(above) + strlen(temperatureAppend)];	// String variable for storing the final string to be transferred

		(void) channel;
		(void) primary;
		(void) user;

		ADC_Reset(ADC0);		// Reset the ADC; Turn it off

		// Getting the average temperature
		for (i = 0; i < ADC_SAMPLESREQUIRED; i++)
			temp += (ADC_Buffer[i]);					// Get the sum of the 200 samples
		temperature = temp / ADC_SAMPLESREQUIRED;		// Get the average of 200 samples
		temperature = convertToCelsius(temperature);	// Get the Temperature in Celsius

		#if CHECK_NEGATIVE_TEMPERATURES
		temperature -= 27;
		#endif

		if (temperature > TEMP_UPPERLIMIT){				// If temperature is above the upper limit
			temp 				 = temperature*10;						// If temperature is 32.11435, this would give temp = 321;
			temperatureAppend[0] = (temp/100)+48;					// temperatureAppend[0] = '3'
			temp 				 = temp%100;							// temp = 21
			temperatureAppend[1] = (temp/10)+48;					// temperatureAppend[1] = '2'
			temp 				 = temp%10;								// temp = 1
			temperatureAppend[2] = '.';								// temperatureAppend[2] = '.'
			temperatureAppend[3] = (temp)+48;						// temperatureAppend[3] = '1'
			temperatureAppend[4] = 'C';								// temperatureAppend[4] = 'C'
			temperatureAppend[5] = '\r';
			temperatureAppend[6] = '\n';
			temperatureAppend[7] = '\0';

			strcpy(temp_string,above);					// Copy the contents of above and concatenate it with the contents in c1.
			strcat(temp_string,temperatureAppend);

			if (BLE_VERBOSE){
				if (NOTIFY_ALERTS){
					if (DMA_IDLE){
					LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;				// Enable DMA wake up for LEUART TX in EM2
					DMA_IDLE = false;
					// Activate DMA transfers for LEUART TX
					DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)temp_string, strlen(temp_string) - 1);
					}
				}
			}
			else {
				smartHomeMessage[8] = temperatureAppend[3];
				smartHomeMessage[7] = temperatureAppend[2];
				smartHomeMessage[6] = temperatureAppend[1];
				smartHomeMessage[5] = temperatureAppend[0];
				smartHomeMessage[4] = '+';
			}
		}
		else if ((temperature >= 0) && (temperature < TEMP_LOWERLIMIT)){		// If temperature is above the lower limit
			temp 				 = temperature*10;						// If temperature is 32.11435, this would give temp = 321;
			temperatureAppend[0] = (temp/100)+48;					// temperatureAppend[0] = '3'
			temp 				 = temp%100;							// temp = 21
			temperatureAppend[1] = (temp/10)+48;					// temperatureAppend[1] = '2'
			temp 				 = temp%10;								// temp = 1
			temperatureAppend[2] = '.';								// temperatureAppend[2] = '.'
			temperatureAppend[3] = (temp)+48;						// temperatureAppend[3] = '1'
			temperatureAppend[4] = 'C';								// temperatureAppend[4] = 'C'
			temperatureAppend[5] = '\r';
			temperatureAppend[6] = '\n';
			temperatureAppend[7] = '\0';

			strcpy(temp_string,below);					// Copy the contents of above and concatenate it with the contents in c1.
			strcat(temp_string,temperatureAppend);

			if (BLE_VERBOSE){
				if (NOTIFY_ALERTS){
					if (DMA_IDLE){
					LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;				// Enable DMA wake up for LEUART TX in EM2
					DMA_IDLE = false;
					// Activate DMA transfers for LEUART TX
					DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)temp_string, strlen(temp_string) - 1);
					}
				}
			}
			else {
				smartHomeMessage[8] = temperatureAppend[3];
				smartHomeMessage[7] = temperatureAppend[2];
				smartHomeMessage[6] = temperatureAppend[1];
				smartHomeMessage[5] = temperatureAppend[0];
				smartHomeMessage[4] = '+';
			}
		}
		else if (temperature < 0){
			temp_temp 			 = temperature * -1;
			temp 				 = temp_temp*10;						// If temperature is 32.11435, this would give temp = 321;
			temperatureAppend[0] = (temp/100)+48;					// temperatureAppend[0] = '3'
			temp 				 = temp%100;							// temp = 21
			temperatureAppend[1] = (temp/10)+48;					// temperatureAppend[1] = '2'
			temp 				 = temp%10;								// temp = 1
			temperatureAppend[2] = '.';								// temperatureAppend[2] = '.'
			temperatureAppend[3] = (temp)+48;						// temperatureAppend[3] = '1'
			temperatureAppend[4] = 'C';								// temperatureAppend[4] = 'C'
			temperatureAppend[5] = '\r';
			temperatureAppend[6] = '\n';
			temperatureAppend[7] = '\0';

			strcpy(temp_string,below_negative);					// Copy the contents of above and concatenate it with the contents in c1.
			strcat(temp_string,temperatureAppend);

			if (BLE_VERBOSE){
				if (NOTIFY_ALERTS){
					if (DMA_IDLE){
					LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;				// Enable DMA wake up for LEUART TX in EM2
					DMA_IDLE = false;
					// Activate DMA transfers for LEUART TX
					DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)temp_string, strlen(temp_string) - 1);
					}
				}
			}
			else {
				smartHomeMessage[8] = temperatureAppend[3];
				smartHomeMessage[7] = temperatureAppend[2];
				smartHomeMessage[6] = temperatureAppend[1];
				smartHomeMessage[5] = temperatureAppend[0];
				smartHomeMessage[4] = '-';
			}
		}

		if (!switchToRPi){
			if (!BLE_VERBOSE) {
				if (temperature >= 0){
					smartHomeMessage[4] = '+';

					temp 				 = temperature*10;
					smartHomeMessage[5] = (temp/100)+48;
					temp 				 = temp%100;
					smartHomeMessage[6] = (temp/10)+48;
					temp 				 = temp%10;
					smartHomeMessage[7] = '.';
					smartHomeMessage[8] = (temp)+48;
				}
				else if (temperature < 0){
					smartHomeMessage[4] = '-';

					temp_temp 			 = temperature * -1;
					temp 				 = temp_temp*10;
					smartHomeMessage[5] = (temp/100)+48;
					temp 				 = temp%100;
					smartHomeMessage[6] = (temp/10)+48;
					temp 				 = temp%10;
					smartHomeMessage[7] = '.';
					smartHomeMessage[8] = (temp)+48;

				}

				if (NOTIFY_ALERTS){
					if (DMA_IDLE){
						LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;				// Enable DMA wake up for LEUART TX in EM2
						DMA_IDLE = false;
						// Activate DMA transfers for LEUART TX
						DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)smartHomeMessage, strlen(smartHomeMessage) - 1);
					}
				}
			}
		}

		DMA_ADC_transferComplete = true;
		unblockSleepMode(EM1);	// Unblock EM1
		blockSleepMode(EM2);	// To Enter EM2
	}
	// DMA TX Call Back
	else{
		(void) channel;
		(void) primary;
		(void) user;

		// Disable DMA wake-up from LEUART1 TX
		LEUART0->CTRL &= ~LEUART_CTRL_TXDMAWU;

		DMA_IDLE = true;
	}
}

// LETIMER0 ISR
void LETIMER0_IRQHandler(void){
	// Clear the Interrupt
	LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);	// Clearing both underflow and comp1 match interrupts

	if (LIGHTSENSE_TIMER == LIGHTSENSE_TIMEOUT){
		GPIO_PinOutClear(LED_PORT, LED1_PIN);		// Turn LED off; LIGHT SENSE
		if (!BLE_VERBOSE) {
			smartHomeMessage[2] = 'N';		// Yes for Light Sensor
		}
	}
	if (LCSENSE_TIMER == LCSENSE_TIMEOUT){
		GPIO_PinOutClear(LED_PORT, LED0_PIN);		// Turn LED off; LC SENSE
		if (!BLE_VERBOSE) {
			smartHomeMessage[1] = 'N';		// Yes for LC Sensor
		}
	}
	LIGHTSENSE_TIMER 	= LIGHTSENSE_TIMEOUT;
	LCSENSE_TIMER 		= LCSENSE_TIMEOUT;

	if (!NOTIFY_ALERTS){							// The timer for snoozing Alerts
		snoozeTime--;
		if (!snoozeTime){
			NOTIFY_ALERTS = true;
			snoozeTime = ALERTS_SNOOZECOUNT;
		}
	}

	unblockSleepMode(EM2);						// Unblock EM3 to enter EM1 during DMA Transfers
	blockSleepMode(EM1);						// To be able to go to EM1
	DMA_ActivateBasic(DMA_CHANNEL_ADC, true, false, (void *)ADC_Buffer, (void *)&(ADC0->SINGLEDATA), ADC_SAMPLESREQUIRED - 1);	// DMA Reset
	ADC_setup();								// ADC Setup
	ADC_Start(ADC0, adcStartSingle);			// ADC Start
	DMA_ADC_transferComplete = false;
}

void LESENSE_IRQHandler(void){
	//uint32_t sensorStatus;
	// Clear the interrupt
	LESENSE_IntClear(LESENSE_IF_CH8 | LESENSE_IF_CH7 | LESENSE_IF_CH6);				// Raised by LC Sense

	if ((LESENSE_ScanResultGet() >= (LESENSE_CHANNELMASK_8)) && (LESENSE_ScanResultGet() <= LESENSE_CHANNELMASK_11)){
		//GPIO_PinOutSet(LED_PORT, LED1_PIN);			// Turn on LED1 for LIGHT SENSE
		SECURITY_ON = !SECURITY_ON;
		if (!SECURITY_ON){
			GPIO_PinOutClear(LED_PORT, LED0_PIN);	// When Security Off requested; Turn Off LEDs immediately
			GPIO_PinOutClear(LED_PORT, LED1_PIN);
			smartHomeMessage[3] = 'N';		// Security Off
			// Turn off LC and Light sense channels
			/*LESENSE_ScanStop();
			LESENSE_ChannelEnable(LIGHTSENSE_LESENSE_CHANNEL, false, false);
			LESENSE_ChannelEnable(LCSENSE_LESENSE_CHANNEL, false, false);*/
			//PROXIMITY_off();
			// Try completely disabling
			GPIO_PinModeSet(PROXIMITY_PORT, PROXIMITY_PIN, gpioModeDisabled, 0);
			//GPIO_PinOutClear(PROXIMITY_SENSOR_PORT, PROXIMITY_PIN);
			/*INT_Disable();
			MSC_Init();
			EE_Write(&ee_security_on, 0);
			INT_Enable();*/
		}
		else {
			smartHomeMessage[3] = 'Y';
			/*LESENSE_ScanStart();
			LESENSE_ChannelEnable(LIGHTSENSE_LESENSE_CHANNEL, true, false);
			LESENSE_ChannelEnable(LCSENSE_LESENSE_CHANNEL, true, true);*/
			//PROXIMITY_on();
			GPIO_PinModeSet(PROXIMITY_PORT, PROXIMITY_PIN, gpioModePushPullDrive, 0);
			GPIO_PinOutSet(PROXIMITY_PORT, PROXIMITY_PIN);
			/*INT_Disable();
			MSC_Init();
			EE_Write(&ee_security_on, 1);
			INT_Enable();*/
		}
	}
	if (SECURITY_ON){
		// If LC SENSE and LIGHT SENSE are triggered
		if (LESENSE_ScanResultGet() == (LESENSE_CHANNELMASK_6 | LESENSE_CHANNELMASK_7)){
			GPIO_PinOutSet(LED_PORT, LED0_PIN);			// Turn on LED0 for LC SENSE
			GPIO_PinOutSet(LED_PORT, LED1_PIN);			// Turn on LED1 for LIGHT SENSE
			LCSENSE_TIMER 		= LCSENSE_TIMER_RESET;		// Reset the timer; If the timer state is reset, then it means there has been another detection
			LIGHTSENSE_TIMER 	= LIGHTSENSE_TIMER_RESET;
			if (NOTIFY_ALERTS){
				if (BLE_VERBOSE){
					if (DMA_IDLE){
						LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;
						DMA_IDLE = false;
						DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)windowOpenLightOn, strlen(windowOpenLightOn) - 1);	// Because there is a null character included in both the strings
					}
				}
				else {
					smartHomeMessage[2] = 'Y';		// Yes for Light Sensor
					smartHomeMessage[1] = 'Y';		// Yes for LC Sensor
				}
			}
		}
		// If only LC SENSE is triggered
		else if (LESENSE_ScanResultGet() == (LESENSE_CHANNELMASK_7)){
			GPIO_PinOutSet(LED_PORT, LED0_PIN);			// Turn on LED0 for LC SENSE
			LCSENSE_TIMER = LCSENSE_TIMER_RESET;
			if (NOTIFY_ALERTS){
				if (BLE_VERBOSE){
					if (DMA_IDLE){
						LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;
						DMA_IDLE = false;
						DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)windowOpen, strlen(windowOpen) - 1);	// Because there is a null character included in both the strings
					}
				}
				else {
					smartHomeMessage[2] = 'N';		// Yes for Light Sensor
					smartHomeMessage[1] = 'Y';		// Yes for LC Sensor
				}
			}
		}
		// If only LIGHT SENSE is triggered
		else if (LESENSE_ScanResultGet() == (LESENSE_CHANNELMASK_6)){
			GPIO_PinOutSet(LED_PORT, LED1_PIN);			// Turn on LED1 for LIGHT SENSE
			LIGHTSENSE_TIMER = LIGHTSENSE_TIMER_RESET;
			if (NOTIFY_ALERTS){
				if (BLE_VERBOSE){
					if (DMA_IDLE){
						LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;
						DMA_IDLE = false;
						DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)lightOn, strlen(lightOn) - 1);	// Because there is a null character included in both the strings
					}
				}
				else {
					smartHomeMessage[2] = 'Y';		// Yes for Light Sensor
					smartHomeMessage[1] = 'N';		// Yes for LC Sensor
				}
			}
		}
	}
}

// LEUART0 ISR
void LEUART0_IRQHandler(){
	// Store and reset pending interrupts
	leuartif = LEUART_IntGet(LEUART0);
	LEUART_IntClear(LEUART0, leuartif);

	if (leuartif & LEUART_IF_SIGF){
		int temp = 0, i = 0;
		float temp_temp = 0;
		char temperatureAppend[TEMPERATURE_SIZE];						// Variable used to store the temperature in characters
		char temp_string[strlen(return_temp) + strlen(temperatureAppend)];	// String variable for storing the final string to be transferred
		char c[strlen(command_RetTemp)];						// To save the Received command

		// Stop the LEUART reception and DMA Transfers
		for (i = 0; i < ((sizeof(command_RetTemp)/sizeof(char)) - 1); i++){
			c[i] = Rx_Buffer[i];
		}
		c[(sizeof(command_RetTemp)/sizeof(char)) - 1] = '\0';

		if (!strcmp(command_RetTemp,c)){
			// If valid Command Received; Initiate DMA transfer of the temperature value; Else Initiate an error message transfer
			if (temperature < 0){
				temp_temp = temperature * -1;
			}
			else temp_temp = temperature;
			temp 				 = temp_temp*10;						// If temperature is 32.11435, this would give temp = 321;
			temperatureAppend[0] = (temp/100)+48;					// temperatureAppend[0] = '3'
			temp 				 = temp%100;							// temp = 21
			temperatureAppend[1] = (temp/10)+48;					// temperatureAppend[1] = '2'
			temp 				 = temp%10;								// temp = 1
			temperatureAppend[2] = '.';								// temperatureAppend[2] = '.'
			temperatureAppend[3] = (temp)+48;						// temperatureAppend[3] = '1'
			temperatureAppend[4] = 'C';								// temperatureAppend[4] = 'C'
			temperatureAppend[5] = '\r';
			temperatureAppend[6] = '\n';
			temperatureAppend[7] = '\0';

			if (temperature < 0)
				strcpy(temp_string,return_temp_neg);				// Copy the contents of above and concatenate
			else strcpy(temp_string,return_temp);					// Copy the contents of above and concatenate
			strcat(temp_string,temperatureAppend);

			if (BLE_VERBOSE){
				if (DMA_IDLE){
					LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;				// Enable DMA wake up for LEUART TX in EM2
					DMA_IDLE = false;
					DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)temp_string, strlen(temp_string) - 1);
				}
			}
			memset(Rx_Buffer,'\0',sizeof(Rx_Buffer));
		}
		else if (!strcmp(command_alertsSnooze,Rx_Buffer)){
			NOTIFY_ALERTS = false;
			memset(Rx_Buffer,'\0',sizeof(Rx_Buffer));
			//if (BLE_VERBOSE){
			if (DMA_IDLE){
				LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;				// Enable DMA wake up for LEUART TX in EM2
				DMA_IDLE = false;
				DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)alertsSnoozed, strlen(alertsSnoozed) - 1);
			}
			//}
		}
		else if (!strcmp(command_securityOn,Rx_Buffer)){
			memset(Rx_Buffer,'\0',sizeof(Rx_Buffer));
			SECURITY_ON = true;
			//if (BLE_VERBOSE){
			if (DMA_IDLE){
				LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;				// Enable DMA wake up for LEUART TX in EM2
				DMA_IDLE = false;
				DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)securityOn, strlen(securityOn) - 1);
			}
			//}
			if (!BLE_VERBOSE) {
				smartHomeMessage[3] = 'Y';
			}
		}
		else if (!strcmp(command_securityOff,Rx_Buffer)){			// To turn the security off
			memset(Rx_Buffer,'\0',sizeof(Rx_Buffer));
			SECURITY_ON = false;
			//if (BLE_VERBOSE){
				if (DMA_IDLE){
					LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;				// Enable DMA wake up for LEUART TX in EM2
					DMA_IDLE = false;
					DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)securityOff, strlen(securityOff) - 1);
				}
			//}
			if (!BLE_VERBOSE){
				smartHomeMessage[3] = 'N';
			}
		}
		else if (!strcmp(command_smartHomeMessage,Rx_Buffer)){		// To reqeust the custom 9 byte data packet
			memset(Rx_Buffer,'\0',sizeof(Rx_Buffer));
			SECURITY_ON = false;
			//if (BLE_VERBOSE){
				if (DMA_IDLE){
					LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;				// Enable DMA wake up for LEUART TX in EM2
					DMA_IDLE = false;
					DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)smartHomeMessage, strlen(smartHomeMessage) - 1);
				}
			//}
			if (!BLE_VERBOSE){
				smartHomeMessage[3] = 'N';
			}
		}
		else {
			//if (BLE_VERBOSE){
				if (DMA_IDLE){
					LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;				// Enable DMA wake up for LEUART TX in EM2
					DMA_IDLE = false;
					// Activate DMA for LEUART TX transfers
					DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)cmdError, strlen(cmdError) - 1);
				}
			//}
			memset(Rx_Buffer,'\0',sizeof(Rx_Buffer));
		}
		LEUART_IntEnable(LEUART0, LEUART_IF_RXDATAV);
		DMA_ActivateBasic(DMA_CHANNEL_LEUART_RX, true, false, NULL, NULL, DMA_LEUARTRX_TRANSFERS-1);
	}
	else if (leuartif & LEUART_IF_RXDATAV){
		memset(Rx_Buffer,'\0',sizeof(Rx_Buffer));
		LEUART_IntDisable(LEUART0, LEUART_IF_RXDATAV);
	}
}

// LETIMER Setup routine
void LETIMER_setup(void){
	// Get the required values for COMP0
	LETIMER_TOP = SAMPLEPERIOD * ULFRCO_Freq;

	// Initial Values compare values for COMP0
	LETIMER_CompareSet(LETIMER0, 0, LETIMER_TOP);		  // COMP0 is used as Timer TOP value
	LETIMER_Init(LETIMER0, &letimerInit);			// Initialize LETIMER

	// Enable Interrupts
    LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);						// Enable underflow interrupt
    NVIC_EnableIRQ(LETIMER0_IRQn);						// Enable LETIMER0 interrupt vector in NVIC
}


/* ***************** Code for code correlation starts here *************************/
#if DEBUG_MODE
// Source: d2l upload: https://learn.colorado.edu/d2l/le/content/156926/viewContent/2334709/View
/*************************************************************************
* @brief Configure trace output for energyAware Profiler
* @note  Enabling trace will add 80uA current for the EFM32_Gxxx_STK.
*        DK's needs to be initialized with SPI-mode:
* @verbatim BSP_Init(BSP_INIT_DK_SPI); @endverbatim
****************************************************************************/
void BSP_TraceSwoSetup(void){
  /* Enable GPIO clock */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;

  /* Enable Serial wire output pin */
  GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;

  /* Set correct location */
  /* This location is valid for GG, LG and WG! */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC0;

  /* Enable output on correct pin. */
  /* This pin is valid for GG, LG and WG! */
  GPIO->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);
  GPIO->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;

  /* Enable debug clock AUXHFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

  /* Wait until clock is ready */
  while (!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY)) ;

  /* Enable trace in core debug */
  CoreDebug->DHCSR |= CoreDebug_DHCSR_C_DEBUGEN_Msk;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Enable PC and IRQ sampling output */
  DWT->CTRL = 0x400113FF;

  /* Set TPIU prescaler to 16. */
  TPI->ACPR = 15;

  /* Set protocol to NRZ */
  TPI->SPPR = 2;

  /* Disable continuous formatting */
  TPI->FFCR = 0x100;

  /* Unlock ITM and output data */
  ITM->LAR = 0xC5ACCE55;
  ITM->TCR = 0x10009;

  /* ITM Channel 0 is used for UART output */
  ITM->TER |= (1UL << 0);
}
#endif
/***************** Code for code correlation ends here *************************/

// Function to Enable necessary oscillators and clocks
void CMU_setup(void){
	// Enable Oscillators
	CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);	// For LETIMER0; ULFRCO is always ON and cannot be turned off; It is sourcing the WatchDog timer.
	CMU_OscillatorEnable(cmuOsc_AUXHFRCO, true, true);	// Enabling AUXHFRCO for LESENSE

	// Enabling the Low Frequency Clock paths
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);	// Connect LFA clock source to ULFRCO; For LETIMER
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);	// Connect LFB clock source to ULFRCO; For LEUART // Needs a stable clock for correct baud; for flawless transmission

	// Making sure that the HF is 14MHz and not 48MHz
	/*CMU_HFRCOBandSet(cmuHFRCOBand_14MHz);
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
	CMU_OscillatorEnable(cmuOsc_HFXO, false, false);  	// Turn-off the HFXO crystal*/

	CMU_ClockEnable(cmuClock_HFPER, true);				// Enable High periph clock?
	CMU_ClockEnable(cmuClock_CORELE, true);     		// Enable CORELE clock?

	// Clocks for various peripherals
	CMU_ClockEnable(cmuClock_LETIMER0, true);			// Give clock source to LETIMER0
	CMU_ClockEnable(cmuClock_ADC0, true);				// Only ADC0
	CMU_ClockEnable(cmuClock_DMA, true);
	CMU_ClockEnable(cmuClock_LEUART0, true);    		// Enable LEUART1 clock
	CMU_ClockEnable(cmuClock_LESENSE, true);			// Enable clock for LESENSE
	CMU_ClockEnable(cmuClock_ACMP0, true);				// Enable clock only for ACMP0
	CMU_ClockEnable(cmuClock_ACMP1, true);				// Enable clock for ACMP1; For Cap sensor
	CMU_ClockEnable(cmuClock_DAC0, true);				// Enable clock only for DAC0
}

// GPIO Setup Routine
void GPIO_setup(void){
	// Reduce the drive strength of the LEDs
	GPIO_DriveModeSet(LED_PORT, LED_PORT_DRIVEMODE);					// Configure PE with lowest drive current (0.5A) for the LEDs // Port E
	GPIO_DriveModeSet(LIGHTSENSE_EXCITE_PORT, gpioDriveModeLow);	// LIGHTSENSE Excite Pin			// Port D
	GPIO_DriveModeSet(LIGHTSENSE_SENSOR_PORT, gpioDriveModeLow);	// LIGHTSENSE Sensor/Measure Pin
	GPIO_DriveModeSet(gpioPortC, gpioDriveModeHigh);	// LIGHTSENSE Sensor/Measure Pin
	//GPIO_DriveModeSet(PROXIMITY_SENSOR_PORT, gpioDriveModeHigh);											// Port A

	// Setting Pin mode
	GPIO_PinModeSet(LED_PORT, LED0_PIN, LED_PORT_CONFIG, 0);	// Making the pin PE2-LED1 as outputs;
	GPIO_PinModeSet(LED_PORT, LED1_PIN, LED_PORT_CONFIG, 0);	// Making the pin PE3-LED1 as outputs;
	GPIO_PinModeSet(LIGHTSENSE_EXCITE_PORT, LIGHTSENSE_EXCITE_PIN, gpioModePushPull, 0);	// Initialize
	GPIO_PinModeSet(LIGHTSENSE_SENSOR_PORT, LIGHTSENSE_SENSOR_PIN, gpioModeDisabled, 0);
	GPIO_PinModeSet(LCSENSE_PORT, LCSENSE_PIN, gpioModePushPull, 0);	// LC Sense Excite Pin
	GPIO_PinModeSet(BLE_SWITCH_PORT, BLE_SWITCH_PIN, gpioModePushPull, 0);	// BLE Switch
    GPIO_PinModeSet(CAPSENSE_PORT, CAPSENSE_PIN, gpioModeDisabled, 0);
}

// ADC Setup Routine
void ADC_setup(void){
	// Initializing with default vaules
	ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

	// ADC Parameters
	init.timebase = ADC_TimebaseCalc(0);				// Get the Timebase value
	init.prescale = ADC_PrescaleCalc(ADC_CLOCKFREQ, 0);	// Get the required prescaler value
	//init.warmUpMode = adcWarmupFastBG;				// Set to FastBG

	// Initializing ADC
	ADC_Init(ADC0, &init);			// Initialize the ADC

	// Single Conversion setup
	// Single Conversion with Repeat ON
	singleInit.reference  = ADC_REFERENCE;		// Give the reference
	singleInit.input      = ADC_INPUTCHANNEL;	// Select ADC Input channel
	singleInit.resolution = ADC_RESOLUTION;		// Select Resolution
	singleInit.acqTime = ADC_ACQTIMECYCLES;		// Select the required acquisition time
	singleInit.rep = true;						// Repeat mode on

	ADC_InitSingle(ADC0, &singleInit);			// ADC single setup on
}

// LEUART Setup Routine
void LEUART_setup(void)
{
	/* Defining the LEUART1 initialization data */
	LEUART_Init_TypeDef leuart0Init;
	leuart0Init.enable   = leuartEnable;        // Activate Rx and Tx
	leuart0Init.refFreq  = 0;                   // Inherit the clock frequency from the LEUART clock source
	leuart0Init.baudrate = LEUART0_BAUDRATE;    // Baudrate = 9600 bps
	leuart0Init.databits = LEUART0_DATABITS;    // Each LEUART frame contains 8 databits
	leuart0Init.parity   = LEUART0_PARITY;      // No parity bits in use
	leuart0Init.stopbits = LEUART0_STOPBITS;    // Setting the number of stop bits in a frame to 2 bitperiods

	LEUART_Init(LEUART0, &leuart0Init);

	// Route LEUART1 TX,RX pin to DMA location 0
	LEUART0->ROUTE = LEUART_ROUTE_TXPEN | LEUART_ROUTE_RXPEN | LEUART_ROUTE_LOCATION_LOC0;

	// Enable GPIO for LEUART1. TX is on D4
	GPIO_PinModeSet(gpioPortD, 4, gpioModePushPull, 0);
	// Enable GPIO for LEUART1. RX is on D5
	GPIO_PinModeSet(gpioPortD, 5, gpioModeInputPull, 0);

	#if LEUART0_LOOP_BACK
	LEUART0->CTRL |= LEUART_CTRL_LOOPBK;
	#endif

	LEUART0->CTRL |= LEUART_CTRL_RXDMAWU;				// Enable DMA wake up for LEUART RX in EM2
	// Start Reception of commands
	DMA_ActivateBasic(DMA_CHANNEL_LEUART_RX, true, false, (void *)Rx_Buffer, (void *)&(LEUART0->RXDATA), DMA_LEUARTRX_TRANSFERS-1);

	// Set the Signal frame
	LEUART0->SIGFRAME = LEUART0_SIGFRAME;							// Set LEUART signal frame

    // Interrupts for LEUART0
    LEUART_IntEnable(LEUART0, LEUART_IF_RXDATAV | LEUART_IF_SIGF);	// Enable Signal frame and Receive valid data interrupt
    NVIC_EnableIRQ(LEUART0_IRQn);						// Enable LEUART interrupt vector in NVIC
}

// DMA Setup Routine
void DMA_setup(void){
	// Initializing with default values
	DMA_Init_TypeDef        dmaInit;

	// Initializing the DMA
	dmaInit.hprot        = 0;
	dmaInit.controlBlock = dmaControlBlock;
	DMA_Init(&dmaInit);
}

// Routine for DMA setup for ADC
void DMA_ADC_Setup(){
	DMA_CfgChannel_TypeDef  chnlCfg;		// DMA Channel configuration for ADC Channel
	DMA_CfgDescr_TypeDef    descrCfg;		// DMA Channel Descriptor for ADC Channel

	// Setting up call-back function
	cb[DMA_CHANNEL_ADC].cbFunc  = DMA_CallBack;		// CallBack function for DMA's ADC Channel
	cb[DMA_CHANNEL_ADC].userPtr = NULL;

	// Setting up channel
	chnlCfg.highPri   = DMA_ADC_PRIORITY;						// Set the priority for the ADC Channel
	chnlCfg.enableInt = true;						// Enabling Call-back interrupt
	chnlCfg.select    = DMAREQ_ADC0_SINGLE;			// Selecting Channel
	chnlCfg.cb       = &(cb[DMA_CHANNEL_ADC]);		// Specify call-back function pointer
	DMA_CfgChannel(DMA_CHANNEL_ADC, &chnlCfg);

	// Setting up channel descriptor
	descrCfg.dstInc  = dmaDataInc2;					// DMA Destination address pointer incrementer
	descrCfg.srcInc  = dmaDataIncNone;				// DMA Source address pointer incrementer
	descrCfg.size    = dmaDataSize2;				// DMA transfer block size
	descrCfg.arbRate = DMA_ADC_ARBITRATE;			// DMA Channel Arbitrate rate
	descrCfg.hprot   = 0;
	DMA_CfgDescr(DMA_CHANNEL_ADC, true, &descrCfg);	// Assigning the Channel Descriptor

	// Starting transfer. Using Basic mode since every transfer must be initiated by the ADC.
	DMA_ActivateBasic(DMA_CHANNEL_ADC, true, false, (void *)ADC_Buffer, (void *)&(ADC0->SINGLEDATA), ADC_SAMPLESREQUIRED - 1);
}

// Routine for DMA setup for LEUART RX
void DMA_LEUART_RX_Setup(){
	DMA_CfgChannel_TypeDef  chnlCfg;		// DMA Channel configuration for LEUART RX Channel
	DMA_CfgDescr_TypeDef    descrCfg;		// DMA Channel descriptor for LEUART RX Channel

	// Setting up call-back function
	cb[DMA_CHANNEL_LEUART_RX].cbFunc  = DMA_CallBack/*DMA_LEUART_CallBack*/;	// CallBack function for DMA's LEUART-RX Channel
	cb[DMA_CHANNEL_LEUART_RX].userPtr = NULL;

	// Setting up channel
	chnlCfg.highPri   = DMA_LEUART_RX_PRIORITY;		// Set the priority for the ADC Channel
	chnlCfg.enableInt = false;						// Enabling Call-back interrupt
	chnlCfg.select    = DMAREQ_LEUART0_RXDATAV;		// Selecting Channel
	chnlCfg.cb       = &(cb[DMA_CHANNEL_LEUART_RX]);// Specify call-back function pointer
	DMA_CfgChannel(DMA_CHANNEL_LEUART_RX, &chnlCfg);// Initialize LEUART RX channel configuration

	// Setting up channel descriptor
	descrCfg.dstInc  = dmaDataInc1;					// DMA Destination address pointer incrementer
	descrCfg.srcInc  = dmaDataIncNone;				// DMA Source address pointer incrementer
	descrCfg.size    = dmaDataSize1;				// DMA transfer block size
	descrCfg.arbRate = DMA_LEUARTRX_ARBITRATE;		// DMA Channel Arbitrate rate
	descrCfg.hprot   = 0;
	DMA_CfgDescr(DMA_CHANNEL_LEUART_RX, true, &descrCfg);	// Configuring the Channel Descriptor
}

// Routine for DMA setup for LEUART TX
void DMA_LEUART_TX_Setup(){
	DMA_CfgChannel_TypeDef  chnlCfg;		// DMA Channel configuration for LEUART TX Channel
	DMA_CfgDescr_TypeDef    descrCfg;		// DMA Channel descriptor for LEUART TX Channel

	// Setting up call-back function
	cb[DMA_CHANNEL_LEUART_TX].cbFunc  = DMA_CallBack/*DMA_LEUART_CallBack*/;		// CallBack function for DMA's LEUART-TX Channel
	cb[DMA_CHANNEL_LEUART_TX].userPtr = NULL;

	// Setting up channel
	chnlCfg.highPri   = DMA_LEUART_TX_PRIORITY;							// Set the priority for the ADC Channel
	chnlCfg.enableInt = true;							// Enabling Call-back interrupt
	chnlCfg.select    = DMAREQ_LEUART0_TXBL;			// Selecting Channel
	chnlCfg.cb        = &(cb[DMA_CHANNEL_LEUART_TX]); 	// Specify call-back function pointer
	DMA_CfgChannel(DMA_CHANNEL_LEUART_TX, &chnlCfg);

	// Setting up channel descriptor
	descrCfg.dstInc  = dmaDataIncNone;					// DMA Destination address pointer incrementer
	descrCfg.srcInc  = dmaDataInc1;						// DMA Source address pointer incrementer
	descrCfg.size    = dmaDataSize1;					// DMA transfer block size
	descrCfg.arbRate = DMA_LEUARTTX_ARBITRATE;			// DMA Channel Arbitrate rate
	descrCfg.hprot   = 0;
	DMA_CfgDescr(DMA_CHANNEL_LEUART_TX, true, &descrCfg);	// Configuring the Channel Descriptor
}

// Function to Send Commands
void BLE_CommandTx(char * BLE_CMD, char * RESPONSE_CHK){
	int i = 0, retries = BLE_TIMEOUT;			// i - Variable for index in for loop
	do {
		memset(Rx_Buffer,'\0',sizeof(Rx_Buffer));		// Clear the DMA Rx Buffer
		DMA_ActivateBasic(DMA_CHANNEL_LEUART_RX, true, false, (void *)Rx_Buffer, (void *)&(LEUART0->RXDATA), DMA_LEUARTRX_TRANSFERS-1);	// Reset the DMA transfers
		for (i = 0; i < strlen(BLE_CMD); i++)	// Transmit the command
			LEUART_Tx(LEUART0,BLE_CMD[i]);
		retries--;								// Decrease the Retries count by 1
	} while (strcmp(Rx_Buffer,RESPONSE_CHK) && (retries != 0));	// Check if max retries reached or a proper response has been received
	// To glow LED1 if there was an error even after the given number retries
	/*if (strcmp(Rx_Buffer,RESPONSE_CHK) && (retries == 0))
		GPIO_PinOutClear(LED_PORT, LED1_PIN); */
	memset(Rx_Buffer,'\0',sizeof(Rx_Buffer));	// Clear the Rx_Buffer before exiting
}

// Function to manipulate Bluetooth Module (Adafruit Bluefruit UART Friend)
void BluetoothManipulation(){
	int i = 0, retries = BLE_TIMEOUT;

	// FactoryReset
	for (i = 0; i < strlen(FACTORYRESET); i++)
		LEUART_Tx(LEUART0,FACTORYRESET[i]);

	// Switch to CMD Mode
	BLE_CommandTx(SWITCHMODE,check_1OK);
/*	do {
		memset(Rx_Buffer,'\0',sizeof(Rx_Buffer));
		DMA_ActivateBasic(DMA_CHANNEL_LEUART_RX, true, false, (void *)Rx_Buffer, (void *)&(LEUART0->RXDATA), DMA_LEUARTRX_TRANSFERS-1);
		for (i = 0; i < strlen(SWITCHMODE); i++)
			LEUART_Tx(LEUART0,SWITCHMODE[i]);
		retries--;
	} while (strcmp(Rx_Buffer,check_1OK) && (retries != 0));
	retries = BLE_TIMEOUT;*/

	// Echo
	BLE_CommandTx(ECHO,check_OK);
/*	do {
		memset(Rx_Buffer,'\0',sizeof(Rx_Buffer));
		DMA_ActivateBasic(DMA_CHANNEL_LEUART_RX, true, false, (void *)Rx_Buffer, (void *)&(LEUART0->RXDATA), DMA_LEUARTRX_TRANSFERS-1);
		for (i = 0; i < strlen(ECHO); i++)
			LEUART_Tx(LEUART0,ECHO[i]);
		retries--;
	} while(strcmp(Rx_Buffer,check_OK) && (retries != 0));
	retries = BLE_TIMEOUT;*/

	// Gap Intervals; ConnIntervals and Advertising Intervals
	BLE_CommandTx(GAP_INTERVAL,check_OK);

	// Mode LED
	BLE_CommandTx(HW_MODE_LED,check_OK);
/*	do {
		memset(Rx_Buffer,'\0',sizeof(Rx_Buffer));
		DMA_ActivateBasic(DMA_CHANNEL_LEUART_RX, true, false, (void *)Rx_Buffer, (void *)&(LEUART0->RXDATA), DMA_LEUARTRX_TRANSFERS-1);
		for (i = 0; i < strlen(HW_MODE_LED); i++)
			LEUART_Tx(LEUART0,HW_MODE_LED[i]);
		retries--;
	} while (strcmp(Rx_Buffer,check_OK) && (retries != 0));
	retries = BLE_TIMEOUT;*/

	// Tx Power
	BLE_CommandTx(TX_POWERLEVEL,check_OK);
/*	do {
		memset(Rx_Buffer,'\0',sizeof(Rx_Buffer));
		DMA_ActivateBasic(DMA_CHANNEL_LEUART_RX, true, false, (void *)Rx_Buffer, (void *)&(LEUART0->RXDATA), DMA_LEUARTRX_TRANSFERS-1);
		for (i = 0; i < strlen(TX_POWERLEVEL); i++)
			LEUART_Tx(LEUART0,TX_POWERLEVEL[i]);
		retries--;
	} while (strcmp(Rx_Buffer,check_OK) && (retries != 0));
	retries = BLE_TIMEOUT;*/

	// Do a System Reset
	for (i = 0; i < strlen(SYS_RESET); i++)
			LEUART_Tx(LEUART0,SYS_RESET[i]);

	// Switch back to UART Data Mode
	BLE_CommandTx(SWITCHMODE,check_OK);
/*	do {
		memset(Rx_Buffer,'\0',sizeof(Rx_Buffer));
		DMA_ActivateBasic(DMA_CHANNEL_LEUART_RX, true, false, (void *)Rx_Buffer, (void *)&(LEUART0->RXDATA), DMA_LEUARTRX_TRANSFERS-1);
		for (i = 0; i < strlen(SWITCHMODE); i++)
			LEUART_Tx(LEUART0,SWITCHMODE[i]);
		retries--;
	} while (strcmp(Rx_Buffer,check_0OK) && (retries != 0));*/
}

// Dac peripheral setup
void DAC_setup(void){
	// DAC Configuration
	DAC_Init_TypeDef dacInit;
	dacInit.refresh      = dacRefresh8;
	dacInit.reference    = dacRefVDD;
	dacInit.outMode      = dacOutputPin;
	dacInit.convMode     = dacConvModeContinuous;
	dacInit.prescale     = 0;
	dacInit.lpEnable     = false;
	dacInit.ch0ResetPre  = false;
	dacInit.outEnablePRS = false;
	dacInit.sineEnable   = false;
	dacInit.diff         = false;

	// Initialize DAC0
	DAC_Init(DAC0, &dacInit);

	// Setup DAC0 for LC Sensor
	writeDataDAC(DAC0, (unsigned int) LCSENSE_DAC_DATA, LCSENSE_DAC_CHANNEL);
}

// Write the data to the correct channel
void writeDataDAC(DAC_TypeDef *dac, unsigned int value, unsigned int ch){
	// Write the DAC Value to the Register
	if (!ch)
		dac->CH0DATA = value;	    // Write into Channel 0
	else
		dac->CH1DATA = value;		// Write into Channel 1
}

// ACMP setup for LC sense and Light Sense
void ACMP_setup(void){
	// ACMP Configuration
	ACMP_Init_TypeDef acmpInit;
	acmpInit.fullBias                 = false;
	acmpInit.halfBias                 = true;
	acmpInit.biasProg                 = 0xE;
	acmpInit.interruptOnFallingEdge   = false;
	acmpInit.interruptOnRisingEdge    = false;
	acmpInit.warmTime                 = acmpWarmTime512;
	acmpInit.hysteresisLevel          = acmpHysteresisLevel0;
	acmpInit.inactiveValue            = false;
	acmpInit.lowPowerReferenceEnabled = false;
	acmpInit.vddLevel                 = ACMP_VDD_SCALE;
	acmpInit.enable                   = false;

	// Initialize ACMP0
	ACMP_Init(ACMP0, &acmpInit);

	// Disable ACMP0 out to a pin.
	ACMP_GPIOSetup(ACMP0, 0, false, false);
	// Select VDD as negative reference
	// Positive reference is controlled by LESENSE
	ACMP_ChannelSet(ACMP0, ACMP_NEG_REF, LESENSE_ACMP_CHANNEL);
}

// For Capsense
void ACMP1_setup(void){
	// Configuration structure for ACMP
	// See application note document for description of the different settings.
	ACMP_CapsenseInit_TypeDef acmpInit =
	  {
	    .fullBias                 = true,            //Configured according to application note
	    .halfBias                 = true,            //Configured according to application note
	    .biasProg                 = 0x5,             //Configured according to application note
	    .warmTime                 = acmpWarmTime512, //LESENSE uses a fixed warmup time
	    .hysteresisLevel          = acmpHysteresisLevel5, //Configured according to application note
	    .resistor                 = acmpResistor0,   //Configured according to application note
	    .lowPowerReferenceEnabled = false,           //LP-reference can introduce glitches with captouch
	    .vddLevel                 = 0x30,            //Configured according to application note
	    .enable                   = false            //LESENSE enables the ACMP
	  };

	// Disable ACMP0 out to a pin.
	ACMP_GPIOSetup(ACMP1, 0, false, false);
	// Select VDD as negative reference

	/* Initialize ACMP in capsense mode*/
	ACMP_CapsenseInit(ACMP1, &acmpInit);
}

// LESENSE setup
void LESENSE_setup(void){
	// LESENSE Configuration
	LESENSE_Init_TypeDef initLesense;

	// Core Configuration
	initLesense.coreCtrl.scanStart    = lesenseScanStartPeriodic;
	initLesense.coreCtrl.prsSel       = lesensePRSCh0;
	initLesense.coreCtrl.scanConfSel  = lesenseScanConfDirMap;
	initLesense.coreCtrl.invACMP0     = false;
	initLesense.coreCtrl.invACMP1     = false;
	initLesense.coreCtrl.dualSample   = false;
	initLesense.coreCtrl.storeScanRes = false;
	initLesense.coreCtrl.bufOverWr    = true;
	initLesense.coreCtrl.bufTrigLevel = lesenseBufTrigHalf;
	initLesense.coreCtrl.wakeupOnDMA  = lesenseDMAWakeUpDisable;
	initLesense.coreCtrl.biasMode     = lesenseBiasModeDutyCycle;
	initLesense.coreCtrl.debugRun     = false;

	// Time Configuration
	initLesense.timeCtrl.startDelay     = 0;

	// Peripheral Configuration
	initLesense.perCtrl.dacCh0Data     = lesenseDACIfData;
	initLesense.perCtrl.dacCh0ConvMode = lesenseDACConvModeSampleOff;
	initLesense.perCtrl.dacCh0OutMode  = lesenseDACOutModeDisable;
	initLesense.perCtrl.dacCh1Data     = lesenseDACIfData;
	initLesense.perCtrl.dacCh1ConvMode = lesenseDACConvModeSampleOff;
	initLesense.perCtrl.dacCh1OutMode  = lesenseDACOutModePin;
	initLesense.perCtrl.dacPresc       = 31;
	initLesense.perCtrl.dacRef         = lesenseDACRefBandGap; 			// BandGap is needed for ACMP to work //For only LC Sensor use lesenseDACRefVdd;
	initLesense.perCtrl.acmp0Mode      = lesenseACMPModeMux;			// LC Sense Doesnt work for lesenseACMPModeMuxThres
	initLesense.perCtrl.acmp1Mode      = lesenseACMPModeMuxThres;		// For Cap sense
	initLesense.perCtrl.warmupMode     = lesenseWarmupModeNormal;

	// Descriptor Configuration
	initLesense.decCtrl.decInput  = lesenseDecInputSensorSt;
	initLesense.decCtrl.initState = 0;
	initLesense.decCtrl.chkState  = false;
	initLesense.decCtrl.intMap    = true;
	initLesense.decCtrl.hystPRS0  = false;
	initLesense.decCtrl.hystPRS1  = false;
	initLesense.decCtrl.hystPRS2  = false;
	initLesense.decCtrl.hystIRQ   = false;
	initLesense.decCtrl.prsCount  = false;
	initLesense.decCtrl.prsChSel0 = lesensePRSCh0;
	initLesense.decCtrl.prsChSel1 = lesensePRSCh1;
	initLesense.decCtrl.prsChSel2 = lesensePRSCh2;
	initLesense.decCtrl.prsChSel3 = lesensePRSCh3;

	// Initialize LESENSE on Reset
	LESENSE_Init(&initLesense, true);

	// Ambient Light Sensor Channel Initialization
	LESENSE_ChDesc_TypeDef initLIGHTSENSE;

	initLIGHTSENSE.enaScanCh     = true;                      	// Enable scan channel.
	initLIGHTSENSE.enaPin        = false;                     	// Enable the assigned pin on scan channel.
	initLIGHTSENSE.enaInt        = true;                      	// Enable interrupts on channel.
	initLIGHTSENSE.chPinExMode   = lesenseChPinExHigh;        	// GPIO pin is high during the excitation period.
	initLIGHTSENSE.chPinIdleMode = lesenseChPinIdleDis;       	// GPIO pin is low during the idle period.
	initLIGHTSENSE.useAltEx      = true;                      	// Use alternate excitation pins for excitation.
	initLIGHTSENSE.shiftRes      = false;                     	// Disabled to shift results from this channel to the decoder register.
	initLIGHTSENSE.invRes        = false;                     	// Disabled to invert the scan result bit.
	initLIGHTSENSE.storeCntRes   = false;                      	// Enabled to store counter value in the result buffer.
	initLIGHTSENSE.exClk         = lesenseClkLF;              	// Use the LF clock for excitation timing.
	initLIGHTSENSE.sampleClk     = lesenseClkLF;              	// Use the LF clock for sample timing.
	initLIGHTSENSE.exTime        = 0x01U;                     	// Excitation time is set to 1(+1) excitation clock cycles.
	initLIGHTSENSE.sampleDelay   = 0x01U;                     	// Sample delay is set to 1(+1) sample clock cycles.
	initLIGHTSENSE.measDelay     = 0x00U;                     	// Measure delay is set to 0 excitation clock cycles.
	initLIGHTSENSE.acmpThres     = LIGHTSENSE_ACMP_THRESHOLD; 	// ACMP threshold has been set to 0x38.
	initLIGHTSENSE.sampleMode    = lesenseSampleModeACMP;      	// ACMP will be used in comparison.
	initLIGHTSENSE.intMode       = lesenseSetIntLevel;      	// Interrupt is generated if the sensor triggers
	initLIGHTSENSE.cntThres      = 0x0000U;                 	// Counter threshold has been set to 0x00.
	initLIGHTSENSE.compMode      = lesenseCompModeLess;       	// Compare mode has been set to trigger interrupt on "less".

	// Channel 6 Configuration for LightSense
	LESENSE_ChannelConfig(&initLIGHTSENSE, LCSENSE_LIGHTSENSE_CHANNEL);

	// LC Sensor Channel Initialization
	LESENSE_ChDesc_TypeDef initLCSENSE;

	// LCSense Configuration
	initLCSENSE.enaScanCh     = true;
	initLCSENSE.enaPin        = true;
	initLCSENSE.enaInt        = true;
	initLCSENSE.chPinExMode   = lesenseChPinExLow;
	initLCSENSE.chPinIdleMode = lesenseChPinIdleDis;
	initLCSENSE.useAltEx      = false;
	initLCSENSE.shiftRes      = false;
	initLCSENSE.invRes        = false;
	initLCSENSE.storeCntRes   = true;
	initLCSENSE.exClk         = lesenseClkHF;
	initLCSENSE.sampleClk     = lesenseClkLF;
	initLCSENSE.exTime        = 0x07;
	initLCSENSE.sampleDelay   = 0x01;
	initLCSENSE.measDelay     = 0x00;
	initLCSENSE.acmpThres     = 0x00;
	initLCSENSE.sampleMode    = lesenseSampleModeCounter;
	initLCSENSE.intMode       = lesenseSetIntLevel;
	initLCSENSE.cntThres      = 0x0000;
	initLCSENSE.compMode      = lesenseCompModeGreaterOrEq;

	// Channel 7 Configuration for LC SENSE
	LESENSE_ChannelConfig(&initLCSENSE, LCSENSE_LESENSE_CHANNEL);

	// Capacitive Sensor Channel Initialization
	LESENSE_ChDesc_TypeDef initCapSENSE;

	// CapSense Configuration
	initCapSENSE.enaScanCh     = true;
	initCapSENSE.enaPin        = true;
	initCapSENSE.enaInt        = true;
	initCapSENSE.chPinExMode   = lesenseChPinExLow;
	initCapSENSE.chPinIdleMode = lesenseChPinIdleDis;
	initCapSENSE.useAltEx      = false;
	initCapSENSE.shiftRes      = false;
	initCapSENSE.invRes        = false;
	initCapSENSE.storeCntRes   = true;
	initCapSENSE.exClk         = lesenseClkHF;
	initCapSENSE.sampleClk     = lesenseClkLF;
	initCapSENSE.exTime        = 0x00;
	initCapSENSE.sampleDelay   = 0x30;														// Set the sample delay
	initCapSENSE.measDelay     = 0x00;
	initCapSENSE.acmpThres     = 0x30U;
	initCapSENSE.sampleMode    = lesenseSampleModeCounter;
	initCapSENSE.intMode       = lesenseSetIntPosEdge;
	initCapSENSE.cntThres      = 0x0000;
	initCapSENSE.compMode      = lesenseCompModeLess;

	// Channel 8 Configuration for Cap Sense
	LESENSE_ChannelConfig(&initCapSENSE, CAPSENSE_LESENSE_CHANNEL);

	// LC Sense Calibration
	if (MANUAL_THRESHOLD)
			LESENSE_ChannelThresSet(LCSENSE_LESENSE_CHANNEL, 0, LCSENSE_MANUAL_THRESHOLD + LCSENSE_THRESHOLD_HYSTERESIS);
		else{
			LESENSE_ScanFreqSet(0, LCSENSE_SCAN_FREQ_Calb);		// LESENSE Scan frequency for Calibration of LC Sensor
			if (CHECK_LCSENSOR_COUNT) while(1)
			// LESENSE Calibration
			LCSensor_Calibration(LCSENSE_LESENSE_CHANNEL);
	}

	// Cap Sense Calibration
	CapSensor_Calibration(CAPSENSE_LESENSE_CHANNEL);

	// Alternate Excitation Initialization
	LESENSE_ConfAltEx_TypeDef initAltEx = LESENSE_LIGHTSENSE_ALTEX_CONF;
	// Set the Alternate Excitation pin										// Here we use only Ambient Light
	LESENSE_AltExConfig(&initAltEx);

	// Enable all the other channels before starting scan
	LESENSE_ChannelEnable(LIGHTSENSE_LESENSE_CHANNEL, true, false);
	LESENSE_ChannelEnable(LCSENSE_LESENSE_CHANNEL, true, true);
	LESENSE_ChannelEnable(CAPSENSE_LESENSE_CHANNEL, true, true);

	// Clock Prescalers
	LESENSE_ClkDivSet(lesenseClkLF, lesenseClkDiv_1);		// Set Clock divisor for LF Clock
	LESENSE_ClkDivSet(lesenseClkHF, lesenseClkDiv_1);		// Set Clock divisor for HF Clock

	// Set the scan frequency
	LESENSE_ScanFreqSet(0, LCSENSE_SCAN_FREQ);

	// Enable LESENSE Interrupt
	NVIC_EnableIRQ(LESENSE_IRQn);

	/* Start scan. */
	LESENSE_ScanStart();
}

// Calibration routine for the Cap sense
void CapSensor_Calibration(uint8_t chIdx){
	uint8_t i;
	uint32_t calibValue;

	// Enable Scan and Pin on selected Channel
	LESENSE_ChannelEnable(chIdx, true, true);

	// Disable Scan and Pin on other channels
	for(i=0; i<LESENSE_MAX_CHANNELS; i++)
		if(i!=chIdx)
			LESENSE_ChannelEnable(i, false, false);

	// Start Scan
	LESENSE_ScanStart();

	// Waiting for the buffer to be full
	while(!(LESENSE->STATUS & LESENSE_STATUS_BUFFULL));

	calibValue = LESENSE_ScanResultDataBufferGet(0);
	// Use the last result as counter threshold
	LESENSE_ChannelThresSet(chIdx, 0x30, calibValue - 600);

	// Stop Scan
	LESENSE_ScanStop();

	// Clear the result buffer
	LESENSE_ResultBufferClear();
}

//LC Sensor calibration routine; TO check the number of counts
void LCSensor_Calibration(uint8_t chIdx){
	uint8_t i;
	uint32_t calibValue;

	// Enable Scan and Pin on selected Channel
	LESENSE_ChannelEnable(chIdx, true, true);

	// Disable Scan and Pin on other channels
	for(i=0; i<LESENSE_MAX_CHANNELS; i++)
		if(i!=chIdx)
			LESENSE_ChannelEnable(i, false, false);

	// Start Scan
	LESENSE_ScanStart();

	// Waiting for the buffer to be full
	while(!(LESENSE->STATUS & LESENSE_STATUS_BUFFULL));

	calibValue = LESENSE_ScanResultDataBufferGet(BUFFER_INDEX_LAST);
	// Use the last result as counter threshold
	LESENSE_ChannelThresSet(chIdx, 0, calibValue - 2);

	// Stop Scan
	LESENSE_ScanStop();

	// Clear the result buffer
	LESENSE_ResultBufferClear();
}

// EEPROM initialization
void EEPROM_Initialization(){
	   // Initialize the eeprom emulator using 3 pages.
	  if ( !EE_Init(EEPROM_PAGES_NUMBER) ) {

	     // If the initialization fails we have to take some measure
	     // to obtain a valid set of pages. In this example we simply
	     // format the pages
	    EE_Format(EEPROM_PAGES_NUMBER);
	  }

	   // All variables should be declared prior to any writes.
	  EE_DeclareVariable(&ee_security_on);
}

// Read the settings right after reset
void EEPROM_ReadSettings(){

	   //  Enables the flash controller for writing.
	  MSC_Init();

	  EE_Read(&ee_security_on, &readValue);
	  if (readValue == 0)
		  SECURITY_ON = false;
	  else
		  SECURITY_ON = true;
}

// Intrusion detected
void intrusionDetectedISR(){
	smartHomeMessage[0] = 'Y';		// Yes for intrusion detection
	INTRUSION_DETECTED = true;
}

// Intruder left the area
void intruderLeftISR(){
	smartHomeMessage[0] = 'N';		// No if intruder left
	INTRUSION_DETECTED = false;
}

// Setup for proximity sensor
void PROXIMITY_initSetup() {
	GPIO_DriveModeSet(PROXIMITY_PORT, gpioDriveModeHigh);
	//GPIO_PinOutSet(PROXIMITY_PORT, PROXIMITY_PIN);

	proximitySensorSIG.fall(&intrusionDetectedISR);
	proximitySensorSIG.rise(&intruderLeftISR);

}

void PROXIMITY_on(){
	//GPIO_PinModeSet(PROXIMITY_PORT, PROXIMITY_PIN, gpioModePushPullDrive, 0);
	GPIO_PinOutSet(PROXIMITY_PORT, PROXIMITY_PIN);
}

void PROXIMITY_off() {
	// Try completely disabling
	//GPIO_PinModeSet(PROXIMITY_PORT, PROXIMITY_PIN, gpioModeDisabled, 0);
	GPIO_PinOutClear(PROXIMITY_SENSOR_PORT, PROXIMITY_PIN);
}

// Function to switch from sending data every 4 seconds to waiting for command
void switchTORPi_ISR() {
	switchToRPi = !switchToRPi;
	if (switchToRPi) {
		GPIO_PinOutSet(LED_PORT, LED0_PIN);
		if (DMA_IDLE){
			LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;				// Enable DMA wake up for LEUART TX in EM2
			DMA_IDLE = false;
			// Activate DMA transfers for LEUART TX
			DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)switchingToRPi, strlen(switchingToRPi) - 1);
		}
	}
	else {
		GPIO_PinOutSet(LED_PORT, LED1_PIN);
		if (DMA_IDLE){
			LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;				// Enable DMA wake up for LEUART TX in EM2
			DMA_IDLE = false;
			// Activate DMA transfers for LEUART TX
			DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)switchingBackFromRPi, strlen(switchingBackFromRPi) - 1);
		}
	}
}

void switchTORPi_setup() {
	switchTORPi_button.rise(&switchTORPi_ISR);
	//22switchTORPi_button.fall(&switchTORPi_ISR);
}

int main(void){
	CHIP_Init();
	/*EEPROM_Initialization();
	EEPROM_ReadSettings();*/
	blockSleepMode(EM2);
	pc.printf("Welcome");
	INT_Disable();

	#if DEBUG_MODE
	BSP_TraceSwoSetup();    		// power analysis for specific code section
	#endif

	// Peripheral Initialization
	CMU_setup();			// Setup all oscillators and clocks required
	GPIO_setup();			// Setup the LEDs, LESENSE excitation/measurement pins
	DAC_setup();			// Setup required DAC output for LC Sense excitation
	ACMP_setup();			// ACMP setup for LC Sense comparision
	ACMP1_setup();
	LETIMER_setup();		// Letimer for turning the LED off in 2 seconds every time the LC sensor is triggered
	ADC_setup();			// ADC Setup for Temperature Sensor
	LESENSE_setup();		// LESENSE initialization

	// Peripheral initialization for Communication using Bluefruit
	DMA_setup();					// Base DMA setup
	DMA_ADC_Setup();				// DMA Setup for ADC
    DMA_LEUART_TX_Setup();			// DMA setup for LEUART Tx
    DMA_LEUART_RX_Setup();			// DMA setup for LEUART Rx
	LEUART_setup();					// LEUART setup

	PROXIMITY_initSetup();			// Setup the Proximity Sensor
	switchTORPi_setup();			// switch to sending data every 4 seconds to waiting for command to send the packet; Using Pushbutton0

	#if BLE_SWITCH
	GPIO_PinModeSet(BLE_SWITCH_PORT, BLE_SWITCH_PIN, gpioModePushPull, 1);	// BLE Switch
	#endif

	#if BLE_CONFIGURATION
	BluetoothManipulation();
	#endif

	INT_Enable();

	// EEPROM emulation failed; So, the default values max and min values are set to be 30 and 15 respectively
	TEMP_UPPERLIMIT = 30;
	TEMP_LOWERLIMIT = 15;

	while(1){
		sleep();	// Sleep tight
	}
}
