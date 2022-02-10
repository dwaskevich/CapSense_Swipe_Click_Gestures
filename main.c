/******************************************************************************
* File Name:   main.c
*
* Description: This project started with Empty PSoC6 Application for ModusToolbox.
*  created 23-Jan-2020
*  by David Waskevich
*  DJJW - MBR3 Host sample application
*
*  Use I2C interface to talk to MBR3 eval kit shield
*
*  The circuit:
*  - Use I2C on shield header to communicate with MBR3 shield.
*  - Note - using CY8CKIT-042 kit to mimic CY8CMBR3106S
*
*  Updates:
*    24-Jan-2021 - adding OLED display aesthetics
*    25-Jan-2021 - cleaning up code/comments and eliminating "magic numbers"
*    				- using #defines in mtb_ssd1306_i2c.h
*    			 - added centroid value display in center of speed bar
*    			 - added string array to hold event type names for printf
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "mtb_ssd1306.h"
#include "mtb_ssd1306_i2c.h"
#include "GUI.h"
#include "stdio.h"

/* Set SysTick interval to 1 msec */
#define SYSTICK_INTERVAL (CY_SYSCLK_IMO_FREQ / 1000)

/* I2C slave address to communicate with */
#define I2C_SLAVE_ADDR          (0x08UL)

/* Define slider centroid register address */
#define SLAVE_STATUS_REGISTER_ADDRESS (0xb0)

/* I2C transaction timeout */
#define I2C_TIMEOUT_MS				(100u)

/* I2C bus frequency */
#define I2C_FREQ                (400000UL)

/* I2C pins */
#define I2C_SCL					(P10_0)
#define I2C_SDA					(P10_1)

/* Set macro to '1' to enable centroid logging, '0' to disable logging */
#define ENABLE_CENTROID_LOGGING     (1u)

/* Finite state machine states for gesture decode (djjw)
    WAIT_FOR_TOUCH - wait for centroid value to be different than #define SLIDER_NO_TOUCH
	DECODE_GESTURE - finger event detected, decode gesture (look for centroid movement >= GESTURE_POSITION_THRESHOLD)
    WAIT_FOR_LIFTOFF - no temporal component to gesture algorithm at this point ... just wait for liftoff */
typedef enum
{
    WAIT_FOR_TOUCH = 0x01u,
    DECODE_GESTURE = 0x02u,
    WAIT_FOR_LIFTOFF = 0x03u
} GESTURE_DECODE_STATE;

/* Gesture decode types (djjw)
    NO_GESTURE - what it sounds like
    TOUCHDOWN_GESTURE - finger event detected, waiting for next event
    CLICK_GESTURE - occurs if lift-off happens with no centroid displacement within MAX_TOUCH_DURATION_FOR_CLICK
	SWIPE_GESTURE_FAST_RIGHT - swipe gesture determined by centroid displacement and elapsed time
    SWIPE_GESTURE_SLOW_RIGHT - swipe gesture determined by centroid displacement and elapsed time
    SWIPE_GESTURE_FAST_LEFT - swipe gesture determined by centroid displacement and elapsed time
    SWIPE_GESTURE_FAST_LEFT - swipe gesture determined by centroid displacement and elapsed time */
typedef enum
{
  NO_GESTURE,
  TOUCHDOWN,
  CLICK_GESTURE,
  SWIPE_GESTURE_FAST_RIGHT,
  SWIPE_GESTURE_SLOW_RIGHT,
  SWIPE_GESTURE_FAST_LEFT,
  SWIPE_GESTURE_SLOW_LEFT,
  CLICK_HOLD_DRAG
} GESTURE_TYPE;

//char gestureNames[][20] = {"NO_GESTURE", "TOUCHDOWN", "CLICK", "FAST_RIGHT", "SLOW_RIGHT", "FAST_LEFT", "SLOW_LEFT", "CLICK/HOLD/DRAG"};
char *gestureNames[] = {"NO_GESTURE", "TOUCHDOWN", "CLICK", "FAST_RIGHT", "SLOW_RIGHT", "FAST_LEFT", "SLOW_LEFT", "CLICK/HOLD/DRAG"};

// Displacement threshold for swipe gestures (units are in centroid values)
#define GESTURE_DISPLACEMENT_THRESHOLD 25u

// Temporal gesture settings (units are msec)
#define MIN_TOUCH_DURATION 10u
#define MAX_TOUCH_DURATION_FOR_CLICK 150u
#define MAX_TOUCH_DURATION_FOR_SWIPE 250u
#define SWIPE_FAST_SLOW_THRESHOLD 100u

#define SLIDER_RESOLUTION 100u
#define SLIDER_NO_TOUCH 0xFFu
#define CLICK_INCREMENT 5u
#define SPEED_INCREMENT (SLIDER_RESOLUTION / 8)
#define MAX_SPEED_BAR_VALUE (SLIDER_RESOLUTION)

#define NEW_DISPLAY_TIME_TO_LIVE 750u

#define INCLUDE_CENTROID_ON_OLED (0u)
#define LAST_ROW_OF_OLED (MTB_DISPLAY_SIZE_Y - 1)
#define LAST_COLUMN_OF_OLED (MTB_DISPLAY_SIZE_X - 1)
#define OLED_GESTURE_LINE_NUM	(2)
#define OLED_CENTROID_LINE_NUM	(4)

#define MAIN_LOOP_DELAY_MSEC 1u

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Variable for storing character read from terminal */
uint8_t uart_read_value;

/* Status register address inside Slave for getting centroid posistion */
const unsigned char statusRegAddr[] = {SLAVE_STATUS_REGISTER_ADDRESS};

volatile uint16_t gestureSysTickTimer = 0;

cy_rslt_t result;
cyhal_i2c_t i2c_obj;

bool newDisplayUpdateFlag = false;
unsigned long newDisplayTimeToLive;
bool newDisplayCentroidFlag = false;
uint8_t centroid;
int speedBarValue, previousSpeedBarValue = 0;
char str [80];
int fontSizeX, fontSizeY;
uint8_t wtfFlag = 0;

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/*****************************************************************************
* Function Prototypes
*****************************************************************************/
void Decode_Gestures(void);
void printDisplacement(uint32, uint32, GESTURE_TYPE);
void SysTickISRCallback(void);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
    	handle_error();
    }

    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_IMO, SYSTICK_INTERVAL);

    /* find next available SysTick callback slot */
	for(uint8_t i = 0; i < CY_SYS_SYST_NUM_OF_CALLBACKS; i++)
	{
		if(Cy_SysTick_GetCallback(i) == NULL)
		{
			Cy_SysTick_SetCallback(i, SysTickISRCallback);
			break;
		}
	}

    /* Initialize the I2C to use with the OLED display */
	result = cyhal_i2c_init(&i2c_obj, I2C_SDA, I2C_SCL, NULL);

	/* Initialize the OLED display */
	result = mtb_ssd1306_init_i2c(&i2c_obj);

    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
	result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
								 CY_RETARGET_IO_BAUDRATE);

	/* retarget-io init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS)
	{
		handle_error();
	}

	/* Initialize the User LED */
	result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
							 CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

	/* GPIO init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS)
	{
		handle_error();
	}

	/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	printf("\x1b[2J\x1b[;H");

	printf("*************** "
		   "PSoC 6 MCU: empty project "
		   "***************\r\n\n");

	printf("djjw gestures!!!\r\n\n");

	printf("OLED display enabled ...\r\n\r\n");

	GUI_Init();
	GUI_SetBkColor(GUI_WHITE);
	GUI_SetColor(GUI_BLACK);
	GUI_SetTextMode(GUI_TM_NORMAL);
	GUI_DispStringHCenterAt(" PSoC6 Gestures v1 \r\n", (MTB_DISPLAY_SIZE_X)/2, 0);
	GUI_SetBkColor(GUI_BLACK);
	GUI_SetColor(GUI_WHITE);
	GUI_SetTextMode(GUI_TM_NORMAL);
	fontSizeX = GUI_GetCharDistX('0');
	fontSizeY = GUI_GetFontDistY();

	GUI_DrawRoundedRect(0, (LAST_ROW_OF_OLED - fontSizeY - 2), LAST_COLUMN_OF_OLED, LAST_ROW_OF_OLED, 2);
	GUI_GotoXY((MTB_DISPLAY_SIZE_X/2 - 9), (LAST_ROW_OF_OLED - fontSizeY));
	GUI_DispDec(speedBarValue, 3);

    for (;;)
    {
		/* run the decode gestures state machine */
    	Decode_Gestures();

    	/* set main loop delay */
		cyhal_system_delay_ms(MAIN_LOOP_DELAY_MSEC);

		/* main loop tasks - updating OLED display */
		if(true == newDisplayUpdateFlag)
		  {
		    /* clear display information if TIME_TO_LIVE is expired */
		    if(gestureSysTickTimer - newDisplayTimeToLive >= NEW_DISPLAY_TIME_TO_LIVE)
		    {
		    	GUI_GotoXY(0, OLED_GESTURE_LINE_NUM * fontSizeY);
		    	GUI_DispCEOL();
		    	GUI_GotoXY(0, OLED_CENTROID_LINE_NUM * fontSizeY);
		    	GUI_DispCEOL();
		    	newDisplayUpdateFlag = false;
		    }
		  }

		  if(0 != wtfFlag) /* print centroid tracking information to OLED display */
		  {
		    speedBarValue = centroid;
		    GUI_GotoXY(0, OLED_CENTROID_LINE_NUM * fontSizeY);
		    sprintf(str, "wtf 0x%02X", centroid);
		    GUI_DispString(str);
		  }

		  if(speedBarValue != previousSpeedBarValue) /* update speed bar animation and value on OLED display */
		  {
		    previousSpeedBarValue = speedBarValue;
		    GUI_SetDrawMode(GUI_DM_REV);
		    GUI_FillRoundedRect(1, (LAST_ROW_OF_OLED - fontSizeY - 1), (LAST_COLUMN_OF_OLED - 1), (LAST_ROW_OF_OLED - 1), 1);
		    GUI_SetDrawMode(GUI_DM_NORMAL);
		    GUI_FillRoundedRect(1, (LAST_ROW_OF_OLED - fontSizeY - 1), (speedBarValue * (LAST_COLUMN_OF_OLED - 1))/100, (LAST_ROW_OF_OLED - 1), 1);
		    GUI_GotoXY((MTB_DISPLAY_SIZE_X/2 - 15), (LAST_ROW_OF_OLED - fontSizeY));
		    GUI_DispChar(' ');
		    GUI_GotoXY((MTB_DISPLAY_SIZE_X/2 - 9), (LAST_ROW_OF_OLED - fontSizeY));
			GUI_DispDec(speedBarValue, 3);
			GUI_DispChar(' ');
		  }
    }
}

/*******************************************************************************
* Function Name: Decode_Gestures
********************************************************************************
* Summary:
* In this function, flick gestures are decoded by interpreting linear slider
* centroid displacements (GESTURE_DISPLACEMENT_THRESHOLD).
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void Decode_Gestures()
{
   #if ENABLE_CENTROID_LOGGING
		// djjw - adding a print log for diagnostics
		#define FALSE (0u)
		#define TRUE (1u)
		#define MAX_LOG_ENTRIES 600u
		static uint16 centroidLog[MAX_LOG_ENTRIES][2];
		static uint16 logIndex = 0;
		static uint16 gestureTriggeredIndex;
		static char touchIsActive = FALSE;
        static uint32 gestureEvent;
		uint16 i;
	#endif /* ENABLE_CENTROID_LOGGING */

	cy_rslt_t result;
	static GESTURE_DECODE_STATE currentState = WAIT_FOR_TOUCH;
	static uint32 touchdownCentroidValue;
	static uint32 gesture = NO_GESTURE;
	static uint32 previousGesture = NO_GESTURE;

	/* Get the touch position (centroid) of the slider */

	/* Set register address in slave, then read data. */
	result = cyhal_i2c_master_write(&i2c_obj, I2C_SLAVE_ADDR, statusRegAddr, sizeof(statusRegAddr), I2C_TIMEOUT_MS, false);
	if(CY_RSLT_SUCCESS != result)
	{
		printf("I2C write error, return status = 0x%04x\r\n", (unsigned int) result);
		cyhal_system_delay_ms(5000);
//		handle_error();
	}

	/* read centroid value */
	result = cyhal_i2c_master_read(&i2c_obj, I2C_SLAVE_ADDR, &centroid, sizeof(centroid), I2C_TIMEOUT_MS, true);
	if(CY_RSLT_SUCCESS != result)
	{
		printf("I2C error, return status = 0x%04x\r\n", (unsigned int) result);
		cyhal_system_delay_ms(5000);
//		handle_error();
	}

	/* process gesture state machine */
	switch( currentState )
	{
		case	WAIT_FOR_TOUCH:
				if(SLIDER_NO_TOUCH != centroid)
				{
					touchdownCentroidValue = centroid; /* record touchdown value */
					gestureSysTickTimer = 0; /* reset gestureSysTickTimer to place touchdown at time zero */
					gesture = TOUCHDOWN; /* post gesture */
					currentState = DECODE_GESTURE; /* move to next state */
				}
				break;

		case	DECODE_GESTURE:
				if(SLIDER_NO_TOUCH == centroid) /* liftoff event happened (i.e. no Touch detected) */
				{
					if(gestureSysTickTimer < MIN_TOUCH_DURATION || gestureSysTickTimer > MAX_TOUCH_DURATION_FOR_CLICK)
                    {
						gesture = NO_GESTURE; /* abandon gesture if lift-off occurs quicker than MIN_TOUCH_DURATION or lasts longer than MAX */
                        #if ENABLE_CENTROID_LOGGING
                            gestureTriggeredIndex = logIndex; /* record scan index for log dump later */
                            gestureEvent = gesture;
                        #endif /* ENABLE_CENTROID_LOGGING */
                    }
					else
                    {
						gesture = CLICK_GESTURE; /* click occurs if lift-off happens with no centroid displacement */
                        #if ENABLE_CENTROID_LOGGING
                            gestureTriggeredIndex = logIndex; /* record scan index for log dump later */
                            gestureEvent = gesture;
                        #endif /* ENABLE_CENTROID_LOGGING */
                    }
					currentState = WAIT_FOR_TOUCH; /* go back to WAIT_FOR_TOUCH state */
				}
				else /* liftoff hasn't happened yet ... still in active gesture decode */
				{
					if(gestureSysTickTimer <= MAX_TOUCH_DURATION_FOR_SWIPE)
					{
						/* test for right swipe conditions (i.e. has there been enough displacement) */
						if(touchdownCentroidValue < (SLIDER_RESOLUTION - GESTURE_DISPLACEMENT_THRESHOLD) && \
							centroid > (touchdownCentroidValue + GESTURE_DISPLACEMENT_THRESHOLD))
						{
							/* now test for speed */
							if(gestureSysTickTimer < SWIPE_FAST_SLOW_THRESHOLD)
							{
								gesture = SWIPE_GESTURE_FAST_RIGHT;
								#if ENABLE_CENTROID_LOGGING
									gestureTriggeredIndex = logIndex; /* record scan index for log dump later */
									gestureEvent = gesture;
								#endif /* ENABLE_CENTROID_LOGGING */
							}
							else
							  if(gestureSysTickTimer < MAX_TOUCH_DURATION_FOR_SWIPE)
							  {
								gesture = SWIPE_GESTURE_SLOW_RIGHT;
								#if ENABLE_CENTROID_LOGGING
									gestureTriggeredIndex = logIndex; /* record scan index for log dump later */
									gestureEvent = gesture;
								#endif /* ENABLE_CENTROID_LOGGING */
							  }
							currentState = WAIT_FOR_LIFTOFF;
						}

						/* test for left swipe conditions (i.e. has there been enough displacement) */
						if(touchdownCentroidValue > (0 + GESTURE_DISPLACEMENT_THRESHOLD)  && \
							centroid < (touchdownCentroidValue - GESTURE_DISPLACEMENT_THRESHOLD))
						{
							/* now test for speed */
							if(gestureSysTickTimer < SWIPE_FAST_SLOW_THRESHOLD)
							{
								gesture = SWIPE_GESTURE_FAST_LEFT;
								#if ENABLE_CENTROID_LOGGING
									gestureTriggeredIndex = logIndex; /* record scan index for log dump later */
									gestureEvent = gesture;
								#endif /* ENABLE_CENTROID_LOGGING */
							}
							else
							  if(gestureSysTickTimer < MAX_TOUCH_DURATION_FOR_SWIPE)
							  {
								gesture = SWIPE_GESTURE_SLOW_LEFT;
								#if ENABLE_CENTROID_LOGGING
									gestureTriggeredIndex = logIndex; /* record scan index for log dump later */
									gestureEvent = gesture;
								#endif /* ENABLE_CENTROID_LOGGING */
							  }
							currentState = WAIT_FOR_LIFTOFF;
						}
					}
					else
					{
						/* test for click and hold */
						if(gestureSysTickTimer >= MAX_TOUCH_DURATION_FOR_SWIPE) /* activate CLICK_HOLD_DRAG */
						{
							gesture = CLICK_HOLD_DRAG;
							#if ENABLE_CENTROID_LOGGING
							  gestureTriggeredIndex = logIndex; /* record scan index for log dump later */
							  gestureEvent = gesture;
							#endif /* ENABLE_CENTROID_LOGGING */
							currentState = WAIT_FOR_LIFTOFF;
						}
					}
				}
				break;

		case	WAIT_FOR_LIFTOFF:
				if(SLIDER_NO_TOUCH == centroid)
				{
					gesture = NO_GESTURE;
					currentState = WAIT_FOR_TOUCH;
				}
				else
					newDisplayTimeToLive = gestureSysTickTimer;
				break;

		default:
				break;
	}

	/* Place gesture actions here */
	if(gesture != previousGesture)
	{
		previousGesture = gesture;
	    switch(gesture)
	    {
	        case NO_GESTURE:
//				printf("NO GESTURE\r\n");
	        	cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
	            break;
	        case TOUCHDOWN:
//				printf("TOUCHDOWN\r\n");
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
	            break;
			case CLICK_GESTURE:
				printf("CLICK\r\n");
				GUI_GotoXY(0, OLED_GESTURE_LINE_NUM * fontSizeY);
				GUI_DispString("CLICK");
				GUI_DispCEOL();
				cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
				speedBarValue += CLICK_INCREMENT;
				if(speedBarValue >= MAX_SPEED_BAR_VALUE)
					speedBarValue = MAX_SPEED_BAR_VALUE;
				newDisplayUpdateFlag = true;
				newDisplayTimeToLive = gestureSysTickTimer;
	            break;
	        case SWIPE_GESTURE_FAST_RIGHT:
                printf("FAST SWIPE RIGHT %lu (touchdown value = %lu, time = %d msec)\r\n", (centroid - touchdownCentroidValue), touchdownCentroidValue, gestureSysTickTimer);
                GUI_GotoXY(0, OLED_GESTURE_LINE_NUM * fontSizeY);
                GUI_DispString("FAST_RIGHT");
                GUI_DispCEOL();
                printDisplacement(touchdownCentroidValue, centroid, gesture);
                speedBarValue += SPEED_INCREMENT * 2;
                if(speedBarValue >= MAX_SPEED_BAR_VALUE)
                	speedBarValue = MAX_SPEED_BAR_VALUE;
                newDisplayUpdateFlag = true;
                newDisplayTimeToLive = gestureSysTickTimer;
	            break;
			case SWIPE_GESTURE_SLOW_RIGHT:
                printf("SLOW SWIPE RIGHT %lu (touchdown value = %lu, time = %d msec)\r\n", (centroid - touchdownCentroidValue), touchdownCentroidValue, gestureSysTickTimer);
                GUI_GotoXY(0, OLED_GESTURE_LINE_NUM * fontSizeY);
				GUI_DispString("SLOW_RIGHT");
				GUI_DispCEOL();
                printDisplacement(touchdownCentroidValue, centroid, gesture);
                speedBarValue += SPEED_INCREMENT;
                if(speedBarValue >= MAX_SPEED_BAR_VALUE)
                	speedBarValue = MAX_SPEED_BAR_VALUE;
                newDisplayUpdateFlag = true;
                newDisplayTimeToLive = gestureSysTickTimer;
	            break;
	        case SWIPE_GESTURE_FAST_LEFT:
                printf("FAST SWIPE LEFT %lu (touchdown value = %lu, time = %d msec)\r\n", (touchdownCentroidValue - centroid), touchdownCentroidValue, gestureSysTickTimer);
                GUI_GotoXY(0, OLED_GESTURE_LINE_NUM * fontSizeY);
				GUI_DispString("FAST_LEFT");
				GUI_DispCEOL();
                printDisplacement(touchdownCentroidValue, centroid, gesture);
                speedBarValue -= SPEED_INCREMENT *2;
                if(speedBarValue <= 0)
                	speedBarValue = 0;
                newDisplayUpdateFlag = true;
                newDisplayTimeToLive = gestureSysTickTimer;
	            break;
			case SWIPE_GESTURE_SLOW_LEFT:
                printf("SLOW SWIPE LEFT %lu (touchdown value = %lu, time = %d msec)\r\n", (touchdownCentroidValue - centroid), touchdownCentroidValue, gestureSysTickTimer);
                GUI_GotoXY(0, OLED_GESTURE_LINE_NUM * fontSizeY);
				GUI_DispString("SLOW_LEFT");
				GUI_DispCEOL();
                printDisplacement(touchdownCentroidValue, centroid, gesture);
                speedBarValue -= SPEED_INCREMENT;
                if(speedBarValue <= 0)
                	speedBarValue = 0;
                newDisplayUpdateFlag = true;
                newDisplayTimeToLive = gestureSysTickTimer;
	            break;
			case CLICK_HOLD_DRAG:
				wtfFlag = 1;
				printf("CLICK_HOLD_DRAG %lu (touchdown value = %lu, time = %d msec)\r\n", (touchdownCentroidValue - centroid), touchdownCentroidValue, gestureSysTickTimer);
				GUI_GotoXY(0, OLED_GESTURE_LINE_NUM * fontSizeY);
				GUI_DispString("CLICK_HOLD_DRAG");
				GUI_DispCEOL();
				printDisplacement(touchdownCentroidValue, centroid, gesture);
				cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
				speedBarValue = centroid;
				newDisplayUpdateFlag = true;
				newDisplayTimeToLive = gestureSysTickTimer;
				break;
			default:
				break;
	    }
	}

	if(WAIT_FOR_TOUCH == currentState)
	{
		cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
		wtfFlag = 0;
	}

	#if ENABLE_CENTROID_LOGGING
		/* djjw - 19-Nov-2018 ... adding centroid logging for diagnostics */
		if(WAIT_FOR_TOUCH != currentState )
		{
			touchIsActive = TRUE; /* touch is active, start logging centroids */

			centroidLog[logIndex][0] = centroid;
			centroidLog[logIndex][1] = gestureSysTickTimer;

			logIndex++;
			if(logIndex > MAX_LOG_ENTRIES)
				logIndex = MAX_LOG_ENTRIES;
		}
		else if(TRUE == touchIsActive) /* this marks an end to the currently active touch event ... print log to UART */
		{
			printf("Centroid Log - gesture triggered at scan #%d\n\rScan\tCentroid\tTime(ms)\n\r", gestureTriggeredIndex);
			for(i = 0; i < logIndex; i++)
			{
				printf("%d\t%d\t\t%d", i, centroidLog[i][0], centroidLog[i][1]);
				if(i == gestureTriggeredIndex)
					printf(" *** triggered ***\n\r");
				else
					printf("\n\r");
			}
			printf("End of log - touch was active for %d scans\n\r", logIndex);
			printf("Touch event was %s (%lu) ... new speed is %d\r\n\n", gestureNames[gestureEvent], gestureEvent, speedBarValue);
            touchIsActive = FALSE;
		}
		else /* this is the default/idle "no touch" state ... just keep resetting the Log index counter */
			logIndex = 0;
	#endif /* ENABLE_CENTROID_LOGGING */
}

void printDisplacement(uint32 touchdownCentroidValue, uint32 centroid, GESTURE_TYPE gesture)
{
    if(gesture == SWIPE_GESTURE_FAST_LEFT || gesture == SWIPE_GESTURE_FAST_RIGHT || gesture == SWIPE_GESTURE_SLOW_LEFT || gesture == SWIPE_GESTURE_SLOW_RIGHT || gesture == CLICK_HOLD_DRAG)
    {
		if(centroid > touchdownCentroidValue)
		{
			printf("Displacement = %lu, Centroid at trigger = %lu, Touchdown = %lu \r\n\n", (centroid - touchdownCentroidValue), centroid, touchdownCentroidValue);
		}
		else
		{
			printf("Displacement = %lu, Centroid at trigger = %lu, Touchdown = %lu \r\n\n", (touchdownCentroidValue - centroid), centroid, touchdownCentroidValue);
		}
    }
}

/*******************************************************************************
* Function Name: SysTickISRCallback
********************************************************************************
*
* Summary:
*  This API is called from SysTick timer interrupt handler to update the
*  sysTickCount counter.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void SysTickISRCallback(void)
{
    gestureSysTickTimer++;
}

/* [] END OF FILE */
