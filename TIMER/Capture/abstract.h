 ******************** (C) COPYRIGHT 2010 NXPSemiconductors *******************
 * @file    TIMER\Capture\abstract.txt 
 * @author  NXP MCU SW Application Team
 * @version 2.0
 * @date    
 * @brief   Description of the TIMER Capture example.
 ******************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 ******************************************************************************
  
@Example description:
	Purpose:
		This example describes how to use Capture Timer function.
	Process:
		We use Timer 0 to take a snapshot of the timer value when an input signal 
		on CAP0.0 (pin P1.26) transitions.
		Timer configuration:
			- Prescaler in microsecond value
			- prescaler value = 1000000us = 1s
			- Use channel, CAPn.0
			- Enable capture both on rising and falling edge
			- Generate capture interrupt
		Whenever capture interrupt occurs, TIMER interrupt service routine will be invoke 
		to get captured time and display it into serial display
		Changing connect P1.26 with GND and VCC whenever want to capture time.

@Directory contents:
	\EWARM: includes EWARM (IAR) project and configuration files
	\Keil:	includes RVMDK (Keil)project and configuration files 
	 
	lpc17xx_libcfg.h: Library configuration file - include needed driver library for this example 
	makefile: Example's makefile (to build with GNU toolchain)
	timer_capture.c: Main program

@How to run:
	Hardware configuration:		
		This example was tested on:
			Keil MCB1700 with LPC1768 vers.1
				These jumpers must be configured as following:
				- VDDIO: ON
				- VDDREGS: ON 
				- VBUS: ON
				- Remain jumpers: OFF
			IAR LPC1768 KickStart vers.A
				These jumpers must be configured as following:
				- PWR_SEL: depend on power source
				- DBG_EN : ON
				- Remain jumpers: OFF
				
	Serial display configuration: (e.g: TeraTerm, Hyperterminal, Flash Magic...) 
		� 115200bps 
		� 8 data bit 
		� No parity 
		� 1 stop bit 
		� No flow control 
	
	Running mode:
		This example can run on RAM/ROM mode.
					
		Note: If want to burn hex file to board by using Flash Magic, these jumpers need
		to be connected:
			- MCB1700 with LPC1768 ver.1:
				+ RST: ON
				+ ISP: ON
			- IAR LPC1768 KickStart vers.A:
				+ RST_E: ON
				+ ISP_E: ON
		
		(Please reference "LPC1000 Software Development Toolchain" - chapter 4 "Creating and working with
		LPC1000CMSIS project" for more information)
	
	Step to run:
		- Step 1: Build example.
		- Step 2: Burn hex file into board (if run on ROM mode)
		- Step 3: Connect UART0 on this board to COM port on your computer
		- Step 4: Configure hardware and serial display as above instruction 
		- Step 5: Run example and observe timer capture value on serial display
		Changing connect P1.26 with GND and VCC whenever want to capture time.
		
		(Pls see "LPC17xx Example Description" document - chapter "Examples > TIMER > Capture"
		for more details)
		
@Tip:
	- Open \EWARM\*.eww project file to run example on IAR
	- Open \RVMDK\*.uvproj project file to run example on Keil