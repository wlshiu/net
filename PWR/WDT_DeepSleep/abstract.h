 ******************** (C) COPYRIGHT 2010 NXPSemiconductors *******************
 * @file    PWR\WDT_DeepSleep\abstract.txt 
 * @author  NXP MCU SW Application Team
 * @version 2.0
 * @date    
 * @brief   Description of the WDT DeepSleep example.
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
		This example describes how to enter system in DeepSleep mode and wake up it
		by using WDT Interrupt
	Process:
		WDT setting:
			- Source clock: IRC oscillator
			- Interrupt mode
			- Timeout = 2000000 us = 2s
		Enter system in DeepSleep by using CLKPWR_DeepSleep function (call __WFI())
		After 2s, WDT timeout and generate interrupt to wake up system.
					
@Directory contents:
	\EWARM: includes EWARM (IAR) project and configuration files
	\Keil:	includes RVMDK (Keil)project and configuration files 
	
	lpc17xx_libcfg.h: Library configuration file - include needed driver library for this example 
	makefile: Example's makefile (to build with GNU toolchain)
	wdt_deepsleep.c: Main program

@How to run:
	Hardware configuration:		
		This example was tested only on:
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
				
		Serial display configuration:(e.g: TeraTerm, Hyperterminal, Flash Magic...) 
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
		- Step 5: Run example and observe on serial display
					+ Press '1' to enter system in DeepSleep mode
					+ After 2s, WDT timeout and wakes system up
			
		(Pls see "LPC17xx Example Description" document - chapter "Examples > PWR > WDT_DeepSleep"
		for more details)
		
@Tip:
	- Open \EWARM\*.eww project file to run example on IAR
	- Open \RVMDK\*.uvproj project file to run example on Keil