 ******************** (C) COPYRIGHT 2010 NXPSemiconductors *******************
 * @file    SysTick\10ms_base\abstract.txt 
 * @author  NXP MCU SW Application Team
 * @version 2.0
 * @date    
 * @brief   Description of the SysTick base 10ms example.
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
		This example describes how to configure System Tick timer to generate 
		interrupt each 10ms
	Process:
		In this example, System Tick timer is clocked internal by the CPU clock
		In this case, CPU clock = cclk = 100MHz
		System Tick timer configure:
			- time interval = 10ms
			- Enable System Tick interrupt
		After each 10ms, System Tick will generate interrupt, interrupt service routine
		'SysTick_Handler( )' will be invoke and toggle P0.0 pin.
		Use oscilloscope to observe signal on P0.0 and measure time between falling and rising 
		edge, it would be: 10ms.
					
@Directory contents:
	\EWARM: includes EWARM (IAR) project and configuration files
	\Keil:	includes RVMDK (Keil)project and configuration files 
	 
	lpc17xx_libcfg.h: Library configuration file - include needed driver library for this example 
	makefile: Example's makefile (to build with GNU toolchain)
	10ms_base.c: Main program

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
		- Step 3: Configure hardware as above instruction 
		- Step 4: Run example, observe System Tick operation via P0.0 signal
			
		(Pls see "LPC17xx Example Description" document - chapter "Examples > SysTick > 10ms_base"
		for more details)
		
@Tip:
	- Open \EWARM\*.eww project file to run example on IAR
	- Open \RVMDK\*.uvproj project file to run example on Keil