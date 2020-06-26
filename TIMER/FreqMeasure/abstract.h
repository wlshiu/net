 ******************** (C) COPYRIGHT 2010 NXPSemiconductors *******************
 * @file    TIMER\FreqMeasure\abstract.txt 
 * @author  NXP MCU SW Application Team
 * @version 2.0
 * @date    
 * @brief   Description of the TIMER Match example in generating
 *			2 different frequency signals.
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
		This example describes how to use Timer to measure a signal's frequency.
	Process:
		1) Initialize UART0 and display information menu.
		2) Ask user to input frequency for a test signal.
		   Configure P1.26 as CAP0.0, P0.6 as MAT2.0.
		3) Configure TIMER2 as follow:
			- Prescale register = 1 us.
			- Match register = 1 / ((frequency)*1us*2)
							 = 500000 / freuqency
			- No interrupt, no stop but reset timer counter on match.
		   Configure TIMER0 as follow:
			- Prescale register = 1 us.
			- Capture register: channel 0, capture on rising edge, generate interrupt
			on capture.
		4) Configure TIMER0 ISR:
			- Prepare 5 interrupt times, in which timer counter and prescale are reset,
			for stable.
			- After this 5 interrupt times, get the capture value, raise done flag.
		5) Start TIMER0 and TIMER2.
		   Wait for done flag, then calculate and print out the measure frequency by:
		      one cycle = capture value * 1us (as TIMER0 configuration).
		      signal frequency = 1/one cycle = 1,000,000 / capture value.
		   Press ESC if we intend to test with other frequencies.
		   TIMER0 and TIMER2 are de-initialized.

@Directory contents:
	\EWARM: includes EWARM (IAR) project and configuration files
	\Keil:	includes RVMDK (Keil)project and configuration files 
	 
	lpc17xx_libcfg.h: Library configuration file - include needed driver library for this example 
	makefile: Example's makefile (to build with GNU toolchain)
	freqmeasure.c: Main program

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
		- Step 5: Run example
				  Use a wire connect P0.6 and P1.26
				  Use PC's terminal to select the test signal on P0.26 and see
				  the result measured frequency printed out. 
		(Pls see "LPC17xx Example Description" document - chapter "Examples > TIMER > FreqMeasure"
		for more details)
		
@Tip:
	- Open \EWARM\*.eww project file to run example on IAR
	- Open \RVMDK\*.uvproj project file to run example on Keil