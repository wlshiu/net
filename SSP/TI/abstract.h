 ******************** (C) COPYRIGHT 2010 NXPSemiconductors *******************
 * @file    SSP\TI\abstract.txt 
 * @author  NXP MCU SW Application Team
 * @version 2.0
 * @date    
 * @brief   Description of the SSP TI example.
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
		This example describes how to use SSP peripheral with TI frame format.
	Process:
		This example uses two SSP peripherals in TI frame format, one is set 
		as master mode and the other is set as slave mode.
		SSP master/slave configuration:
			- CPHA = 0: data is sampled on the first clock edge of SCK.
			- CPOL = 0: SCK is active high
			- Clock rate = 1MHz
			- DSS = 8: 8 bits per transfer
			- MSTR = 0/1: SSP operates in Slave/Master mode
			- FRF= 1: MicroWire Frame format
		After initialize buffer, SPI master will transfer/receive data to/from SSP slave
		in INTERRUPT mode.
		After transmittion completed, receive and transmit buffer will be compared, if they 
		are not similar, the program will enter infinite loop and a error notice will
		be displayed. 

@Directory contents:
	\EWARM: includes EWARM (IAR) project and configuration files
	\Keil:	includes RVMDK (Keil)project and configuration files 
	
	lpc17xx_libcfg.h: Library configuration file - include needed driver library for this example 
	makefile: Example's makefile (to build with GNU toolchain)
	ti_test.c: Main program

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
				
		SSP Connection:
			- P0.15 <-> P0.7 : SCK
		 	- P0.16 <-> P0.6 : SSEL 
		  	- P0.17 <-> P0.8 : MISO
		  	- P0.18 <-> P0.9 : MOSI
						
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
		- Step 5: Run example, observe SSP transfer result on serial display
		
		(Pls see "LPC17xx Example Description" document - chapter "Examples > SSP > TI"
		for more details)
		
@Tip:
	- Open \EWARM\*.eww project file to run example on IAR
	- Open \RVMDK\*.uvproj project file to run example on Keil