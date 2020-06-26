 ******************** (C) COPYRIGHT 2010 NXPSemiconductors *******************
 * @file    SPI\LoopBack\abstract.txt 
 * @author  NXP MCU SW Application Team
 * @version 2.0
 * @date    
 * @brief   Description of the SPI Loopback example.
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
		This example describes how to test SPI operation by using loop-back mode
	Process:
		SPI configuration:
			- CPHA = 1: data is sampled on the second clock edge of SCK.
			- CPOL = 1: SCK is active low
			- Clock rate = 2MHz
			- LSBF = 0: SPI data is transferred MSB first 
			- BITS = 10: 10 bits per transfer
			- MSTR = 1: SPI operates in Master mode
		After initialize buffer, SPI will transfer data to itself (loop-back mode)
		by calling 'SPI_ReadWrite( )' function and use POLLING mode to send/receive data.
		After transmittion completed, receive and transmit buffer will be compare, it they 
		are not similare, the program will be enter infinite loop and a error notice will
		be displayed.  

@Directory contents:
	\EWARM: includes EWARM (IAR) project and configuration files
	\Keil:	includes RVMDK (Keil)project and configuration files 
	
	lpc17xx_libcfg.h: Library configuration file - include needed driver library for this example 
	makefile: Example's makefile (to build with GNU toolchain)
	spi_loopback_test.c: Main program

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
				
		SPI connection:
	 	 	- P0.17 - MISO
	  		- P0.18 - MOSI
	  		MOSI pin must be cross-connected with MISO pin (loop-back mode)	
	  			
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
		- Step 5: Run example and observe SPI transfer result from serial display
		
		(Pls see "LPC17xx Example Description" document - chapter "Examples > SPI > LoopBack" 
		for more details)
		
@Tip:
	- Open \EWARM\*.eww project file to run example on IAR
	- Open \RVMDK\*.uvproj project file to run example on Keil