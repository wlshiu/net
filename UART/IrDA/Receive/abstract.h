 ******************** (C) COPYRIGHT 2010 NXPSemiconductors *******************
 * @file    UART\IrDA\Receive\abstract.txt 
 * @author  NXP MCU SW Application Team
 * @version 2.0
 * @date    
 * @brief   Description of the UART IrDA example.
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
		This example, together with the transmit example, describes how to use UART in IrDA mode
	Process:
		UART0/ UART1 configuration:
			� 9600bps 
			� 8 data bit 
			� No parity 
			� 1 stop bit 
			� No flow control 
			- Receive and transmit enable
		UART3 configuration:
			� 9600bps 
			� 8 data bit 
			� No parity 
			� 1 stop bit 
			� No flow control 
			- Enable IrDA mode
		GPIO P2.2-P2.6, P1.28, P1.29, P1.31 are configured as output for display the received byte.
		
		UART will print welcome screen first, then UART3 keep reading the income irda signal and
		output to 8 leds bank the received value.
		
		Note: If using this example to print with UART1, pls add conversion type (LPC_UART_TypeDef *)LPC_UART1
		because UART1 has different structure type
		Ex: UART_Send((LPC_UART_TypeDef *)LPC_UART1, menu1, sizeof(menu1), BLOCKING);
		
@Directory contents:
	\EWARM: includes EWARM (IAR) project and configuration files
	\Keil:	includes RVMDK (Keil)project and configuration files 
	
	lpc17xx_libcfg.h: Library configuration file - include needed driver library for this example 
	makefile: Example's makefile (to build with GNU toolchain)
	uart_irda_receive.c: Main program

@How to run:
	Hardware configuration:		
		This example was tested only on:
			Keil MCB1700 with LPC1768 vers.1
				These jumpers must be configured as following:
				- VDDIO: ON
				- VDDREGS: ON 
				- VBUS: ON
				- LED: ON
				- SPK: OFF
				- Remain jumpers: OFF
				
			UART connection:
				Add in a simple hardware: (1 infrared receive led, 1 150R resistor)
					- An infrared receiver led which has its anode connected to 3.3V, cathode connected
					to a 150R resistor. The other end of this resistor connects to GND.
					- Connect a wire between P0.26, SPK jumper, and this infrared receiver led's cathode.
			
			
	
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
				  Also burn the transmit example hex file to the other board.
		- Step 3: Connect UART0 on this board to COM port on your computer
		- Step 4: Configure hardware and serial display as above instruction 
		- Step 5: Run both examples
				  Point the infrared transmit led to the infrared received led66
				  Observe the 8 leds bank and compare with the value send out by the transmit example.
		
		(Pls see "LPC17xx Example Description" document - chapter "Examples > UART > IrDA - Receive"
		for more details)
		
@Tip:
	- Open \EWARM\*.eww project file to run example on IAR
	- Open \RVMDK\*.uvproj project file to run example on Keil
