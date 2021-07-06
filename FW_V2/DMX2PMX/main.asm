//
//
//
//		8888888b.  888b     d888 Y88b   d88P  .d8888b.  8888888b.  888b     d888 Y88b   d88P 
//		888  "Y88b 8888b   d8888  Y88b d88P  d88P  Y88b 888   Y88b 8888b   d8888  Y88b d88P  
//		888    888 88888b.d88888   Y88o88P          888 888    888 88888b.d88888   Y88o88P   
//		888    888 888Y88888P888    Y888P         .d88P 888   d88P 888Y88888P888    Y888P    
//		888    888 888 Y888P 888    d888b     .od888P"  8888888P"  888 Y888P 888    d888b    
//		888    888 888  Y8P  888   d88888b   d88P"      888        888  Y8P  888   d88888b   
//		888  .d88P 888   "   888  d88P Y88b  888"       888        888   "   888  d88P Y88b  
//		8888888P"  888       888 d88P   Y88b 888888888  888        888       888 d88P   Y88b 
//
//
//***********************************************************************************************                                                                                
//                                                                                    
//								   DMX512 to Dual PMX Converter                                                                                   
//										   Version: 1.0
//									  Build date: 05.04.2020
//										  Author: Gabs'e
//										File: main.asm
//								   Target MCU: Microchip ATtiny1634
//										Licence: CC BY-NC-SA 
//								PMX Protocoll Description: https://www.claypaky.it/media/documents/Pulsar_RS232-423(PMX)_SerialDataProtocol_EN.pdf
//
//***********************************************************************************************
//
//	Attiny Pinout:
//					ADDRESS SW = 0-7= PORTA
//					PMX1	= UART0	= PB0
//					DMX		= UART1 = PB1
//					PMX2	= USI	= PB2
//					ADDRESS SW	= 8	= PB3
//					PMX CLOCK	= PC0&PC1
//					LED		= LED	= PC2
//
//	Fuse Settings: 
//						EXTENDED:	0xF5
//						HIGH:		0xD4
//						LOW:		0xCF
//					
//
//	Changelog:	
//				V0.0..........Fork from DMX2SGM
//				V1.0..........First Release
//
//
//***********************************************************************************************

.equ F_CPU = 12000000							// CPU Clock
.equ NUM_CH= 24									// Number of PMX Channels per Output. Maximal 120. Keep value as low as possible for faster response. (Updating all 120 channel takes 0,25s in 7 bit mode & up to 0,375s in 8 bit mode)
.equ PMXMODE= 7									// PMX bit mode. Either 7 or 8 bit mode. Use 7 bit mode for ClayPacky Golden Scan 1 exclusively. Use 8 Bit mode for all other fixtures and for a mix of both.


//********************************Registers*********************************

.def DMX_ADDRESS_L	=	R0						// Register for DIP DMX address LSB
.def DMX_ADDRESS_H	=	R1						// Register for DIP DMX address MSB
.def DMX_ADDRESS_L_C=	R2						// Register for DIP DMX address LSB compare value
.def DMX_ADDRESS_H_C=	R3						// Register for DIP DMX address MSB compare value
.def DMX_BYTE_H		=	R4						// Register for last DMX byte number MSB
.def DMX_BYTE_L		=	R5						// Register for last DMX byte number LSB
.def DMX_DATA		=	R6						// Register for current DMX byte
.def PMX1_DATA		=	R7						// Register for PMX1 data RAM address
.def PMX2_DATA		=	R8						// Register for PMX2 data RAM address
.def PMX_8TH_BIT	=	R9						// Register for 8th bit content in 8 bit PMX Mode
.def S_SREG			=	R10						// Register to restore the SREG after Interrupts
.def PMX1_LAST		=	R11						// Register for last changed PMX1 channel
.def PMX2_LAST		=	R12						// Register for last changed PMX2 channel

.def FLAGS			=	R16						// Register for general status flags
.def PMX_FLAGS		=	R17						// Register for PMX status flags
.def PMX1_ADDRESS	=	R18						// Register for PMX1 address
.def PMX2_ADDRESS	=	R19						// Register for PMX2 address
.def PMX1_REFRESH_C	=	R20						// Register for PMX1 refresh counter
.def PMX2_REFRESH_C	=	R21						// Register for PMX2 refresh counter
.def PMX_COUNTER	=	R22						// Register for compared PMX byte counter
.def TEMP_I			=	R23						// Register for temporary values during interrupts
.def TEMP			=	R24						// Register for temporary values in the main loop
.def USI_DAT		=	R25						// Register for USI data
// MAIN_ADRESS		=	X => R26 & R27			// Address pointer for main Loop
// PMX_ADDRESS		=	Y => R28 & R29			// Address pointer for PMX interrupts
// DMX_ADDRESS		=	Z => R30 & R31			// Address pointer for DMX interrupt


//******************************General FLAGS*******************************

.equ DSNB			=	0						// DMX Skip Next Byte
.equ DRSB			=	1						// DMX Read Start Byte
.equ LEDT			=	2						// LED Toggle
.equ USIS			=	3						// USI State


//********************************PMX FLAGS*********************************

.equ UPX1			=	0						// Update PMX1
.equ AOD1			=	1						// Address or Data Byte PMX1
.equ PAGE1			=	2						// Send Page PMX1
.equ BYTE81			=	3						// Send 8th Byte PMX1

.equ UPX2			=	4						// Update PMX2
.equ AOD2			=	5						// Address or Data Byte PMX2
.equ PAGE2			=	6						// Send Page PMX2
.equ BYTE82			=	7						// Send 8th Byte PMX2


//*****************************Interrupt Vectors*****************************

.org 0x0000
	RJMP MAIN									// Jump to MAIN on program start

.org OC1Aaddr
	RJMP TC1CMA									// Jump to TC1CMA on OC1Aaddr interrupt

.org UDRE0addr
	RJMP PMX1_INTERRUPT							// Jump to PMX1_INTERRUPT on UDRE0addr interrupt

.org URXC1addr
	RJMP DMX_INTERRUPT							// Jump to DMX_INTERRUPT on URXC1addr interrupt

.org USI_OVFaddr
	RJMP PMX2_INTERRUPT							// Jump to PMX2_INTERRUPT on USI_OVFaddr interrupt


//*****************************Init Controller******************************
main:

	// Clear work registers
	CLR R0										// Clear register 0
	CLR R1										// Clear register 1
	CLR R2										// Clear register 2
	CLR R3										// Clear register 3
	CLR R4										// Clear register 4
	CLR R5										// Clear register 5
	CLR R6										// Clear register 6
	CLR R7										// Clear register 7
	CLR R8										// Clear register 8
	CLR R9										// Clear register 9
	CLR R10										// Clear register 10
	CLR R11										// Clear register 11
	CLR R12										// Clear register 12
	CLR R13										// Clear register 13
	CLR R14										// Clear register 14
	CLR R15										// Clear register 15
	CLR R16										// Clear register 16
	CLR R17										// Clear register 17
	CLR R18										// Clear register 18
	CLR R19										// Clear register 19
	CLR R20										// Clear register 20
	CLR R21										// Clear register 21
	CLR R22										// Clear register 22
	CLR R23										// Clear register 23
	CLR R24										// Clear register 24
	CLR R25										// Clear register 25
	CLR R26										// Clear register 26
	CLR R27										// Clear register 27
	CLR R28										// Clear register 28
	CLR R29										// Clear register 29
	CLR R30										// Clear register 30
	CLR R31										// Clear register 31

	// Init Stack Pointer
	LDI	TEMP,	LOW(RAMEND)						// Init StackPointer to end of RAM
	OUT	SPL,	TEMP							// Init StackPointer to end of RAM
	LDI	TEMP,	HIGH(RAMEND)					// Init StackPointer to end of RAM
	OUT	SPH,	TEMP							// Init StackPointer to end of RAM

	// Init RAM
	CLR TEMP									// Clear TEMP
	LDI ZL,	LOW(SRAM_START)						// Set Z pointer LSB to Start of RAM
	LDI ZH,	HIGH(SRAM_START)					// Set Z pointer MSB to Start of RAM

clr_ram:
	ST Z+,	TEMP								// Copy TEMP to RAM and increment Z pointer
	CPI ZL,	LOW(RAMEND+1)						// Check if End of RAM reached
		BRNE clr_ram								// Jump to clr_ram if End of RAM was not reached
	CPI ZH,	HIGH(RAMEND+1)						// Check if End of RAM reachedd
		BRNE clr_ram								// Jump to clr_ram if End of RAM was not reached
	

//*****************************Init USI for PMX2****************************
	
	LDI TEMP,	1<<USIOIE | 1<<USIWM0 | 1<<USICS1// Set USI to 3Wire mode, clock from PC1 & interrupt on USI overflow
	OUT USICR,	TEMP							// Set USI to 3Wire mode, clock from PC1 & interrupt on USI overflow
	SER TEMP									// Set USI output to 1
	OUT USIDR,	TEMP							// Set USI output to 1
	LDI TEMP,	1<<USIOIF | 8					// Reset USI Overflow Flag & Set USI Counter to 8 offsetting the PMX Interrupts against each other
	OUT USISR,	TEMP							// Reset USI Overflow Flag & Set USI Counter to 8 offsetting the PMX Interrupts against each other
	SBI DDRB,	PB2								// Set PMX2 pin to output
	LDI TEMP,	NUM_CH							// Load number of channels into TEMP
	MOV PMX2_LAST,	TEMP						// Copy number of channels into PMX2_LAST


//**************************Init USART0 for PMX1****************************

	LDI TEMP,	1<<UCSZ00 | 1<<UCSZ01 | 1<<UMSEL00// Set USART to Synchronous mode & 8 data bits (Synchronous mode allows reuse of the USART clock generator for the USI)
	OUT UCSR0C, TEMP							// Set USART to Synchronous mode & 8 data bits (Synchronous mode allows reuse of the USART clock generator for the USI)
	LDI TEMP,	1<<UDRIE0 | 1<<TXEN0			// Enable USART 0 transmitt & TX complete interrupt
	OUT UCSR0B, TEMP							// Enable USART 0 transmitt & TX complete interrupt	
	LDI TEMP,	HIGH ((F_CPU/(9600*2))-1)		// Set USART 0 baud register MSB to 9600Hz
	OUT UBRR0H, TEMP							// Set USART 0 baud register MSB to 9600Hz
	LDI TEMP,	LOW ((F_CPU/(9600*2))-1)		// Set USART 0 baud register LSB to 9600Hz
	OUT UBRR0L, TEMP							// Set USART 0 baud register LSB to 9600Hz
	SBI DDRC,	PC0								// Enable clock output
	

//****************************Init USART1 for DMX***************************

	LDI TEMP,	1<<RXCIE1 | 1<<RXEN1			// Enable USART 1 recive & RX interrupt
	STS UCSR1B, TEMP							// Enable USART 1 recive & RX interrupt
	LDI TEMP,	1<<USBS1 | 1<<UCSZ11 | 1<<UCSZ10// Set USART 1 to 8 databits & 2 stopbits
	STS UCSR1C,	TEMP							// Set USART 1 to 8 databits & 2 stopbits
	LDI TEMP,	HIGH ((F_CPU/(250000*16))-1)	// Set USART 1 baud register MSB to 250kHz
	STS UBRR1H, TEMP							// Set USART 1 baud register MSB to 250kHz
	LDI TEMP,	LOW ((F_CPU/(250000*16))-1)		// Set USART 1 baud register LSB to 250kHz
	STS UBRR1L, TEMP							// Set USART 1 baud register LSB to 250kHz
	SBI PUEB,	PB1								// Enable DMX pullup


//******************************Init Timer for LED**************************

	LDI TEMP,	1<<OCIE1A						// Enable Timer/Counter1 Compare Match A interrupt
	OUT TIMSK,	TEMP							// Enable Timer/Counter1 Compare Match A interrupt	
	LDI TEMP,	HIGH (((F_CPU/1024)/33)-1)		// Set Timer/Counter1 Compare Register A MSB to about 30ms (33Hz)
	STS OCR1AH,	TEMP							// Set Timer/Counter1 Compare Register A MSB to about 30ms (33Hz)		
	LDI TEMP,	LOW (((F_CPU/1024)/33)-1)		// Set Timer/Counter1 Compare Register A LSB to about 30ms (33Hz)
	STS OCR1AL,	TEMP							// Set Timer/Counter1 Compare Register A LSB to about 30ms (33Hz)
	CLR TEMP									// Reset Timer/Counter1
	STS TCNT1L, TEMP							// Reset Timer/Counter1
	STS TCNT1H, TEMP							// Reset Timer/Counter1
	STS TCCR1A, TEMP							// Reset Timer/Counter1 Control Register A
	LDI TEMP,	1<<WGM12 | 1<<CS10 | 1<<CS12	// Set Timer/Counter1 Control Register B to Clear Timer on Compare A & Clock/1024
	STS TCCR1B, TEMP							// Set Timer/Counter1 Control Register B to Clear Timer on Compare A & Clock/1024
	SBI DDRC,	PC2								// Set LED pin to output
	SBI PORTC,	PC2								// Turn LED ON


//******************************General Init********************************
	
	LDI TEMP,	1<<WDE							// Enable Watchdog Timer @ 16ms
	OUT WDTCSR,	TEMP							// Enable Watchdog Timer @ 16ms
	LDI TEMP,	0x00							// Set PortA to input 
	OUT DDRA,	TEMP							// Set PortA to input 
	LDI TEMP,	0xFF							// Enable PortA pullup
	OUT PUEA,	TEMP							// Enable PortA pullup
	CBI DDRB,	PB3								// Set pin PB3 to input
	SBI PUEB,	PB3								// Enable pin PB3 pullup
	SER TEMP									// Load TEMP with value for XOR
	IN DMX_ADDRESS_L,	PINB					// Read DMX address bit 8 from DIP switch
	EOR DMX_ADDRESS_L,	TEMP					// Invert register
	BST DMX_ADDRESS_L,	PB3						// Copy DMX address PB3 to T
	BLD DMX_ADDRESS_H,	0						// Copy T to DMX_ADDRESS_H bit 0
	IN DMX_ADDRESS_L,	PINA					// Read DMX address bit 0-7 from DIP switch
	EOR DMX_ADDRESS_L,	TEMP					// Invert register
	LDI XH,	0x02								// Set Main address MSB to 2
	LDI YH, 0x01								// Set PMX address MSB to 1
	LDI ZH, 0x01								// Set DMX address MSB to 1

	.if PMXMODE == 8
	LDI TEMP,	0x40							// Set Bit 6
	MOV PMX_8TH_BIT,	TEMP					// Copy Bit 6 and 7 into PMX_8TH_BIT
	.endif

	SEI											// Enable global interrupts


//********************************main loop*********************************
loop:

		// Check if PMX1 needs new data
		SBRC PMX_FLAGS,	UPX1					// Check if PMX1 update flag is clear
			RJMP PMX2								// Jump to PMX2 if flag is clear
		CLR PMX_COUNTER							// Clear PMX_COUNTER
		MOV XL,	PMX1_LAST						// Copy last changed PMX 1 channel into XL

	PMX1_CHECK:
		// Check if all channels have been checked already. Refresh data if no channel has changed. 
		INC PMX_COUNTER							// Increment PMX_COUNTER
		CPI PMX_COUNTER, NUM_CH+1				// Check if all channels have been checked for update
			BREQ PMX1_UPDATE						// Jump to PMX1_UPDATE if all channels have been checked, but no channel has changed

		// Reset if XL is over NUM_CH
		CPI XL, NUM_CH-1						// Check if NUM_CH is reached (overflow)
			BRNE PMX1_CONT							// Jump to PMX1_CONT if not reached
		LDI XL, 0xFF							// Clear XL if reached

	PMX1_CONT:
		// Check if data has changed. Jump back if no change has occoured. Write change into RAM @0x0200 + PMX1 Address. 
		INC XL									// Increment XL
		DEC XH									// Decrement pointer to memmory bank 0x01xx
		LD PMX1_ADDRESS,	X					// Load compare value from RAM
		INC XH									// Increment pointer to memmory bank 0x02xx
		LD TEMP,	X							// Load data from RAM	
		CP TEMP,	PMX1_ADDRESS				// Compare data bytes
			BREQ PMX1_CHECK							// Jump to PMX1_CHECK if Data is equal
		ST X,	PMX1_ADDRESS					// Store data back into RAM for next compare in data is not equal
		MOV PMX1_ADDRESS,	XL					// Send page address
		MOV PMX1_DATA,	XL						// Copy address to PMX1_DATA
		MOV PMX1_LAST,	XL						// Copy last transmit address into PMX1_LAST
		SBR  PMX_FLAGS,	1<<UPX1					// Set Update PMX1 flag
		RJMP PMX2								// Jump to PMX2
		
	PMX1_UPDATE:
		// Check if all channels have been refreshed, and send page
		INC PMX1_REFRESH_C						// Increment PMX1_REFRESH_C
		CPI PMX1_REFRESH_C,	NUM_CH				// Compare if all channels have been updated
			BREQ PMX1_PAGE							// Jump to PMX1_PAGE if all channels have been refreshed, and sending the page is pending
			BRLO PMX1_REFRESH						// Jump to PMX1_REFRESH if not every channel has been updated
		CLR PMX1_REFRESH_C						// Clear PMX1_REFRESH_C

	PMX1_REFRESH:
		// Send DataByte
		MOV PMX1_ADDRESS,	PMX1_REFRESH_C		// Copy PMX1_REFRESH_C to PMX1_ADDRESS
		MOV PMX1_DATA,	PMX1_REFRESH_C			// Copy PMX1_REFRESH_C to PMX1_DATA
		SBR  PMX_FLAGS, 1<<UPX1					// Set Update PMX1 flag
		RJMP PMX2								// Jump to PMX2

	PMX1_PAGE:
		// Send Page
		SBR PMX_FLAGS,	1<<UPX1 | 1<<PAGE1		// Set Update PMX1 and Page PMX1 flag


	PMX2:
		// Check if PMX2 needs new data
		SBRC PMX_FLAGS,	UPX2					// Check if PMX2 update flag is clear
			RJMP CONTINUE							// Jump to CONTINUE if flag is clear
		CLR PMX_COUNTER							// Clear PMX_COUNTER
		MOV XL,	PMX2_LAST						// Copy last changed PMX 2 channel into XL

	PMX2_CHECK:
		// Check if all channels have been checked already. Refresh data if no channel has changed. 
		INC PMX_COUNTER							// Increment PMX_COUNTER
		CPI PMX_COUNTER,	NUM_CH+1			// Check if all channels have been checked for update
			BREQ PMX2_UPDATE						// Jump to PMX2_UPDATE if all channels have been checked, but no channel has changed

		// Reset if XL is over NUM_CH*2
		CPI XL,	NUM_CH*2-1						// Check if NUM_CH is reached (overflow)
			BRNE PMX2_CONT							// Jump to PMX2_CONT if not reached
		LDI XL, NUM_CH-1							// Reset XL to NUM_CH if reached

	PMX2_CONT:
		// Check if data has changed. jump back if no change has occoured. Write change into RAM @0x0200 + PMX1 Address. 
		INC XL									// Increment XL
		DEC XH									// Decrement pointer to memmory bank 0x01xx
		LD PMX2_ADDRESS,	X					// Load compare value from RAM
		INC XH									// Increment Pointer to memmory bank 0x02xx
		LD TEMP,	X							// Load data from RAM	
		CP TEMP,	PMX2_ADDRESS				// Compare data bytes
			BREQ PMX2_CHECK							// Jump to PMX2_CHECK if data is equal
		ST X,	PMX2_ADDRESS					// Store data back into RAM for next compare
		MOV PMX2_ADDRESS,	XL					// Send page address
		MOV PMX2_DATA,	XL						// Copy address to PMX2_DATA
		MOV PMX2_LAST,	XL						// Copy last transmit address into PMX2_LAST
		SBR  PMX_FLAGS, 1<<UPX2					// Set Update PMX2 flag
		RJMP CONTINUE							// Jump to CONTINUE
		
	PMX2_UPDATE:
		// Check if all channels have been refreshed, and send page
		INC PMX2_REFRESH_C						// Increment PMX2_REFRESH_C
		CPI PMX2_REFRESH_C,	NUM_CH*2			// Compare if all channels have been updated
			BREQ PMX2_PAGE							// Jump to PMX2_PAGE if all channels have been refreshed, and sending the page is pending
			BRLO PMX2_REFRESH						// Jump to PMX2_REFRESH if not every channel has been updated
		LDI PMX2_REFRESH_C,	NUM_CH				// Clear PMX2_REFRESH_C and continue

	PMX2_REFRESH:
		// Send DataByte
		MOV PMX2_ADDRESS,	PMX2_REFRESH_C		// Copy PMX2_REFRESH_C to PMX2_ADDRESS
		MOV PMX2_DATA,		PMX2_REFRESH_C		// Copy PMX2_ADDRESS to PMX2_DATA
		SBR  PMX_FLAGS,		1<<UPX2				// Set Update PMX2 flag
		RJMP CONTINUE							// Jump to CONTINUE

	PMX2_PAGE:
		// Send Page
		SBR PMX_FLAGS,	1<<UPX2 | 1<<PAGE2		// Set Update PMX2 and Page PMX2 flag


	CONTINUE:
		// Check if DIP switch address has changed, and force a device reset by not reseting the watchdog if address has changed.
		SER TEMP								// Load TEMP with value for XOR
		IN DMX_ADDRESS_L_C,	PINB				// Read DMX address bit 8 from DIP switch
		EOR DMX_ADDRESS_L_C,	TEMP			// Invert register
		BST DMX_ADDRESS_L_C,	PB3				// Copy DMX address PB3 to T
		BLD DMX_ADDRESS_H_C,	0				// Copy T to DMX_ADDRESS_H_C bit 0
		IN DMX_ADDRESS_L_C,	PINA				// Read DMX address bit 0-7 from DIP switch
		EOR DMX_ADDRESS_L_C,	TEMP			// Invert register
		CPSE DMX_ADDRESS_L_C,	DMX_ADDRESS_L	// Compare new address LSB with set address
			RJMP loop								// Jump back to loop if address LSB has changed, forcing a device reset
		CPSE DMX_ADDRESS_H_C,	DMX_ADDRESS_H	// Compare new address MSB with set address
			RJMP loop								// Jump back to loop if address MSB has changed, forcing a device reset
		WDR										// Reset watchdog timer
		RJMP loop								// Jump back to loop


//****************Timer/Counter1 Compare Match A Interrupt *****************
TC1CMA:
	
		// Check if DMX has updated since the last Interrupt, and flash the LED if update has ocoured. Turn the LED ON otherwise.	
		IN S_SREG,	SREG						// Save SREG
		SBI PINC,	PC2							// Toggle LED
		SBRS FLAGS,	LEDT						// Check DMX status for LED toggle
			SBI PORTC, PC2							// Turn LED ON
		CBR FLAGS,	1<<LEDT						// Clear LEDT flag
		OUT SREG,	S_SREG						// Restore SREG
		RETI									// Exit interrupt


//***************************DMX Input Interrupt****************************
DMX_INTERRUPT:

		// Get Data from the UART, check if special DMX condition has ocoured. Save the recived DMX Data to RAM starting from 0x0100 with the first DMX channel selected by the DIP Switch. Toggle the LED if valid DMX Frames are recived.
		LDS TEMP_I,	UCSR1A						// Load data from UCSR1A to TEMP_I register
		LDS	DMX_DATA,	UDR1					// Load data from USART data register to DMX_DATA register
		IN S_SREG,	SREG						// Save SREG
		SBRC TEMP_I,	DOR1					// Check dataoverrun flag
			RJMP SKIPNEXT							// Jump to SKIPNEXT if dataoverrunn occoured (drop current DMX frame, as dataoverruns should never happen)
		SBRC TEMP_I,	FE1						// Chek frameerror flag
			RJMP FRAMEERROR							// Jump to FRAMEERROR if frameerror occoured (marks the beginning of a new DMX frame) 
		SBRS FLAGS,	DSNB						// Check DMX status for skip next flag
			RJMP SKIPNEXT							// Jump to SKIPNEXT
		SBRC FLAGS,	DRSB						// Check DMX status for startbyte
			RJMP	STARTBYTE						// Jump to STARTBYTE
		SBR FLAGS,	1<<LEDT						// Set flag to toggle the LED
		CP DMX_BYTE_L,	DMX_ADDRESS_L			// Check DMX address MSB
		CPC DMX_BYTE_H,	DMX_ADDRESS_H			// Check DMX address LSB
			BRLO ADDRESSLOW							// Jump to ADDRESSLOW if DMX byte is below DMX address 
		SBRC DMX_BYTE_H,	1					// Check if all 512 bytes have been recived
			RJMP SKIPNEXT							// Jump to SKIPNEXT if all 512 bytes have been recived. (should never happen to recive more than 512 bytes)
		CPI ZL,	NUM_CH*2						// Check if all used channels have been recived
			BREQ ADDRESSOVER						// Jump to ADDRESSOVER if all necessarry channels have been recived
	.if PMXMODE == 7						// If 7 Bit mode active
		LSR DMX_DATA							// Drop the last databit if device is running in 7 bit mode to avoid unecesarry PMX updates if only bit 0 changed
	.endif
		ST Z+, DMX_DATA								// Copy DMX_DATA to RAM and increment Z pointer

	ADDRESSLOW:
		// Increment the DMX_BYTE
		INC DMX_BYTE_L							// Increment DMXBYTE L by 1
			BRNE		NINCDMXH						// Jump to NINCDMXH when DMX_BYTE_L hasn't overflown
		INC DMX_BYTE_H							// Increment DMXBYTE H by 1
	NINCDMXH:
		OUT SREG,	S_SREG						// Restore SREG
		RETI									// Exit Interrupt

	SKIPNEXT:
		// Drop recived data until next DMX Frame starts
		CBR FLAGS,	1<<DSNB						// Clear FLAGSATUS to skip til next frameerror
	ADDRESSOVER:
		OUT SREG,	S_SREG						// Restore SREG
		RETI									// Exit interrupt

	FRAMEERROR:
		// Reset X Pointer & DMX_BYTE counter. Set DMX status to check the startbyte on the next interrupt
		LDI ZL,	0x00							// Reset DMXADRESS LSB
		LDI ZH, 0x01							// Reset DMXADRESS MSB
		CLR DMX_BYTE_L							// Reset DMXBYTE LSB
		CLR DMX_BYTE_H							// Reset DMXBYTE MSB
		SBR FLAGS,	1<<DRSB	| 1<<DSNB			// Set DMX Skip Next Byte & DMX Read Start Byte flag to check startbyte on next interrupt
		OUT SREG,	S_SREG						// Restore SREG
		RETI									// Exit Interrupt

	STARTBYTE:
		// Check if recived startbit is valid (zero). Otherwise drop the incoming DMX Frame.
		CPSE DMX_DATA,	DMX_BYTE_H				// Check startbit
			RJMP SKIPNEXT							// Jump to SKIPNEXT if startbit is not 0
		CBR FLAGS,	1<<DRSB						// Clear DMX Read Start Byte flag
		OUT SREG,	S_SREG						// Restore SREG
		RETI									// Exit interrupt
		
			
//**************************PMX1 Output Interrupt***************************
PMX1_INTERRUPT:

		// Check if in address, data or page phase of PMX transfer
		IN S_SREG,	SREG						// Save SREG
		SBRC PMX_FLAGS,	AOD1					// Check if address flag is set 
			RJMP PMX1_DATA_SEND						// Jump to PMX1_DATA if flag is set
		SBR PMX_FLAGS,	1<<AOD1					// Set Address or Data Byte PMX1 flag
		SBR PMX1_ADDRESS, 1<<7					// Set address bit
		SBRC PMX_FLAGS,	PAGE1					// Check if page flag is set 
			LDI PMX1_ADDRESS,	249					// Send page command
		OUT UDR0, PMX1_ADDRESS					// Output PMX1_ADDRESS
		OUT SREG, S_SREG						// Restore SREG
		RETI									// Exit interrupt

	PMX1_DATA_SEND:
		// Send PMX Data. In 8 bit mode databit 0 is checked and a additional databit is transmitted in the next interrupt if bit was HIGH
	.if PMXMODE == 7						// If 7 bit mode active
		MOV YL,	PMX1_DATA						// Copy RAM address into address pointer
		LD PMX1_DATA,	Y						// Get data from RAM
		SBRC PMX_FLAGS,	PAGE1					// Check if PMX1 page flag is set 
			CLR PMX1_DATA							// Send Page 0
		CBR PMX_FLAGS,	1<<AOD1 | 1<<UPX1 |	1<<PAGE1// Clear Address or Data Byte PMX1, Update PMX1 and Send Page PMX1 flag
		OUT UDR0,	PMX1_DATA					// Output PMX1_DATA
		OUT SREG,	S_SREG						// Restore SREG
		RETI									// Exit interrupt

	.else									// If 8 bit mode active
		SBRC PMX_FLAGS,	BYTE81					// Check If 8th bit flag is set 
			RJMP PMX1_8THBIT_DATA_SEND				// Jump to PMX1_DATA if flag is set
		MOV YL,	PMX1_DATA						// Copy RAM address into address pointer
		LD PMX1_DATA,	Y						// Get data from RAM
		SBRC PMX_FLAGS,	PAGE1					// Check if page flag is set 
			CLR PMX1_DATA						// Send page 0
		CBR PMX_FLAGS,	1<<AOD1 | 1<<UPX1 | 1<<PAGE1// Clear Address or Data Byte PMX1, Update PMX1 and Send Page PMX1 flag
		SBRC PMX1_DATA,	0						// Check if bit0 is set 
			SBR PMX_FLAGS, 1<<AOD1 | 1<<UPX1 | 1<<BYTE81// Set Address or Data Byte PMX1, Update PMX1 and Send 8th Byte PMX1 flag
		LSR PMX1_DATA							// Drop the last databit
		OUT UDR0,	PMX1_DATA					// Output PMX1_DATA
		OUT SREG,	S_SREG						// Restore SREG
		RETI									// Exit interrupt
	PMX1_8THBIT_DATA_SEND: 
		CBR PMX_FLAGS, 1<<AOD1 | 1<<UPX1 | 1<<BYTE81// Clear Address or Data Byte PMX1, Update PMX1 and Send 8th Byte PMX1 flag
		OUT UDR0,	PMX_8TH_BIT					// Output PMX1_DATA
		OUT SREG,	S_SREG						// Restore SREG
		RETI									// Exit interrupt

	.endif


//**************************PMX2 Output Interrupt***************************
PMX2_INTERRUPT:

		// Check if in first or second part of UART transfer
		SBRC FLAGS,	USIS						// Check USI State
			RJMP USISEND2							// Jump to USISEND2 if flag is set

		// Check if in address, data or page phase of PMX transfer
		IN S_SREG,	SREG						// Save SREG
		SBRC PMX_FLAGS,	AOD2					// Check if address flag is set 
			RJMP PMX2_DATA_SEND						// Jump to PMX2_DATA if flag is set
		SBR PMX_FLAGS,	1<<AOD2					// Set Address or Data Byte PMX2 flag
		SBR PMX2_ADDRESS,	1<<7				// Set address bit
		SUBI PMX2_ADDRESS,	NUM_CH				// Subtract channel offset
		SBRC PMX_FLAGS,	PAGE2					// Check if page flag is set 
			LDI PMX2_ADDRESS,	249					// Send page command
		MOV TEMP_I,	PMX2_ADDRESS				// Copy data to TEMP_I for output
		RJMP USISEND1							// Jump to USISEND1

	PMX2_DATA_SEND:
		// Send PMX Data. In 8 bit mode databit 0 is checked and a additional databit is transmitted in the next interrupt if bit was HIGH
	.if PMXMODE == 7						// If 7 bit mode active
		MOV YL,	PMX2_DATA						// Copy RAM address into address pointer
		LD PMX2_DATA,	Y						// Get data from RAM
		SBRC PMX_FLAGS,	PAGE2					// Check if PMX2 page flag is set 
			CLR PMX2_DATA							// Send Page 0
		CBR PMX_FLAGS,	1<<AOD2 | 1<<UPX2 |	1<<PAGE2// Clear Address or Data Byte PMX2, Update PMX2 and Send Page PMX2 flag
		MOV TEMP_I, 	PMX2_DATA				// Copy data to TEMP_I for output
		RJMP USISEND1							// Jump to USISEND1


	.else									// If 8 bit mode active
		SBRC PMX_FLAGS,	BYTE82					// Check if 8th bit flag is set 
			RJMP PMX2_8THBIT_DATA_SEND				// Jump to PMX2_DATA if flag is set
		MOV YL,	PMX2_DATA						// Copy RAM address into address pointer
		LD PMX2_DATA,	Y						// Get data from RAM
		SBRC PMX_FLAGS,	PAGE2					// Check if page flag is set 
			CLR PMX2_DATA							// Send page 0
		CBR PMX_FLAGS,	1<<AOD2 | 1<<UPX2 | 1<<PAGE2// Clear Address or Data Byte PMX2, Update PMX2 and Send Page PMX2 flag
		SBRC PMX2_DATA,	0						// Check if bit0 is set 
			SBR PMX_FLAGS,	1<<AOD2 | 1<<UPX2 | 1<<BYTE82// Set Address or Data Byte PMX2, Update PMX2 and Send 8th Byte PMX2 flag
		LSR PMX2_DATA							// Drop the last databit
		MOV TEMP_I,	PMX2_DATA					// Copy data to TEMP_I for output
		RJMP USISEND1							// Jump to USISEND1
	PMX2_8THBIT_DATA_SEND:
		CBR PMX_FLAGS,	1<<AOD2 | 1<<UPX2 | 1<<BYTE82// Clear Address or Data Byte PMX2, Update PMX2 and Send 8th Byte PMX2 flag
		MOV TEMP_I,	PMX_8TH_BIT					// Copy data to TEMP_I for output

	.endif


	USISEND1:
		// Load the startbit, add databits 0-5 in flipped bitorder and output it to the USI. Prepare the stopbit and databit 5-7 in flipped bit order. Set the USI to output 6 bits
		CLR USI_DAT									// Clear USI_DAT
		SBR FLAGS,	1<<USIS							// Set USI sate flag
		BST TEMP_I,	0								// Copy TEMP_I bit 0 to T
		BLD USI_DAT,	6							// Copy T to USI_DAT bit 6
		BST TEMP_I,	1								// Copy TEMP_I bit 1 to T
		BLD USI_DAT,	5							// Copy T to USI_DAT bit 5
		BST TEMP_I,	2								// Copy TEMP_I bit 2 to T
		BLD USI_DAT,	4							// Copy T to USI_DAT bit 4
		BST TEMP_I,	3								// Copy TEMP_I bit 3 to T
		BLD USI_DAT,	3							// Copy T to USI_DAT bit 3
		BST TEMP_I,	4								// Copy TEMP_I bit 4 to T
		BLD USI_DAT,	2							// Copy T to USI_DAT bit 2
		BST TEMP_I,	5								// Copy TEMP_I bit 5 to T
		BLD USI_DAT,	1							// Copy T to USI_DAT bit 1
		OUT USIDR,	USI_DAT							// Output USI_DAT to USIDR
		LDI USI_DAT,	1<<USIOIF | 4				// Reset USI Overflow Flag & Set USI Counter to 4
		OUT USISR,	USI_DAT							// Reset USI Overflow Flag & Set USI Counter to 4
		LDI USI_DAT,	0x10						// Load Stopbit into USI_DAT
		BLD USI_DAT,	7							// Copy T to USI_DAT bit 7
		BST TEMP_I,	6								// opy TEMP_I bit 6 to T
		BLD USI_DAT,	6							// Copy T to USI_DAT bit 6
		BST TEMP_I,	7								// Copy TEMP_I bit 7 to T
		BLD USI_DAT,	5							// Copy T to USI_DAT bit 5
		OUT SREG,	S_SREG							// Restore SREG
		RETI										// Exit interrupt	
	
	
	USISEND2:
		// Output prepared data to the USI. Set the USI to output 6 bits if USI interrupt was on a rising clock edge. Add a extra half bit otherwise to correct the clockpase.
		CBR FLAGS,	1<<USIS							// Reset USI state flag
		OUT USIDR,	USI_DAT							// Output USI_DAT to USIDR
		LDI USI_DAT,	1<<USIOIF | 8				// Reset USI Overflow Flag & Set USI Counter to 8
		SBIS PINC,	PC1								// Check clock phase
			LDI USI_DAT,	1<<USIOIF | 9				// Correct clock phase (Interrupt on rising edge, output on falling edge)
		OUT USISR,	USI_DAT							// Reset USI Overflow Flag & Set USI Counter to 8/9
		OUT SREG,	S_SREG							// Restore SREG
		RETI										// Exit interrupt


//******************************Programm End *******************************

.org FLASHEND-9
.db "DMX2PMX V1 by Gabs'e"

.if NUM_CH > 120 | NUM_CH < 1
.error "Maximal 120 Cannels per Output Allowed!"
.endif

.if PMXMODE == 7
.message "Assembling in 7 Bit Mode."

.elif PMXMODE == 8
.message "Assembling in 8 Bit Mode."

.else
.error "Invalid PMX Mode! Check parameter!"

.endif


//********************************File End *********************************