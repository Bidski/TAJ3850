#include <asf.h>
#include <string.h>
#include "uart.h"
#include "main.h"
#include "ui.h"
#include "RAM.h"

/********************************************
 *				CONSTANTS					*
 ********************************************/
#define OFF							0
#define ON							1
#define FALSE						0
#define TRUE						1

#define NUM_BUSES					0x04																												// Number of motor buses plus the USB bus.
#define NUM_IDS_PER_BUS				0x0F																												// Arbitrary maximum for the number of IDs per motor bus.
#define BUS_1_MOTORS				0x00
#define BUS_2_MOTORS				0x01
#define BUS_3_MOTORS				0x02
//#define BUS_4_MOTORS				0x03
//#define BUS_5_MOTORS				0x04
#define BUS_6_USB					0x03

#define MAX_PARAMETERS				0x7F

/********************************************
 *			TYPES AND ENUMS					*
 ********************************************/
enum ERROR
{
	NO_ERROR			= 0x00,
	INPUT_VOLTAGE_ERROR	= 0x01,
	ANGLE_LIMIT_ERROR	= 0x02,
	OVERHEATING_ERROR	= 0x04,
	RANGE_ERROR			= 0x08,
	CHECKSUM_ERROR		= 0x10,
	OVERLOAD_ERROR		= 0x20,
	INSTRUCTION_ERROR	= 0x40
};

enum INSTRUCTION
{
	PING				= 0x01,
	READ				= 0x02,
	WRITE				= 0x03,
	REG_WRITE			= 0x04,
	ACTION				= 0x05,
	RESET				= 0x06,
	SYNC_WRITE			= 0x83,
	BULK_READ			= 0x92
};

typedef union
{
	uint8_t			packet[MAX_PARAMETERS + 6];
	
	struct
	{
		uint16_t	nPreamble;
		uint8_t		nID;																																// ID of the device to send the message to.
		uint8_t		nLength;																															// Length of the message after this point.
		uint8_t		nInstruction;																														// Instruction to perform. Or returned error if this is a response.
		uint8_t		nParameters[MAX_PARAMETERS];																										// The parameters for the message. May be none. Upper limit of MAX_PARAMETERS.
		uint8_t		nChecksum;																															// Check Sum = ~(0xFF & (ID + Length + Error + Parameter1 + ... + Parameter N))
	} INSTRUCTION_PACKET;
} PACKET;

/********************************************
 *			FUNCTION PROTOTYPES				*
 ********************************************/
void	initRAM(void);
uint8_t	calculateChecksum(const PACKET packet);
uint8_t isValidInstruction(uint8_t instruction);

/********************************************
 *				GLOBALS						*
 ********************************************/
__attribute__((__section__(".userpage"))) uint8_t RAM[RAM_TABLE_SIZE];																					// Store RAM table in flash user page.

static usart_options_t			usart_options;

static PACKET					txCircBuffer[NUM_BUSES][NUM_IDS_PER_BUS] = {[0 ... (NUM_BUSES - 1)][0 ... (NUM_IDS_PER_BUS -  1)].packet = {0}};		// Transmit circular buffers.
static PACKET					rxCircBuffer[NUM_BUSES][NUM_IDS_PER_BUS] = {[0 ... (NUM_BUSES - 1)][0 ... (NUM_IDS_PER_BUS -  1)].packet = {0}};		// Receive circular buffers.
static uint8_t					txHead[NUM_BUSES] = {0};																								// Pointer to the head of each transmit circular buffer.
static uint8_t					txTail[NUM_BUSES] = {0};																								// Pointer to the tail of each transmit circular buffer.
static uint16_t					txPosition[NUM_BUSES] = {0};																							// Pointer to the position within the head of each transmit circular buffer.
static uint8_t					rxHead[NUM_BUSES] = {0};																								// Pointer to the head of each receive circular buffer.
static uint8_t					rxTail[NUM_BUSES] = {0};																								// Pointer to the tail of each receive circular buffer.
static uint16_t					rxPosition[NUM_BUSES] = {0};																							// Pointer to the position within the head of each receive circular buffer.
static volatile avr32_usart_t	*BUS[NUM_BUSES] = {0};																									// Handle to each USART device.

uint8_t					readUSB = 0;
uint8_t					triggerUSART = 0;



void initRAM(void)
{
	uint8_t temp[RAM_TABLE_SIZE] = {MODEL_NUMBER_L_DEFAULT, MODEL_NUMBER_H_DEFAULT, FIRMWARE_VERSION_DEFAULT, DYNAMIXEL_ID_DEFAULT, BAUD_RATE_DEFAULT};
		
	// Populate RAM with default values.
	flashc_memset((void *)RAM, 0x00, 8, RAM_TABLE_SIZE, true);
	flashc_memcpy((void *)RAM, temp, RAM_TABLE_SIZE, true);
	
	BUS[0] = ((avr32_usart_t*)AVR32_USART0_ADDRESS);
	BUS[1] = ((avr32_usart_t*)AVR32_USART1_ADDRESS);
	BUS[2] = ((avr32_usart_t*)AVR32_USART2_ADDRESS);
}

uint8_t isValidInstruction(uint8_t instruction)
{
	switch (instruction)
	{
		case PING:
		case READ:
		case WRITE:
		case REG_WRITE:
		case ACTION:
		case RESET:
		case SYNC_WRITE:
		case BULK_READ:
		{
			return(NO_ERROR);
		}
		
		default:
		{
			return(INSTRUCTION_ERROR);
		}
	}
}

uint8_t calculateChecksum(const PACKET packet)
{
	uint32_t checksum = packet.INSTRUCTION_PACKET.nID + packet.INSTRUCTION_PACKET.nLength + packet.INSTRUCTION_PACKET.nInstruction;
	uint8_t i;
	
	for (i = 0; i < (packet.INSTRUCTION_PACKET.nLength - 2); i++)
	{
		checksum += packet.INSTRUCTION_PACKET.nParameters[i];
	}
	
	return((uint8_t)(~(checksum & 0x000000FF)));
}

void receiveUSBData(void)
{
	if (udi_cdc_is_rx_ready() == 0)
	{
		// No data to read.
		readUSB = 0;
		return;
	}

	if (udi_cdc_read_buf(&rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]].packet[rxPosition[BUS_6_USB]], 1) != 0)
	{
		// Failed to read a character from the USB.
		udi_cdc_signal_framing_error();
	}

	else
	{
		// We read a character, now sanity check it.
		
		// Verify each byte of the preamble.
		if ((rxPosition[BUS_6_USB] == 0) && (rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]].packet[0] != 0xFF))
		{
			// Preamble is wrong. Ignore this character and start again.
			rxPosition[BUS_6_USB] = 0;
		}
		
		else if ((rxPosition[BUS_6_USB] == 1) && (rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]].packet[1] != 0xFF))
		{
			// Preamble is wrong. Ignore this character and start again.
			rxPosition[BUS_6_USB] = 0;
		}
		
		// When we read in the last parameter we need to jump to the checksum position.
		// There is a total of 5 bytes before the parameters, less 1 since we start counting from 0. Less another 1 since we need to 
		// The length field contains the number of parameters + 2. So, the number of bytes to reach the last parameter is
		// 5 - 1 + (length - 2) = length + 2.
		// The checksum is the last byte in the structure. With a structure length of MAX_PARATERS + 6, this equates to MAX_PARAMETS + 5.
		else if ((rxPosition[BUS_6_USB] > 3) && (rxPosition[BUS_6_USB] == (rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]].INSTRUCTION_PACKET.nLength + 2)))
		{
			rxPosition[BUS_6_USB] = MAX_PARAMETERS + 5;
		}

		// We just read in the checksum byte, so we are done.
		// The checksum byte can be verified later.
		else if (rxPosition[BUS_6_USB] == (MAX_PARAMETERS + 5))
		{										
			if (++rxHead[BUS_6_USB] == NUM_BUSES)
			{
				rxHead[BUS_6_USB] = 0;
			}

			rxPosition[BUS_6_USB] = 0;
		}

		// There is no sanity checking to perform on this byte.
		else
		{
			rxPosition[BUS_6_USB]++;
		}
	}	
}

void processPacket(void)
{
	uint8_t bus, txBus;
	uint8_t error;
	
	// Check each bus for anything to process.
	for (bus = BUS_1_MOTORS; bus <= BUS_6_USB; bus++)
	{
		// We need only check the receive buffers to see if there is anything to process.
		// The transmit buffers are filled as a result of our processing.
		while (rxTail[bus] != rxHead[bus])
		{
			// Verify checksum and instruction.
			error = isValidInstruction(rxCircBuffer[bus][rxTail[bus]].INSTRUCTION_PACKET.nInstruction);
			error |= (calculateChecksum(rxCircBuffer[bus][rxTail[bus]]) != rxCircBuffer[bus][rxTail[bus]].INSTRUCTION_PACKET.nChecksum) ? CHECKSUM_ERROR : NO_ERROR;
			
			if (error != NO_ERROR)
			{
				// Generate an error response.
				txCircBuffer[bus][txHead[bus]].INSTRUCTION_PACKET.nPreamble		= 0xFFFF;
				txCircBuffer[bus][txHead[bus]].INSTRUCTION_PACKET.nID			= RAM[DYNAMIXEL_ID];
				txCircBuffer[bus][txHead[bus]].INSTRUCTION_PACKET.nLength		= 0x02;
				txCircBuffer[bus][txHead[bus]].INSTRUCTION_PACKET.nInstruction	= error;
				txCircBuffer[bus][txHead[bus]].INSTRUCTION_PACKET.nChecksum		= calculateChecksum(txCircBuffer[bus][txHead[bus]]);
						
				// Increment the head pointer for the current transmit circular buffer.
				if (++txHead[bus] == NUM_BUSES)
				{
					txHead[bus] = 0;
				}
				
				// Increment the tail pointer for the current receive circular buffer.
				if (++rxTail[bus] == NUM_BUSES)
				{
					rxTail[bus] = 0;
				}
	
				error = NO_ERROR;		
				continue;
			}

			// Check the ID field and see if this message was meant for us.
			else if ((rxCircBuffer[bus][rxTail[bus]].INSTRUCTION_PACKET.nID == RAM[DYNAMIXEL_ID]) || (rxCircBuffer[bus][rxTail[bus]].INSTRUCTION_PACKET.nID == DYNAMIXEL_ID_BROADCAST))
			{
				// Packet is meant for us.
				switch (rxCircBuffer[bus][rxTail[bus]].INSTRUCTION_PACKET.nInstruction)
				{
					case PING:
					{
						// Generate response packet. (Replicate received packet across all buses if broadcasting).
						/*
						 * Packet Specifics:
						 * Preamble...:	0xFFFF
						 * ID.........:	Dynamixel ID.
						 * Length.....:	0x02
						 * Instruction:	0x01
						 * Parameter..: No parameters.
						 * Checksum...:	~((instruction + length + id) & 0xFF).
						 */
						
						// Generate response packet.
						txCircBuffer[bus][txHead[bus]].INSTRUCTION_PACKET.nPreamble		= 0xFFFF;
						txCircBuffer[bus][txHead[bus]].INSTRUCTION_PACKET.nID			= RAM[DYNAMIXEL_ID];
						txCircBuffer[bus][txHead[bus]].INSTRUCTION_PACKET.nLength		= 0x02;
						txCircBuffer[bus][txHead[bus]].INSTRUCTION_PACKET.nInstruction	= NO_ERROR;
						txCircBuffer[bus][txHead[bus]].INSTRUCTION_PACKET.nChecksum		= calculateChecksum(txCircBuffer[bus][txHead[bus]]);
						
						if (++txHead[bus] == NUM_BUSES)
						{
							txHead[bus] = 0;
						}
					}
			
					case READ:
					{
						// Read from specified register addresses. (Replicate packet across all buses if broadcasting).
						/*
						 * Packet Specifics:
						 * Preamble...:	0xFFFF
						 * ID.........:	Dynamixel ID.
						 * Length.....:	0x04
						 * Instruction:	0x02
						 * Parameter..: Address to start reading from.
						 *				Number of bytes to read.
						 * Checksum...:	~((sum of parameters + instruction + length + id) & 0xFF).
						 */
					}
			
					case WRITE:
					{
						// Write to specified register addresses. (Replicate packet across all buses if broadcasting).
						/*
						 * Packet Specifics:
						 * Preamble...:	0xFFFF
						 * ID.........:	Dynamixel ID.
						 * Length.....:	3 + number of parameters.
						 * Instruction:	0x03
						 * Parameter..: Address to start writing to.
						 *				First byte.
						 *				...
						 *				N-th byte.
						 * Checksum...:	~((sum of parameters + instruction + length + id) & 0xFF).
						 */
					}
			
					case REG_WRITE:
					{
						// Register instructions. (Replicate packet across all buses if broadcasting).
						/*
						 * Packet Specifics:
						 * Preamble...:	0xFFFF
						 * ID.........:	Dynamixel ID.
						 * Length.....:	3 + number of parameters.
						 * Instruction:	0x04
						 * Parameter..: Address to start writing to.
						 *				First byte.
						 *				...
						 *				N-th byte.
						 * Checksum...:	~((sum of parameters + instruction + length + id) & 0xFF).
						 */
					}
			
					case ACTION:
					{
						// Activate registered instructions. (Replicate packet across all buses if broadcasting).
						// Error if REG WRITE has not yet occurred.
						/*
						 * Packet Specifics:
						 * Preamble...:	0xFFFF
						 * ID.........:	Dynamixel ID.
						 * Length.....:	0x02
						 * Instruction:	0x05
						 * Parameter..: No parameters.
						 * Checksum...:	~((instruction + length + id) & 0xFF).
						 */
					}
			
					case RESET:
					{
						// Reset local RAM. (Replicate packet across all buses if broadcasting).
						/*
						 * Packet Specifics:
						 * Preamble...:	0xFFFF
						 * ID.........:	Dynamixel ID.
						 * Length.....:	0x02
						 * Instruction:	0x06
						 * Parameter..: No parameters.
						 * Checksum...:	~((instruction + length + id) & 0xFF).
						 */
					}
			
					case SYNC_WRITE:
					{
						// Generate multiple write packets. Route them to the correct bus.
						/*
						 * Packet Specifics:
						 * Preamble...:	0xFFFF
						 * ID.........:	0xFE
						 * Length.....:	4 + (Number of Dynamixels to write to) * (Length of data for each Dynamixel)
						 * Instruction:	0x83
						 * Parameter..: Address to start reading from.
						 *				Number of bytes to read.
						 *				ID of first dynamixel to read from.
						 *				First byte.
						 *				...
						 *				N-th byte.
						 *				Address to start reading from.
						 *				Number of bytes to read.
						 *				ID of next dynamixel to read from.
						 *				First byte.
						 *				...
						 *				N-th byte.
						 * Checksum...:	~((sum of parameters + instruction + length + id) & 0xFF).
						 */
					}
			
					case BULK_READ:
					{
						// Generate multiple read packets. Route them to the correct bus.
						/*
						 * Packet Specifics:
						 * Preamble...:	0xFFFF
						 * ID.........:	0xFE
						 * Length.....:	3 + (Number of Dynamixels to read from) * 3
						 * Instruction:	0x92
						 * Parameter..: 0x00
						 *				Address to start reading from.
						 *				ID of first dynamixel to read from.
						 *				Number of bytes to read.
						 *				Address to start reading from.
						 *				ID of next dynamixel to read from.
						 *				Number of bytes to read.
						 * Checksum...:	~((sum of parameters + instruction + length + id) & 0xFF).
						 */
					}
			
					default:
					{
						// This case is handled above.
						break;
					}
				}
						
				// Replicate received packet across all buses if broadcasting.
				if (rxCircBuffer[bus][rxTail[bus]].INSTRUCTION_PACKET.nID == DYNAMIXEL_ID_BROADCAST)
				{
					for (txBus = BUS_1_MOTORS; txBus <= BUS_6_USB; txBus++)
					{
						// We don't want to broadcast back on to the bus that the message was received on.
						if (txBus != bus)
						{
							txCircBuffer[txBus][txHead[txBus]].INSTRUCTION_PACKET.nPreamble		= rxCircBuffer[bus][rxTail[bus]].INSTRUCTION_PACKET.nPreamble;
							txCircBuffer[txBus][txHead[txBus]].INSTRUCTION_PACKET.nID			= rxCircBuffer[bus][rxTail[bus]].INSTRUCTION_PACKET.nID;
							txCircBuffer[txBus][txHead[txBus]].INSTRUCTION_PACKET.nLength		= rxCircBuffer[bus][rxTail[bus]].INSTRUCTION_PACKET.nLength;
							txCircBuffer[txBus][txHead[txBus]].INSTRUCTION_PACKET.nInstruction	= rxCircBuffer[bus][rxTail[bus]].INSTRUCTION_PACKET.nInstruction;
							memcpy(&txCircBuffer[txBus][txHead[txBus]].INSTRUCTION_PACKET.nParameters, &rxCircBuffer[bus][rxTail[bus]].INSTRUCTION_PACKET.nParameters, (rxCircBuffer[bus][rxTail[bus]].INSTRUCTION_PACKET.nLength - 2) * sizeof(uint8_t));
							txCircBuffer[txBus][txHead[txBus]].INSTRUCTION_PACKET.nChecksum		= rxCircBuffer[bus][rxTail[bus]].INSTRUCTION_PACKET.nChecksum;
						
							// Increment the head pointer for the current transmit circular buffer.
							if (++txHead[txBus] == NUM_BUSES)
							{
								txHead[txBus] = 0;
							}
						}
					}
				}
			}
	
			else
			{
				// Packet is meant for a different device. So route it to the correct USART accordingly.
				;
			}
			
			// Increment the tail pointer for the current receive circular buffer.
			rxTail[bus]++;
		}
	}

	for (bus = BUS_1_MOTORS; bus < BUS_6_USB; bus++)
	{
		// If UART is open
		if (BUS[bus]->imr & AVR32_USART_IER_RXRDY_MASK)
		{
			// Enable UART TX interrupt to send a new value
			BUS[bus]->ier = AVR32_USART_IER_TXRDY_MASK;
		}
	}
}

void motorBusIntteruptController(uint8_t motorBus)
{	
	int txData;
	
    // There is a message being received from one of the buses.
	if (BUS[motorBus]->csr & AVR32_USART_CSR_RXRDY_MASK)
	{
		if (usart_read_char(BUS[motorBus], (int *)&rxCircBuffer[motorBus][txTail[motorBus]].packet[txPosition[motorBus]]) != USART_SUCCESS)
		{
			// Receiver was not ready or an error occurred.
			usart_reset_status(BUS[motorBus]);
			BUS[motorBus]->idr = AVR32_USART_IER_RXRDY_MASK;
		}

		else
		{
			// We read a character, now sanity check it.
	    
			// Verify each byte of the preamble.
			if ((rxPosition[motorBus] == 0) && (rxCircBuffer[motorBus][rxHead[motorBus]].packet[0] != 0xFF))
			{
				// Preamble is wrong. Ignore this character and start again.
				rxPosition[motorBus] = 0;
			}
	    
			else if ((rxPosition[motorBus] == 1) && (rxCircBuffer[motorBus][rxHead[motorBus]].packet[1] != 0xFF))
			{
				// Preamble is wrong. Ignore this character and start again.
				rxPosition[motorBus] = 0;
			}
	    
			// When we read in the last parameter we need to jump to the checksum position.
			// There is a total of 5 bytes before the parameters, less 1 since we start counting from 0.
			// The length field contains the number of parameters + 2. So, the number of bytes to reach the last parameter is
			// 5 - 1 + (length - 2) = length + 2.
			// The checksum is the last byte in the structure. With a structure length of MAX_PARATERS + 6, this equates to MAX_PARAMETS + 5.
			else if (rxPosition[motorBus] == (rxCircBuffer[motorBus][rxHead[motorBus]].INSTRUCTION_PACKET.nLength + 2))
			{
				rxPosition[motorBus] = MAX_PARAMETERS + 5;
			}

			// We just read in the checksum byte, so we are done.
			// The checksum byte can be verified later.
			else if (rxPosition[motorBus] == (MAX_PARAMETERS + 5))
			{
				if (++rxHead[motorBus] == NUM_BUSES)
				{
					rxHead[motorBus] = 0;
				}

				rxPosition[motorBus] = 0;
			}

			// There is no sanity checking to perform on this byte.
			else
			{
				rxPosition[motorBus]++;
			}
		}
	}

	// There is a message to be transmitted to one of the buses.
	if (BUS[motorBus]->csr & AVR32_USART_CSR_TXRDY_MASK)
	{
		if (txTail[motorBus] != txHead[motorBus])
		{
			if (usart_tx_ready(BUS[motorBus]))
			{
				// Write byte to the Transmitter Holding Register.
				txData = txCircBuffer[motorBus][txTail[motorBus]].packet[txPosition[motorBus]];
			    BUS[motorBus]->THR.txchr = txData & 0x1FF;

				udi_cdc_putc(txData & 0x1FF);
				
				// Check to see if we just wrote the last parameter.

				// When we write the last parameter we need to jump to the checksum position.
				// There is a total of 5 bytes before the parameters, less 1 since we start counting from 0.
				// The length field contains the number of parameters + 2. So, the number of bytes to reach the last parameter is
				// 5 - 1 + (length - 2) = length + 2.
				// The checksum is the last byte in the structure. With a structure length of MAX_PARATERS + 6, this equates to MAX_PARAMETS + 5.
				if (txPosition[motorBus] == (txCircBuffer[motorBus][txTail[motorBus]].INSTRUCTION_PACKET.nLength + 2))
				{
					txPosition[motorBus] = MAX_PARAMETERS + 5;
				}

				// We just wrote the checksum byte, so we are done.
				else if (txPosition[motorBus] == (MAX_PARAMETERS + 5))
				{
					if (++txTail[motorBus] == NUM_BUSES)
					{
						txTail[motorBus] = 0;
					}

					txPosition[motorBus] = 0;
				}

				else
				{
					txPosition[motorBus]++;
				}
			}
		}
    
		else
		{
			// Nothing to transmit.
			usart_reset_status(BUS[motorBus]);
			BUS[motorBus]->idr = AVR32_USART_IER_TXRDY_MASK;
		}
	}
}

__attribute__((__interrupt__)) static void usart_interrupt(void)
{
	motorBusIntteruptController(1);
}

void usb_rx_notify(uint8_t port)
{
	uint8_t bus;
	
	readUSB = 1;
	
	// Loop through all USARTs and, if they are open, trigger the TX interrupt.
	for (bus = BUS_1_MOTORS; bus < BUS_6_USB; bus++)
	{
		// If UART is open
		if (BUS[bus]->imr & AVR32_USART_IER_RXRDY_MASK)
		{
			// Enable UART TX interrupt to send a new value
			BUS[bus]->ier = AVR32_USART_IER_TXRDY_MASK;
		}
	}
		
	/*
	 * All received messages should be of the following form:
	 * 2 Byte preamble.     Should always be 0xFFFF.
	 * 1 Byte ID.           Should be 0x00-0xFE. 0xFE = Broadcast ID. This is the ID of the destination Dynamixel.
	 * 1 Byte Length.       This is the length of the packet beyond this point. Should be the number of parameters + 2.
	 * 1 Byte Instruction.  0x01 PING           Ping a Dynamixel.                                                                   Parameters = 0.
	 *                      0x02 READ_DATA      Reads from Dynamixel.                                                               Parameters = 2.
	 *                      0x03 WRITE_DATA     Writes to Dynamixel.                                                                Parameters = 2 or more.
	 *                      0x04 REG WRITE      Same as WRITE_DATA, but action is not taken until ACTION instruction is received.   Parameters = 2 or more.
	 *                      0x05 ACTION         Initiates registered motions (from REG WRITE instruction).                          Parameters = 0.
	 *                      0x06 RESET          Perform factory reset on Dynamixels.                                                Parameters = 0.
	 *                      0x83 SYNC WRITE     Used to control multiple Dynamixels simultaneously.                                 Parameters = 4 or more.
	 *                      0x92 BULK READ      Used to read from multiple Dynamixels simultaneously.                               Parameters = 4 or more.
	 * N-Byte Parameter.    N = Length - 2. This is the packet data.
	 * 1 Byte Checksum.     Checksum = ~((ID + Length + Instruction + Parameter1 + .... = Parameter N) & 0xFF).
	 */
	
	/*
	 * All return messages (from a Dynamixel) should be of the following form:
	 * 2 Byte preamble.		Should always be 0xFFFF
	 * 1 Byte ID.			Should be 0x00-0xFE. This is the ID of the source Dynamixel.
	 * 1 Byte Length.		This is the length of the packet beyond this point. Should be the number of parameters + 2.
	 * 1 Byte Error.		Bit 7: UNUSED.
	 *						Bit 6: Instruction Error. Undefined instruction or use of ACTION without REG WRITE.
	 *						Bit 5: Overload Error. Current load cannot be controlled by the set torque.
	 *						Bit 4: Checksum Error. Incorrect checksum.
	 *						Bit 3: Range Error. Command is out of range.
	 *						Bit 2: Overheating Error. Internal Dynamixel temperature is out of range of operating temperature.
	 *						Bit 1: Angle Limit Error. Goal position is out of range from CW Angle Limit to CCW Angle Limit.
	 *						Bit 0: Input Voltage Error. Applied input voltage is out of range.
	 * N-Byte Parameter.	N = Length - 2. This is the packet data.
	 * 1 Byte Checksum.		Checksum = ~((ID + Length + Instruction + Parameter1 + .... = Parameter N) & 0xFF).
	 */	
}


void uart_config(uint8_t port, usb_cdc_line_coding_t * cfg)
{
	// Initialise RAM table.
	initRAM();

	// Options for USART.
	usart_options.baudrate		= 921600; // RAM[BAUD_RATE] * 500000; // 921600;
	usart_options.charlength	= 0x08;
	usart_options.paritytype	= USART_NO_PARITY;
	usart_options.stopbits		= 1;
	usart_options.channelmode	= USART_NORMAL_CHMODE;
}

void uart_open(uint8_t port)
{
	// Enable interrupt with priority higher than USB
	irq_register_handler(usart_interrupt, USART_IRQ, 3);

	// Initialize it in RS232 mode.
	sysclk_enable_pba_module(USART_SYSCLK);

	// Calculate baud rate parameters.
	unsigned int over = (sysclk_get_pba_hz() >= 16 * usart_options.baudrate) ? 16 : 8;
	unsigned int cd_fp = (0x08 * sysclk_get_pba_hz()) / (over * usart_options.baudrate) + 0.5;
	unsigned int cd = cd_fp >> 3;
	unsigned int fp = cd_fp & 0x07;
	
	// Set required fields in USART Mode Register.
	if ((cd >= 1) || (cd <= 0x0000FFFF))
	{
		USART->mr = 0;
		USART->MR.over = (over == 8); // (over == 16);
		USART->MR.chrl = 0x3;
		USART->MR.par = USART_NO_PARITY;
		
		USART->brgr = 0;
		USART->BRGR.cd = cd;
		USART->BRGR.fp = fp;
		
		USART->cr = 0;
		USART->CR.rxen = 1;
		USART->CR.txen = 1;
		
		USART->idr = 0xFFFFFFFF;

		USART->ier = 0;
		USART->IER.rxrdy = 1;
		USART->IER.txrdy = 1;		
	}

	else
	{
		// Baud rate is not possible with the provided clock.
		udi_cdc_putc(0x00);
		return;
	}
}

void uart_close(uint8_t port)
{
	// Disable interrupts
	// Close RS232 communication
	USART->idr = 0xFFFFFFFF;
}
