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

#define NUM_IDS_PER_BUS				0x0F																												// Arbitrary maximum for the number of IDs per motor bus.
#define BUS_1_MOTORS				0x00
#define BUS_2_MOTORS				0x01
#define BUS_3_MOTORS				0x02
//#define BUS_4_MOTORS				0x03
//#define BUS_5_MOTORS				0x04
#define BUS_6_USB					0x03
#define NUM_BUSES					BUS_6_USB + 1																										// Number of motor buses plus the USB bus.

#define MAX_PACKET_LENGTH			0x84
#define MAX_PARAMETERS				MAX_PACKET_LENGTH - 6
#define PACKET_PREAMBLE_1_OFFSET	0x00
#define PACKET_PREAMBLE_2_OFFSET	0x01
#define PACKET_ID_OFFSET			0x02
#define PACKET_LENGTH_OFFSET		0x03
#define PACKET_INSTRUCTION_OFFSET	0x04
#define PACKET_PARAMETERS_OFFSET	0x05
#define PACKET_CHECKSUM_OFFSET		MAX_PACKET_LENGTH - 1


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

/********************************************
 *			FUNCTION PROTOTYPES				*
 ********************************************/
void	initRAM(void);
uint8_t	calculateChecksum(const volatile uint8_t *packet);
uint8_t isValidInstruction(uint8_t instruction);

/********************************************
 *				GLOBALS						*
 ********************************************/
static volatile usart_options_t	usart_options;

static volatile uint8_t			txCircBuffer[NUM_BUSES][NUM_IDS_PER_BUS][MAX_PACKET_LENGTH] = {[0 ... (NUM_BUSES - 1)][0 ... (NUM_IDS_PER_BUS -  1)] = {0}};		// Transmit circular buffers.
static volatile uint8_t			rxCircBuffer[NUM_BUSES][NUM_IDS_PER_BUS][MAX_PACKET_LENGTH] = {[0 ... (NUM_BUSES - 1)][0 ... (NUM_IDS_PER_BUS -  1)] = {0}};		// Receive circular buffers.
static volatile uint8_t			txHead[NUM_BUSES] = {0};																								// Pointer to the head of each transmit circular buffer.
static volatile uint8_t			txTail[NUM_BUSES] = {0};																								// Pointer to the tail of each transmit circular buffer.
static volatile uint8_t			txPosition[NUM_BUSES] = {0};																							// Pointer to the position within the head of each transmit circular buffer.
static volatile uint8_t			rxHead[NUM_BUSES] = {0};																								// Pointer to the head of each receive circular buffer.
static volatile uint8_t			rxTail[NUM_BUSES] = {0};																								// Pointer to the tail of each receive circular buffer.
static volatile uint8_t			rxPosition[NUM_BUSES] = {0};																							// Pointer to the position within the head of each receive circular buffer.
static volatile avr32_usart_t	*BUS[NUM_BUSES] = {0};																									// Handle to each USART device.

const uint8_t DEFAULT_RAM[RAM_TABLE_SIZE] = {MODEL_NUMBER_L_DEFAULT, MODEL_NUMBER_H_DEFAULT, FIRMWARE_VERSION_DEFAULT, DYNAMIXEL_ID_DEFAULT, BAUD_RATE_DEFAULT};
	
__attribute__((__section__(".flash_nvram"))) static uint8_t RAM[RAM_TABLE_SIZE];																					// Store RAM table in flash NVRAM.
//__attribute__((__section__(".userpage"))) static uint8_t RAM[RAM_TABLE_SIZE];																					// Store RAM table in flash user page.



void initRAM(void)
{
	// Populate RAM with default values.
	flashc_memset((void *)RAM, 0x00, 8, RAM_TABLE_SIZE * sizeof(uint8_t), true);
	flashc_memcpy((void *)&RAM, (void *)&DEFAULT_RAM, RAM_TABLE_SIZE * sizeof(uint8_t), true);
	
	BUS[BUS_1_MOTORS] = ((avr32_usart_t*)AVR32_USART0_ADDRESS);
	BUS[BUS_2_MOTORS] = ((avr32_usart_t*)AVR32_USART1_ADDRESS);
	BUS[BUS_3_MOTORS] = ((avr32_usart_t*)AVR32_USART2_ADDRESS);
	BUS[BUS_6_USB] = NULL;
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

uint8_t calculateChecksum(const volatile uint8_t *packet)
{
	uint16_t checksum = packet[PACKET_ID_OFFSET] + packet[PACKET_LENGTH_OFFSET] + packet[PACKET_INSTRUCTION_OFFSET];
	uint8_t parameter;
	
	for (parameter = 0; parameter < (packet[PACKET_LENGTH_OFFSET] - 2); parameter++)
	{
		checksum += packet[PACKET_PARAMETERS_OFFSET + parameter];
	}
	
	return((uint8_t)(~(checksum & 0x00FF)));
}

void transmitUSBData(void)
{
	irqflags_t flags;
	
	if (udi_cdc_is_tx_ready() == TRUE)
	{
		flags = cpu_irq_save();
		
		if (txTail[BUS_6_USB] != txHead[BUS_6_USB])
		{
			// Write byte to the Transmitter Holding Register.
			// Need to cast away volatile qualifier.
			if (udi_cdc_write_buf((void *)&txCircBuffer[BUS_6_USB][txTail[BUS_6_USB]][txPosition[BUS_6_USB]], 1) == 0)
			{
				// Check to see if we just wrote the last parameter.

				// When we write the last parameter we need to jump to the checksum position.
				// There is a total of 5 bytes before the parameters, less 1 since we start counting from 0.
				// The length field contains the number of parameters + 2. So, the number of bytes to reach the last parameter is
				// 5 - 1 + (length - 2) = length + 2.
				// The checksum is the last byte in the structure. With a structure length of MAX_PARATERS + 6, this equates to MAX_PARAMETS + 5.
				if (txPosition[BUS_6_USB] == (txCircBuffer[BUS_6_USB][txTail[BUS_6_USB]][PACKET_LENGTH_OFFSET] + 2))
				{
					txPosition[BUS_6_USB] = PACKET_CHECKSUM_OFFSET;
				}

				// We just wrote the checksum byte, so we are done.
				else if (txPosition[BUS_6_USB] == PACKET_CHECKSUM_OFFSET)
				{
					txTail[BUS_6_USB]++;
					
					if (txTail[BUS_6_USB] >= NUM_BUSES)
					{
						txTail[BUS_6_USB] = 0;
					}

					txPosition[BUS_6_USB] = 0;
				}

				else
				{
					txPosition[BUS_6_USB]++;
				}
			}
		}
		
		cpu_irq_restore(flags);
	}
	
	else
	{
		udi_cdc_signal_overrun();
	}
}

void receiveUSBData(void)
{
	static uint8_t rxBuffer[MAX_PACKET_LENGTH] = {0};
	static uint8_t bufferPosition = 0;
	irqflags_t flags;

	while (udi_cdc_is_rx_ready())
	{	
		// Need to cast away volatile qualifier.
		if (udi_cdc_read_buf(&rxBuffer[bufferPosition], 1) == 0)
		{
			// We read a character, now sanity check it.
		
			// Verify each byte of the preamble.
			if ((bufferPosition == 0) && (rxBuffer[PACKET_PREAMBLE_1_OFFSET] != 0xFF))
			{
				// Preamble is wrong. Ignore this character and start again.
				bufferPosition = 0;
			}
		
			else if ((bufferPosition == 1) && (rxBuffer[PACKET_PREAMBLE_2_OFFSET] != 0xFF))
			{
				// Preamble is wrong. Ignore this character and start again.
				bufferPosition = 0;
			}
			
			else if ((bufferPosition == 2) && (rxBuffer[PACKET_ID_OFFSET] == 0xFF))
			{
				// We got another 0xFF, assume it is part of a preamble and wait for an ID byte to arrive.
				;
			}
			
			// When we read in the last parameter we need to jump to the checksum position.
			// There is a total of 5 bytes before the parameters, less 1 since we start counting from 0. Less another 1 since we need to 
			// The length field contains the number of parameters + 2. So, the number of bytes to reach the last parameter is
			// 5 - 1 + (length - 2) = length + 2.
			// The checksum is the last byte in the structure. With a structure length of MAX_PARATERS + 6, this equates to MAX_PARAMETS + 5.
			else if ((bufferPosition > PACKET_LENGTH_OFFSET) && (bufferPosition == (rxBuffer[PACKET_LENGTH_OFFSET] + 2)))
			{
				bufferPosition = PACKET_CHECKSUM_OFFSET;
			}

			// We just read in the checksum byte, so we are done.
			// The checksum byte can be verified later.
			else if (bufferPosition == PACKET_CHECKSUM_OFFSET)
			{
				flags = cpu_irq_save();

				// Copy the received buffer into the head of the receive circular buffer.
				memcpy(&rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]], &rxBuffer, MAX_PACKET_LENGTH * sizeof(uint8_t));
				
				// Increment the head position.
				rxHead[BUS_6_USB]++;

				if (rxHead[BUS_6_USB] >= NUM_BUSES)
				{
					rxHead[BUS_6_USB] = 0;
				}

				cpu_irq_restore(flags);
				
				// Clear out our local buffer.
				memset(&rxBuffer, 0x00, MAX_PACKET_LENGTH * sizeof(uint8_t));
				
				// Reset our counter.
				bufferPosition = 0;
			}

			// There is no sanity checking to perform on this byte.
			else
			{
				bufferPosition++;
			}
		}
	}
	
	/*
	irqflags_t flags;
	uint8_t count, slot, startPosition, size;
	
	if (udi_cdc_is_rx_ready())
	{
		count = 0;
		
		flags = cpu_irq_save();

		count = (udi_cdc_get_nb_received_data() < MAX_PACKET_LENGTH) ? udi_cdc_get_nb_received_data() : MAX_PACKET_LENGTH;
		
		// Need to cast away volatile qualifier.
		if (udi_cdc_read_buf((uint8_t *)&rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]][PACKET_INSTRUCTION_OFFSET], count) == 0)
		{
			rxPosition[BUS_6_USB] += count;
			
			if (rxPosition[BUS_6_USB] >= PACKET_LENGTH_OFFSET)
			{
				do 
				{
					slot = (rxHead[BUS_6_USB] + 1 == NUM_BUSES) ? 0 : rxHead[BUS_6_USB] + 1;
					startPosition = rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]][PACKET_LENGTH_OFFSET] + 4;
					size = rxPosition[BUS_6_USB] - rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]][PACKET_LENGTH_OFFSET] - 4;
						
					if (rxPosition[BUS_6_USB] > (rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]][PACKET_LENGTH_OFFSET] + 4))
					{
						// Copy excess data to the next slot in the circular buffer.
						memcpy((uint8_t *)&rxCircBuffer[BUS_6_USB][slot][0], (uint8_t *)&rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]][startPosition], size * sizeof(uint8_t));
						
						rxPosition[BUS_6_USB] = size;
						
						// Find the checksum byte and move it to its correct position.
						rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]][PACKET_CHECKSUM_OFFSET] = rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]][startPosition - 1];
						
						rxHead[BUS_6_USB]++;

						if (rxHead[BUS_6_USB] >= NUM_BUSES)
						{
							rxHead[BUS_6_USB] = 0;
						}
					}
					
					else if (rxPosition[BUS_6_USB] == (rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]][PACKET_LENGTH_OFFSET] + 4))
					{							
						// Find the checksum byte and move it to its correct position.
						rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]][PACKET_CHECKSUM_OFFSET] = rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]][startPosition - 1];
						
						rxHead[BUS_6_USB]++;

						if (rxHead[BUS_6_USB] >= NUM_BUSES)
						{
							rxHead[BUS_6_USB] = 0;
						}
					}
				} while(rxPosition[BUS_6_USB] > (rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]][PACKET_LENGTH_OFFSET] + 4));
			}
		}
*/
/*		
		// Need to cast away volatile qualifier.
		if (udi_cdc_read_buf((uint8_t *)&rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]][rxPosition[BUS_6_USB]], 1) == 0)
		{
			// We read a character, now sanity check it.
		
			// Verify each byte of the preamble.
			if ((rxPosition[BUS_6_USB] == 0) && (rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]][PACKET_PREAMBLE_1_OFFSET] != 0xFF))
			{
				// Preamble is wrong. Ignore this character and start again.
				rxPosition[BUS_6_USB] = 0;
			}
		
			else if ((rxPosition[BUS_6_USB] == 1) && (rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]][PACKET_PREAMBLE_2_OFFSET] != 0xFF))
			{
				// Preamble is wrong. Ignore this character and start again.
				rxPosition[BUS_6_USB] = 0;
			}
			
			else if ((rxPosition[BUS_6_USB] == 2) && (rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]][PACKET_ID_OFFSET] == 0xFF))
			{
				// We got another 0xFF, assume it is part of a preamble and wait for an ID byte to arrive.
				;
			}
			
			// When we read in the last parameter we need to jump to the checksum position.
			// There is a total of 5 bytes before the parameters, less 1 since we start counting from 0. Less another 1 since we need to 
			// The length field contains the number of parameters + 2. So, the number of bytes to reach the last parameter is
			// 5 - 1 + (length - 2) = length + 2.
			// The checksum is the last byte in the structure. With a structure length of MAX_PARATERS + 6, this equates to MAX_PARAMETS + 5.
			else if ((rxPosition[BUS_6_USB] > PACKET_LENGTH_OFFSET) && (rxPosition[BUS_6_USB] == (rxCircBuffer[BUS_6_USB][rxHead[BUS_6_USB]][PACKET_LENGTH_OFFSET] + 2)))
			{
				rxPosition[BUS_6_USB] = PACKET_CHECKSUM_OFFSET;
			}

			// We just read in the checksum byte, so we are done.
			// The checksum byte can be verified later.
			else if (rxPosition[BUS_6_USB] == PACKET_CHECKSUM_OFFSET)
			{										
				rxHead[BUS_6_USB]++;

				if (rxHead[BUS_6_USB] >= NUM_BUSES)
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

		cpu_irq_restore(flags);
	}
*/
}

void processPacket(void)
{
	uint8_t bus, txBus;
	uint8_t error;
	uint8_t packet[MAX_PACKET_LENGTH];
	irqflags_t flags;
	
	// Check each bus for anything to process.
	for (bus = BUS_1_MOTORS; bus <= BUS_6_USB; bus++)
	{
		flags = cpu_irq_save();
		
		// We need only check the receive buffers to see if there is anything to process.
		// The transmit buffers are filled as a result of our processing.
		while (rxTail[bus] != rxHead[bus])
		{
			memcpy(&packet, &rxCircBuffer[bus][rxTail[bus]], MAX_PACKET_LENGTH * sizeof(uint8_t));

			udi_cdc_putc(packet[PACKET_PREAMBLE_1_OFFSET]);
			udi_cdc_putc(packet[PACKET_PREAMBLE_2_OFFSET]);
			udi_cdc_putc(packet[PACKET_ID_OFFSET]);
			udi_cdc_putc(packet[PACKET_LENGTH_OFFSET]);
			udi_cdc_putc(packet[PACKET_INSTRUCTION_OFFSET]);
			udi_cdc_putc(packet[PACKET_CHECKSUM_OFFSET]);

			// For packets received via USB, verify checksum and instruction.
			if (bus == BUS_6_USB)
			{
				error = isValidInstruction(packet[PACKET_INSTRUCTION_OFFSET]);
				error |= (calculateChecksum(packet) != packet[PACKET_CHECKSUM_OFFSET]) ? CHECKSUM_ERROR : NO_ERROR;
			
				if (error != NO_ERROR)
				{
					// Generate an error response.
					txCircBuffer[BUS_6_USB][txHead[BUS_6_USB]][PACKET_PREAMBLE_1_OFFSET]		= 0xFF;
					txCircBuffer[BUS_6_USB][txHead[BUS_6_USB]][PACKET_PREAMBLE_2_OFFSET]		= 0xFF;
					txCircBuffer[BUS_6_USB][txHead[BUS_6_USB]][PACKET_ID_OFFSET]				= RAM[DYNAMIXEL_ID];
					txCircBuffer[BUS_6_USB][txHead[BUS_6_USB]][PACKET_LENGTH_OFFSET]			= 0x02;
					txCircBuffer[BUS_6_USB][txHead[BUS_6_USB]][PACKET_INSTRUCTION_OFFSET]		= error;
					txCircBuffer[BUS_6_USB][txHead[BUS_6_USB]][PACKET_CHECKSUM_OFFSET]			= calculateChecksum(txCircBuffer[BUS_6_USB][txHead[BUS_6_USB]]);
				
					// Increment the head pointer for the current transmit circular buffer.
					txHead[BUS_6_USB]++;

					if (txHead[BUS_6_USB] >= NUM_BUSES)
					{
						txHead[BUS_6_USB] = 0;
					}
				
					// Increment the tail pointer for the current receive circular buffer.
					rxTail[BUS_6_USB]++;

					if (rxTail[BUS_6_USB] >= NUM_BUSES)
					{
						rxTail[BUS_6_USB] = 0;
					}
				
					error = NO_ERROR;
					continue;
				}
			}

			// Check the ID field and see if this message was meant for us.
			if ((packet[PACKET_ID_OFFSET] == RAM[DYNAMIXEL_ID]) || (packet[PACKET_ID_OFFSET] == DYNAMIXEL_ID_BROADCAST))
			{
				// Packet is meant for us.
				switch (packet[PACKET_INSTRUCTION_OFFSET])
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
						txCircBuffer[bus][txHead[bus]][PACKET_PREAMBLE_1_OFFSET]		= 0xFF;
						txCircBuffer[bus][txHead[bus]][PACKET_PREAMBLE_2_OFFSET]		= 0xFF;
						txCircBuffer[bus][txHead[bus]][PACKET_ID_OFFSET]				= RAM[DYNAMIXEL_ID];
						txCircBuffer[bus][txHead[bus]][PACKET_LENGTH_OFFSET]			= 0x02;
						txCircBuffer[bus][txHead[bus]][PACKET_INSTRUCTION_OFFSET]		= NO_ERROR;
						txCircBuffer[bus][txHead[bus]][PACKET_CHECKSUM_OFFSET]			= calculateChecksum(txCircBuffer[bus][txHead[bus]]);
						
						txHead[bus]++;

						if (txHead[bus] >= NUM_BUSES)
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
				if (packet[PACKET_ID_OFFSET] == DYNAMIXEL_ID_BROADCAST)
				{
					for (txBus = 0; txBus < NUM_BUSES; txBus++)
					{
						// We don't want to broadcast back on to the bus that the message was received on.
						if (txBus != bus)
						{
							memcpy(&txCircBuffer[txBus][txHead[txBus]], &packet, MAX_PACKET_LENGTH);
						
							// Increment the head pointer for the current transmit circular buffer.
							txHead[txBus]++;

							if (txHead[txBus] >= NUM_BUSES)
							{
								txHead[txBus] = 0;
							}
						}
					}
				}
			}
	
			else
			{
				// Packet is meant for a different device. So route it to the correct bus.
				if (bus == BUS_6_USB)
				{
					// Packet came from the USB bus, so find the correct USART to route it to.
					;
				}
				
				else
				{
					// Packet came from a USART and it was not addressed to us, so route it to the USB.
					memcpy(&txCircBuffer[BUS_6_USB][txHead[BUS_6_USB]], &packet, MAX_PACKET_LENGTH);
							
					// Increment the head pointer for the current transmit circular buffer.
					txHead[BUS_6_USB]++;

					if (txHead[BUS_6_USB] >= NUM_BUSES)
					{
						txHead[BUS_6_USB] = 0;
					}
				}
			}
			
			// Increment the tail pointer for the current receive circular buffer.
			rxTail[bus]++;
		}
							
		cpu_irq_restore(flags);
	}

	for (bus = 0; bus < (NUM_BUSES - 1); bus++)
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
	static uint8_t rxBuffer[MAX_PACKET_LENGTH] = {0};
	static uint8_t rxBufferPosition = 0;
	static uint8_t txBuffer[MAX_PACKET_LENGTH] = {0};
	static uint8_t txBufferPosition = 0;
	
	// Sanity check input parameter.
	if (motorBus >= BUS_6_USB)
	{
		return;
	}
	
	// Check for any errors.
	if (BUS[motorBus]->csr & (AVR32_USART_CSR_OVRE_MASK | AVR32_USART_CSR_FRAME_MASK | AVR32_USART_CSR_PARE_MASK))
	{
		// Receiver was not ready or an error occurred.
		usart_reset_status(BUS[motorBus]);
	}

    // There is a message being received from one of the buses.
	if (BUS[motorBus]->csr & AVR32_USART_CSR_RXRDY_MASK)
	{
		rxBuffer[rxBufferPosition] = BUS[motorBus]->rhr & 0xFF;
		
		// We read a character, now sanity check it.
	    
		// Verify each byte of the preamble.
		if ((rxBufferPosition == 0) && (rxBuffer[PACKET_PREAMBLE_1_OFFSET] != 0xFF))
		{
			// Preamble is wrong. Ignore this character and start again.
			rxBufferPosition = 0;
		}
	    
		else if ((rxBufferPosition == 1) && (rxBuffer[PACKET_PREAMBLE_2_OFFSET] != 0xFF))
		{
			// Preamble is wrong. Ignore this character and start again.
			rxBufferPosition = 0;
		}
		
		else if ((rxBufferPosition == 2) && (rxBuffer[PACKET_ID_OFFSET] == 0xFF))
		{
			// We got another 0xFF, assume it is part of a preamble and wait for an ID byte to arrive.
			;
		}
		
		// When we read in the last parameter we need to jump to the checksum position.
		// There is a total of 5 bytes before the parameters, less 1 since we start counting from 0.
		// The length field contains the number of parameters + 2. So, the number of bytes to reach the last parameter is
		// 5 - 1 + (length - 2) = length + 2.
		// The checksum is the last byte in the structure. With a structure length of MAX_PARATERS + 6, this equates to MAX_PARAMETS + 5.
		else if ((rxBufferPosition > PACKET_LENGTH_OFFSET) && (rxBufferPosition == (rxBuffer[PACKET_LENGTH_OFFSET] + 2)))
		{
			rxBufferPosition = PACKET_CHECKSUM_OFFSET;
		}

		// We just read in the checksum byte, so we are done.
		// The checksum byte can be verified later.
		else if (rxBufferPosition == PACKET_CHECKSUM_OFFSET)
		{
			// Copy the received buffer into the head of the receive circular buffer.
			memcpy(&rxCircBuffer[motorBus][rxHead[motorBus]], &rxBuffer, MAX_PACKET_LENGTH * sizeof(uint8_t));
				
			// Increment the head position.
			rxHead[motorBus]++;

			if (rxHead[motorBus] >= NUM_BUSES)
			{
				rxHead[motorBus] = 0;
			}

			// Clear out our local buffer.
			memset(&rxBuffer, 0x00, MAX_PACKET_LENGTH * sizeof(uint8_t));
			
			// Reset our counter.
			rxBufferPosition = 0;
		}

		// There is no sanity checking to perform on this byte.
		else
		{
			rxBufferPosition++;
		}
		
/*
		rxCircBuffer[motorBus][rxHead[motorBus]][rxPosition[motorBus]] = BUS[motorBus]->rhr & 0xFF;
		
		// We read a character, now sanity check it.
		
		// Verify each byte of the preamble.
		if ((rxPosition[motorBus] == 0) && (rxCircBuffer[motorBus][rxHead[motorBus]][PACKET_PREAMBLE_1_OFFSET] != 0xFF))
		{
			// Preamble is wrong. Ignore this character and start again.
			rxPosition[motorBus] = 0;
		}
		
		else if ((rxPosition[motorBus] == 1) && (rxCircBuffer[motorBus][rxHead[motorBus]][PACKET_PREAMBLE_2_OFFSET] != 0xFF))
		{
			// Preamble is wrong. Ignore this character and start again.
			rxPosition[motorBus] = 0;
		}
		
		else if ((rxPosition[motorBus] == 2) && (rxCircBuffer[motorBus][rxHead[motorBus]][PACKET_ID_OFFSET] == 0xFF))
		{
			// We got another 0xFF, assume it is part of a preamble and wait for an ID byte to arrive.
			;
		}
		
		// When we read in the last parameter we need to jump to the checksum position.
		// There is a total of 5 bytes before the parameters, less 1 since we start counting from 0.
		// The length field contains the number of parameters + 2. So, the number of bytes to reach the last parameter is
		// 5 - 1 + (length - 2) = length + 2.
		// The checksum is the last byte in the structure. With a structure length of MAX_PARATERS + 6, this equates to MAX_PARAMETS + 5.
		else if ((rxPosition[motorBus] > PACKET_LENGTH_OFFSET) && (rxPosition[motorBus] == (rxCircBuffer[motorBus][rxHead[motorBus]][PACKET_LENGTH_OFFSET] + 2)))
		{
			rxPosition[motorBus] = PACKET_CHECKSUM_OFFSET;
		}

		// We just read in the checksum byte, so we are done.
		// The checksum byte can be verified later.
		else if (rxPosition[motorBus] == PACKET_CHECKSUM_OFFSET)
		{
			rxHead[motorBus]++;

			if (rxHead[motorBus] == NUM_BUSES)
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
*/
	}

	// There is a message to be transmitted to one of the buses.
	if (BUS[motorBus]->csr & AVR32_USART_CSR_TXRDY_MASK)
	{
		// If the transmit buffer is empty, copy the next one over.
		if (txBuffer[0] == 0x00)
		{
			if (txTail[motorBus] != txHead[motorBus])
			{
				memcpy(&txBuffer, &txCircBuffer[motorBus][txTail[motorBus]], MAX_PACKET_LENGTH * sizeof(uint8_t));
			
				// Increment the tail pointer.
				txTail[motorBus]++;

				if (txTail[motorBus] >= NUM_BUSES)
				{
					txTail[motorBus] = 0;
				}
			}
    
			else
			{
				// Nothing to transmit.
				usart_reset_status(BUS[motorBus]);
				BUS[motorBus]->idr = AVR32_USART_IER_TXRDY_MASK;
			}
		}

		else		
		{
			if (usart_tx_ready(BUS[motorBus]))
			{
				// Write byte to the Transmitter Holding Register.
			    BUS[motorBus]->THR.txchr = txBuffer[txBufferPosition] & 0xFF;
				
				// Check to see if we just wrote the last parameter.

				// When we write the last parameter we need to jump to the checksum position.
				// There is a total of 5 bytes before the parameters, less 1 since we start counting from 0.
				// The length field contains the number of parameters + 2. So, the number of bytes to reach the last parameter is
				// 5 - 1 + (length - 2) = length + 2.
				// The checksum is the last byte in the structure. With a structure length of MAX_PARATERS + 6, this equates to MAX_PARAMETS + 5.
				if (txBufferPosition == (txBuffer[PACKET_LENGTH_OFFSET] + 2))
				{
					txBufferPosition = PACKET_CHECKSUM_OFFSET;
				}

				// We just wrote the checksum byte, so we are done.
				else if (txBufferPosition == PACKET_CHECKSUM_OFFSET)
				{
					// Clear out our local buffer.
					memset(&txBuffer, 0x00, MAX_PACKET_LENGTH * sizeof(uint8_t));

					// Reset our counter.			
					txBufferPosition = 0;
				}

				else
				{
					txBufferPosition++;
				}
			}
		}
	}
}

__attribute__((__interrupt__)) static void usart_interrupt(void)
{
	motorBusIntteruptController(BUS_2_MOTORS);
}

void usb_rx_notify(uint8_t port)
{
	uint8_t bus;
	
	// Loop through all USARTs and, if they are open, trigger the TX interrupt.
	for (bus = 0; bus < (NUM_BUSES - 1); bus++)
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
