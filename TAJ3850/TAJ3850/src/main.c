#include <asf.h>
#include "conf_usb.h"
#include "ui.h"
#include "uart.h"

static volatile bool main_b_cdc_enable = false;

/*! \brief Main function. Execution starts here.
 */
int main(void)
{
	// Initialise system clocks.
	sysclk_init();
	
	// Enable all interrupts.
	irq_initialize_vectors();
	cpu_irq_enable();
	
	// Initialise board GPIO mappings.
	board_init();
	
	// Initialize the sleep manager
	sleepmgr_init();

	// Start USB stack to authorize VBus monitoring
	udc_start();

	while (true)
	{
		// Check  to see if we need to process any packets.
		processPacket();
		
		// Check to see if there is data in the USB TX FIFO to transmit.
		transmitUSBData();
		
		sleepmgr_enter_sleep();
	}
}

void main_suspend_action(void)
{
	;
}

void main_resume_action(void)
{
	;
}

void main_sof_action(void)
{
	;
}

bool main_cdc_enable(uint8_t port)
{
	main_b_cdc_enable = true;
	
	// Open communication
	uart_open(port);
	
	return(true);
}

void main_cdc_disable(uint8_t port)
{
	main_b_cdc_enable = false;
	
	// Close communication
	uart_close(port);
}

void main_cdc_set_dtr(uint8_t port, bool b_enable)
{
	;
}


void main_vbus_action(bool b_high)
{
	if (b_high)
	{
		// Attach USB Device
		udc_attach();
	}
	
	else
	{
		// Vbus not present
		udc_detach();
	}
}