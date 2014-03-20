#ifndef ASF_H
#define ASF_H

/*
 * This file includes all API header files for the selected drivers from ASF.
 * Note: There might be duplicate includes required by more than one driver.
 *
 * The file is automatically generated and will be re-written when
 * running the ASF driver selector tool. Any changes will be discarded.
 */

// From module: ADC - Analog-to-digital Converter
#include <adc.h>

// From module: Common SAM compiler driver
#include <compiler.h>
#include <status_codes.h>

// From module: Delay routines
#include <delay.h>

// From module: EEFC - Enhanced Embedded Flash Controller
#include <efc.h>

// From module: Flash - SAM Flash Service API
#include <flash_efc.h>

// From module: GPIO - General purpose Input/Output
#include <gpio.h>

// From module: Generic board support
#include <board.h>
#include <led.h>

// From module: IOPORT - General purpose I/O service
#include <ioport.h>

// From module: Interrupt management - SAM implementation
#include <interrupt.h>

// From module: PIO - Parallel Input/Output Controller
#include <pio.h>

// From module: PMC - Power Management Controller
#include <pmc.h>
#include <sleep.h>

// From module: Part identification macros
#include <parts.h>

// From module: SAM3U startup code
#include <exceptions.h>

// From module: Sleep manager - SAM implementation
#include <sam/sleepmgr.h>
#include <sleepmgr.h>

// From module: System Clock Control - SAM3U implementation
#include <sysclk.h>

// From module: TWI - Two-wire Interface
#include <twi.h>

// From module: USART - Univ. Syn Async Rec/Trans
#include <usart.h>

// From module: USB CDC Protocol
#include <usb_protocol_cdc.h>

// From module: USB Device CDC (Single Interface Device)
#include <udi_cdc.h>

// From module: USB Device Stack Core (Common API)
#include <udc.h>
#include <udd.h>

// From module: pio_handler support enabled
#include <pio_handler.h>

#endif // ASF_H
