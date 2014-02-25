#ifndef _UART_H_
#define _UART_H_

/*! \brief Called by CDC interface
 * Callback running when CDC device have received data
 */
void usb_rx_notify(uint8_t port);

/*! \brief Configures communication line
 *
 * \param cfg      line configuration
 */
void uart_config(uint8_t port, usb_cdc_line_coding_t * cfg);

/*! \brief Opens communication line
 */
void uart_open(uint8_t port);

/*! \brief Closes communication line
 */
void uart_close(uint8_t port);

void processPacket(void);
void receiveUSBData(void);
void transmitUSBData(void);
void motorBusIntteruptController(uint8_t motorBus);

#endif // _UART_H_
