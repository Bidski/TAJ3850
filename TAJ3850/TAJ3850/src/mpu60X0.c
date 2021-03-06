#include "main.h"
#include "mpu60X0.h"

int initMPU60X0(void)
{
	twi_options_t twi_options;
	
	twi_options.master_clk	= BOARD_MCK;
	twi_options.speed		= MPU60X0_SPEED;
	twi_options.chip		= MPU60X0_BASE_ADDRESS;

	// Enable External Interrupt Controller Line.
	// Enable interrupts with priority higher than USB.
	pio_set_input(PIOA, MPU60X0_BODY_INT_PIN, MPU60X0_BODY_INT_FUNCTION);
	pio_configure_interrupt(PIOA, MPU60X0_BODY_INT_PIN, MPU60X0_BODY_INT_FUNCTION);
	pio_handler_set(PIOA, ID_PIOA, MPU60X0_BODY_INT_PIN, PIO_IT_RISE_EDGE, MPU60X0BodyInterrupt);
	
	pio_set_input(PIOA, MPU60X0_HEAD_INT_PIN, MPU60X0_HEAD_INT_FUNCTION);
	pio_configure_interrupt(PIOA, MPU60X0_HEAD_INT_PIN, MPU60X0_HEAD_INT_FUNCTION);
	pio_handler_set(PIOA, ID_PIOA, MPU60X0_HEAD_INT_PIN, PIO_IT_RISE_EDGE, MPU60X0HeadInterrupt);
	
	NVIC_EnableIRQ(PIOA_IRQn);
	pio_enable_interrupt(PIOA, MPU60X0_BODY_INT_PIN);
	pio_enable_interrupt(PIOA, MPU60X0_HEAD_INT_PIN);
	
	// Initialize TWI driver with options.
	if (TWI_SUCCESS != twi_master_init(TWI0, &twi_options))
	{
		// Disable the interrupts.
		pio_disable_interrupt(PIOA, MPU60X0_BODY_INT_PIN);
		pio_disable_interrupt(PIOA, MPU60X0_HEAD_INT_PIN);
	
		return(-1);
	}

	// Enable both RX and TX
	TWI0->TWI_IER = TWI_IER_TXRDY | TWI_IER_RXRDY;

	return(0);
}

int configureMPU60X0(int8_t mpuAddress, int8_t gyroFullScale, int8_t accelFullScale, int8_t DPLFConfig)
{
	uint8_t data;
	
	// Send a device reset and tell the device to derive its clock from a PLL using the X-axis of the gyroscope as a base.
	data = MPU60X0_PWR_MGMT_1_BYTE;
	
	if (sendMPUPacket(mpuAddress, MPU60X0_PWR_MGMT_1, &data, 1) != 0)
	{
		return(-1);
	}
	
	// Delay for 100ms.
	cpu_delay_ms(100, sysclk_get_cpu_hz());

	// Send the signal path reset byte.
	// Just to ensure the entire device has been reset.
	data = MPU60X0_SIGNAL_PATH_RESET_BYTE;
	
	if (sendMPUPacket(mpuAddress, MPU60X0_SIGNAL_PATH_RESET, &data, 1) != 0)
	{
		return(-1);
	}
	
	// Delay for 100ms.
	cpu_delay_ms(100, sysclk_get_cpu_hz());

	// Send the configuration byte.
	// Set EXT_SYNC off and DLPF to 260Hz bandwidth or a user defined value.
	data = (DPLFConfig > -1) ? (((MPU60X0_CONFIG_BYTE) & 0xF8) | DPLFConfig) : MPU60X0_CONFIG_BYTE;
	
	if (sendMPUPacket(mpuAddress, MPU60X0_CONFIG, &data, 1) != 0)
	{
		return(-1);
	}
	
	// Configure the gyroscope.
	// Set the full scale range of the gyroscope to GYRO_FS_SEL (defined in header file) or a user defined value.
	data = (gyroFullScale > -1) ? (((MPU60X0_GYRO_CONFIG_BYTE) & 0xE3) | (gyroFullScale << 3)) : MPU60X0_GYRO_CONFIG_BYTE;
	
	if (sendMPUPacket(mpuAddress, MPU60X0_GYRO_CONFIG, &data, 1) != 0)
	{
		return(-1);
	}
	
	// Configure the accelerometer.
	// Set the full scale range of the accelerometer to ACCEL_FS_SEL (defined in header file) or a user defined value.
	data = (accelFullScale > -1) ? (((MPU60X0_ACCEL_CONFIG_BYTE) & 0xE3) | (accelFullScale << 3)) : MPU60X0_ACCEL_CONFIG_BYTE;
	
	if (sendMPUPacket(mpuAddress, MPU60X0_ACCEL_CONFIG, &data, 1) != 0)
	{
		return(-1);
	}
	
	// Configure the interrupt generation byte.
	// Set the MPU60X0 to only generate interrupts when a set of sensor readings are available.
	data = MPU60X0_INT_ENABLE_BYTE;
	
	if (sendMPUPacket(mpuAddress, MPU60X0_INT_ENABLE, &data, 1) != 0)
	{
		return(-1);
	}

	return(0);
}

int disableMPU60X0(void)
{
	uint8_t data;
	
	// Put MPUs into sleep mode.
	data = MPU60X0_MPU60X0_SLEEP_MODE;
			
	if (sendMPUPacket(MPU60X0_ADDRESS_BODY, MPU60X0_PWR_MGMT_1, &data, 1) != 0)
	{
		return(-1);
	}

	if (sendMPUPacket(MPU60X0_ADDRESS_HEAD, MPU60X0_PWR_MGMT_1, &data, 1) != 0)
	{
		return(-1);
	}

	// Close TWI/I2C port.
	TWI0->TWI_IDR = 0xFFFFFFFF;

	// Disable the interrupts.
	pio_disable_interrupt(PIOA, MPU60X0_BODY_INT_PIN);
	pio_disable_interrupt(PIOA, MPU60X0_HEAD_INT_PIN);
}

int8_t sendMPUPacket(int8_t mpuAddress, uint8_t mpuRegisterAddress, volatile void *data, uint8_t dataLength)
{
	twi_packet_t packet;
	int8_t bError = 0;
	
	// Address the MPU in the body.
	packet.chip = mpuAddress;
	
	// We want to write to the PWR_MGMT_1 register.
	packet.addr[0] = mpuRegisterAddress;
	packet.addr[1] = 0;
	packet.addr[2] = 0;
	
	// It is a 1 byte address.
	packet.addr_length = 1;

	// Write a byte telling the MPU to reset and use a PLL clock derived
	// from the X-axis of the gyroscope.
	packet.buffer = (void *)data;

	// How many bytes do we want to write.
	packet.length = dataLength;

	if (TWI_SUCCESS != twi_master_write(TWI0, &packet))
	{
		bError = -1;
	}
	
	return(bError);
}

int requestMPUPacket(int8_t mpuAddress, uint8_t mpuRegisterAddress, volatile void *data, uint8_t dataLength)
{
	twi_packet_t packet;
	int8_t bError = 0;

	// Address the MPU in the body.
	packet.chip = mpuAddress;
	
	// We want to write to the PWR_MGMT_1 register.
	packet.addr[0] = mpuRegisterAddress;
	packet.addr[1] = 0;
	packet.addr[2] = 0;
	
	// It is a 1 byte address.
	packet.addr_length = 1;

	// Write a byte telling the MPU to reset and use a PLL clock derived
	// from the X-axis of the gyroscope.
	packet.buffer = (void *)data;

	// How many bytes do we want to write.
	packet.length = dataLength;

	if (TWI_SUCCESS != twi_master_read(TWI0, &packet))
	{
		bError = -1;
	}
	
	return(bError);
}

void MPU60X0IntteruptController(uint8_t mpuAddress, uint8_t sensor, uint8_t *gyroReadingOffset, uint8_t *accelReadingOffset, uint8_t *tempReadingOffset)
{
	// Request sensor readings.
	requestMPUPacket(mpuAddress,  GYRO_XOUT_H,  &gyroReadings[sensor][ *gyroReadingOffset + 0], 1);
	requestMPUPacket(mpuAddress,  GYRO_XOUT_L,  &gyroReadings[sensor][ *gyroReadingOffset + 1], 1);
	requestMPUPacket(mpuAddress,  GYRO_YOUT_H,  &gyroReadings[sensor][ *gyroReadingOffset + 2], 1);
	requestMPUPacket(mpuAddress,  GYRO_YOUT_L,  &gyroReadings[sensor][ *gyroReadingOffset + 3], 1);
	requestMPUPacket(mpuAddress,  GYRO_ZOUT_H,  &gyroReadings[sensor][ *gyroReadingOffset + 4], 1);
	requestMPUPacket(mpuAddress,  GYRO_ZOUT_L,  &gyroReadings[sensor][ *gyroReadingOffset + 5], 1);
	requestMPUPacket(mpuAddress, ACCEL_XOUT_H, &accelReadings[sensor][*accelReadingOffset + 0], 1);
	requestMPUPacket(mpuAddress, ACCEL_XOUT_L, &accelReadings[sensor][*accelReadingOffset + 1], 1);
	requestMPUPacket(mpuAddress, ACCEL_YOUT_H, &accelReadings[sensor][*accelReadingOffset + 2], 1);
	requestMPUPacket(mpuAddress, ACCEL_YOUT_L, &accelReadings[sensor][*accelReadingOffset + 3], 1);
	requestMPUPacket(mpuAddress, ACCEL_ZOUT_H, &accelReadings[sensor][*accelReadingOffset + 4], 1);
	requestMPUPacket(mpuAddress, ACCEL_ZOUT_L, &accelReadings[sensor][*accelReadingOffset + 5], 1);
	requestMPUPacket(mpuAddress,   TEMP_OUT_H,  &tempReadings[sensor][ *tempReadingOffset + 0], 1);
	requestMPUPacket(mpuAddress,   TEMP_OUT_L,  &tempReadings[sensor][ *tempReadingOffset + 1], 1);
	
	*gyroReadingOffset	+= 6;
	*accelReadingOffset	+= 6;
	*tempReadingOffset	+= 2;
	
	if (*gyroReadingOffset >= ((NUM_SENSOR_READINGS_TO_AVG * 6) << 1))
	{
		*gyroReadingOffset = 0;
	}
	
	if (*accelReadingOffset >= ((NUM_SENSOR_READINGS_TO_AVG * 6) << 1))
	{
		*accelReadingOffset = 0;
	}
	
	if (*tempReadingOffset >= ((NUM_SENSOR_READINGS_TO_AVG * 1) << 1))
	{
		*tempReadingOffset = 0;
	}
}

void MPU60X0BodyInterrupt(uint32_t a, uint32_t b)
{
	static uint8_t  gyroReadingOffset = 0;
	static uint8_t accelReadingOffset = 0;
	static uint8_t  tempReadingOffset = 0;
	
	MPU60X0IntteruptController(MPU60X0_ADDRESS_BODY, 0, &gyroReadingOffset, &accelReadingOffset, &tempReadingOffset);	
	
	// Mark interrupt as handled.
	pio_disable_interrupt(PIOA, MPU60X0_BODY_INT_PIN);

	// Average the stored readings.
	storeAveragedReadings();
}

void MPU60X0HeadInterrupt(uint32_t a, uint32_t b)
{
	static uint8_t  gyroReadingOffset = 0;
	static uint8_t accelReadingOffset = 0;
	static uint8_t  tempReadingOffset = 0;
	
	MPU60X0IntteruptController(MPU60X0_ADDRESS_HEAD, 1, &gyroReadingOffset, &accelReadingOffset, &tempReadingOffset);
	
	// Mark interrupt as handled.
	pio_disable_interrupt(PIOA, MPU60X0_HEAD_INT_PIN);

	// Average the stored readings.
	storeAveragedReadings();
}

void storeAveragedReadings(void)
{
	float result = 0.0;
	sensorLocations sensorLocation;
	sensorAxes sensorAxis;
	uint8_t reading;

	for (sensorLocation = SENSOR_LOCATION_BODY; sensorLocation != SENSOR_LOCATION_HEAD; sensorLocation++)
	{
		for (sensorAxis = SENSOR_AXIS_X; sensorAxis != SENSOR_AXIS_Z; sensorAxis += 2)
		{
			for (reading = 0; reading < NUM_SENSOR_READINGS_TO_AVG; reading += 6)
			{
				result += (float)((gyroReadings[sensorLocation][reading + sensorAxis] << 8) | gyroReadings[sensorLocation][reading + sensorAxis + 1]);
			}

			// Average the readings and convert to SI units.
			result = result * (GRAVITY / NUM_SENSOR_READINGS_TO_AVG);
			
			// (RAM + (GYRO_X_B0 + sensorAxis) + (sensorLocation * (GYRO_X_H0 - GYRO_X_B0))))
			// RAM_Start_Address + (Sensor_Axis_Offset) + (Sensor_Location_Offset)
			flash_write((RAM + (GYRO_X_B0 + sensorAxis) + (sensorLocation * (GYRO_X_H0 - GYRO_X_B0))), &result, sizeof(float), false);
			
			result = 0.0;
		}
	}
	
	for (sensorLocation = SENSOR_LOCATION_BODY; sensorLocation != SENSOR_LOCATION_HEAD; sensorLocation++)
	{
		for (sensorAxis = SENSOR_AXIS_X; sensorAxis != SENSOR_AXIS_Z; sensorAxis += 2)
		{
			for (reading = 0; reading < NUM_SENSOR_READINGS_TO_AVG; reading += 6)
			{
				result += (float)((accelReadings[sensorLocation][reading + sensorAxis] << 8) | accelReadings[sensorLocation][reading + sensorAxis + 1]);
			}
			
			// Average the readings and convert to SI units.
			result = result * (RADIANS_PER_DEGREE / NUM_SENSOR_READINGS_TO_AVG);
			
			// (RAM + (ACCEL_X_B0 + sensorAxis) + (sensorLocation * (ACCEL_X_H0 - ACCEL_X_B0))))
			// RAM_Start_Address + (Sensor_Axis_Offset) + (Sensor_Location_Offset)
			flash_write((RAM + (ACCEL_X_B0 + sensorAxis) + (sensorLocation * (ACCEL_X_H0 - ACCEL_X_B0))), &result, sizeof(float), false);
			
			result = 0.0;
		}
	}
	
	for (sensorLocation = SENSOR_LOCATION_BODY; sensorLocation != SENSOR_LOCATION_HEAD; sensorLocation++)
	{
		for (reading = 0; reading < NUM_SENSOR_READINGS_TO_AVG; reading += 2)
		{
			// Convert readings to SI units.
			result += (float)CONVERT_TEMP_READING(((tempReadings[sensorLocation][reading + sensorAxis] << 8) | tempReadings[sensorLocation][reading + sensorAxis + 1]));
		}
			
		// Average the readings.
		result /= NUM_SENSOR_READINGS_TO_AVG;
			
		// (RAM + TEMP_B0 + (sensorLocation * (TEMP_H0 - TEMP_B0))))
		// RAM_Start_Address + Sensor_Offset + (Sensor_Location_Offset)
		flash_write((RAM + TEMP_B0 + (sensorLocation * (TEMP_H0 - TEMP_B0))), &result, sizeof(float), false);
			
		result = 0.0;
	}
}