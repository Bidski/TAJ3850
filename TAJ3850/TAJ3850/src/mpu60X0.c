#include "main.h"
#include "mpu60X0.h"

int initMPU60X0(void)
{
	twi_options_t twi_options;
	
	twi_options.pba_hz = FOSC0;
	twi_options.speed  = MPU60X0_SPEED;
	twi_options.chip   = MPU60X0_BASE_ADDRESS;

	// Enable External Interrupt Controller Line.
	eic_enable_line(&AVR32_EIC, MPU60X0_BODY_INT_LINE);
	eic_enable_line(&AVR32_EIC, MPU60X0_HEAD_INT_LINE);

	// Enable interrupts with priority higher than USB.
	irq_register_handler(MPU60X0BodyInterrupt, MPU60X0_BODY_INT_IRQ, 3);
	irq_register_handler(MPU60X0HeadInterrupt, MPU60X0_HEAD_INT_IRQ, 3);

	// Initialize TWI driver with options.
	if (TWI_SUCCESS != twi_master_init(&AVR32_TWI, &twi_options))
	{
		// Disable the interrupts.
		eic_disable_line(&AVR32_EIC, MPU60X0_HEAD_INT_LINE);
		eic_disable_line(&AVR32_EIC, MPU60X0_BODY_INT_LINE);
	
		return(-1);
	}

	// Enable both RX and TX
	(&AVR32_TWI)->ier = AVR32_TWI_IER_TXRDY_MASK | AVR32_TWI_IER_RXRDY_MASK;

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

int8_t sendMPUPacket(int8_t mpuAddress, uint8_t mpuRegisterAddress, volatile void *data, uint8_t dataLength)
{
	twi_package_t packet;
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

	if (TWI_SUCCESS != twi_master_write(&AVR32_TWI, &packet))
	{
		bError = -1;
	}
	
	return(bError);
}

int requestMPUPacket(int8_t mpuAddress, uint8_t mpuRegisterAddress, volatile void *data, uint8_t dataLength)
{
	twi_package_t packet;
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

	if (TWI_SUCCESS != twi_master_read(&AVR32_TWI, &packet))
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

__attribute__((__interrupt__)) void MPU60X0BodyInterrupt(void)
{
	static uint8_t  gyroReadingOffset = 0;
	static uint8_t accelReadingOffset = 0;
	static uint8_t  tempReadingOffset = 0;
	
	MPU60X0IntteruptController(MPU60X0_ADDRESS_BODY, 0, &gyroReadingOffset, &accelReadingOffset, &tempReadingOffset);	
	
	// Mark interrupt as handled.
	eic_clear_interrupt_line(&AVR32_EIC, MPU60X0_BODY_INT_LINE);
}

__attribute__((__interrupt__)) void MPU60X0HeadInterrupt(void)
{
	static uint8_t  gyroReadingOffset = 0;
	static uint8_t accelReadingOffset = 0;
	static uint8_t  tempReadingOffset = 0;
	
	MPU60X0IntteruptController(MPU60X0_ADDRESS_HEAD, 1, &gyroReadingOffset, &accelReadingOffset, &tempReadingOffset);
	
	// Mark interrupt as handled.
	eic_clear_interrupt_line(&AVR32_EIC, MPU60X0_HEAD_INT_LINE);
}

void storeAveragedReadings(sensorLocations sensorLocation, sensorTypes sensorType, sensorAxes sensorAxis)
{
	float result = 0.0;
	uint8_t i;
	
	switch (sensorType)
	{
		case SENSOR_TYPE_GYROSCOPE:
		{
			for (i = 0; i < NUM_SENSOR_READINGS_TO_AVG; i = i + 6)
			{
				result += (float)((gyroReadings[sensorLocation][i + sensorAxis] << 8) | gyroReadings[sensorLocation][i + sensorAxis]);
			}
			
			if (sensorLocation == SENSOR_LOCATION_BODY)
			{
				*((float *)(RAM + GYRO_X_B0)) = result / NUM_SENSOR_READINGS_TO_AVG;
			}
	
			else
			{
				*((float *)(RAM + GYRO_X_H0)) = result / NUM_SENSOR_READINGS_TO_AVG;
			}

			break;
		}
		
		case SENSOR_TYPE_ACCELEROMETER:
		{
			for (i = 0; i < NUM_SENSOR_READINGS_TO_AVG; i = i + 6)
			{
				result += (float)((accelReadings[sensorLocation][i + sensorAxis] << 8) | accelReadings[sensorLocation][i + sensorAxis]);
			}
			
			if (sensorLocation == SENSOR_LOCATION_BODY)
			{
				*((float *)(RAM + ACCEL_X_B0)) = result / NUM_SENSOR_READINGS_TO_AVG;
			}
			
			else
			{
				*((float *)(RAM + ACCEL_X_H0)) = result / NUM_SENSOR_READINGS_TO_AVG;
			}

			break;
		}
		
		case SENSOR_TYPE_TEMPERATURE:
		{
			for (i = 0; i < NUM_SENSOR_READINGS_TO_AVG; i = i + 2)
			{
				result += (float)((tempReadings[sensorLocation][i + sensorAxis] << 8) | tempReadings[sensorLocation][i + sensorAxis]);
			}
			
			if (sensorLocation == SENSOR_LOCATION_BODY)
			{
				*((float *)(RAM + TEMP_B0)) = result / NUM_SENSOR_READINGS_TO_AVG;
			}
			
			else
			{
				*((float *)(RAM + TEMP_H0)) = result / NUM_SENSOR_READINGS_TO_AVG;
			}
			
			break;
		}
		
		default:
		{
			break;
		}
	}
	
//	return(result / NUM_SENSOR_READINGS_TO_AVG);
}