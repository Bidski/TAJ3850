#ifndef _MPU60X0_H_
#define _MPU60X0_H_

#include "RAM.h"

/********************************************
 *					ENUMS					*
 ********************************************/
typedef enum
{
	SENSOR_LOCATION_BODY	= 0x00,
	SENSOR_LOCATION_HEAD	= 0x01
} sensorLocations;

typedef enum
{
	SENSOR_TYPE_GYROSCOPE		= 0x00,
	SENSOR_TYPE_ACCELEROMETER	= 0x01,
	SENSOR_TYPE_TEMPERATURE		= 0x02
} sensorTypes;

typedef enum
{
	SENSOR_AXIS_X = 0x00,
	SENSOR_AXIS_Y = 0x02,
	SENSOR_AXIS_Z = 0x04
} sensorAxes;

/********************************************
 *				CONSTANTS					*
 ********************************************/
#define GRAVITY						9.80665												// Earths gravitational constant for converting accelerometer readings to SI units.
#define RADIANS_PER_DEGREE			0.0174532925										// 1 degree = 0.0174532925 radians. For converting gyroscope readings to SI units.
#define CONVERT_TEMP_READING(x)		(((int16_t)(x) / 0.0029411765) + 36.53)				// Convert temperature reading to degrees centigrade. C = (reading / 340) + 36.53.

#define MPU60X0_BASE_ADDRESS		0x67												// MPU60X0 TWI (7-bit) base address b1101000.
#define MPU60X0_ADDRESS_BODY		MPU60X0_BASE_ADDRESS + 0							// Address for MPU60X0 in the body  b1101000.
#define MPU60X0_ADDRESS_HEAD		MPU60X0_BASE_ADDRESS + 1							// Address for MPU60X0 in the head  b1101001.

#define MPU60X0_SPEED				400000		// Speed of TWI (400kHz)

#define MPU60X0_BODY_INT_PIN		PIO_PA25
#define MPU60X0_HEAD_INT_PIN		PIO_PA26
#define MPU60X0_BODY_INT_FUNCTION	(PIO_TYPE_PIO_INPUT | PIO_IT_RISE_EDGE | PIO_DEFAULT)
#define MPU60X0_HEAD_INT_FUNCTION	(PIO_TYPE_PIO_INPUT | PIO_IT_RISE_EDGE | PIO_DEFAULT)

#define NUM_SENSORS					0x03
#define NUM_SENSOR_LOCATIONS		0x02
#define NUM_SENSOR_READINGS_TO_AVG	0x10


// MPU Configuration options.
#define SMPRT_DIV					25			// Sample divider rate register.
#define SAMPLE_RATE_DIV				0			// 8-bit unsigned number.

#define MPU60X0_CONFIG				26			// Config register.

												// FSYNC Location Bit
#define EXT_SYNC_DISABLED			0			// Input disabled.
#define EXT_SYNC_TEMP_OUT_L			1			// TEMP_OUT_L[0]
#define EXT_SYNC_GYRO_XOUT_L		2			// GYRO_XOUT_L[0]
#define EXT_SYNC_GYRO_YOUT_L		3			// GYRO_YOUT_L[0]
#define EXT_SYNC_GYRO_ZOUT_L		4			// GYRO_ZOUT_L[0]
#define EXT_SYNC_ACCEL_XOUT_L		5			// ACCEL_XOUT_L[0]
#define EXT_SYNC_ACCEL_YOUT_L		6			// ACCEL_YOUT_L[0]
#define EXT_SYNC_ACCEL_ZOUT_L		7			// ACCEL_ZOUT_L[0]

												//	  Accelerometer		    Gyroscope			Fs
												// Bandwidth	Delay	Bandwidth	Delay
												//	  (Hz)		(ms)	   (Hz)		(ms)	  (kHz)
#define DLPF_CFG_BW_260				0			//	 260		  0		   256		 0.98		8
#define DLPF_CFG_BW_184				1			//	 184		  2		   188		 1.9		1
#define DLPF_CFG_BW_94				2			//	  94		  3		    98		 2.8		1
#define DLPF_CFG_BW_44				3			//	  44		 4.9		42		 4.8		1
#define DLPF_CFG_BW_21				4			//	  21		 8.5		20		 8.3		1
#define DLPF_CFG_BW_10				5			//	  10		13.8		10		13.4		1
#define DLPF_CFG_BW_5				6			//	   5		19.0		 5		18.6		1
#define DLPF_CFG_BW_RESERVED		7

#define EXT_SYNC_SET				EXT_SYNC_DISABLED
#define DLPF_CFG					DLPF_CFG_BW_260

#define MPU60X0_GYRO_CONFIG			27			// Gyroscope config register.

												// Gyroscope Full Scale Range
#define GYRO_FS_SEL_250				0			// +/- 250  degrees/second
#define GYRO_FS_SEL_500				1			// +/- 500  degrees/second
#define GYRO_FS_SEL_1000			2			// +/- 1000 degrees/second
#define GYRO_FS_SEL_2000			3			// +/- 2000 degrees/second	

#define GYRO_SELF_TEST_X			0			// Set to enable self-test function.
#define GYRO_SELF_TEST_Y			0
#define GYRO_SELF_TEST_Z			0

#define GYRO_FS_SEL					GYRO_FS_SEL_500

#define MPU60X0_ACCEL_CONFIG		28			// Accelerometer config register.

												// Accelerometer Full Scale Range
#define ACCEL_FS_SEL_2				0			// +/- 2  g
#define ACCEL_FS_SEL_4				1			// +/- 4  g
#define ACCEL_FS_SEL_8				2			// +/- 8  g
#define ACCEL_FS_SEL_16				3			// +/- 16 g	

#define ACCEL_SELF_TEST_X			0			// Set to enable self-test function.
#define ACCEL_SELF_TEST_Y			0
#define ACCEL_SELF_TEST_Z			0

#define ACCEL_FS_SEL				ACCEL_FS_SEL_8

#define MPU60X0_FIFO_EN				35			// FIFO enable register.

#define TEMP_FIFO_EN				0			// Set to enable writing of corresponding registers (H and L) to the FIFO.
#define XG_FIFO_EN					0
#define YG_FIFO_EN					0
#define ZG_FIFO_EN					0
#define ACCEL_FIFO_EN				0
#define SLV2_FIFO_EN				0
#define SLV1_FIFO_EN				0
#define SLV0_FIFO_EN				0

#define MPU60X0_I2C_MST_CTRL		36			// I2C master control register.

#define MUL_MST_EN					0			// Set to enable multi-master capability.
#define WAIT_FOR_ES					0			// Set to delay Data Ready Interrupt until External Sensor data is ready.
#define SLV3_FIFO_EN				0			// See FIFO_EN register.
#define I2C_MST_P_NSR				0			// Controls I2C Masters transition from one slave read to the next slave read. 0 = restart be tween reads.
#define I2C_MST_CLK					0			// 4-bit unsigned value. Configure I2C master clock speed divider.

#define MPU60X0_INT_PIN_CFG			55			// INT Pin/Bypass enable register.

#define INT_LEVEL					0			// 0 = INT pin is active high.
#define INT_OPEN					0			// 0 = INT pin is push-pull. 1 = open drain.
#define LATCH_INT_EN				0			// 0 = INT pin emits 50us pulse. 1 = INT pin high until cleared.
#define INT_RD_CLEAR				0			// 0 = Clear interrupts by reading INT_STATUS register. 1 = Clear interrupts on any read operation.
#define FSYNC_INT_LEVEL				0			// 0 = FSYNC pin (when used as an interrupt for host processor) is active high.
#define FSYNC_INT_EN				0			// 0 = Prevent FSYNC pin from causing an interrupt.
#define I2C_BYPASS_EN				0			// 0 = Host processor cannot directly access the auxiliary I2C bus.

#define MPU60X0_INT_ENABLE			56			// Interrupt Enable register.

#define FIFO_OFLOW_EN				0			// 0 = Do not generate an interrupt when FIFO overflows.
#define I2C_MST_INT_EN				0			// 0 = Prevent all of the I2C Master Interrupt sources from generating an interrupt.
#define DATA_RDY_EN					1			// 1 = Enable Data Ready interrupt (occurs when all sensor data registers have been written to).

#define MPU60X0_INT_STATUS			58			// Interrupt Status register. (READ ONLY)

#define FIFO_OFLOW_INT				0x08		// Indicates whether a FIFO Overflow interrupt has occurred.
#define I2C_MST_INT_INT				0x04		// Indicates whether a I2C Master interrupt has occurred.
#define DATA_RDY_INT				0x01		// Indicates whether a Data Ready interrupt has occurred.

#define ACCEL_XOUT_H				59			// Accelerometer measurement registers. (READ ONLY)
#define ACCEL_XOUT_L				60			// Data stored in 16-bit 2's complement form.
#define ACCEL_YOUT_H				61
#define ACCEL_YOUT_L				62
#define ACCEL_ZOUT_H				63
#define ACCEL_ZOUT_L				64

#define TEMP_OUT_H					65			// Temperature measurement registers. (READ ONLY)
#define TEMP_OUT_L					66			// 16-bit signed value.
												// Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53

#define GYRO_XOUT_H					67			// Gyroscope measurement registers. (READ ONLY)
#define GYRO_XOUT_L					68			// Data stored in 16-bit 2's complement form.
#define GYRO_YOUT_H					69
#define GYRO_YOUT_L					70
#define GYRO_ZOUT_H					71
#define GYRO_ZOUT_L					72

#define MPU60X0_SIGNAL_PATH_RESET	104			// Signal path reset register. (WRITE ONLY)
#define GYRO_RESET					0x04		// Use to reset the analog and digital signal paths of the gyroscope, accelerometer, and temperature sensors (does not clear registers).
#define ACCEL_RESET					0x02
#define TEMP_RESET					0x01

#define MPU60X0_USER_CTRL			106			// User Control register.

#define MPU60X0_PWR_MGMT_1			107			// Power Management 1 register.
#define DEVICE_RESET				0x80		// Reset the device. Reverts all registers to their factory defaults.
#define MPU_SLEEP					0x40		// Put MPU into sleep mode.
#define CYCLE						0x20		// CYCLE = 1, SLEEP = 0: MPU cycles between sleep mode and waking up to take a single sample from active sensors.
#define TEMP_DIS					0x08		// Set to disable temperature sensor.

#define CLKSEL_INTERNAL				0
#define CLKSEL_PLL_GYRO_X			1
#define CLKSEL_PLL_GYRO_Y			2
#define CLKSEL_PLL_GYRO_Z			3
#define CLKSEL_PLL_EXT_32K			4
#define CLKSEL_PLL_EXT_19M			5
#define CLKSEL_PLL_RESERVED			6
#define CLKSEL_RESET				7

#define CLKSEL						CLKSEL_PLL_GYRO_X

#define MPU60X0_PWR_MGMT_2			108			// Power Management 2 register.
#define STBY_XA						0x20		// Put Accelerometer X-axis into standby mode.
#define STBY_YA						0x10		// Put Accelerometer Y-axis into standby mode.
#define STBY_ZA						0x08		// Put Accelerometer Z-axis into standby mode.
#define STBY_XG						0x04		// Put Gyroscope X-axis into standby mode.
#define STBY_YG						0x02		// Put Gyroscope Y-axis into standby mode.
#define STBY_ZG						0x01		// Put Gyroscope Z-axis into standby mode.

// Defines the rate at which the MPU cycles between sleep and awake. (Bits 6 and 7 of PWR_MGMT_2).
#define LP_WAKE_CTRL_1_25			0			// 1.25 Hz
#define LP_WAKE_CTRL_5				1			// 5 Hz
#define LP_WAKE_CTRL_20				2			// 20 Hz
#define LP_WAKE_CTRL_40				3			// 40 Hz

#define FIFO_COUNT_H				114			// FIFO count registers. (READ ONLY)
#define FIFO_COUNT_L				115			// 16-bit unsigned value. Counts the number of bytes currently in the FIFO buffer.

#define FIFO_R_W					116			// FIFO Read Write. Use to read data from the FIFO. Reads in order of register number (register 73 to 96 depending on which are enabled).
	
#define WHO_AM_I					117			// Use to verify device identity. Should contain 0x68 = b0110100

#define MPU60X0_CONFIG_BYTE				(EXT_SYNC_SET << 3) | DLPF_CFG
#define MPU60X0_GYRO_CONFIG_BYTE		(GYRO_SELF_TEST_X << 7) | (GYRO_SELF_TEST_Y << 6) | (GYRO_SELF_TEST_Z << 5) | (GYRO_FS_SEL << 3)
#define MPU60X0_ACCEL_CONFIG_BYTE		(ACCEL_SELF_TEST_X << 7) | (ACCEL_SELF_TEST_Y << 6) | (ACCEL_SELF_TEST_Z << 5) | (ACCEL_FS_SEL << 3)
#define MPU60X0_FIFO_EN_BYTE			(TEMP_FIFO_EN << 7) | (XG_FIFO_EN << 6) | (YG_FIFO_EN << 5) | (ZG_FIFO_EN << 4) | (ACCEL_FIFO_EN << 3) | (SLV2_FIFO_EN << 2) |  (SLV1_FIFO_EN << 1) |  SLV0_FIFO_EN
#define MPU60X0_I2C_MST_CTRL_BYTE		(MUL_MST_EN << 7) | (WAIT_FOR_ES << 6) | (SLV3_FIFO_EN << 5) | (I2C_MST_P_NSR << 4) | I2C_MST_CLK
#define MPU60X0_INT_PIN_CFG_BYTE		(INT_LEVEL << 7) | (INT_OPEN << 6) | (LATCH_INT_EN << 5) | (INT_RD_CLEAR << 4) | (FSYNC_INT_LEVEL << 3) | (FSYNC_INT_EN << 2) | (I2C_BYPASS_EN << 1)
#define MPU60X0_INT_ENABLE_BYTE			(FIFO_OFLOW_EN << 4) | (I2C_MST_INT_EN << 3) | DATA_RDY_EN
#define MPU60X0_PWR_MGMT_1_BYTE			DEVICE_RESET | CLKSEL
#define MPU60X0_SIGNAL_PATH_RESET_BYTE	GYRO_RESET | ACCEL_RESET | TEMP_RESET
#define MPU60X0_MPU60X0_SLEEP_MODE		MPU_SLEEP
#define MPU60X0_PWR_MGMT_2_BYTE			STBY_XA | STBY_YA | STBY_ZA | STBY_XG | STBY_YG | STBY_ZG

// RESET VALUE FOR ALL REGISTERS: 0x00
// RESET VALUE FOR REGISTER PWR_MGMT_1 (107):  0x40
// RESET VALUE FOR REGISTER WHO_AM_I   (117):  0x68

/********************************************
 *			FUNCTION PROTOTYPES				*
 ********************************************/
int		initMPU60X0(void);
int		configureMPU60X0(int8_t mpuAddress, int8_t gyroFullScale, int8_t accelFullScale, int8_t DPLFConfig);
int		disableMPU60X0(void);
int8_t	sendMPUPacket(int8_t mpuAddress, uint8_t mpuRegisterAddress, volatile void *data, uint8_t dataLength);
int		requestMPUPacket(int8_t mpuAddress, uint8_t mpuRegisterAddress, volatile void *data, uint8_t dataLength);

void	MPU60X0HeadInterrupt(uint32_t a, uint32_t b);
void	MPU60X0BodyInterrupt(uint32_t a, uint32_t b);
void	MPU60X0IntteruptController(uint8_t mpuAddress, uint8_t sensor, uint8_t *gyroReadingOffset, uint8_t *accelReadingOffset, uint8_t *tempReadingOffset);

void	storeAveragedReadings(void);

/********************************************
 *				GLOBALS						*
 ********************************************/
static volatile uint8_t					 gyroReadings[NUM_SENSOR_LOCATIONS][(NUM_SENSOR_READINGS_TO_AVG * 3) << 1]	= {[0 ... (NUM_SENSOR_LOCATIONS - 1)] = {0}};		// Gyroscope readings from the head and body (hi and lo bytes).
static volatile uint8_t					accelReadings[NUM_SENSOR_LOCATIONS][(NUM_SENSOR_READINGS_TO_AVG * 3) << 1]	= {[0 ... (NUM_SENSOR_LOCATIONS - 1)] = {0}};		// Accelerometer readings from the head and body (hi and lo bytes).
static volatile uint8_t					 tempReadings[NUM_SENSOR_LOCATIONS][(NUM_SENSOR_READINGS_TO_AVG * 1) << 1]	= {[0 ... (NUM_SENSOR_LOCATIONS - 1)] = {0}};		// Temperature readings from the head and body (hi and lo bytes).


#endif // _MPU60X0_H_