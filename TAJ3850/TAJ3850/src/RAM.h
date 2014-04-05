/*
 * RAM.h
 *
 * Created: 19/02/2014 9:17:37 AM
 *  Author: student
 */ 


#ifndef RAM_H_
#define RAM_H_

/*
	00 (0x00) Model Number(L)			Default: 0x0A. (READ ONLY)
	01 (0x01) Model Number(H)			Default: 0x0F. (READ ONLY)
	02 (0x02) Firmware Version			Default: 0x01. (READ ONLY)
	03 (0x03) Dynamixel ID				Default: 200 (0xC8).
	04 (0x04) Dynamixel Baud Rate		Default: 0x06 (0x01 = 0.5Mbps, 0x02 = 1Mbps, 0x03 = 1.5bps, 0x04 = 2Mbps, 0x05 = 2.5Mbps, 0x06 = 3Mbps).
	----------------------------
	20 (0X14) Dynamixel Power			0 = Dynamixels OFF.
	21 (0X15) LED Panel					Bits 7-3 UNUSED. Bit 2 = LED4 Status. Bit 1 = LED3 Status. Bit 0 = LED2 Status.
	22 (0X16) LED 5 (L)					Bit 15 UNUSED.
	23 (0X17) LED 5 (H)					Bits 14-10 Blue value. Bits 9-5 Green value. Bits 4-0 Red value.
	24 (0X18) LED 6 (L)					Bit 15 UNUSED.
	25 (0X19) LED 6 (H)					Bits 14-10 Blue value. Bits 9-5 Green value. Bits 4-0 Red value.
	26 (0X1A) Button					Bits 7-2 UNUSED. Bit 1 = Status of START button. Bit 0 = Status of MODE button. (READ ONLY)
	27 (0X1B) MPU Settings				Bit 7 UNUSED. Bits 6-4 DLPF. Bits 3-2 Gyro. Bits 1-0 Accelerometer.
	----------------------------
	BEGIN READ ONLY
	40 (0x28) Gyro_X_B					32-bit float. Gyroscope X-Axis reading.
	41 (0x29)
	42 (0x2A)
	43 (0x2B)
	44 (0x2C) Gyro_Y_B					32-bit float. Gyroscope Y-Axis reading.
	45 (0x2D)
	46 (0x2E)
	47 (0x2F)
	48 (0x30) Gyro_Z_B					32-bit float. Gyroscope Z-Axis reading.
	49 (0x31)
	50 (0x32)
	51 (0x33)
	52 (0x34) Acc_X_B					32-bit float. Accelerometer X-Axis reading.
	53 (0x35)
	54 (0x36)
	55 (0x37)
	56 (0x38) Acc_Y_B					32-bit float. Accelerometer Y-Axis reading.
	57 (0x39)
	58 (0x3A)
	59 (0x3B)
	60 (0x3C) Acc_Z_B					32-bit float. Accelerometer Z-Axis reading.
	61 (0x3D)
	62 (0x3E)
	63 (0x3F)
	64 (0x40) Temp_B					32-bit float. Temperature reading in degrees celcius.
	65 (0x41)
	66 (0x42)
	67 (0x43)
	68 (0x44) Gyro_X_H					32-bit float. Gyroscope X-Axis reading.
	69 (0x45)
	70 (0x46)
	71 (0x47)
	72 (0x48) Gyro_Y_H					32-bit float. Gyroscope Y-Axis reading.
	73 (0x49)
	74 (0x4A)
	75 (0x4B)
	76 (0x4C) Gyro_Z_H					32-bit float. Gyroscope Z-Axis reading.
	77 (0x4D)
	78 (0x4E)
	79 (0x4F)
	80 (0x50) Acc_X_H					32-bit float. Accelerometer X-Axis reading.
	81 (0x51)
	82 (0x52)
	83 (0x53)
	84 (0x54) Acc_Y_H					32-bit float. Accelerometer Y-Axis reading.
	85 (0x55)
	86 (0x56)
	87 (0x57)
	88 (0x58) Acc_Z_H					32-bit float. Accelerometer Z-Axis reading.
	89 (0x59)
	90 (0x5A)
	91 (0x5B)
	92 (0x5C) Temp_H					32-bit float. Temperature reading in degrees celcius.
	93 (0x5D)
	94 (0x5E)
	95 (0x5F)
	END READ ONLY
*/

// We only need to store model number, firmware version, dynamixel id, and baud rate in flash memory.
// All other fields are there to provide a transparent interface for the ODROID.
//#define RAM_TABLE_LAST_ADDRESS		0x5F

#define RAM_TABLE_SIZE				0x60
#define MODEL_NUMBER_L				0x00
#define MODEL_NUMBER_H				0x01
#define FIRMWARE_VERSION			0x02
#define DYNAMIXEL_ID				0x03
#define BAUD_RATE					0x04

#define DYNAMIXEL_POWER				0x14
#define LED_PANEL					0x15
#define LED_5_L						0x16
#define LED_5_H						0x17
#define LED_6_L						0x18
#define LED_6_H						0x19
#define BUTTON						0x1A

#define GYRO_X_B0					0x28
#define GYRO_X_B1					0x29
#define GYRO_X_B2					0x2A
#define GYRO_X_B3					0x2B
#define GYRO_Y_B0					0x2C
#define GYRO_Y_B1					0x2D
#define GYRO_Y_B2					0x2E
#define GYRO_Y_B3					0x2F
#define GYRO_Z_B0					0x30
#define GYRO_Z_B1					0x31
#define GYRO_Z_B2					0x32
#define GYRO_Z_B3					0x33
#define ACCEL_X_B0					0x34
#define ACCEL_X_B1					0x35
#define ACCEL_X_B2					0x36
#define ACCEL_X_B3					0x37
#define ACCEL_Y_B0					0x38
#define ACCEL_Y_B1					0x39
#define ACCEL_Y_B2					0x3A
#define ACCEL_Y_B3					0x3B
#define ACCEL_Z_B0					0x3C
#define ACCEL_Z_B1					0x3D
#define ACCEL_Z_B2					0x3E
#define ACCEL_Z_B3					0x3F
#define TEMP_B0						0x40
#define TEMP_B1						0x41
#define TEMP_B2						0x42
#define TEMP_B3						0x43
#define GYRO_X_H0					0x44
#define GYRO_X_H1					0x45
#define GYRO_X_H2					0x46
#define GYRO_X_H3					0x47
#define GYRO_Y_H0					0x48
#define GYRO_Y_H1					0x49
#define GYRO_Y_H2					0x4A
#define GYRO_Y_H3					0x4B
#define GYRO_Z_H0					0x4C
#define GYRO_Z_H1					0x4D
#define GYRO_Z_H2					0x4E
#define GYRO_Z_H3					0x4F
#define ACCEL_X_H0					0x50
#define ACCEL_X_H1					0x51
#define ACCEL_X_H2					0x52
#define ACCEL_X_H3					0x53
#define ACCEL_Y_H0					0x54
#define ACCEL_Y_H1					0x55
#define ACCEL_Y_H2					0x56
#define ACCEL_Y_H3					0x57
#define ACCEL_Z_H0					0x58
#define ACCEL_Z_H1					0x59
#define ACCEL_Z_H2					0x5A
#define ACCEL_Z_H3					0x5B
#define TEMP_H0						0x5C
#define TEMP_H1						0x5D
#define TEMP_H2						0x5E
#define TEMP_H3						0x5F

#define MODEL_NUMBER_L_DEFAULT		0x0A
#define MODEL_NUMBER_H_DEFAULT		0x0F
#define FIRMWARE_VERSION_DEFAULT	0x01
#define DYNAMIXEL_ID_DEFAULT		0xC8
#define BAUD_RATE_DEFAULT			0x06

#define DYNAMIXEL_ID_BROADCAST		0xFE

extern uint8_t RAM[RAM_TABLE_SIZE];

#endif /* RAM_H_ */