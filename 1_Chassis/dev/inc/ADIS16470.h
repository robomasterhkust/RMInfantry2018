/*
 * ADIS16470.h
 *
 *  Created on: 28 Apr 2018
 *      Author: Alex Wong
 */

#ifndef INC_ADIS16470_H_
#define INC_ADIS16470_H_

#define ADIS16470_SPID			&SPID4

#define ADIS16470_pin           GPIOE_SPI4_ADIS_NSS
#define ADIS16470_port          GPIOE

#define ADIS16470_BUFFER_SIZE	18

typedef struct imu_data_t{

	uint16_t diag_stat;
	int16_t x_gyro_out;
	int16_t y_gyro_out;
	int16_t z_gyro_out;
	int16_t x_accl_out;
	int16_t y_accl_out;
	int16_t z_accl_out;
	int16_t temp_out;
	uint16_t count_out;

}imu_data_t;

#endif /* INC_ADIS16470_H_ */
