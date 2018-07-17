/*
 * ADIS16470.c
 *
 *  Created on: 28 Apr 2018
 *      Author: Alex Wong
 */

#include "ch.h"
#include "hal.h"
#include "ADIS16470.h"

static imu_data_t adis_imu;

static const SPIConfig adis16470SpiCfg = {

  NULL,
  ADIS16470_port,
  ADIS16470_pin,
  SPI_CR1_MSTR | SPI_CR1_DFF | SPI_CR1_BR_2 | SPI_CR1_BR_0 | SPI_CR1_CPHA | SPI_CR1_CPOL

};

static uint16_t buffer[ADIS16470_BUFFER_SIZE];

static uint16_t gyro_req_reg(SPIDriver* spidriver, uint16_t regaddr) {

	uint16_t regdata;

	spiAcquireBus(spidriver);
	spiSelect(spidriver);
	spiSend(spidriver, 1, &regaddr);
	spiUnselect(spidriver);

	spiSelect(spidriver);
	spiReceive(spidriver, 1, &regdata);
	spiUnselect(spidriver);

	spiReleaseBus(spidriver);

	return regdata;

}

static int16_t words[20];
static int32_t data[1];

#define ADIS16470_UPDATE_PERIOD		MS2ST(5)
static THD_WORKING_AREA(adis16470Thd_wa, 1024);
static THD_FUNCTION(adis16470Thd, p) {

	(void)p;

	static uint16_t temp = 0x6800;

	while(true) {

		spiAcquireBus(ADIS16470_SPID);
		spiSelect(ADIS16470_SPID);
		spiSend(ADIS16470_SPID, 1, &temp);
		//spiUnselect(spidriver);

		//spiSelect(spidriver);
		spiReceive(ADIS16470_SPID, 11, &words);
		spiUnselect(ADIS16470_SPID);

		spiReleaseBus(ADIS16470_SPID);

		chThdSleep(MS2ST(1));

	}

}

void adis16470_init(void) {

	memset((void*)&adis_imu, 0, sizeof(imu_data_t));
	memset((void*)&buffer, 0, ADIS16470_BUFFER_SIZE);

	memset((void*)&words, 0, sizeof(uint16_t) * 2);
	memset((void*)&data, 0, sizeof(int32_t));

	(*ADIS16470_SPID).rxdmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
	(*ADIS16470_SPID).txdmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
	spiStart(ADIS16470_SPID, &adis16470SpiCfg);

	chThdCreateStatic(adis16470Thd_wa, sizeof(adis16470Thd_wa),
					  NORMALPRIO + 7, adis16470Thd, NULL);

}
