/*******************************************************************************
 * @file        BQ76920.c
 * @brief       C Library for BQ76920 an Analog Front-End Battery Manangement System
 * @version     2.0
 * @author      Nawat Kitiphuwadon, extensively modified by Ignatius Djaynurdin
 * @date        2023-7-04
 ********************************************************************************

 MIT License
 Copyright (c) 2018 ETA Systems
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

#include "BQ76920.h"
#include "kalmanfilter.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

void BQ76920_Initialise(BQ76920_t *BMS, I2C_HandleTypeDef *i2cHandle) {
	// Device StartUp
	BMS->i2cHandle = i2cHandle;
	BMS->SOH = 100.0f;

	// Set SYS_STAT to 0xff
	uint8_t SYS_STAT_VAL = 0xff;
	BQ76920_WriteRegister(BMS, SYS_STAT, &SYS_STAT_VAL);

	// Set CC_CFG to 0x19
	uint8_t CC_CFG_REG = 0x19;
	BQ76920_WriteRegister(BMS, CC_CFG, &CC_CFG_REG);

	// Set ADC_EN and TEMP_SEL
	uint8_t SYS_CTRL1_REG = 0x10;
	BQ76920_WriteRegister(BMS, SYS_CTRL1, &SYS_CTRL1_REG);

	// Set CC_EN
	uint8_t SYS_CTRL2_REG = 0x43; // Set to 0x43 if no SOC
	BQ76920_WriteRegister(BMS, SYS_CTRL2, &SYS_CTRL2_REG);

	// Read ADCGAIN
	uint8_t regData[2] = { 0u, 0u };	// declare buffer
	BQ76920_ReadRegister(BMS, ADCGAIN1, &regData[0]);
	BQ76920_ReadRegister(BMS, ADCGAIN2, &regData[1]);
	uint8_t ADCGAIN = (((regData[1] & 0xE0) >> 5) | ((regData[0] & 0x0C) << 1));
	uint16_t GAIN = ADCGAIN + 365;
	BMS->GAIN = GAIN;

	// Read ADCOFFSET
	BQ76920_ReadRegister(BMS, ADCOFFSET, &regData[0]);
	int8_t OFFSET = (int8_t) regData[0];
	BMS->OFFSET = OFFSET;

	// Set protect1 register
	uint8_t PROTECT1_REG;
	BQ76920_ReadRegister(BMS, PROTECT1, &PROTECT1_REG);
	PROTECT1_REG = PROTECT1_REG | (SDC_100us_delay << 3);
	PROTECT1_REG = PROTECT1_REG | SCD_Threshold_89mV;
	BQ76920_WriteRegister(BMS, PROTECT1, &PROTECT1_REG);

	// Set protect2 register
	uint8_t PROTECT2_REG;
	BQ76920_ReadRegister(BMS, PROTECT2, &PROTECT2_REG);
	PROTECT2_REG = PROTECT2_REG | (ODC_160ms_delay << 5);
	PROTECT2_REG = PROTECT2_REG | (OCD_Threshold_8mV);
	BQ76920_WriteRegister(BMS, PROTECT2, &PROTECT2_REG);

	// Set protect3 register
	uint8_t PROTECT3_REG;
	BQ76920_ReadRegister(BMS, PROTECT3, &PROTECT3_REG);
	PROTECT3_REG = PROTECT3_REG | (UV_Delay_4s << 6);
	PROTECT3_REG = PROTECT3_REG | (OV_Delay_4s << 4);
	BQ76920_WriteRegister(BMS, PROTECT3, &PROTECT3_REG);

	// Set OV_Trip register
	uint16_t OV = grossOV * 1000;
	uint16_t temp = (uint16_t) ((float) (OV - BMS->OFFSET)
			/ ((float) (BMS->GAIN) / 1000));
	temp = (temp & 0x0ff0) >> 4;
	uint8_t OV_TRIP_FULL = temp & 0xff;
	BQ76920_WriteRegister(BMS, OV_TRIP, &OV_TRIP_FULL);

	// Set UV_Trip register
	uint16_t UV = grossUV * 1000;
	temp =
			(uint16_t) ((float) (UV - BMS->OFFSET)
					/ ((float) (BMS->GAIN) / 1000));
	temp = (temp & 0x0ff0) >> 4;
	uint8_t UV_TRIP_FULL = temp & 0xff;
	BQ76920_WriteRegister(BMS, UV_TRIP, &UV_TRIP_FULL);

	BMS->SOH = 45.0;
	BMS->SOHEnergy = 45.0;
	BMS->SOHCapacity = 45.0;
	BMS->SOHOCV = 45.0;
}

float getCellVoltage(BQ76920_t *BMS, int cell) {
	if ((cell == VC1) || (cell == VC2) || (cell == VC3) || (cell == VC4)) {
		uint8_t regData[2] = { 0u, 0u };	// Declare buffer
		BQ76920_ReadRegister(BMS, cell, &regData[0]); // Read Hi
		BQ76920_ReadRegister(BMS, cell + 1, &regData[1]); // Read Lo
		uint16_t VoltageCellRaw = (((regData[0] & 0x3f) << 8)) | regData[1];
		float VoltageCell = (((float) (BMS->GAIN) / 1000)
				* ((float) VoltageCellRaw) + (float) (BMS->OFFSET)) / 1000;
		// Stored data in struct for balance Cell
		switch (cell) {
		case VC1:
			BMS->Vcell[0] = VoltageCell;
			break;
		case VC2:
			BMS->Vcell[1] = VoltageCell;
			break;
		case VC3:
			BMS->Vcell[2] = VoltageCell;
			break;
		case VC4:
			BMS->Vcell[3] = VoltageCell;
			break;
		default:
			break;
		}
		// Return float value of requested cell
		return VoltageCell;
	} else
		return 0;
}

float getPackVoltage(BQ76920_t *BMS) {
	uint8_t regData[2] = { 0u, 0u };	// Declare buffer
	BQ76920_ReadRegister(BMS, BAT_LO, &regData[0]);	// Read data in BAT_LO register
	BQ76920_ReadRegister(BMS, BAT_HI, &regData[1]); // Read data in BAT_HI register
	// Return float value of Vpack
	BMS->Vpack = (float) ((regData[1] << 8) | regData[0]) * 4
			* (BMS->GAIN / 1000000.0f) + (4 * (BMS->OFFSET) / 1000);
	return BMS->Vpack;
}

float getCurrent(BQ76920_t *BMS) {
	uint8_t regData[2] = { 0u, 0u };	// Declare buffer
	BQ76920_ReadRegister(BMS, CC_LO, &regData[0]);
	BQ76920_ReadRegister(BMS, CC_HI, &regData[1]);

	int16_t CurrentRaw = (int16_t) ((regData[1] << 8) | regData[0]);

	// Check if CurrentRaw is negative
	if (CurrentRaw & 0x8000) {
		// Perform 2's complement conversion
		CurrentRaw = -(~CurrentRaw + 1);
	}

	if (CurrentRaw == 1 || CurrentRaw == -1) {
		CurrentRaw = 0;
	}

	float Current = (float) CurrentRaw * 8.44; // in uV
	Current = Current / RSENSE; // in mA

	return Current; // in mA
}

float SOCPack(BQ76920_t *BMS, float PackCurrent, float Vpack) {
	// Determine threshold.
	float fullCapacityEnergy = grossCapacity * 3600 * 4 * nominalPackV
			* (BMS->SOH / 100.0f); // in mW
	float fullCapacityCurrent = grossCapacity * 3600 * 4 * (BMS->SOH / 100.0f); // in mA

	// Take account for Round Trip Efficiency
	if (PackCurrent > 0) {
		PackCurrent = PackCurrent * ROUND_TRIP_EFFICIENCY;
	}

	// Sensor Reading
	BMS->wattUsage += PackCurrent * Vpack;
	BMS->currentUsage += PackCurrent;

	// Calculation
	BMS->SOCEnergy = (float) ((fullCapacityEnergy + BMS->wattUsage) * 100.0f
			/ fullCapacityEnergy);
	BMS->SOCCapacity = (float) ((fullCapacityCurrent + BMS->currentUsage)
			* 100.0f / fullCapacityCurrent);

	// Kalman Filter
	KalmanFilter kf_soc;
	kalman_filter_init(&kf_soc, BMS->SOCEnergy, 1.0f, 0.05f); // Tune the initial_estimate_error_cov value based on the system
	float fused_soc = kalman_filter_update(&kf_soc, BMS->SOCCapacity, 0.05f);
	BMS->SOC = fused_soc;

	// Return
	return BMS->SOC;
}

float SOHPack(BQ76920_t *BMS) {
	// Determine threshold.
	int32_t EnergyCapacity = grossCapacity * 3600 * 4 * nominalPackV;
	int32_t AmpereCapacity = grossCapacity * 3600 * 4;
	float FullOCV = netOV;

	// Determine the lowest value to be the new indicator for full Value. For SOHOCV.
	for (int i = 0; i <= 3; i++) {
		if (BMS->Vcell[i] < FullOCV) {
			BMS->smallestV = BMS->Vcell[i];
		}
	}

	// Calculation
	BMS->SOHEnergy = (float) (BMS->wattUsage * 100.0f / EnergyCapacity);
	BMS->SOHCapacity = (float) (BMS->currentUsage * 100.0f / AmpereCapacity);
	BMS->SOHOCV = (float) (BMS->smallestV * 100.0f / FullOCV);

	// Kalman Filter
	KalmanFilter kf_soh;
	kalman_filter_init(&kf_soh, BMS->SOHEnergy, 1.0f, 0.05f); // Tune the initial_estimate_error_cov value based on the system
	float fused_soh = kalman_filter_update(&kf_soh, BMS->SOHCapacity, 0.05f);
	fused_soh = kalman_filter_update(&kf_soh, BMS->SOHOCV, 0.05f);
	BMS->SOH = fused_soh;

	// Reset
	BMS->wattUsage = 0;
	BMS->currentUsage = 0;

	// Return
	return BMS->SOH;
}

void readAlert(BQ76920_t *BMS) {
	uint8_t temp;
	BQ76920_ReadRegister(BMS, SYS_STAT, &temp);
	for (int i = 0; i <= 7; i++) {
		uint8_t tmp = temp;
		BMS->Alert[i] = (tmp >> i) & 1;
	}
}

void EnableBalanceCell(BQ76920_t *BMS, float PackCurrent) {
	if (PackCurrent <= 0.0f) {
		uint8_t notBalancing = 0x00;
		BQ76920_WriteRegister(BMS, CELLBAL1, &notBalancing);
	} else {
		float maxDifference = 0;
		int balanceCase = 0;
		uint8_t balancingFlags = 0x01;
		// Iterate over the array and find the maximum difference
		for (int i = 0; i < 3; i++) {
			float difference = fabsf(BMS->Vcell[i] - BMS->Vcell[i + 1]);
			if (difference > maxDifference) {
				maxDifference = difference;
				balanceCase = i;
			}
		}
		// Set the balancing flags based on the balanceCase
		if (balanceCase == 1) {
			balancingFlags = 0x02;
		} else if (balanceCase == 2) {
			balancingFlags = 0x04;
		}
		BQ76920_WriteRegister(BMS, CELLBAL1, &balancingFlags);
	}
}

void turnCHGOn(BQ76920_t *BMS) {
	uint8_t temp;
	BQ76920_ReadRegister(BMS, SYS_CTRL2, &temp);
	temp = temp | 1;
	BQ76920_WriteRegister(BMS, SYS_CTRL2, &temp);
}

void turnDSGOn(BQ76920_t *BMS) {
	uint8_t temp;
	BQ76920_ReadRegister(BMS, SYS_CTRL2, &temp);
	temp = temp | 2;
	BQ76920_WriteRegister(BMS, SYS_CTRL2, &temp);
}

void turnCHGOff(BQ76920_t *BMS) {
	uint8_t temp;
	BQ76920_ReadRegister(BMS, SYS_CTRL2, &temp);
	temp = temp & 254;
	BQ76920_WriteRegister(BMS, SYS_CTRL2, &temp);
}

void turnDSGOff(BQ76920_t *BMS) {
	uint8_t temp;
	BQ76920_ReadRegister(BMS, SYS_CTRL2, &temp);
	temp = temp & 253;
	BQ76920_WriteRegister(BMS, SYS_CTRL2, &temp);
}

void CLEAR_SYS_STAT(BQ76920_t *BMS) {
	uint8_t temp = 0xff;
	BQ76920_WriteRegister(BMS, SYS_STAT, &temp);
}

uint8_t getAlert(BQ76920_t *BMS, uint8_t k) {
	return BMS->Alert[k];
}

uint8_t checkUV(float Vcell[4]) {
	if (Vcell[0] <= netUV || Vcell[1] <= netUV || Vcell[2] <= netUV
			|| Vcell[3] <= netUV) {
		return 1;
	}
	return 0;
}

uint8_t checkNotUV(float Vcell[4], uint8_t UV) {
	if (UV && Vcell[0] >= netUV + thresholdRange
			&& Vcell[1] >= netUV + thresholdRange
			&& Vcell[2] >= netUV + thresholdRange
			&& Vcell[3] >= netUV + thresholdRange) {
		return 1;
	}
	return 0;
}

uint8_t checkOV(float Vcell[4]) {
	if (Vcell[0] >= grossOV || Vcell[1] >= grossOV || Vcell[2] >= grossOV
			|| Vcell[3] >= grossOV) {
		return 1;
	}
	return 0;
}

uint8_t checkNotOV(float Vcell[4], uint8_t OV) {
	if (OV && Vcell[0] <= grossOV - thresholdRange
			&& Vcell[1] <= grossOV - thresholdRange
			&& Vcell[2] <= grossOV - thresholdRange
			&& Vcell[3] <= grossOV - thresholdRange) {
		return 1;
	}
	return 0;
}

// Debug
void justWrite1(BQ76920_t *BMS) { // For Debug
	uint8_t temp = 0xcd;
	BQ76920_WriteRegister(BMS, CC_CFG, &temp);
}

float justGetter1(BQ76920_t *BMS) { // For Debug
	return BMS->SOCEnergy;
}

float justGetter2(BQ76920_t *BMS) { // For Debug
	return BMS->SOCCapacity;
}

float justGetter3(BQ76920_t *BMS) { // For Debug
	return BMS->SOH;
}

float justGetter4(BQ76920_t *BMS) { // For Debug
	return BMS->SOHEnergy;
}

float justGetter5(BQ76920_t *BMS) { // For Debug
	return BMS->SOHCapacity;
}

float justGetter6(BQ76920_t *BMS) { // For Debug
	return BMS->SOHOCV;
}

int32_t justGetter7(BQ76920_t *BMS) { // For Debug
	return BMS->wattUsage;
}

int32_t justGetter8(BQ76920_t *BMS) { // For Debug
	return BMS->currentUsage;
}

uint8_t justRead1(BQ76920_t *BMS) { // For Debug
	uint8_t temp;
	BQ76920_ReadRegister(BMS, CELLBAL1, &temp);
	return temp;
}

uint8_t justRead2(BQ76920_t *BMS) { // For Debug
	uint8_t temp;
	BQ76920_ReadRegister(BMS, SYS_CTRL2, &temp);
	return temp;
}

// Low Level Implementation
void BQ76920_ReadRegister(BQ76920_t *BMS, uint8_t reg, uint8_t *data) {
	HAL_I2C_Mem_Read(BMS->i2cHandle, BQ76920_ADDRESS, reg,
	I2C_MEMADD_SIZE_8BIT, data, 1, 200);
}

void BQ76920_WriteRegister(BQ76920_t *BMS, uint8_t reg, uint8_t *data) {
	HAL_I2C_Mem_Write(BMS->i2cHandle, BQ76920_ADDRESS, reg,
	I2C_MEMADD_SIZE_8BIT, data, 1, 200);
}

