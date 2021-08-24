#pragma once

#include <stdio.h>
#include "stm32f4xx_hal.h"

#define MAX30102_ADDR   0x57<<1

extern I2C_HandleTypeDef hi2c1;
extern void Error_Handler(void);

void MX_I2C1_Init(void);
void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
uint8_t MAX30102_getINT1(void);
uint8_t MAX30102_getINT2(void);
void MAX30102_enableAFULL(void);
void MAX30102_disableAFULL(void);
void MAX30102_enableDATARDY(void);
void MAX30102_disableDATARDY(void);
void MAX30102_enableALCOVF(void);
void MAX30102_disableALCOVF(void);
void MAX30102_enablePROXINT(void);
void MAX30102_disablePROXINT(void);
void MAX30102_enableDIETEMPRDY(void);
void MAX30102_disableDIETEMPRDY(void);
void MAX30102_softReset(void);
void MAX30102_shutDown(void);
void MAX30102_wakeUp(void);
void MAX30102_setLEDMode(uint8_t mode);
void MAX30102_setADCRange(uint8_t adcRange);
void MAX30102_setSampleRate(uint8_t sampleRate);
void MAX30102_setPulseWidth(uint8_t pulseWidth);
void MAX30102_setPulseAmplitudeRed(uint8_t amplitude);
void MAX30102_setPulseAmplitudeIR(uint8_t amplitude);
void MAX30102_setPulseAmplitudeGreen(uint8_t amplitude);
void MAX30102_setPulseAmplitudeProximity(uint8_t amplitude);
void MAX30102_setProximityThreshold(uint8_t threshMSB);
void MAX30102_enableSlot(uint8_t slotNumber, uint8_t device);
void MAX30102_disableSlots(void);
void MAX30102_setFIFOAverage(uint8_t numberOfSamples);
void MAX30102_clearFIFO(void);
void MAX30102_enableFIFORollover(void);
void MAX30102_disableFIFORollover(void);
void MAX30102_setFIFOAlmostFull(uint8_t numberOfSamples);
uint8_t MAX30102_getWritePointer(void);
uint8_t MAX30102_getReadPointer(void);
float MAX30102_readTemperature(void);
float MAX30102_readTemperatureF(void);
void MAX30102_setPROXINTTHRESH(uint8_t val);
void MAX30102_readRevisionID(void);
uint8_t MAX30102_getRevisionID(void);
uint8_t MAX30102_readPartID(void);
void MAX30102_setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange);
void MAX30102_FIFOWrite(uint8_t* data);