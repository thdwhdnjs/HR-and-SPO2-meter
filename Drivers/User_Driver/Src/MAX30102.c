#include "MAX30102.h"
#include "main.h"



// Status Registers
static const uint8_t MAX30102_INTSTAT1 =		0x00;
static const uint8_t MAX30102_INTSTAT2 =		0x01;
static const uint8_t MAX30102_INTENABLE1 =		0x02;
static const uint8_t MAX30102_INTENABLE2 =		0x03;

// FIFO Registers
static const uint8_t MAX30102_FIFOWRITEPTR = 	        0x04;
static const uint8_t MAX30102_FIFOOVERFLOW = 	        0x05;
static const uint8_t MAX30102_FIFOREADPTR = 	        0x06;
static const uint8_t MAX30102_FIFODATA =		0x07;

// Configuration Registers
static const uint8_t MAX30102_FIFOCONFIG = 		0x08;
static const uint8_t MAX30102_MODECONFIG = 		0x09;
static const uint8_t MAX30102_PARTICLECONFIG = 	        0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30102_LED1_PULSEAMP = 	        0x0C;
static const uint8_t MAX30102_LED2_PULSEAMP = 	        0x0D;
static const uint8_t MAX30102_LED3_PULSEAMP = 	        0x0E;
static const uint8_t MAX30102_LED_PROX_AMP = 	        0x10;
static const uint8_t MAX30102_MULTILEDCONFIG1 =         0x11;
static const uint8_t MAX30102_MULTILEDCONFIG2 =         0x12;

// Die Temperature Registers
static const uint8_t MAX30102_DIETEMPINT = 		0x1F;
static const uint8_t MAX30102_DIETEMPFRAC = 	        0x20;
static const uint8_t MAX30102_DIETEMPCONFIG = 	        0x21;

// Proximity Function Registers
static const uint8_t MAX30102_PROXINTTHRESH = 	        0x30;

// Part ID Registers
static const uint8_t MAX30102_REVISIONID = 		0xFE;
static const uint8_t MAX30102_PARTID = 			0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30105 Commands
// Interrupt configuration (pg 13, 14)
static const uint8_t MAX30102_INT_A_FULL_MASK =		~0x80;
static const uint8_t MAX30102_INT_A_FULL_ENABLE = 	0x80;
static const uint8_t MAX30102_INT_A_FULL_DISABLE = 	0x00;

static const uint8_t MAX30102_INT_DATA_RDY_MASK =       ~0x80;
static const uint8_t MAX30102_INT_DATA_RDY_ENABLE =	0x40;
static const uint8_t MAX30102_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX30102_INT_ALC_OVF_MASK = ~0x80;
static const uint8_t MAX30102_INT_ALC_OVF_ENABLE = 	0x20;
static const uint8_t MAX30102_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX30102_INT_PROX_INT_MASK = ~0x80;
static const uint8_t MAX30102_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX30102_INT_PROX_INT_DISABLE = 0x00;

static const uint8_t MAX30102_INT_DIE_TEMP_RDY_MASK = ~0x80;
static const uint8_t MAX30102_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX30102_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX30102_SAMPLEAVG_MASK =	~0x80;
static const uint8_t MAX30102_SAMPLEAVG_1 = 	0x00;
static const uint8_t MAX30102_SAMPLEAVG_2 = 	0x20;
static const uint8_t MAX30102_SAMPLEAVG_4 = 	0x40;
static const uint8_t MAX30102_SAMPLEAVG_8 = 	0x60;
static const uint8_t MAX30102_SAMPLEAVG_16 = 	0x80;
static const uint8_t MAX30102_SAMPLEAVG_32 = 	0xA0;

static const uint8_t MAX30102_ROLLOVER_MASK = 	0xEF;
static const uint8_t MAX30102_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX30102_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX30102_A_FULL_MASK = 	0xF0;

// Mode configuration commands (page 19)
static const uint8_t MAX30102_SHUTDOWN_MASK = 	0x7F;
static const uint8_t MAX30102_SHUTDOWN = 		0x80;
static const uint8_t MAX30102_WAKEUP = 			0x00;

static const uint8_t MAX30102_RESET_MASK = 		0xBF;
static const uint8_t MAX30102_RESET = 			0x40;

static const uint8_t MAX30102_MODE_MASK = 		0xF8;
static const uint8_t MAX30102_MODE_REDONLY = 	0x02;
static const uint8_t MAX30102_MODE_REDIRONLY = 	0x03;


// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX30102_ADCRANGE_MASK = 	0x9F;
static const uint8_t MAX30102_ADCRANGE_2048 = 	0x00;
static const uint8_t MAX30102_ADCRANGE_4096 = 	0x20;
static const uint8_t MAX30102_ADCRANGE_8192 = 	0x40;
static const uint8_t MAX30102_ADCRANGE_16384 = 	0x60;

static const uint8_t MAX30102_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30102_SAMPLERATE_50 = 	0x00;
static const uint8_t MAX30102_SAMPLERATE_100 = 	0x04;
static const uint8_t MAX30102_SAMPLERATE_200 = 	0x08;
static const uint8_t MAX30102_SAMPLERATE_400 = 	0x0C;
static const uint8_t MAX30102_SAMPLERATE_800 = 	0x10;
static const uint8_t MAX30102_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30102_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30102_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30102_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30102_PULSEWIDTH_69 = 	0x00;
static const uint8_t MAX30102_PULSEWIDTH_118 = 	0x01;
static const uint8_t MAX30102_PULSEWIDTH_215 = 	0x02;
static const uint8_t MAX30102_PULSEWIDTH_411 = 	0x03;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX30102_SLOT1_MASK = 	0xF8;
static const uint8_t MAX30102_SLOT2_MASK = 	0x8F;
static const uint8_t MAX30102_SLOT3_MASK = 	0xF8;
static const uint8_t MAX30102_SLOT4_MASK = 	0x8F;

static const uint8_t SLOT_NONE = 		0x00;
static const uint8_t SLOT_RED_LED = 		0x01;
static const uint8_t SLOT_IR_LED = 		0x02;


static const uint8_t MAX_30102_EXPECTEDPARTID = 0x15;


void bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  // Grab current register context
  uint8_t originalContents;
  HAL_I2C_Mem_Read(&hi2c1,MAX30102_ADDR|0x01,reg,1,&originalContents,1,50);

  // Zero-out the portions of the register we're interested in
  originalContents = (originalContents & mask) | thing;

  // Change contents
  HAL_I2C_Mem_Write(&hi2c1,MAX30102_ADDR,reg,1,&originalContents,1,50);
}

uint8_t MAX30102_getINT1(void) {
  uint8_t data;
  HAL_I2C_Mem_Read(&hi2c1,MAX30102_ADDR|0x01,MAX30102_INTSTAT1,1,&data,1,50);
  return data;
}

uint8_t MAX30102_getINT2(void) {
  uint8_t data;
  HAL_I2C_Mem_Read(&hi2c1,MAX30102_ADDR|0x01,MAX30102_INTSTAT2,1,&data,1,50);
  return data;
}

void MAX30102_enableAFULL(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK,MAX30102_INT_A_FULL_ENABLE);
}
void MAX30102_disableAFULL(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_DISABLE);
}

void MAX30102_enableDATARDY(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_ENABLE);
}
void MAX30102_disableDATARDY(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_DISABLE);
}

void MAX30102_enableALCOVF(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_ENABLE);
}
void MAX30102_disableALCOVF(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_DISABLE);
}

void MAX30102_enablePROXINT(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_PROX_INT_MASK, MAX30102_INT_PROX_INT_ENABLE);
}
void MAX30102_disablePROXINT(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_PROX_INT_MASK, MAX30102_INT_PROX_INT_DISABLE);
}

void MAX30102_enableDIETEMPRDY(void) {
  bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_ENABLE);
}
void MAX30102_disableDIETEMPRDY(void) {
  bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_DISABLE);
}

//End Interrupt configuration

void MAX30102_softReset(void) {
  bitMask(MAX30102_MODECONFIG, MAX30102_RESET_MASK, MAX30102_RESET);

  // Poll for bit to clear, reset is then complete
  // Timeout after 100ms
  uint8_t response;
  unsigned long startTime = HAL_GetTick();
  while (HAL_GetTick() - startTime < 100)
  {
    
    HAL_I2C_Mem_Read(&hi2c1,MAX30102_ADDR|0x01,MAX30102_MODECONFIG,1,&response,1,50);
    if ((response & MAX30102_RESET) == 0) break; //We're done!
    HAL_Delay(1); //Let's not over burden the I2C bus
  }
}

void MAX30102_shutDown(void) {
  // Put IC into low power mode (datasheet pg. 19)
  // During shutdown the IC will continue to respond to I2C commands but will
  // not update with or take new readings (such as temperature)
  bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_SHUTDOWN);
}

void MAX30102_wakeUp(void) {
  // Pull IC out of low power mode (datasheet pg. 19)
  bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_WAKEUP);
}

void MAX30102_setLEDMode(uint8_t mode) {
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  // See datasheet, page 19
  bitMask(MAX30102_MODECONFIG, MAX30102_MODE_MASK, mode);
}

void MAX30102_setADCRange(uint8_t adcRange) {
  // adcRange: one of MAX30105_ADCRANGE_2048, _4096, _8192, _16384
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_ADCRANGE_MASK, adcRange);
}

void MAX30102_setSampleRate(uint8_t sampleRate) {
  // sampleRate: one of MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_SAMPLERATE_MASK, sampleRate);
}

void MAX30102_setPulseWidth(uint8_t pulseWidth) {
  // pulseWidth: one of MAX30105_PULSEWIDTH_69, _188, _215, _411
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_PULSEWIDTH_MASK, pulseWidth);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void MAX30102_setPulseAmplitudeRed(uint8_t amplitude) {
  HAL_I2C_Mem_Write(&hi2c1,MAX30102_ADDR,MAX30102_LED1_PULSEAMP,1,&amplitude,1,50);
}

void MAX30102_setPulseAmplitudeIR(uint8_t amplitude) {
  HAL_I2C_Mem_Write(&hi2c1,MAX30102_ADDR,MAX30102_LED2_PULSEAMP,1,&amplitude,1,50);
}

void MAX30102_setPulseAmplitudeGreen(uint8_t amplitude) {
  HAL_I2C_Mem_Write(&hi2c1,MAX30102_ADDR,MAX30102_LED3_PULSEAMP,1,&amplitude,1,50);
}

void MAX30102_setPulseAmplitudeProximity(uint8_t amplitude) {
  HAL_I2C_Mem_Write(&hi2c1,MAX30102_ADDR,MAX30102_LED_PROX_AMP,1,&amplitude,1,50);
}

void MAX30102_setProximityThreshold(uint8_t threshMSB) {
  // Set the IR ADC count that will trigger the beginning of particle-sensing mode.
  // The threshMSB signifies only the 8 most significant-bits of the ADC count.
  // See datasheet, page 24.
  HAL_I2C_Mem_Write(&hi2c1,MAX30102_ADDR,MAX30102_PROXINTTHRESH,1,&threshMSB,1,50);
}

//Given a slot number assign a thing to it
//Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
//Assigning a SLOT_RED_LED will pulse LED
//Assigning a SLOT_RED_PILOT will ??
void MAX30102_enableSlot(uint8_t slotNumber, uint8_t device) {

  uint8_t originalContents;

  switch (slotNumber) {
    case (1):
      bitMask(MAX30102_MULTILEDCONFIG1, MAX30102_SLOT1_MASK, device);
      break;
    case (2):
      bitMask(MAX30102_MULTILEDCONFIG1, MAX30102_SLOT2_MASK, device << 4);
      break;
    case (3):
      bitMask(MAX30102_MULTILEDCONFIG2, MAX30102_SLOT3_MASK, device);
      break;
    case (4):
      bitMask(MAX30102_MULTILEDCONFIG2, MAX30102_SLOT4_MASK, device << 4);
      break;
    default:
      //Shouldn't be here!
      break;
  }
}

//Clears all slot assignments
void MAX30102_disableSlots(void) {
  HAL_I2C_Mem_Write(&hi2c1,MAX30102_ADDR,MAX30102_MULTILEDCONFIG1,1,0,1,50);
  HAL_I2C_Mem_Write(&hi2c1,MAX30102_ADDR,MAX30102_MULTILEDCONFIG2,1,0,1,50);
}

//
// FIFO Configuration
//

//Set sample average (Table 3, Page 18)
void MAX30102_setFIFOAverage(uint8_t numberOfSamples) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_SAMPLEAVG_MASK, numberOfSamples);
}

//Resets all points to start in a known state
//Page 15 recommends clearing FIFO before beginning a read
void MAX30102_clearFIFO(void) {
  HAL_I2C_Mem_Write(&hi2c1,MAX30102_ADDR,MAX30102_FIFOWRITEPTR,1,0,1,50);
  HAL_I2C_Mem_Write(&hi2c1,MAX30102_ADDR,MAX30102_FIFOOVERFLOW,1,0,1,50);
  HAL_I2C_Mem_Write(&hi2c1,MAX30102_ADDR,MAX30102_FIFOREADPTR,1,0,1,50);
}

//Enable roll over if FIFO over flows
void MAX30102_enableFIFORollover(void) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_ENABLE);
}

//Disable roll over if FIFO over flows
void MAX30102_disableFIFORollover(void) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_DISABLE);
}

//Set number of samples to trigger the almost full interrupt (Page 18)
//Power on default is 32 samples
//Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
void MAX30102_setFIFOAlmostFull(uint8_t numberOfSamples) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_A_FULL_MASK, numberOfSamples);
}

//Read the FIFO Write Pointer
uint8_t MAX30102_getWritePointer(void) {
  uint8_t data;
  HAL_I2C_Mem_Read(&hi2c1,MAX30102_ADDR|0x01,MAX30102_FIFOWRITEPTR,1,&data,1,50);
  return data;
}

//Read the FIFO Read Pointer
uint8_t MAX30102_getReadPointer(void) {
  uint8_t data;
  HAL_I2C_Mem_Read(&hi2c1,MAX30102_ADDR|0x01,MAX30102_FIFOREADPTR,1,&data,1,50);
  return data;
}


// Die Temperature
// Returns temp in C
float MAX30102_readTemperature(void) {
	
  //DIE_TEMP_RDY interrupt must be enabled
  uint8_t response;
  uint8_t data = 0x01;
  // Step 1: Config die temperature register to take 1 temperature sample
  HAL_I2C_Mem_Write(&hi2c1,MAX30102_ADDR,MAX30102_DIETEMPCONFIG,1,&data,1,50);
  
  // Poll for bit to clear, reading is then complete
  // Timeout after 100ms
  unsigned long startTime = HAL_GetTick();
  while (HAL_GetTick() - startTime < 100)
  {
    //uint8_t response = readRegister8(_i2caddr, MAX30105_DIETEMPCONFIG); //Original way
    //if ((response & 0x01) == 0) break; //We're done!
    
	//Check to see if DIE_TEMP_RDY interrupt is set
        HAL_I2C_Mem_Read(&hi2c1,MAX30102_ADDR|0x01,MAX30102_INTSTAT2,1,&response,1,50);
    if ((response & MAX30102_INT_DIE_TEMP_RDY_ENABLE) > 0) break; //We're done!
    HAL_Delay(1); //Let's not over burden the I2C bus
  }
  //TODO How do we want to fail? With what type of error?
  //? if(millis() - startTime >= 100) return(-999.0);

  // Step 2: Read die temperature register (integer)
  uint8_t tempInt; 
  HAL_I2C_Mem_Read(&hi2c1,MAX30102_ADDR|0x01,MAX30102_DIETEMPINT,1,&tempInt,1,50);
  uint8_t tempFrac;
  HAL_I2C_Mem_Read(&hi2c1,MAX30102_ADDR|0x01,MAX30102_DIETEMPFRAC,1,&tempFrac,1,50);
 //Causes the clearing of the DIE_TEMP_RDY interrupt

  // Step 3: Calculate temperature (datasheet pg. 23)
  return (float)tempInt + ((float)tempFrac * 0.0625);
}

// Returns die temp in F
float MAX30102_readTemperatureF(void) {
  float temp = readTemperature();

  if (temp != -999.0) temp = temp * 1.8 + 32.0;

  return (temp);
}

// Set the PROX_INT_THRESHold
void MAX30102_setPROXINTTHRESH(uint8_t val) {
  HAL_I2C_Mem_Write(&hi2c1,MAX30102_ADDR,MAX30102_PROXINTTHRESH,1,&val,1,50);
}



//
// Device ID and Revision
//

uint8_t revisionID;
void MAX30102_readRevisionID(void) {
  HAL_StatusTypeDef status = HAL_OK;
  
  status=HAL_I2C_Mem_Read(&hi2c1,MAX30102_ADDR|0x01,MAX30102_REVISIONID,1,&revisionID,1,50);
  if(status != HAL_OK)
    printf("i2c error");
    
  
}

uint8_t MAX30102_getRevisionID(void) {
  return revisionID;
  
}

uint8_t MAX30102_readPartID(void) {
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t data;
  status=HAL_I2C_Mem_Read(&hi2c1,MAX30102_ADDR|0x01,MAX30102_PARTID,1,&data,1,50);
  if(status != HAL_OK)
    printf("i2c error");
    
  return data;
}

void MAX30102_setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange) {
  MAX30102_softReset(); //Reset all configuration, threshold, and data registers to POR values

  //FIFO Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //The chip will average multiple samples of same type together if you wish
  if (sampleAverage == 1) MAX30102_setFIFOAverage(MAX30102_SAMPLEAVG_1); //No averaging per FIFO record
  else if (sampleAverage == 2) MAX30102_setFIFOAverage(MAX30102_SAMPLEAVG_2);
  else if (sampleAverage == 4) MAX30102_setFIFOAverage(MAX30102_SAMPLEAVG_4);
  else if (sampleAverage == 8) MAX30102_setFIFOAverage(MAX30102_SAMPLEAVG_8);
  else if (sampleAverage == 16) MAX30102_setFIFOAverage(MAX30102_SAMPLEAVG_16);
  else if (sampleAverage == 32) MAX30102_setFIFOAverage(MAX30102_SAMPLEAVG_32);
  else MAX30102_setFIFOAverage(MAX30102_SAMPLEAVG_4);

  //setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
  MAX30102_enableFIFORollover(); //Allow FIFO to wrap/roll over
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Mode Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if (ledMode > 1) MAX30102_setLEDMode(MAX30102_MODE_REDIRONLY); //Red and IR
  else MAX30102_setLEDMode(MAX30102_MODE_REDONLY); //Red only
  
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Particle Sensing Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if(adcRange < 4096) MAX30102_setADCRange(MAX30102_ADCRANGE_2048); //7.81pA per LSB
  else if(adcRange < 8192) MAX30102_setADCRange(MAX30102_ADCRANGE_4096); //15.63pA per LSB
  else if(adcRange < 16384) MAX30102_setADCRange(MAX30102_ADCRANGE_8192); //31.25pA per LSB
  else if(adcRange == 16384) MAX30102_setADCRange(MAX30102_ADCRANGE_16384); //62.5pA per LSB
  else MAX30102_setADCRange(MAX30102_ADCRANGE_2048);

  if (sampleRate < 100) MAX30102_setSampleRate(MAX30102_SAMPLERATE_50); //Take 50 samples per second
  else if (sampleRate < 200) MAX30102_setSampleRate(MAX30102_SAMPLERATE_100);
  else if (sampleRate < 400) MAX30102_setSampleRate(MAX30102_SAMPLERATE_200);
  else if (sampleRate < 800) MAX30102_setSampleRate(MAX30102_SAMPLERATE_400);
  else if (sampleRate < 1000) MAX30102_setSampleRate(MAX30102_SAMPLERATE_800);
  else if (sampleRate < 1600) MAX30102_setSampleRate(MAX30102_SAMPLERATE_1000);
  else if (sampleRate < 3200) MAX30102_setSampleRate(MAX30102_SAMPLERATE_1600);
  else if (sampleRate == 3200) MAX30102_setSampleRate(MAX30102_SAMPLERATE_3200);
  else MAX30102_setSampleRate(MAX30102_SAMPLERATE_50);

  //The longer the pulse width the longer range of detection you'll have
  //At 69us and 0.4mA it's about 2 inches
  //At 411us and 0.4mA it's about 6 inches
  if (pulseWidth < 118) MAX30102_setPulseWidth(MAX30102_PULSEWIDTH_69); //Page 26, Gets us 15 bit resolution
  else if (pulseWidth < 215) MAX30102_setPulseWidth(MAX30102_PULSEWIDTH_118); //16 bit resolution
  else if (pulseWidth < 411) MAX30102_setPulseWidth(MAX30102_PULSEWIDTH_215); //17 bit resolution
  else if (pulseWidth == 411) MAX30102_setPulseWidth(MAX30102_PULSEWIDTH_411); //18 bit resolution
  else MAX30102_setPulseWidth(MAX30102_PULSEWIDTH_69);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //LED Pulse Amplitude Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Default is 0x1F which gets us 6.4mA
  //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

  MAX30102_setPulseAmplitudeRed(powerLevel);
  MAX30102_setPulseAmplitudeIR(powerLevel);
  //MAX30102_setPulseAmplitudeGreen(powerLevel);
  //MAX30102_setPulseAmplitudeProximity(powerLevel);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Multi-LED Mode Configuration, Enable the reading of the three LEDs
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  MAX30102_enableSlot(1, SLOT_RED_LED);
  if (ledMode > 1) MAX30102_enableSlot(2, SLOT_IR_LED);
  
  //enableSlot(1, SLOT_RED_PILOT);
  //enableSlot(2, SLOT_IR_PILOT);
  //enableSlot(3, SLOT_GREEN_PILOT);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  MAX30102_clearFIFO(); //Reset the FIFO before we begin checking the sensor
}
void MAX30102_FIFOWrite(uint8_t* data)
{
  HAL_I2C_Mem_Read(&hi2c1,MAX30102_ADDR|0x01,MAX30102_FIFODATA,1,data,6,20);
}