#include "spi.h"
#include "ina229.h"



#define DIV_ROUND_CLOSEST(__DIVIDEND__, __DIVISOR__) (((__DIVIDEND__) + ((__DIVISOR__)/2)) / (__DIVISOR__))


uint16_t ina229_ReadReg_16(uint16_t Address)
{
  uint16_t data = 0;
  uint8_t dt[2];

  ReadRegister_16(Address, dt);
  data = (dt[0] << 8) | dt[1];
  return data;
}


uint32_t ina229_ReadReg_24(uint16_t Address)
{
  uint32_t data = 0;
  uint8_t dt[3];

  ReadRegister_24(Address,dt);
  data = (dt[0] << 16) | (dt[1] << 8) | dt[2];

  return data;
}

void ina229_WriteReg(uint16_t Address, uint16_t Data)
{
  uint8_t dt[2];

  dt[0] = Data >> 8;    /* MSB 1st */
  dt[1] = Data & 0xff;  /* LSB 2nd */


  WriteRegister(Address, dt);
}


void ina229_Reset()
{
  uint16_t cfg = 0;

  /*  Read programmed configuration  */
  cfg = ina229_ReadReg_16(INA229_REG_CONFIG);

  /* Set Reset bit bit */
  cfg |= INA229_REG_CFG_RESET;
  ina229_WriteReg(INA229_REG_CONFIG, cfg);
}


void ina229_Init()
{
  uint16_t cfg = 0;

  /* Reset all registers are to default values */
  ina229_Reset();

  /* Set INA230 configuration */
  cfg = INA229_MODE_CONTINOUS_VBUS_VSHUNT | INA229_VBUSCT_1052 | INA229_VSHCT_1052 | INA229_VTCT_1052 | INA229_AVG_16;
  //cfg = (cfg & ((uint16_t)~(INA229_REG_ADC_CONFIG_MODE_Msk))) | (pConfig->ShuntConvertTime << INA229_REG_ADC_CONFIG_MODE_Pos);
  //cfg = (cfg & ((uint16_t)~(INA229_REG_ADC_CONFIG_VBUSCT_Msk))) | (pConfig->BusConvertTime << INA229_REG_ADC_CONFIG_VBUSCT_Pos);
  //cfg = (cfg & ((uint16_t)~(INA229_REG_ADC_CONFIG_VSHCT_Msk))) | (pConfig->AveragingMode << INA229_REG_ADC_CONFIG_VSHCTPos);
  //cfg = (cfg & ((uint16_t)~(INA229_REG_ADC_CONFIG_VTCT_Msk))) | (pConfig->AveragingMode << INA230_REG_CFG_AVG_Pos);
  //cfg = (cfg & ((uint16_t)~(INA229_REG_ADC_CONFIG_AVG_Msk))) | (pConfig->AveragingMode << INA230_REG_CFG_AVG_Pos);

  ina229_WriteReg(INA229_REG_ADC_CONFIG, cfg);
}

uint16_t ina229_ReadManufacturerID()
{
  return ina229_ReadReg_16(INA229_REG_MANUFACTURER_ID);
}

uint16_t ina229_ReadDeviceID()
{
  return ina229_ReadReg_16(INA229_REG_DEVICE_ID);
}

void ina229_SetCalibration(uint16_t Calibration)
{
  ina229_WriteReg(INA229_REG_SHUNT_CAL, Calibration);
}

/**
  * @brief  Read programmed calibration value
  * @param  Address INA229 address on communication Bus.
  * @retval Calibration Calibration value.
  */
uint16_t ina230_GetCalibration(uint16_t Address)
{
  return ina229_ReadReg_16(INA229_REG_SHUNT_CAL);
}

int32_t ina230_GetCurrent()
{
  int32_t current = ((int32_t)(ina229_ReadReg_24(INA229_REG_CURRENT) << 8)) >> 8;

  return current;
}


/**
  * @brief  Read the bus voltage
  * @param  Address INA229 address on communication Bus.
  * @retval BusVoltage Bus voltage value (in mV)
  */
uint32_t ina229_GetVBus()
{
  uint32_t val;
  uint32_t vbus;
  val = ina229_ReadReg_24(INA229_REG_VBUS) >> 4;
  vbus = (val * 3125 >> 4);
  vbus = DIV_ROUND_CLOSEST(vbus, 1000);
  return (uint32_t)vbus;
}



/**
  * @brief  Read the shunt voltage
  * @param  Address INA229 address on communication Bus.
  * @retval VShunt Shunt voltage value (in mV)
  */
int32_t ina229_GetVShunt(uint8_t ADCRange)
{
  int32_t val;
  int32_t vshunt;
  val = ina229_ReadReg_24(INA229_REG_VSHUNT);
  if(ADCRange ==0 )
	  vshunt = val * 625 >> 1;
  else
	  vshunt = val * 625 >> 3;
  return vshunt;
}
