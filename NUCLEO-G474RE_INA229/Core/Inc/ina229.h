/*
 * ina229.h
 *
 *  Created on: Sep 1, 2021
 *      Author: micro
 */

#ifndef INC_INA229_H_
#define INC_INA229_H_

/******************************* Register Map  ********************************/
#define INA229_REG_CONFIG       	 0x00  /*!< Configuration Register      */
#define INA229_REG_ADC_CONFIG        0x01  /*!< ADC Configuration Register      */
#define INA229_REG_SHUNT_CAL         0x02  /*!< Shunt Calibration Register        */
#define INA229_REG_SHUNT_TEMPCO      0x03  /*!< Shunt Temperature Coefficient Register              */
#define INA229_REG_VSHUNT    		 0x04  /*!< Shunt Voltage Measurement Register            */
#define INA229_REG_VBUS   			 0x05  /*!< Bus Voltage Measurement Register        */
#define INA229_REG_DIETEMP   		 0x06  /*!< Temperature Measurement Register        */
#define INA229_REG_CURRENT   		 0x07  /*!< Current Result Register        */
#define INA229_REG_POWER             0x08  /*!< Power Result register */
#define INA229_REG_ENERGY            0x09  /*!< Energy Result register */
#define INA229_REG_CHARGE            0x0A  /*!< Charge Result register */
#define INA229_REG_DIAG_ALRT         0x0B  /*!< Diagnostic Flags and Alert register */
#define INA229_REG_SOVL              0x0C  /*!< Shunt Overvoltage Threshold register */
#define INA229_REG_SUVL              0x0D  /*!< Shunt Undervoltage Threshold register */
#define INA229_REG_BOVL              0x0E  /*!< Bus Overvoltage Threshold register */
#define INA229_REG_BUVL              0x0F  /*!< Bus Undervoltage Threshold register */
#define INA229_REG_TEMP_LIMIT        0x10  /*!< Temperature Over-Limit Threshold register */
#define INA229_REG_PWR_LIMIT         0x11  /*!< Power Over-Limit Threshold register */
#define INA229_REG_MANUFACTURER_ID   0x3E  /*!< Manufacturer ID register */
#define INA229_REG_DEVICE_ID         0x3F  /*!< Device ID register */

/*******************  Bit definition for Configuration register  **************/
#define INA229_REG_CFG_RESET_Pos        (15U)
#define INA229_REG_CFG_RESET_Msk        (0x1U << INA229_REG_CFG_RESET_Pos)      /*!< 0x8000 */
#define INA229_REG_CFG_RESET            INA229_REG_CFG_RESET_Msk                /*!< Reset Bit */
#define INA229_REG_CFG_RSTACC_Pos        (14U)
#define INA229_REG_CFG_RSTACC_Msk        (0x1U << INA229_REG_CFG_RSTACC_Pos)      /*!< 0x8000 */
#define INA229_REG_CFG_RSTACC            INA229_REG_CFG_RSTACC_Msk                /*!< Reset Bit */
#define INA229_REG_CFG_CONVDLY_Pos        (6U)
#define INA229_REG_CFG_CONVDLY_Msk        (0xFFU << INA229_REG_CFG_CONVDLY_Pos)      /*!< 0x8000 */
#define INA229_REG_CFG_CONVDLY            INA229_REG_CFG_CONVDLY_Msk                /*!< Reset Bit */
#define INA229_REG_CFG_TEMPCOMP_Pos        (5U)
#define INA229_REG_CFG_TEMPCOMP_Msk        (0x1U << INA229_REG_CFG_TEMPCOMP_Pos)      /*!< 0x8000 */
#define INA229_REG_CFG_TEMPCOMP            INA229_REG_CFG_TEMPCOMP_Msk                /*!< Reset Bit */
#define INA229_REG_CFG_ADCRANGE_Pos        (4U)
#define INA229_REG_CFG_ADCRANGE_Msk        (0x1U << INA229_REG_CFG_ADCRANGE_Pos)      /*!< 0x8000 */
#define INA229_REG_CFG_ADCRANGE            INA229_REG_CFG_ADCRANGE_Msk                /*!< Reset Bit */

/*******************  Bit definition for ADC Config register  **************/
#define INA229_REG_ADC_CONFIG_MODE_Pos        (12U)
#define INA229_REG_ADC_CONFIG_MODE_Msk        (0xFU << INA229_REG_ADC_CONFIG_MODE_Pos)      /*!< 0x8000 */
#define INA229_REG_ADC_CONFIG_MODE            INA229_REG_ADC_CONFIG_MODE_Msk                /*!< Reset Bit */
#define INA229_REG_ADC_CONFIG_MODE_0           (0x1U << INA229_REG_ADC_CONFIG_MODE_Pos)       /*!< 0x0001 */
#define INA229_REG_ADC_CONFIG_MODE_1           (0x2U << INA229_REG_ADC_CONFIG_MODE_Pos)       /*!< 0x0002 */
#define INA229_REG_ADC_CONFIG_MODE_2           (0x4U << INA229_REG_ADC_CONFIG_MODE_Pos)       /*!< 0x0004 */
#define INA229_REG_ADC_CONFIG_MODE_3           (0x8U << INA229_REG_ADC_CONFIG_MODE_Pos)       /*!< 0x0004 */

#define INA229_REG_ADC_CONFIG_VBUSCT_Pos        (9U)
#define INA229_REG_ADC_CONFIG_VBUSCT_Msk        (0x7U << INA229_REG_ADC_CONFIG_VBUSCT_Pos)      /*!< 0x8000 */
#define INA229_REG_ADC_CONFIG_VBUSCT            INA229_REG_ADC_CONFIG_VBUSCT_Msk                /*!< Reset Bit */
#define INA229_REG_ADC_CONFIG_VBUSCT_0           (0x1U << INA229_REG_ADC_CONFIG_VBUSCT_Pos)       /*!< 0x0001 */
#define INA229_REG_ADC_CONFIG_VBUSCT_1           (0x2U << INA229_REG_ADC_CONFIG_VBUSCT_Pos)       /*!< 0x0002 */
#define INA229_REG_ADC_CONFIG_VBUSCT_2           (0x4U << INA229_REG_ADC_CONFIG_VBUSCT_Pos)       /*!< 0x0004 */

#define INA229_REG_ADC_CONFIG_VSHCT_Pos        (6U)
#define INA229_REG_ADC_CONFIG_VSHCT_Msk        (0x7U << INA229_REG_ADC_CONFIG_VSHCT_Pos)      /*!< 0x8000 */
#define INA229_REG_ADC_CONFIG_VSHCT            INA229_REG_ADC_CONFIG_VSHCT_Msk                /*!< Reset Bit */
#define INA229_REG_ADC_CONFIG_VSHCT_0           (0x1U << INA229_REG_ADC_CONFIG_VSHCT_Pos)       /*!< 0x0001 */
#define INA229_REG_ADC_CONFIG_VSHCT_1           (0x2U << INA229_REG_ADC_CONFIG_VSHCT_Pos)       /*!< 0x0002 */
#define INA229_REG_ADC_CONFIG_VSHCT_2           (0x4U << INA229_REG_ADC_CONFIG_VSHCT_Pos)       /*!< 0x0004 */

#define INA229_REG_ADC_CONFIG_VTCT_Pos        (3U)
#define INA229_REG_ADC_CONFIG_VTCT_Msk        (0x7U << INA229_REG_ADC_CONFIG_VTCT_Pos)      /*!< 0x8000 */
#define INA229_REG_ADC_CONFIG_VTCT            INA229_REG_ADC_CONFIG_VTCT_Msk                /*!< Reset Bit */
#define INA229_REG_ADC_CONFIG_VTCT_0           (0x1U << INA229_REG_ADC_CONFIG_VTCT_Pos)       /*!< 0x0001 */
#define INA229_REG_ADC_CONFIG_VTCT_1           (0x2U << INA229_REG_ADC_CONFIG_VTCT_Pos)       /*!< 0x0002 */
#define INA229_REG_ADC_CONFIG_VTCT_2           (0x4U << INA229_REG_ADC_CONFIG_VTCT_Pos)       /*!< 0x0004 */


#define INA229_REG_ADC_CONFIG_AVG_Pos        (0U)
#define INA229_REG_ADC_CONFIG_AVG_Msk        (0x7U << INA229_REG_ADC_CONFIG_AVG_Pos)      /*!< 0x8000 */
#define INA229_REG_ADC_CONFIG_AVG            INA229_REG_ADC_CONFIG_AVG_Msk                /*!< Reset Bit */
#define INA229_REG_ADC_CONFIG_AVG_0           (0x1U << INA229_REG_ADC_CONFIG_AVG_Pos)       /*!< 0x0001 */
#define INA229_REG_ADC_CONFIG_AVG_1           (0x2U << INA229_REG_ADC_CONFIG_AVG_Pos)       /*!< 0x0002 */
#define INA229_REG_ADC_CONFIG_AVG_2           (0x4U << INA229_REG_ADC_CONFIG_AVG_Pos)       /*!< 0x0004 */


/*******************  Bit definition for Calibration register  ****************/
#define INA229_REG_CALIBRATION_DATA_Pos (0U)
#define INA229_REG_CALIBRATION_DATA_Msk (0x7FFFU << INA229_REG_CALIBRATION_DATA_Pos) /*!< 0x7FFF */
#define INA229_REG_CALIBRATION_DATA     INA229_REG_CALIBRATION_DATA_Msk              /*!< Measured current */

/******************  Bit definition for Shunt Voltage register  ***************/
#define INA229_REG_VSHUNT_DATA_Pos      (4U)
#define INA229_REG_VSHUNT_DATA_Msk      (0x7FFFFU << INA229_REG_VSHUNT_DATA_Pos) /*!< 0x7FFF */
#define INA229_REG_VSHUNT_DATA          INA229_REG_VSHUNT_DATA_Msk              /*!< Measured shunt voltage */
#define INA229_REG_VSHUNT_SIGN_Pos      (23U)
#define INA229_REG_VSHUNT_SIGN_Msk      (0x1U << INA229_REG_VSHUNT_SIGN_Pos)    /*!< 0x8000 */
#define INA229_REG_VSHUNT_SIGN          INA229_REG_VSHUNT_SIGN_Msk              /*!< Sign */

/*******************  Bit definition for Bus Voltage register  ****************/
#define INA229_REG_VBUS_DATA_Pos        (4U)
#define INA229_REG_VBUS_DATA_Msk        (0xFFFFFU << INA229_REG_VBUS_DATA_Pos)   /*!< 0x7FFF */
#define INA229_REG_VBUS_DATA            INA229_REG_VBUS_DATA_Msk                /*!< Measured bus voltage */


/*********************  Bit definition for Power register  ********************/
#define INA229_REG_PWR_DATA_Pos         (0U)
#define INA229_REG_PWR_DATA_Msk         (0xFFFFFFU << INA229_REG_PWR_DATA_Pos)    /*!< 0xFFFF */
#define INA229_REG_PWR_DATA             INA229_REG_PWR_DATA_Msk                 /*!< Measured power */

/*****************  Bit definition for Current register  **********************/
#define INA229_REG_CURRENT_DATA_Pos     (4U)
#define INA229_REG_CURRENT_DATA_Msk     (0x7FFFFU << INA229_REG_CURRENT_DATA_Pos) /*!< 0x7FFF */
#define INA229_REG_CURRENT_DATA         INA229_REG_CURRENT_DATA_Msk              /*!< Measured current */
#define INA229_REG_CURRENT_SIGN_Pos     (23U)
#define INA229_REG_CURRENT_SIGN_Msk     (0x1U << INA229_REG_CURRENT_SIGN_Pos)   /*!< 0x8000 */
#define INA229_REG_CURRENT_SIGN         INA229_REG_CURRENT_SIGN_Msk             /*!< Sign */

/*****************  Bit definition for Die Temp register  **********************/
#define INA229_REG_DIETEMP_DATA_Pos     (0U)
#define INA229_REG_DIETEMP_DATA_Msk     (0x7FFFU << INA229_REG_DIETEMP_DATA_Pos) /*!< 0x7FFF */
#define INA229_REG_DIETEMP_DATA         INA229_REG_CURRENT_DATA_Msk              /*!< Measured current */
#define INA229_REG_DIETEMP_SIGN_Pos     (15U)
#define INA229_REG_DIETEMP_SIGN_Msk     (0x1U << INA229_REG_DIETEMP_SIGN_Pos)   /*!< 0x8000 */
#define INA229_REG_DIETEMP_SIGN         INA229_REG_CURRENT_SIGN_Msk             /*!< Sign */


/** @defgroup INA229_MODE  INA229 operating mode
  * @{
  */
#define INA229_MODE_POWER_DOWN       (uint16_t)0x0000U                                 /*!< Turn off current into the INA230 inputs               */
#define INA229_MODE_TRIGGERED_VSHUNT INA229_REG_ADC_CONFIG_MODE_0                             /*!< Shunt Voltage, Single shot conversion                 */
#define INA229_MODE_TRIGGERED_VBUS   INA229_REG_ADC_CONFIG_MODE_1                             /*!< Bus voltage, Single shot conversion                   */
#define INA229_MODE_TRIGGERED_VTEMP   INA229_REG_ADC_CONFIG_MODE_2                             /*!< Bus voltage, Single shot conversion                   */
#define INA229_MODE_TRIGGERED_VBUS_VSHUNT   (INA229_REG_ADC_CONFIG_MODE_1 | INA229_REG_ADC_CONFIG_MODE_0)                            /*!< Bus voltage, Single shot conversion                   */

#define INA229_MODE_CONTINOUS_VSHUNT (INA229_REG_ADC_CONFIG_MODE_3 | INA229_REG_ADC_CONFIG_MODE_2 | INA229_REG_ADC_CONFIG_MODE_0)   /*!< Shunt Voltage, Continuous conversion                  */
#define INA229_MODE_CONTINOUS_VBUS   (INA229_REG_ADC_CONFIG_MODE_3 | INA229_REG_ADC_CONFIG_MODE_2 | INA229_REG_ADC_CONFIG_MODE_1)   /*!< Bus voltage, Continuous conversion                    */
#define INA229_MODE_CONTINOUS_VTEMP    (INA229_REG_ADC_CONFIG_MODE_3 | INA229_REG_ADC_CONFIG_MODE)                               /*!< Shunt Voltage and Bus voltage, Continuous conversion  */
#define INA229_MODE_CONTINOUS_VBUS_VSHUNT   (INA229_REG_ADC_CONFIG_MODE_3 | INA229_REG_ADC_CONFIG_MODE_1 | INA229_REG_ADC_CONFIG_MODE_0)                            /*!< Bus voltage, Single shot conversion                   */

/** @defgroup INA229_VBUSCT INA229 Vbus ADC conversion time
  * @{
  */
#define INA229_VBUSCT_50     (uint16_t)0x0000U                                       /*!< ADC conversion time: 140 us  */
#define INA229_VBUSCT_84     INA229_REG_ADC_CONFIG_VBUSCT_0                               /*!< ADC conversion time: 204 us  */
#define INA229_VBUSCT_150     INA229_REG_ADC_CONFIG_VBUSCT_1                               /*!< ADC conversion time: 332 us  */
#define INA229_VBUSCT_280     (INA229_REG_ADC_CONFIG_VBUSCT_1 | INA229_REG_CFG_VBUSCT_0) /*!< ADC conversion time: 588 us  */
#define INA229_VBUSCT_540    INA229_REG_ADC_CONFIG_VBUSCT_2                               /*!< ADC conversion time: 1100 us */
#define INA229_VBUSCT_1052    (INA229_REG_ADC_CONFIG_VBUSCT_2| INA229_REG_ADC_CONFIG_VBUSCT_0)  /*!< ADC conversion time: 2116 us */
#define INA229_VBUSCT_2074    (INA229_REG_ADC_CONFIG_VBUSCT_2| INA229_REG_ADC_CONFIG_VBUSCT_1)  /*!< ADC conversion time: 4156 us */
#define INA229_VBUSCT_4120    INA229_REG_ADC_CONFIG_VBUSCT                                 /*!< ADC conversion time: 8244 us */

/** @defgroup INA229_VSHCT INA229 shunt ADC conversion time
  * @{
  */
#define INA229_VSHCT_50     (uint16_t)0x0000U                                       /*!< ADC conversion time: 140 us  */
#define INA229_VSHCT_84     INA229_REG_ADC_CONFIG_VSHCT_0                               /*!< ADC conversion time: 204 us  */
#define INA229_VSHCT_150     INA229_REG_ADC_CONFIG_VSHCT_1                               /*!< ADC conversion time: 332 us  */
#define INA229_VSHCT_280     (INA229_REG_ADC_CONFIG_VSHCT_1 | INA229_REG_CFG_VSHCT_0) /*!< ADC conversion time: 588 us  */
#define INA229_VSHCT_540    INA229_REG_ADC_CONFIG_VBUSCT_2                               /*!< ADC conversion time: 1100 us */
#define INA229_VSHCT_1052    (INA229_REG_ADC_CONFIG_VSHCT_2| INA229_REG_ADC_CONFIG_VSHCT_0)  /*!< ADC conversion time: 2116 us */
#define INA229_VSHCT_2074    (INA229_REG_ADC_CONFIG_VSHCT_2| INA229_REG_ADC_CONFIG_VSHCT_1)  /*!< ADC conversion time: 4156 us */
#define INA229_VSHCT_4120    INA229_REG_ADC_CONFIG_VSHCT                                 /*!< ADC conversion time: 8244 us */


/** @defgroup INA229_VSHCT INA229 current ADC conversion time
  * @{
  */
#define INA229_VTCT_50     (uint16_t)0x0000U                                       /*!< ADC conversion time: 140 us  */
#define INA229_VTCT_84     INA229_REG_ADC_CONFIG_VTCT_0                               /*!< ADC conversion time: 204 us  */
#define INA229_VTCT_150     INA229_REG_ADC_CONFIG_VTCT_1                               /*!< ADC conversion time: 332 us  */
#define INA229_VTCT_280     (INA229_REG_ADC_CONFIG_VTCT_1 | INA229_REG_CFG_VTCT_0) /*!< ADC conversion time: 588 us  */
#define INA229_VTCT_540    INA229_REG_ADC_CONFIG_VTCT_2                               /*!< ADC conversion time: 1100 us */
#define INA229_VTCT_1052    (INA229_REG_ADC_CONFIG_VTCT_2| INA229_REG_ADC_CONFIG_VTCT_0)  /*!< ADC conversion time: 2116 us */
#define INA229_VTCT_2074    (INA229_REG_ADC_CONFIG_VTCT_2| INA229_REG_ADC_CONFIG_VTCT_1)  /*!< ADC conversion time: 4156 us */
#define INA229_VTCT_4120    INA229_REG_ADC_CONFIG_VTCT                                 /*!< ADC conversion time: 8244 us */

/** @defgroup INA229_AVG INA229 number of averages
  * @{
  */
#define INA229_AVG_1      (uint16_t)0x0000U                                       /*!< Number of averages: 1    */
#define INA229_AVG_4      INA229_REG_ADC_CONFIG_AVG_0                               /*!< Number of averages: 4    */
#define INA229_AVG_16     INA229_REG_ADC_CONFIG_AVG_1                               /*!< Number of averages: 16   */
#define INA229_AVG_64     (INA229_REG_ADC_CONFIG_AVG_1 | INA229_REG_ADC_CONFIG_AVG_0) /*!< Number of averages: 64   */
#define INA229_AVG_128    INA229_REG_ADC_CONFIG_AVG_2                               /*!< Number of averages: 128  */
#define INA229_AVG_256    (INA229_REG_ADC_CONFIG_AVG_2| INA229_REG_ADC_CONFIG_AVG_0)  /*!< Number of averages: 256  */
#define INA229_AVG_512    (INA229_REG_ADC_CONFIG_AVG_2| INA229_REG_ADC_CONFIG_AVG_1)  /*!< Number of averages: 512  */
#define INA229_AVG_1024   INA229_REG_ADC_CONFIG_AVG                                 /*!< Number of averages: 1024 */


#endif /* INC_INA229_H_ */

uint16_t ina229_ReadReg_16(uint16_t Address);
uint32_t ina229_ReadReg_24(uint16_t Address);
void ina229_WriteReg(uint16_t Address, uint16_t Data);
void ina229_Init();
uint16_t ina229_ReadManufacturerID();
uint16_t ina229_ReadDeviceID();
void ina229_SetCalibration(uint16_t Calibration);
uint16_t ina230_GetCalibration(uint16_t Address);
int32_t ina230_GetCurrent();
uint32_t ina229_GetVBus();
int32_t ina229_GetVShunt(uint8_t ADCRange);
