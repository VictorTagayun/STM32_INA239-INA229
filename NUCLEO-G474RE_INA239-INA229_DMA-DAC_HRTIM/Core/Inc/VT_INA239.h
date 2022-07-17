/*
 * VT_INA239.h
 *
 *  Created on: 26 Jun 2022
 *      Author: Victor
 */

//#ifndef INC_VT_INA239_H_
//#define INC_VT_INA239_H_

void VT_INA239_ReadReg(uint8_t Address);
void VT_INA239_ReadReg_16(uint8_t Address);
void VT_INA239_ReadReg_24(uint8_t Address);

/******************************* Register Map  ********************************/
#define INA239_REG_CONFIG       	 0x00  /*!< Configuration Register      */
#define INA239_REG_ADC_CONFIG        0x01  /*!< ADC Configuration Register      */
#define INA239_REG_SHUNT_CAL         0x02  /*!< Shunt Calibration Register        */
#define INA239_REG_VSHUNT    		 0x04  /*!< Shunt Voltage Measurement Register            */
#define INA239_REG_VBUS   			 0x05  /*!< Bus Voltage Measurement Register        */
#define INA239_REG_DIETEMP   		 0x06  /*!< Temperature Measurement Register        */
#define INA239_REG_CURRENT   		 0x07  /*!< Current Result Register        */
#define INA239_REG_POWER             0x08  /*!< Power Result register */
#define INA239_REG_DIAG_ALRT         0x0B  /*!< Diagnostic Flags and Alert register */
#define INA239_REG_SOVL              0x0C  /*!< Shunt Overvoltage Threshold register */
#define INA239_REG_SUVL              0x0D  /*!< Shunt Undervoltage Threshold register */
#define INA239_REG_BOVL              0x0E  /*!< Bus Overvoltage Threshold register */
#define INA239_REG_BUVL              0x0F  /*!< Bus Undervoltage Threshold register */
#define INA239_REG_TEMP_LIMIT        0x10  /*!< Temperature Over-Limit Threshold register */
#define INA239_REG_PWR_LIMIT         0x11  /*!< Power Over-Limit Threshold register */
#define INA239_REG_MANUFACTURER_ID   0x3E  /*!< Manufacturer ID register */
#define INA239_REG_DEVICE_ID         0x3F  /*!< Device ID register */
