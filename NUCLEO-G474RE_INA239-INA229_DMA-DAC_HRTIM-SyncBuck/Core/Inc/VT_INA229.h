/*
 * VT_INA229.h
 *
 *  Created on: 26 Jun 2022
 *      Author: Victor
 */

//#ifndef VT_INA229_H
//#define VT_INA229_H

void VT_INA229_ReadAllReg(void);
void VT_INA229_ReadRegPartial1(void);
void VT_INA229_ReadReg(uint8_t Address);
void VT_INA229_ReadReg_16(uint8_t Address);
void VT_INA229_ReadReg_24(uint8_t Address);
void VT_INA229_ReadReg_40(uint8_t Address);
void VT_INA229_ResetVars(void);

uint16_t combine_2_bytes(uint16_t high_byte, uint16_t low_byte);
uint32_t combine_3_bytes(uint32_t high_byte, uint32_t mid_byte, uint32_t low_byte);
int32_t combine_3_bytes_to_signed_20bits(uint32_t high_byte, uint32_t mid_byte, uint32_t low_byte);
uint64_t combine_5_bytes(uint64_t highhigh_byte, uint64_t high_byte, uint64_t mid_byte, uint64_t low_byte, uint64_t lowlow_byte);

void VT_INA229_WriteReg_16(uint8_t Address, uint16_t data);
void INA229_Write_CONFIG(uint16_t data);
void INA229_Write_ADC_CONFIG(uint16_t data);
void INA229_Write_SHUNT_CAL(uint16_t data);
void INA229_Write_SHUNT_TEMPCO(uint16_t data);
void INA229_Write_DIAG_ALERT(uint16_t data);
void INA229_Write_SOVL(uint16_t data);
void INA229_Write_SUVL(uint16_t data);
void INA229_Write_BOVL(uint16_t data);
void INA229_Write_BUVL(uint16_t data);
void INA229_Write_TEMP_LIMIT(uint16_t data);
void INA229_Write_POWER_LIMIT(uint16_t data);


//uint16_t combine_2_bytes(uint16_t high_byte, uint16_t low_byte);
//uint32_t combine_3_bytes(uint32_t high_byte, uint32_t mid_byte, uint32_t low_byte);
//uint64_t combine_5_bytes(uint64_t highhigh_byte, uint64_t high_byte, uint64_t mid_byte, uint64_t low_byte, uint64_t lowlow_byte);

//// SPI transmit data
//uint8_t INA229_msg_lenght_cntr = 0;
//uint8_t INA229_send_packet[100], INA229_send_packet_decoder[100];
//uint8_t INA229_recv_packet[100], INA229_recv_packet_decoder[100];

//uint16_t INA229_REG_CONFIG_val;
//uint16_t INA229_REG_ADC_CONFIG_val;
//uint16_t INA229_REG_SHUNT_CAL_val;
//uint16_t INA229_REG_SHUNT_TEMPCO_val;
//uint32_t INA229_REG_VSHUNT_val;
//uint32_t INA229_REG_VBUS_val;
//uint16_t INA229_REG_DIETEMP_val;
//uint32_t INA229_REG_CURRENT_val;
//uint32_t INA229_REG_POWER_val;
//uint64_t INA229_REG_ENERGY_val;
//uint64_t INA229_REG_CHARGE_val;
//uint16_t INA229_REG_DIAG_ALRT_val;
//uint16_t INA229_REG_SOVL_val;
//uint16_t INA229_REG_SUVL_val;
//uint16_t INA229_REG_BOVL_val;
//uint16_t INA229_REG_BUVL_val;
//uint16_t INA229_REG_TEMP_LIMIT_val;
//uint16_t INA229_REG_PWR_LIMIT_val;
//uint16_t INA229_REG_MANUFACTURER_ID_val;
//uint16_t INA229_REG_DEVICE_ID_val;

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
