/*
 * VT_INA229.c
 *
 *  Created on: 26 Jun 2022
 *      Author: Victor
 */

#include "main.h"
#include "VT_INA229.h"

// SPI transmit data
uint8_t INA229_msg_lenght_cntr = 0;
uint8_t INA229_send_packet[100], INA229_send_packet_mirror[100];
uint8_t INA229_recv_packet[100], INA229_recv_packet_mirror[100];

void VT_INA229_ReadReg(uint8_t Address)
{
	if ((Address == INA229_REG_CONFIG) ||
			(Address == INA229_REG_ADC_CONFIG) ||
			(Address == INA229_REG_SHUNT_CAL) ||
			(Address == INA229_REG_SHUNT_TEMPCO) ||
			(Address == INA229_REG_DIETEMP) ||
			(Address == INA229_REG_DIAG_ALRT) ||
			(Address == INA229_REG_SOVL) ||
			(Address == INA229_REG_SUVL) ||
			(Address == INA229_REG_BOVL) ||
			(Address == INA229_REG_BUVL) ||
			(Address == INA229_REG_TEMP_LIMIT) ||
			(Address == INA229_REG_PWR_LIMIT) ||
			(Address == INA229_REG_MANUFACTURER_ID) ||
			(Address == INA229_REG_DEVICE_ID))
	{
		VT_INA229_ReadReg_16(Address);
	} else if ((Address == INA229_REG_VSHUNT) ||
			(Address == INA229_REG_VBUS) ||
			(Address == INA229_REG_CURRENT) ||
			(Address == INA229_REG_POWER))
	{
		VT_INA229_ReadReg_24(Address);
	} else if ((Address == INA229_REG_ENERGY) ||
			(Address == INA229_REG_CHARGE))
	{
		VT_INA229_ReadReg_40(Address);
	}

}


/*
#define INA229_REG_CONFIG       	 0x00  !< Configuration Register
#define INA229_REG_ADC_CONFIG        0x01  !< ADC Configuration Register
#define INA229_REG_SHUNT_CAL         0x02  !< Shunt Calibration Register
#define INA229_REG_SHUNT_TEMPCO      0x03  !< Shunt Temperature Coefficient Register
#define INA229_REG_DIETEMP   		 0x06  !< Temperature Measurement Register
#define INA229_REG_DIAG_ALRT         0x0B  !< Diagnostic Flags and Alert register
#define INA229_REG_SOVL              0x0C  !< Shunt Overvoltage Threshold register
#define INA229_REG_SUVL              0x0D  !< Shunt Undervoltage Threshold register
#define INA229_REG_BOVL              0x0E  !< Bus Overvoltage Threshold register
#define INA229_REG_BUVL              0x0F  !< Bus Undervoltage Threshold register
#define INA229_REG_TEMP_LIMIT        0x10  !< Temperature Over-Limit Threshold register
#define INA229_REG_PWR_LIMIT         0x11  !< Power Over-Limit Threshold register
#define INA229_REG_MANUFACTURER_ID   0x3E  !< Manufacturer ID register
#define INA229_REG_DEVICE_ID         0x3F  !< Device ID register */
void VT_INA229_ReadReg_16(uint8_t Address)
{
	INA229_send_packet_mirror[INA229_msg_lenght_cntr] = Address  + 1;
	INA229_send_packet[INA229_msg_lenght_cntr] = (Address << 2) + 1;
	INA229_msg_lenght_cntr++; // increment cntr for data
	INA229_msg_lenght_cntr = INA229_msg_lenght_cntr + 2; // leave space for 16bit data
}

/*
#define INA229_REG_VSHUNT    		 0x04  !< Shunt Voltage Measurement Register
#define INA229_REG_VBUS   			 0x05  !< Bus Voltage Measurement Register
#define INA229_REG_CURRENT   		 0x07  !< Current Result Register
#define INA229_REG_POWER             0x08  !< Power Result register */
void VT_INA229_ReadReg_24(uint8_t Address)
{
	INA229_send_packet_mirror[INA229_msg_lenght_cntr] = Address  + 1;
	INA229_send_packet[INA229_msg_lenght_cntr] = (Address << 2) + 1;
	INA229_msg_lenght_cntr++; // increment cntr for data
	INA229_msg_lenght_cntr = INA229_msg_lenght_cntr + 3; // leave space for 24bit data
}

/*
#define INA229_REG_ENERGY            0x09  !< Energy Result register
#define INA229_REG_CHARGE            0x0A  !< Charge Result register */
void VT_INA229_ReadReg_40(uint8_t Address)
{
	INA229_send_packet_mirror[INA229_msg_lenght_cntr] = Address  + 1;
	INA229_send_packet[INA229_msg_lenght_cntr] = (Address << 2) + 1;
	INA229_msg_lenght_cntr++; // increment cntr for data
	INA229_msg_lenght_cntr = INA229_msg_lenght_cntr + 5; // leave space for 40bit data
}

void VT_INA229_WriteReg_16(uint8_t Address)
{
	INA229_msg_lenght_cntr++;
	INA229_send_packet[INA229_msg_lenght_cntr] = Address;
	INA229_msg_lenght_cntr = INA229_msg_lenght_cntr + 2;
}

void VT_INA229_WriteReg_24(uint8_t Address)
{
	INA229_msg_lenght_cntr++;
	INA229_send_packet[INA229_msg_lenght_cntr] = Address;
	INA229_msg_lenght_cntr = INA229_msg_lenght_cntr + 3;
}

void VT_INA229_WriteReg_40(uint8_t Address)
{
	INA229_msg_lenght_cntr++;
	INA229_send_packet[INA229_msg_lenght_cntr] = Address;
	INA229_msg_lenght_cntr = INA229_msg_lenght_cntr + 5;
}
