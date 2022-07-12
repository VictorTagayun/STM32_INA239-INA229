/*
 * VT_INA229.c
 *
 *  Created on: 26 Jun 2022
 *      Author: Victor
 */

#include "main.h"
#include "VT_INA229.h"
#include <stdio.h>

extern void SPI_DMA_TXRX(void);

// SPI transmit data
uint8_t INA229_msg_lenght_cntr = 0, INA229_msg_lenght_cntr_old;
uint8_t INA229_send_packet[70], INA229_send_packet_decoder[70];
uint8_t INA229_recv_packet[70], INA229_recv_packet_decoder[70];

uint16_t INA229_REG_CONFIG_val;
uint16_t INA229_REG_ADC_CONFIG_val;
uint16_t INA229_REG_SHUNT_CAL_val;
uint16_t INA229_REG_SHUNT_TEMPCO_val;
uint32_t INA229_REG_VSHUNT_val;
uint32_t INA229_REG_VBUS_val;
uint16_t INA229_REG_DIETEMP_val;
uint32_t INA229_REG_CURRENT_val;
uint32_t INA229_REG_POWER_val;
uint64_t INA229_REG_ENERGY_val;
uint64_t INA229_REG_CHARGE_val;
uint16_t INA229_REG_DIAG_ALRT_val;
uint16_t INA229_REG_SOVL_val;
uint16_t INA229_REG_SUVL_val;
uint16_t INA229_REG_BOVL_val;
uint16_t INA229_REG_BUVL_val;
uint16_t INA229_REG_TEMP_LIMIT_val;
uint16_t INA229_REG_PWR_LIMIT_val;
uint16_t INA229_REG_MANUFACTURER_ID_val;
uint16_t INA229_REG_DEVICE_ID_val;

uint16_t combine_2_bytes(uint16_t high_byte, uint16_t low_byte)
{
	return (high_byte << 8) + low_byte;
}

uint32_t combine_3_bytes(uint32_t high_byte, uint32_t mid_byte, uint32_t low_byte)
{
	return (high_byte << 8*2) + (mid_byte << 8) + low_byte;
}

uint64_t combine_5_bytes(uint64_t highhigh_byte, uint64_t high_byte, uint64_t mid_byte, uint64_t low_byte, uint64_t lowlow_byte)
{
	return (highhigh_byte << 8*4) + (high_byte << 8*3) + (mid_byte << 8*2) + (low_byte << 8) + lowlow_byte;
}

void VT_INA229_ReadAllReg(void)
{
//	printf("VT_INA229_ReadAllReg \n");
	// reset msg cntr before constructing the message
	INA229_msg_lenght_cntr = 0;
	VT_INA229_ReadReg(INA229_REG_CONFIG);
	VT_INA229_ReadReg(INA229_REG_ADC_CONFIG);
	VT_INA229_ReadReg(INA229_REG_SHUNT_CAL);
	VT_INA229_ReadReg(INA229_REG_SHUNT_TEMPCO);
	VT_INA229_ReadReg(INA229_REG_VSHUNT);
	VT_INA229_ReadReg(INA229_REG_VBUS);
	VT_INA229_ReadReg(INA229_REG_DIETEMP);
	VT_INA229_ReadReg(INA229_REG_CURRENT);
	VT_INA229_ReadReg(INA229_REG_POWER);
	VT_INA229_ReadReg(INA229_REG_ENERGY);
	VT_INA229_ReadReg(INA229_REG_CHARGE);
	VT_INA229_ReadReg(INA229_REG_DIAG_ALRT);
	VT_INA229_ReadReg(INA229_REG_SOVL);
	VT_INA229_ReadReg(INA229_REG_SUVL);
	VT_INA229_ReadReg(INA229_REG_BOVL);
	VT_INA229_ReadReg(INA229_REG_BUVL);
	VT_INA229_ReadReg(INA229_REG_TEMP_LIMIT);
	VT_INA229_ReadReg(INA229_REG_PWR_LIMIT);
	VT_INA229_ReadReg(INA229_REG_MANUFACTURER_ID);
	VT_INA229_ReadReg(INA229_REG_DEVICE_ID);
	//	SPI_DMA_TXRX();
}

void VT_INA229_ReadRegPartial1(void)
{
//	printf("VT_INA229_ReadRegPartial1 \n");
	// reset msg cntr before constructing the message
	INA229_msg_lenght_cntr = 0;
	VT_INA229_ReadReg(INA229_REG_CONFIG);
	VT_INA229_ReadReg(INA229_REG_ADC_CONFIG);
	VT_INA229_ReadReg(INA229_REG_SHUNT_CAL);
	VT_INA229_ReadReg(INA229_REG_SHUNT_TEMPCO);
	VT_INA229_ReadReg(INA229_REG_VSHUNT);
	VT_INA229_ReadReg(INA229_REG_VBUS);
	VT_INA229_ReadReg(INA229_REG_DIETEMP);
	VT_INA229_ReadReg(INA229_REG_CURRENT);
	VT_INA229_ReadReg(INA229_REG_POWER);
	VT_INA229_ReadReg(INA229_REG_ENERGY);
	VT_INA229_ReadReg(INA229_REG_CHARGE);
	VT_INA229_ReadReg(INA229_REG_DIAG_ALRT);
	//	SPI_DMA_TXRX();
}

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

void INA229_Write_CONFIG(uint16_t data)
{
	uint8_t Address = 0;
	VT_INA229_WriteReg_16(Address, data);
}

void INA229_Write_ADC_CONFIG(uint16_t data)
{
	uint8_t Address = 1;
	VT_INA229_WriteReg_16(Address, data);
}

void INA229_Write_SHUNT_CAL(uint16_t data)
{
	uint8_t Address = 2;
	VT_INA229_WriteReg_16(Address, data);
}

void INA229_Write_SHUNT_TEMPCO(uint16_t data)
{
	uint8_t Address = 3;
	VT_INA229_WriteReg_16(Address, data);
}

void INA229_Write_DIAG_ALERT(uint16_t data)
{
	uint8_t Address = 0x0b;
	VT_INA229_WriteReg_16(Address, data);
}

void INA229_Write_SOVL(uint16_t data)
{
	uint8_t Address = 0x0c;
	VT_INA229_WriteReg_16(Address, data);
}

void INA229_Write_SUVL(uint16_t data)
{
	uint8_t Address = 0x0d;
	VT_INA229_WriteReg_16(Address, data);
}

void INA229_Write_BOVL(uint16_t data)
{
	uint8_t Address = 0x0e;
	VT_INA229_WriteReg_16(Address, data);
}

void INA229_Write_BUVL(uint16_t data)
{
	uint8_t Address = 0x0f;
	VT_INA229_WriteReg_16(Address, data);
}

void INA229_Write_TEMP_LIMIT(uint16_t data)
{
	uint8_t Address = 0x10;
	VT_INA229_WriteReg_16(Address, data);
}

void INA229_Write_POWER_LIMIT(uint16_t data)
{
	uint8_t Address = 0x11;
	VT_INA229_WriteReg_16(Address, data);
}

void VT_INA229_ResetVars(void)
{
	// reset everything
//	for (uint8_t cntr = 0; cntr < INA229_msg_lenght_cntr; cntr++)
//	{
//		INA229_send_packet[cntr] = 0xff;
//		INA229_recv_packet[cntr] = 0xff;
//		INA229_send_packet_decoder[cntr] = 0;
//		INA229_recv_packet_decoder[cntr] = 0;
//		INA229_msg_lenght_cntr = 0;
//		//		printf("INA229_send_packet[%d] = %x \n", cntr , INA229_send_packet[cntr]);
//	}
	INA229_msg_lenght_cntr_old = INA229_msg_lenght_cntr ;
	INA229_msg_lenght_cntr = 0;
}

void VT_INA229_WriteReg_16(uint8_t Address, uint16_t data)
{
	INA229_send_packet_decoder[INA229_msg_lenght_cntr] = Address + 1; // the address only, does not include Read/Write command
	INA229_send_packet[INA229_msg_lenght_cntr++] = (Address << 2); // the address + Write "0" command, increment INA229_msg_lenght_cntr for next byte
	INA229_send_packet[INA229_msg_lenght_cntr++] = (data >> 8); // high byte, increment INA229_msg_lenght_cntr for next byte
	INA229_send_packet[INA229_msg_lenght_cntr++] = data; // low byte, increment INA229_msg_lenght_cntr for next byte
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
	INA229_send_packet_decoder[INA229_msg_lenght_cntr] = Address  + 1; // the address only, does not include Read/Write command
	INA229_send_packet[INA229_msg_lenght_cntr] = (Address << 2) + 1; // the address + Read "1" command, increment INA229_msg_lenght_cntr for next byte
	INA229_msg_lenght_cntr++; // increment cntr for dataH, increment INA229_msg_lenght_cntr for next byte
	INA229_msg_lenght_cntr++; // increment cntr for dataL, increment INA229_msg_lenght_cntr for next byte
	// original code below
	//INA229_msg_lenght_cntr = INA229_msg_lenght_cntr + 2; // leave space for 24bit data
	// modified
	INA229_msg_lenght_cntr++; // increment cntr for next packet or start of address
}

/*
#define INA229_REG_VSHUNT    		 0x04  !< Shunt Voltage Measurement Register
#define INA229_REG_VBUS   			 0x05  !< Bus Voltage Measurement Register
#define INA229_REG_CURRENT   		 0x07  !< Current Result Register
#define INA229_REG_POWER             0x08  !< Power Result register */
void VT_INA229_ReadReg_24(uint8_t Address)
{
	INA229_send_packet_decoder[INA229_msg_lenght_cntr] = Address  + 1; // the address only, does not include Read/Write command
	INA229_send_packet[INA229_msg_lenght_cntr] = (Address << 2) + 1; // the address + Read "1" command, increment INA229_msg_lenght_cntr for next byte
	INA229_msg_lenght_cntr++; // increment cntr for data, increment INA229_msg_lenght_cntr for next byte
	INA229_msg_lenght_cntr = INA229_msg_lenght_cntr + 3; // leave space for 24bit data, increment INA229_msg_lenght_cntr for next byte
}

/*
#define INA229_REG_ENERGY            0x09  !< Energy Result register
#define INA229_REG_CHARGE            0x0A  !< Charge Result register */
void VT_INA229_ReadReg_40(uint8_t Address)
{
	INA229_send_packet_decoder[INA229_msg_lenght_cntr] = Address  + 1; // the address only, does not include Read/Write command
	INA229_send_packet[INA229_msg_lenght_cntr] = (Address << 2) + 1; // the address + Read "1" command
	INA229_msg_lenght_cntr++; // increment cntr for data
	INA229_msg_lenght_cntr = INA229_msg_lenght_cntr + 5; // leave space for 40bit data
}
