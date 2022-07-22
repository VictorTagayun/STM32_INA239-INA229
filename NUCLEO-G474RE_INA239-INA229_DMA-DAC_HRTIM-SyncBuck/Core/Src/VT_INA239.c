/*
 * VT_INA239.c
 *
 *  Created on: 26 Jun 2022
 *      Author: Victor
 */

#include "main.h"
#include "VT_INA239.h"

// SPI transmit data
uint8_t INA239_msg_lenght_cntr = 0;
uint8_t INA239_send_packet[100], INA239_send_packet_decoder[100];
uint8_t INA239_recv_packet[100], INA239_recv_packet_decoder[100];

void VT_INA239_ReadReg(uint8_t Address)
{
	if (Address == INA239_REG_POWER)
	{
		VT_INA239_ReadReg_24(Address);
	} else
	{
		VT_INA239_ReadReg_16(Address);
	}

}

void VT_INA239_ReadReg_16(uint8_t Address)
{
	INA239_send_packet_decoder[INA239_msg_lenght_cntr] = Address  + 1;
	INA239_send_packet[INA239_msg_lenght_cntr] = (Address << 2) + 1;
	INA239_msg_lenght_cntr++; // increment cntr for data
	INA239_msg_lenght_cntr = INA239_msg_lenght_cntr + 2; // leave space for 16bit data
}

void VT_INA239_ReadReg_24(uint8_t Address)
{
	INA239_send_packet_decoder[INA239_msg_lenght_cntr] = Address  + 1;
	INA239_send_packet[INA239_msg_lenght_cntr] = (Address << 2) + 1;
	INA239_msg_lenght_cntr++; // increment cntr for data
	INA239_msg_lenght_cntr = INA239_msg_lenght_cntr + 3; // leave space for 24bit data
}

void VT_INA239_WriteReg_16(uint8_t Address)
{
	INA239_msg_lenght_cntr++;
	INA239_send_packet[INA239_msg_lenght_cntr] = Address;
	INA239_msg_lenght_cntr = INA239_msg_lenght_cntr + 2;
}

void VT_INA239_WriteReg_24(uint8_t Address)
{
	INA239_msg_lenght_cntr++;
	INA239_send_packet[INA239_msg_lenght_cntr] = Address;
	INA239_msg_lenght_cntr = INA239_msg_lenght_cntr + 3;
}

void VT_INA239_WriteReg_40(uint8_t Address)
{
	INA239_msg_lenght_cntr++;
	INA239_send_packet[INA239_msg_lenght_cntr] = Address;
	INA239_msg_lenght_cntr = INA239_msg_lenght_cntr + 5;
}

//	// reset msg cntr before constructing the message
//	INA239_msg_lenght_cntr = 0;
//	VT_INA239_ReadReg(INA239_REG_CONFIG);
//	VT_INA239_ReadReg(INA239_REG_ADC_CONFIG);
//	VT_INA239_ReadReg(INA239_REG_SHUNT_CAL);
//	VT_INA239_ReadReg(INA239_REG_VSHUNT);
//	VT_INA239_ReadReg(INA239_REG_VBUS);
//	VT_INA239_ReadReg(INA239_REG_DIETEMP);
//	VT_INA239_ReadReg(INA239_REG_CURRENT);
//	VT_INA239_ReadReg(INA239_REG_POWER);
//	VT_INA239_ReadReg(INA239_REG_DIAG_ALRT);
//	VT_INA239_ReadReg(INA239_REG_SOVL);
//	VT_INA239_ReadReg(INA239_REG_SUVL);
//	VT_INA239_ReadReg(INA239_REG_BOVL);
//	VT_INA239_ReadReg(INA239_REG_BUVL);
//	VT_INA239_ReadReg(INA239_REG_TEMP_LIMIT);
//	VT_INA239_ReadReg(INA239_REG_PWR_LIMIT);
//	VT_INA239_ReadReg(INA239_REG_MANUFACTURER_ID);
//	VT_INA239_ReadReg(INA239_REG_DEVICE_ID);
