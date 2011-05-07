/* ----------------------------------------------------------------------------
 * vd5376_mouse.h  --  Driver for VD5376 Finger mouse
 *
 * Copyright (C) 2010 by Pyeongjeong Lee
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _VD5376_MOUSE_H
#define _VD5376_MOUSE_H

#define Device_Hardware_Revision	0x00
#define Device_Software_Revision	0x01
#define Reference_Setting		0x05
#define Device_Input_Value		0x0F
#define Device_X_Motion			0x21
#define Device_Y_Motion			0x22
#define Device_Motion_Status		0x23
#define Device_Value_Setting		0x27
#define Device_Min_Features		0x29
#define Device_Scaling_X_Motion		0x2A
#define Device_Scaling_Y_Motion		0x2B
#define Device_Features_Count1		0x31
#define Device_Features_Count2		0x32
#define Device_Exposure			0x41
#define Device_Auto_Exposure		0x43
#define Device_Vbat_Convert_Data	0x47
#define Device_Exp_Max_Value		0x4F
#define Device_Image			0x61
#define Device_Frame_Setting		0x62
#define Device_Timer_Interrupt		0x82

// Reference_Setting		0x05
#define Automatic_Power_Management	(1<<0)
#define Laser_Selected			(1<<1)
#define Use_External_Supply		(1<<2)
#define Host_Config_Done		(1<<3)
#define Led_DAC_Driven			(1<<5)
#define FW_Idle_State			(1<<7)

// Device_Input_Value		0x0F
#define PowerDown_Value			(1<<0)
#define Motion_Value			(1<<1)
#define ResetOut_Value			(1<<2)

// Device_Motion_Status		0x23
#define X_OverFlow			(1<<0)
#define Y_OverFlow			(1<<1)
#define Motion_Detect			(1<<2)
#define No_Motion			(1<<3)
#define Motion_Complete			(1<<4)

// Device_Value_Setting		0x27
#define Invert_X			(1<<0)
#define Invert_Y			(1<<1)
#define Swap_XY				(1<<3)
#define Test_Pattern_Enabled		(1<<5)
#define Test_Pattern_Speed		(1<<6)

// Device_Exposure		0x41
#define Auto_Expo_En			(1<<4)

// Device_Frame_Setting		0x62
#define Frame_Dump_Mode_Enable		(1<<0)
#define Frame_Dump_Start		(1<<1)
#define Frame_Ready_for_Download	(1<<2)
#define Frame_Upload_Complete		(1<<3)
#define Frame_PCI_Test_Enable		(1<<4)

// Device_Timer_Interrupt	0x82
#define Timer_ITR_Enable		(1<<1)

#define Wait_for_Delay			(0xff)

#define INTERRUPT_MODE			(0x1)
#define POLLING_MODE			(0x0)

#define STATUS_JESTURE_START		(0x1)
#define STATUS_JESTURE_STOP		(0x2)
#define STATUS_BTN_ON			(0x3)
#define STATUS_BTN_OFF			(0x4)
#define STATUS_MOVE			(0x5)

static unsigned char vd5376_init_reg[][2] = {
	{0x05, 0x2d},
	{Wait_for_Delay,100},
	{0x27, 0x1a},
	{0x2a, 0x08},
	{0x2b, 0x08},
};
#define VD5376_INIT_REGS (sizeof(vd5376_init_reg) / sizeof(vd5376_init_reg[0]))

struct vd5376
{ 
	struct i2c_client		*client;
	struct input_dev		*input;
	struct delayed_work		dwork;
	struct delayed_work		jesture_dwork;
	spinlock_t			lock;
	struct vd5376_platform_data	*pdata;
	int				mode;
	int				jesture;
};

#endif
