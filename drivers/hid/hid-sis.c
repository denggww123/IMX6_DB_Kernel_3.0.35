/*
 *  HID driver for sis 9237/9257 test touchscreens
 *
 *  Copyright (c) 2008 Rafi Rubin
 *  Copyright (c) 2009 Stephane Chatty
 *
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/hid-debug.h>
#include "hid-ids.h"
//for i2c-bridge
#include <linux/usb.h>
#include "usbhid/usbhid.h"
#include <asm/uaccess.h>
#include <linux/types.h>

#ifdef CONFIG_HID_SIS_UPDATE_FW
//for ioctl
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#define INTERNAL_DEVICE_NAME "sis_zeus_hid_touch_device"
#define BRIDGE_DEVICE_NAME "sis_zeus_hid_bridge_touch_device"
#define SIS817_DEVICE_NAME "sis_aegis_hid_touch_device"
#define SISF817_DEVICE_NAME "sis_aegis_hid_bridge_touch_device"
static int sis_char_devs_count = 1;        /* device count */
static int sis_char_major = 0;
static struct cdev sis_char_cdev;
static struct class *sis_char_class = NULL;
//20110111 Tammy system call for tool
static struct hid_device *hid_dev_backup = NULL;  //backup address
static struct urb *backup_urb = NULL;
#endif	//CONFIG_HID_SIS_UPDATE_FW

#define MAX_X			4095
#define MAX_Y			4095
//#define MAX_PRESSURE		2047
#define MAX_SCANTIME		65535
#define MAX_CONTACTID		31

#define MAX_POINT		10
#define HID_DG_SCANTIME		0x000d0056	//new usage not defined in hid.h
#define REPORTID_10		0x10
#define REPORTID_TYPE1		0x30

#define CTRL 0
#define DIR_IN 0x1

struct Point {
	u16 x, y, id, pressure, width, height;
};

struct sis_data {
	int id, total, ReportID, scantime;
	struct Point pt[MAX_POINT];
};


static int pkg_num=0;
static int idx=-1;

/*
 * this driver is aimed at two firmware versions in circulation:
 *  - dual pen/fingedrivers/hid/hid-sis.c:83:r single touch
 *  - finger multitouch, pen not working
 */

static int sis_input_mapping(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
	// No special mappings needed for the pen and single touch 
	if (field->physical == HID_GD_POINTER)
		return -1;
	
	else if (field->physical && (field->physical != HID_GD_POINTER))
		return 0;

#ifdef CONFIG_DEBUG_HID_SIS_INIT
	printk (KERN_INFO "sis_input_mapping : usage->hid = %x\n", usage->hid);
#endif //CONFIG_DEBUG_HID_SIS_INIT

	switch (usage->hid & HID_USAGE_PAGE) {
	case HID_UP_GENDESK:
		switch (usage->hid) {
		case HID_GD_X:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_X);
			input_set_abs_params(hi->input, ABS_X, 
				field->logical_minimum, field->logical_maximum, 0, 0);
			return 1;

		case HID_GD_Y:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_Y);
			input_set_abs_params(hi->input, ABS_Y,
				field->logical_minimum, field->logical_maximum, 0, 0);
			return 1;
		}
		return 0;

	case HID_UP_DIGITIZER:
		switch (usage->hid) {
		/* we do not want to map these for now */
		case HID_DG_CONFIDENCE:
		case HID_DG_INPUTMODE:
		case HID_DG_DEVICEINDEX:
		case HID_DG_CONTACTCOUNT:
		case HID_DG_CONTACTMAX:
		case HID_DG_INRANGE:
/**dsdsds*/
		//new usage for SiS817 Device(for later use)
		case HID_DG_WIDTH:
			//hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MT_TOUCH_MINOR);
			//input_set_abs_params(hi->input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
			//return 1;
		case HID_DG_HEIGHT:	
			//hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MT_TOUCH_MAJOR);
			//input_set_abs_params(hi->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
			//return 1;	
		case HID_DG_TIPPRESSURE:
			//hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MT_PRESSURE);
			//input_set_abs_params(hi->input, ABS_MT_PRESSURE, 0, 2047, 0, 0);
			//return 1;
		case HID_DG_SCANTIME:
			return -1;

		case HID_DG_TIPSWITCH:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_PRESSURE);
			input_set_abs_params(hi->input, ABS_PRESSURE, 0, 1, 0, 0);
			return 1;

		case HID_DG_CONTACTID:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MT_TRACKING_ID);
			input_set_abs_params(hi->input, ABS_MT_TRACKING_ID, 0, 127, 0, 0);
			return 1;
		}
		return 0;

	/*case HID_UP_BUTTON:
		return 0;*/

	case 0xff000000:
		/* ignore HID features */
		return -1;

	}
	/* ignore buttons */
	return 0;
}

//sis_input_mapped : unmapped usage that no use in sis_event
static int sis_input_mapped(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
#ifdef CONFIG_DEBUG_HID_SIS_INIT
	printk (KERN_INFO "sis_input_mapping : usage->hid = %x\n", usage->hid);
#endif //CONFIG_DEBUG_HID_SIS_INIT

	if (usage->type == EV_KEY || usage->type == EV_ABS)
		clear_bit(usage->code, *bit);

	return 0;
}

static void sis_event_emission(struct sis_data *nd, struct input_dev *input)
{
	//int i;
	bool all_touch_up = true;
	//for(i=0; i< nd->total; i++)
	//{

#ifdef CONFIG_DEBUG_HID_SIS_SENDPOINT
		printk(KERN_INFO "MT_event: finger(s)=%d, id=%d, x=%d, y=%d\n", nd->total, nd->pt[i].id, nd->pt[i].x, nd->pt[i].y);
		printk(KERN_INFO "MT_event: pressure=%d, width=%d, height=%d, scantime=%d\n", nd->pt[i].pressure, nd->pt[i].width, nd->pt[i].height, nd->scantime);
#endif //CONFIG_DEBUG_HID_SIS_SENDPOINT

		//checking correction of data
	//	if(nd->pt[0].x > MAX_X || nd->pt[0].y > MAX_Y || nd->pt[0].id > MAX_CONTACTID /*|| nd->scantime > MAX_SCANTIME*/)
		//{
		//	printk(KERN_INFO "point data error : abort sending point this time");
		//	break;
		//}

		//if(nd->pt[i].pressure)
		//{
			//input_report_abs(input, ABS_MT_TOUCH_MAJOR, max(nd->pt[i].height,nd->pt[i].width));
			//input_report_abs(input, ABS_MT_TOUCH_MINOR, min(nd->pt[i].height,nd->pt[i].width));

			input_report_abs(input, ABS_PRESSURE, nd->pt[0].pressure);
			input_report_abs(input, ABS_X, nd->pt[0].x);
			input_report_abs(input, ABS_Y, nd->pt[0].y);
			//input_report_abs(input, ABS_MT_TRACKING_ID, nd->pt[0].id);
			//input_mt_sync(input);
			//printk("ABS_X=%d  ABS_Y=%d\n", nd->pt[0].x,nd->pt[0].y);
			all_touch_up = false;
		//}

		//if(i == (nd->total - 1) && all_touch_up == true)
		//	input_mt_sync(input);
//	}
	input_sync(input);
	//input_sync will be send by hid default flow
}

static void sis_event_clear(struct sis_data *nd, int max)
{
	int i;
	for(i=0; i<max; i++)
	{
		nd->pt[i].id = 0;
		nd->pt[i].x = 0;
		nd->pt[i].y = 0;
		nd->pt[i].pressure = 0;
		nd->pt[i].width = 0;
		nd->pt[i].height = 0;
	}
	nd->scantime = 0;
	idx = -1;
	pkg_num = 0;
}

static int sis_raw_event (struct hid_device *hid, struct hid_report *report,
		                        u8 *raw_data, int size)
{
	struct sis_data *nd = hid_get_drvdata(hid);
	nd->ReportID = raw_data[0];

#ifdef CONFIG_DEBUG_HID_SIS_SENDPOINT
	printk(KERN_INFO "raw_event : ReportID = %d\n", nd->ReportID);
#endif //CONFIG_DEBUG_HID_SIS_SENDPOINT

	hid_set_drvdata(hid, nd);
	return 0;
}

static void sis_event_lastdata(struct hid_device *hid, struct sis_data *nd, struct input_dev *input)
{
	int pkg_n=0;
	
//817 method : original format
	if ( (hid->product == USB_PRODUCT_ID_SIS817_TOUCH || hid->product == USB_PRODUCT_ID_SISF817_TOUCH) && nd->ReportID == REPORTID_10)
	{
#ifdef CONFIG_DEBUG_HID_SIS_SENDPOINT
		printk (KERN_INFO "sis_event_lastdata : 817 original format\n");
#endif //CONFIG_DEBUG_HID_SIS_SENDPOINT

		sis_event_emission(nd, input);
		sis_event_clear(nd, MAX_POINT);
	}
	//817 method : Extend Class Format
	else if ( (hid->product == USB_PRODUCT_ID_SIS817_TOUCH || hid->product == USB_PRODUCT_ID_SISF817_TOUCH) && nd->ReportID != REPORTID_10)
	{
#ifdef CONFIG_DEBUG_HID_SIS_SENDPOINT
		printk (KERN_INFO "sis_event_lastdata : 817 extend format\n");
#endif //CONFIG_DEBUG_HID_SIS_SENDPOINT

		if(nd->total >= 6)
		{				
			idx = 4;
			pkg_num = nd->total;	
		}
		else if(nd->total >= 1)
		{
			sis_event_emission(nd, input);
			sis_event_clear(nd, MAX_POINT);
		}
		else
		{
			if(pkg_num >0)
			{
				nd->total = pkg_num;
				sis_event_emission(nd, input);
				pkg_n = 0;
				sis_event_clear(nd, MAX_POINT);
			}
			else
			{
				sis_event_clear(nd, MAX_POINT);
			}
		}
	}
	else	//816 method
	{
#ifdef CONFIG_DEBUG_HID_SIS_SENDPOINT
		printk (KERN_INFO "sis_event_lastdata : 816 format\n");
#endif //CONFIG_DEBUG_HID_SIS_SENDPOINT

		if(nd->total >= 3)
		{				
			idx = 1;
			pkg_num = nd->total;	
		}
		else if(nd->total >= 1)
		{
			sis_event_emission(nd, input);
			sis_event_clear(nd, MAX_POINT);
		}
		else
		{
			if(pkg_num >0)
			{
				if((pkg_num%2)>0)	
					pkg_n = pkg_num+1;
				else
					pkg_n = pkg_num;
			
				if(pkg_n == (idx + 1) )
				{
					nd->total = pkg_num;
					sis_event_emission(nd, input);
					pkg_n = 0;
					sis_event_clear(nd, MAX_POINT);
				}
			}
			else
			{
				sis_event_clear(nd, MAX_POINT);
			}
		}
	}
}
/*
 * this function is called upon all reports
 * so that we can filter contact point information,
 * decide whether we are in multi or single touch mode
 * and call input_mt_sync after each point if necessary
 */
static int sis_event (struct hid_device *hid, struct hid_field *field,
		                        struct hid_usage *usage, __s32 value)
{
	struct sis_data *nd = hid_get_drvdata(hid);
	//printk (KERN_INFO "sis_event");

        if (hid->claimed & HID_CLAIMED_INPUT) {
		struct input_dev *input = field->hidinput->input;
#ifdef CONFIG_DEBUG_HID_SIS_SENDPOINT
		printk (KERN_INFO "sis_event : usage->hid = %x, value = %d\n", usage->hid, value);
#endif //CONFIG_DEBUG_HID_SIS_SENDPOINT
		switch (usage->hid) {
		case HID_DG_INRANGE:			
			break;

		case HID_DG_TIPSWITCH:
			idx++;
			nd->pt[idx].pressure = !!value;
			break;

		case HID_DG_CONTACTID:
			nd->pt[idx].id = value;
			break;

		case HID_GD_X:
			nd->pt[idx].x = value;
			break;

		case HID_GD_Y:
			nd->pt[idx].y = value;
			break;

		//new usage for SiS817 Extend Class Device
		case HID_DG_SCANTIME:
			nd->scantime = value;
			if ( (nd->ReportID & 0xf0) > REPORTID_TYPE1 )
				sis_event_lastdata(hid, nd, input);
			break;

		case HID_DG_WIDTH:
			nd->pt[idx].width = value;
			break;

		case HID_DG_HEIGHT:
			nd->pt[idx].height = value;
			break;
		
		case HID_DG_TIPPRESSURE:
			nd->pt[idx].pressure = value;
			break;
		//end of new usage for SiS817 Extend Class Device

		case HID_DG_CONTACTCOUNT:
			nd->total = value;
			if ( (nd->ReportID & 0xf0) <= REPORTID_TYPE1 )
				sis_event_lastdata(hid, nd, input);
			break;			
		default:
			//fallback to the generic hidinput handling
			return 0;
		}
	}

	/* we have handled the hidinput part, now remains hiddev */
	if (hid->claimed & HID_CLAIMED_HIDDEV && hid->hiddev_hid_event)
		hid->hiddev_hid_event(hid, field, usage, value);

	return 1;
}

#ifdef CONFIG_HID_SIS_UPDATE_FW
int sis_cdev_open(struct inode *inode, struct file *filp)	//20120306 Yuger ioctl for tool
{
	//20110511, Yuger, kill current urb by method of usbhid_stop
	struct usbhid_device *usbhid;

#ifdef CONFIG_DEBUG_HID_SIS_UPDATE_FW
	printk( KERN_INFO "sis_cdev_open\n" );
#endif //CONFIG_DEBUG_HID_SIS_UPDATE_FW

	if ( !hid_dev_backup )
	{
		printk( KERN_INFO "sis_cdev_open : hid_dev_backup is not initialized yet" );
		return -1;
	}

	usbhid = hid_dev_backup->driver_data;

	//20110602, Yuger, fix bug: not contact usb cause kernel panic
	if( !usbhid )
	{
		printk( KERN_INFO "sis_cdev_open : usbhid is not initialized yet" );
		return -1;
	}
	else if ( !usbhid->urbin )
	{
		printk( KERN_INFO "sis_cdev_open : usbhid->urbin is not initialized yet" );
		return -1;
	}
	else if (hid_dev_backup->vendor == USB_VENDOR_ID_SIS2_TOUCH)
	{
		usb_fill_int_urb(backup_urb, usbhid->urbin->dev, usbhid->urbin->pipe,
			usbhid->urbin->transfer_buffer, usbhid->urbin->transfer_buffer_length,
			usbhid->urbin->complete, usbhid->urbin->context, usbhid->urbin->interval);

                clear_bit( HID_STARTED, &usbhid->iofl );
                set_bit( HID_DISCONNECTED, &usbhid->iofl );

                usb_kill_urb( usbhid->urbin );
                usb_free_urb( usbhid->urbin );
                usbhid->urbin = NULL;
		return 0;
	}
        else	
	{
		printk (KERN_INFO "This is not a SiS device");
		return -801;
	}
}

int sis_cdev_release(struct inode *inode, struct file *filp)
{
	//20110505, Yuger, rebuild the urb which is at the same urb address, then re-submit it
	int ret;
	struct usbhid_device *usbhid;
	unsigned long flags;

#ifdef CONFIG_DEBUG_HID_SIS_UPDATE_FW
	printk( KERN_INFO "sis_cdev_release" );
#endif //CONFIG_DEBUG_HID_SIS_UPDATE_FW
	
	if ( !hid_dev_backup )
	{
		printk( KERN_INFO "sis_cdev_release : hid_dev_backup is not initialized yet" );
		return -1;
	}

	usbhid = hid_dev_backup->driver_data;

	printk( KERN_INFO "sys_sis_HID_start" );

	if( !usbhid )
	{
		printk( KERN_INFO "sis_cdev_release : usbhid is not initialized yet" );
		return -1;
	}

	if( !backup_urb )
	{
		printk( KERN_INFO "sis_cdev_release : urb_backup is not initialized yet" );
		return -1;
	}

	clear_bit( HID_DISCONNECTED, &usbhid->iofl );
	usbhid->urbin = usb_alloc_urb( 0, GFP_KERNEL );

	if( !backup_urb->interval )
	{
		printk( KERN_INFO "sis_cdev_release : urb_backup->interval does not exist" );
		return -1;
	}

	usb_fill_int_urb(usbhid->urbin, backup_urb->dev, backup_urb->pipe, 
		backup_urb->transfer_buffer, backup_urb->transfer_buffer_length, 
		backup_urb->complete, backup_urb->context, backup_urb->interval);
	usbhid->urbin->transfer_dma = usbhid->inbuf_dma;
	usbhid->urbin->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	set_bit( HID_STARTED, &usbhid->iofl );

	//method at hid_start_in
	spin_lock_irqsave( &usbhid->lock, flags );		
	ret = usb_submit_urb( usbhid->urbin, GFP_ATOMIC );
	spin_unlock_irqrestore( &usbhid->lock, flags );
	//yy

#ifdef CONFIG_DEBUG_HID_SIS_UPDATE_FW
	printk( KERN_INFO "sis_cdev_release : ret = %d", ret );
#endif //CONFIG_DEBUG_HID_SIS_UPDATE_FW

	return ret;
}

//SiS 817 only
ssize_t sis_cdev_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int actual_length = 0, timeout = 0;
	u8 *rep_data = NULL;
	u16 size = 0;
	long rep_ret;
	//int i;
	struct usb_interface *intf = to_usb_interface(hid_dev_backup->dev.parent);
	struct usb_device *dev = interface_to_usbdev(intf);

#ifdef CONFIG_DEBUG_HID_SIS_UPDATE_FW
	printk( KERN_INFO "sis_cdev_read\n");
#endif //CONFIG_DEBUG_HID_SIS_UPDATE_FW

	size = (((u16)(buf[64] & 0xff)) << 24) + (((u16)(buf[65] & 0xff)) << 16) + 
		(((u16)(buf[66] & 0xff)) << 8) + (u16)(buf[67] & 0xff);
	timeout = (((int)(buf[68] & 0xff)) << 24) + (((int)(buf[69] & 0xff)) << 16) + 
		(((int)(buf[70] & 0xff)) << 8) + (int)(buf[71] & 0xff);

	rep_data = kzalloc(size, GFP_KERNEL);
	if (!rep_data)
		return -12;

	if ( copy_from_user( rep_data, (void*)buf, size) ) 
	{
		printk( KERN_INFO "copy_to_user fail\n" );
		//free allocated data
		kfree(rep_data);
		rep_data = NULL;
		return -19;
	}

	rep_ret = usb_interrupt_msg(dev, backup_urb->pipe,
		rep_data, size, &actual_length, timeout);

#ifdef CONFIG_DEBUG_HID_SIS_UPDATE_FW
	printk(KERN_INFO "sis_cdev_read : rep_data = ");
	for (i=0; i<8; i++)  
	{
		printk ("%02X ", rep_data[i]);
	}
	printk ("\n");
#endif //CONFIG_DEBUG_HID_SIS_UPDATE_FW

	if( rep_ret == 0 )
	{
		rep_ret = actual_length;
		if ( copy_to_user( (void*)buf, rep_data, rep_ret ) ) 
		{
			printk( KERN_INFO "copy_to_user fail\n" );
			//free allocated data
			kfree( rep_data );
			rep_data = NULL;
			return -19;
		}
	}

	//free allocated data
	kfree( rep_data );
	rep_data = NULL;

#ifdef CONFIG_DEBUG_HID_SIS_UPDATE_FW
	printk( KERN_INFO "sis_cdev_read : rep_ret = %ld\n", rep_ret );
#endif //CONFIG_DEBUG_HID_SIS_UPDATE_FW

	return rep_ret;
}

ssize_t sis_cdev_write( struct file *file, const char __user *buf, size_t count, loff_t *f_pos )
{
	int i, actual_length = 0;
	u8 *tmp_data = NULL;	//include report id
	u8 *rep_data = NULL;
	long rep_ret;
	struct usb_interface *intf = to_usb_interface( hid_dev_backup->dev.parent );
	struct usb_device *dev = interface_to_usbdev( intf );
	struct usbhid_device *usbhid = hid_dev_backup->driver_data;

	if ( hid_dev_backup->product == USB_PRODUCT_ID_SIS817_TOUCH || hid_dev_backup->product == USB_PRODUCT_ID_SISF817_TOUCH )	//817 method
	{
		u16 size = (((u16)(buf[64] & 0xff)) << 24) + (((u16)(buf[65] & 0xff)) << 16) + 
			(((u16)(buf[66] & 0xff)) << 8) + (u16)(buf[67] & 0xff);
		int timeout = (((int)(buf[68] & 0xff)) << 24) + (((int)(buf[69] & 0xff)) << 16) + 
			(((int)(buf[70] & 0xff)) << 8) + (int)(buf[71] & 0xff);

#ifdef CONFIG_DEBUG_HID_SIS_UPDATE_FW
		printk( KERN_INFO "sis_cdev_write : 817 method\n");
#endif //CONFIG_DEBUG_HID_SIS_UPDATE_FW

		rep_data = kzalloc(size, GFP_KERNEL);
		if (!rep_data)
			return -12;

		if ( copy_from_user( rep_data, (void*)buf, size) ) 
		{
			printk( KERN_INFO "copy_to_user fail\n" );
			//free allocated data
			kfree( rep_data );
			rep_data = NULL;
			return -19;
		}

		rep_ret = usb_interrupt_msg( dev, usbhid->urbout->pipe,
			rep_data, size, &actual_length, timeout );

#ifdef CONFIG_DEBUG_HID_SIS_UPDATE_FW
		printk(KERN_INFO "sis_cdev_write : rep_data = ");
		for (i=0; i<size-1; i++)  
		{
			printk ("%02X ", rep_data[i]);
		}
		if (i == size-1)
			printk ("%02X\n", rep_data[i]);
#endif //CONFIG_DEBUG_HID_SIS_UPDATE_FW

		if( rep_ret == 0 )
		{
			rep_ret = actual_length;
			if ( copy_to_user( (void*)buf, rep_data, rep_ret ) ) 
			{
				printk( KERN_INFO "copy_to_user fail\n" );
				//free allocated data
				kfree( rep_data );
				rep_data = NULL;
				return -19;
			}
		}

#ifdef CONFIG_DEBUG_HID_SIS_UPDATE_FW
		printk( KERN_INFO "sis_cdev_write : rep_ret = %ld\n", rep_ret );
#endif //CONFIG_DEBUG_HID_SIS_UPDATE_FW

	}
	else	//816 method
	{

		u8 request = buf[0];
		u8 dir = buf[1];
		u16 value = (((u16)(buf[2] & 0xff)) << 24) + (((u16)(buf[3] & 0xff)) << 16) + (((u16)(buf[4] & 0xff)) << 8) + (u16)(buf[5] & 0xff);
		u16 index = (((u16)(buf[6] & 0xff)) << 24) + (((u16)(buf[7] & 0xff)) << 16) + (((u16)(buf[8] & 0xff)) << 8) + (u16)(buf[9] & 0xff);


		u16 size = (((u16)(buf[29] & 0xff)) << 24) + (((u16)(buf[30] & 0xff)) << 16) + (((u16)(buf[31] & 0xff)) << 8) + (u16)(buf[32] & 0xff);
		int timeout = (((int)(buf[33] & 0xff)) << 24) + (((int)(buf[34] & 0xff)) << 16) + (((int)(buf[35] & 0xff)) << 8) + (int)(buf[36] & 0xff);

#ifdef CONFIG_DEBUG_HID_SIS_UPDATE_FW
		printk( KERN_INFO "sis_cdev_write : 816 method\n");
		printk (KERN_INFO "dir = %d, value = %d, index = %d, timeout = %d\n", dir, value, index, timeout);
#endif //CONFIG_DEBUG_HID_SIS_UPDATE_FW

		rep_data = kzalloc( size , GFP_KERNEL );
		if ( rep_data == 0 ) 
		{
			return -12;
		}

		for( i = 0; i < size; i++)
		{
			rep_data[i] = buf[10+i];
		}

		tmp_data = kzalloc( size + 1, GFP_KERNEL );	//include report id, so size +1

		for( i = 0; i < size; i++ )
		{
			tmp_data[i+1] = rep_data[i];
		}

		buf += 10;

		if( dir & DIR_IN )
		{
			if ( hid_dev_backup->product == USB_PRODUCT_ID_SIS2_TOUCH || hid_dev_backup->product == USB_PRODUCT_ID_NEW_SIS2_TOUCH )
			{
				//20110510, Yuger, for correcting intr data send into interrupt msg(receive, in, endp=2)
				tmp_data[0] = 0x0A;//in

				rep_ret = usb_interrupt_msg( dev, backup_urb->pipe, tmp_data, size+1, &actual_length, timeout );
				//yy

#ifdef CONFIG_DEBUG_HID_SIS_UPDATE_FW
				printk(KERN_INFO "(INT_IN)rep_ret = %ld, actual_length = %d", rep_ret, actual_length);
#endif //CONFIG_DEBUG_HID_SIS_UPDATE_FW

				if( rep_ret == 0 )
				{
					rep_ret = actual_length;
				}

				//20110510, Yuger, for recovering rep_data
				for( i = 0; i < size; i++ )
				{
					rep_data[i] = tmp_data[i+1];
				}
				//yy

#ifdef CONFIG_DEBUG_HID_SIS_UPDATE_FW
				printk(KERN_INFO "(INT_IN)size = %u, dir = %u, rep_ret = %ld, rep_data = %X %X %X", size, dir, rep_ret, rep_data[0], rep_data[1], rep_data[2]);
#endif //CONFIG_DEBUG_HID_SIS_UPDATE_FW

				if ( copy_to_user( (void*)buf, rep_data, rep_ret ) ) 
				{
					printk( KERN_INFO "copy_to_user fail\n" );
					//free allocated data
					kfree( rep_data );
					kfree( tmp_data );
					rep_data = NULL;
					tmp_data = NULL;
					return -19;
				}
			}
			else
			{
				//control message
				rep_ret = usb_control_msg( dev, usb_rcvctrlpipe( dev, CTRL ), 
					request, (USB_DIR_IN|USB_TYPE_VENDOR|USB_RECIP_DEVICE), 
					value, index, rep_data, size, timeout );

#ifdef CONFIG_DEBUG_HID_SIS_UPDATE_FW
				printk ("(CTRL) size = %d, dir = %d, rep_ret = %ld, rep_data = ", size, dir, rep_ret);
				for (i=0; i<size-1; i++)  
				{
					printk ("%02X ", rep_data[i]);
				}
				if (i == size-1)
				printk ("%02X\n", rep_data[i]);
#endif //CONFIG_DEBUG_HID_SIS_UPDATE_FW

				if ( copy_to_user( (void*)buf, rep_data, rep_ret ) ) 
				{
					printk( KERN_INFO "copy_to_user fail\n" );
					//free allocated data
					kfree( rep_data );
					rep_data = NULL;
					return -19;
				}
			}
		}
		else
		{
			if ( hid_dev_backup->product == USB_PRODUCT_ID_SIS2_TOUCH || hid_dev_backup->product == USB_PRODUCT_ID_NEW_SIS2_TOUCH )
			{
				//20110510, Yuger, for correcting intr data send into interrupt msg(send, out, endp=1)
				tmp_data[0] = 0x09;//out

				rep_ret = usb_interrupt_msg( dev, usbhid->urbout->pipe, tmp_data, size + 1, &actual_length, timeout );

				//just return success or not(no need to return actual_length if succeed)

				//20110510, Yuger, for recovering rep_data
				for( i = 0; i < size; i++ )
				{
					rep_data[i] = tmp_data[i+1];
				}
				//yy

#ifdef CONFIG_DEBUG_HID_SIS_UPDATE_FW
				printk(KERN_INFO "(INT_OUT)size = %u, actual_length = %d, rep_ret = %ld, rep_data = %x %x %x", size, actual_length, rep_ret, rep_data[0], rep_data[1], rep_data[2]);
#endif //CONFIG_DEBUG_HID_SIS_UPDATE_FW

				if ( copy_to_user( (void*)buf, rep_data, actual_length-1 ) ) 
				{
					printk( KERN_INFO "copy_to_user fail\n" );
					//free allocated data
					kfree( rep_data );
					kfree( tmp_data );
					rep_data = NULL;
					tmp_data = NULL;
					return -19;
				}
			}
			else
			{
				//control message
				rep_ret = usb_control_msg( dev, usb_sndctrlpipe( dev, CTRL ), 
					request, (USB_DIR_OUT|USB_TYPE_VENDOR|USB_RECIP_DEVICE), 
					value, index, rep_data, 16, timeout );

#ifdef CONFIG_DEBUG_HID_SIS_UPDATE_FW
				printk ("(CTRL) size = %d, dir = %d, rep_ret = %ld, rep_data = ", size, dir, rep_ret);
				for (i=0; i<size-1; i++)  
				{
					printk ("%02X ", rep_data[i]);
				}
				if (i == size-1)
					printk ("%02X\n", rep_data[i]);
#endif //CONFIG_DEBUG_HID_SIS_UPDATE_FW

				if ( copy_to_user( (void*)buf, rep_data, rep_ret ) ) 
				{
					printk( KERN_INFO "copy_to_user fail\n" );
					//free allocated data
					kfree( rep_data );
					rep_data = NULL;
					return -19;
				}
			}
		}
		//free allocated data
		kfree( tmp_data );
		tmp_data = NULL;
	}
	//free allocated data
	kfree( rep_data );
	rep_data = NULL;

#ifdef CONFIG_DEBUG_HID_SIS_UPDATE_FW
	printk( KERN_INFO "End of sis_cdev_write\n" );
#endif //CONFIG_DEBUG_HID_SIS_UPDATE_FW

	return rep_ret;
}

//~TT

//for ioctl
static const struct file_operations sis_cdev_fops = {
	.owner	= THIS_MODULE,
	.read	= sis_cdev_read,
	.write	= sis_cdev_write,
	.open	= sis_cdev_open,
	.release= sis_cdev_release,
};

//for ioctl
static int sis_setup_chardev(struct hid_device *hdev, struct sis_data *nd)
{
	
	dev_t dev = MKDEV(sis_char_major, 0);
	int alloc_ret = 0;
	int cdev_err = 0;
	int input_err = 0;
	struct device *class_dev = NULL;
	void *ptr_err;
	
	printk("sis_setup_chardev.\n");
	
	if (nd == NULL) 
	{
		input_err = -ENOMEM;
		goto error;
	} 

	// dynamic allocate driver handle
	if (hdev->product == USB_PRODUCT_ID_SIS9200_TOUCH)
		alloc_ret = alloc_chrdev_region(&dev, 0, sis_char_devs_count, BRIDGE_DEVICE_NAME);
	else if (hdev->product == USB_PRODUCT_ID_SIS817_TOUCH)
		alloc_ret = alloc_chrdev_region(&dev, 0, sis_char_devs_count, SIS817_DEVICE_NAME);
	else if (hdev->product == USB_PRODUCT_ID_SISF817_TOUCH)
		alloc_ret = alloc_chrdev_region(&dev, 0, sis_char_devs_count, SISF817_DEVICE_NAME);
	else
		alloc_ret = alloc_chrdev_region(&dev, 0, sis_char_devs_count, INTERNAL_DEVICE_NAME);

	if (alloc_ret)
		goto error;
		
	sis_char_major  = MAJOR(dev);
	cdev_init(&sis_char_cdev, &sis_cdev_fops);
	sis_char_cdev.owner = THIS_MODULE;
	cdev_err = cdev_add(&sis_char_cdev, MKDEV(sis_char_major, 0), sis_char_devs_count);
	if (cdev_err) 
		goto error;

	if (hdev->product == USB_PRODUCT_ID_SIS9200_TOUCH)
		printk(KERN_INFO "%s driver(major %d) installed.\n", BRIDGE_DEVICE_NAME, sis_char_major);
	else if (hdev->product == USB_PRODUCT_ID_SIS817_TOUCH)
		printk(KERN_INFO "%s driver(major %d) installed.\n", SIS817_DEVICE_NAME, sis_char_major);
	else if (hdev->product == USB_PRODUCT_ID_SISF817_TOUCH)
		printk(KERN_INFO "%s driver(major %d) installed.\n", SISF817_DEVICE_NAME, sis_char_major);
	else
		printk(KERN_INFO "%s driver(major %d) installed.\n", INTERNAL_DEVICE_NAME, sis_char_major);

	// register class
	if (hdev->product == USB_PRODUCT_ID_SIS9200_TOUCH)
		sis_char_class = class_create(THIS_MODULE, BRIDGE_DEVICE_NAME);
	else if (hdev->product == USB_PRODUCT_ID_SIS817_TOUCH)
		sis_char_class = class_create(THIS_MODULE, SIS817_DEVICE_NAME);
	else if (hdev->product == USB_PRODUCT_ID_SISF817_TOUCH)
		sis_char_class = class_create(THIS_MODULE, SISF817_DEVICE_NAME);
	else
		sis_char_class = class_create(THIS_MODULE, INTERNAL_DEVICE_NAME);

	if(IS_ERR(ptr_err = sis_char_class)) 
	{
		goto err2;
	}

	if (hdev->product == USB_PRODUCT_ID_SIS9200_TOUCH)
		class_dev = device_create(sis_char_class, NULL, MKDEV(sis_char_major, 0), NULL, BRIDGE_DEVICE_NAME);
	else if (hdev->product == USB_PRODUCT_ID_SIS817_TOUCH)
		class_dev = device_create(sis_char_class, NULL, MKDEV(sis_char_major, 0), NULL, SIS817_DEVICE_NAME);
	else if (hdev->product == USB_PRODUCT_ID_SISF817_TOUCH)
		class_dev = device_create(sis_char_class, NULL, MKDEV(sis_char_major, 0), NULL, SISF817_DEVICE_NAME);
	else
		class_dev = device_create(sis_char_class, NULL, MKDEV(sis_char_major, 0), NULL, INTERNAL_DEVICE_NAME);

	if(IS_ERR(ptr_err = class_dev)) 
	{
		goto err;
	}
	
	return 0;
error:
	if (cdev_err == 0)
		cdev_del(&sis_char_cdev);
	if (alloc_ret == 0)
		unregister_chrdev_region(MKDEV(sis_char_major, 0), sis_char_devs_count);
	if(input_err != 0)
	{
		printk("sis_ts_bak error!\n");
	}
err:
	device_destroy(sis_char_class, MKDEV(sis_char_major, 0));
err2:
	class_destroy(sis_char_class);
	return -1;
}
#endif	//CONFIG_HID_SIS_UPDATE_FW

static int sis_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;
    	struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
    	struct usb_device *dev = interface_to_usbdev(intf);
	struct sis_data *nd;
   	u8 *rep_data = NULL;

#ifdef CONFIG_HID_SIS_UPDATE_FW	
	hid_dev_backup = hdev;
#endif	//CONFIG_HID_SIS_UPDATE_FW

	printk(KERN_INFO "sis_probe\n");

#ifdef CONFIG_HID_SIS_UPDATE_FW	
	backup_urb = usb_alloc_urb(0, GFP_KERNEL); //0721test
	if (!backup_urb) {
		dev_err(&hdev->dev, "cannot allocate backup_urb\n");
		return -ENOMEM;
	}
#endif	//CONFIG_HID_SIS_UPDATE_FW

//	command Set_Feature for changing device from mouse to touch device
	rep_data = kmalloc(3,GFP_KERNEL);	//return value will be 0xabcd
	if (hdev->product == USB_PRODUCT_ID_SIS9200_TOUCH)
	{
		if(!rep_data)
			return -ENOMEM;
		rep_data[0] = 0x07;
		rep_data[1] = 0x02;
		rep_data[2] = 0xA9;
	
		ret = usb_control_msg(dev, usb_sndctrlpipe(dev, 0), 
			USB_REQ_SET_CONFIGURATION, (USB_DIR_OUT | USB_TYPE_CLASS | 
			USB_RECIP_INTERFACE), 0x0307, 0, rep_data, 3, 1000);
	}

//	allocate memory for sis_data struct
	nd = kzalloc(sizeof(struct sis_data), GFP_KERNEL);
	if (!nd) {
		dev_err(&hdev->dev, "cannot allocate SiS 9200 data\n");
		kfree(rep_data);
		rep_data = NULL;
		return -ENOMEM;
	}
	
	hid_set_drvdata(hdev, nd);

#ifdef CONFIG_HID_SIS_UPDATE_FW
	//for ioctl
	ret = sis_setup_chardev(hdev, nd);
	if(ret)
	{
		printk( KERN_INFO "sis_setup_chardev fail\n");
	}
#endif	//CONFIG_HID_SIS_UPDATE_FW

	//set noget for not init reports (speed improvement)
	hdev->quirks |= HID_QUIRK_NOGET;

	ret = hid_parse(hdev);
	if (ret) {
		dev_err(&hdev->dev, "parse failed\n");
		goto err_free;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		dev_err(&hdev->dev, "hw start failed\n");
		goto err_free;
	}

err_free:
	kfree(rep_data);
	rep_data = NULL;	
	return ret;
}

static void sis_remove(struct hid_device *hdev)
{
#ifdef CONFIG_HID_SIS_UPDATE_FW
	//for ioctl
	dev_t dev = MKDEV(sis_char_major, 0);

	printk(KERN_INFO "sis_remove\n");
	cdev_del(&sis_char_cdev);
	unregister_chrdev_region(dev, sis_char_devs_count);
	device_destroy(sis_char_class, MKDEV(sis_char_major, 0));
	class_destroy(sis_char_class);
#else	//CONFIG_HID_SIS_UPDATE_FW

	printk(KERN_INFO "sis_remove\n");

#endif	//CONFIG_HID_SIS_UPDATE_FW

	usb_kill_urb(backup_urb);
	usb_free_urb(backup_urb);
	backup_urb = NULL;
	hid_hw_stop(hdev);
	kfree(hid_get_drvdata(hdev));
	hid_set_drvdata(hdev, NULL);
}

static const struct hid_device_id sis_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_SIS2_TOUCH, USB_PRODUCT_ID_SIS2_TOUCH) },	//0x0457, 0x0151
	{ HID_USB_DEVICE(USB_VENDOR_ID_SIS2_TOUCH, USB_PRODUCT_ID_SIS_TOUCH) },		//0x0457, 0x0810
	{ HID_USB_DEVICE(USB_VENDOR_ID_SIS2_TOUCH, USB_PRODUCT_ID_NEW_SIS2_TOUCH) },	//0x0457, 0x0816
	{ HID_USB_DEVICE(USB_VENDOR_ID_SIS2_TOUCH, USB_PRODUCT_ID_SIS9200_TOUCH) },	//0x0457, 0x9200
	{ HID_USB_DEVICE(USB_VENDOR_ID_SIS2_TOUCH, USB_PRODUCT_ID_SIS817_TOUCH) },	//0x0457, 0x0817
	{ HID_USB_DEVICE(USB_VENDOR_ID_SIS2_TOUCH, USB_PRODUCT_ID_SISF817_TOUCH) },	//0x0457, 0xF817
	{ }
};
MODULE_DEVICE_TABLE(hid, sis_devices);



static struct hid_driver sis_driver = {
	.name = "sis",
	.id_table = sis_devices,
	.probe = sis_probe,
	.remove = sis_remove,
	.raw_event = sis_raw_event,
	.input_mapped = sis_input_mapped,
	.input_mapping = sis_input_mapping,
	.event = sis_event,
};

static int __init sis_init(void)
{
	printk(KERN_INFO "sis_init\n");
	return hid_register_driver(&sis_driver);
}

static void __exit sis_exit(void)
{
	printk(KERN_INFO "sis_exit\n");
	hid_unregister_driver(&sis_driver);
}

module_init(sis_init);
module_exit(sis_exit);
MODULE_DESCRIPTION("SiS 817 Touchscreen Driver");
MODULE_LICENSE("GPL");
