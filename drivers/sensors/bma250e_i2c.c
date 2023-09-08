/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 */
/* drivers/input/misc/bma250e_i2c.c
 *
 * Accelerometer device driver for I2C
 *
 * Copyright (C) 2011-2014 ALPS ELECTRIC CO., LTD. All Rights Reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifdef ALPS_ACC_DEBUG
#define DEBUG 1
#endif

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "alps-input.h"
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>

#define I2C_RETRIES		5

#define ACCSNS_DRIVER_NAME	"accelerometer"
#define ACCSNS_LOG_TAG		"[BMA250E], "

/* Register Name for accsns */
#define ACCSNS_XOUT		0x02
#define ACCSNS_YOUT		0x04
#define ACCSNS_ZOUT		0x06
#define ACCSNS_TEMP		0x08
#define ACCSNS_REG0F		0x0F
#define ACCSNS_REG10		0x10
#define ACCSNS_REG11		0x11
#define ACCSNS_REG14		0x14

#define ACCSNS_DATA_ACCESS_NUM	6
#define ACCSNS_3AXIS_NUM	3

#ifdef SUPPORT_ACC_MTP_RECOVERY
#define ACCSNS_BACKUP_DATA_NUM	27
#endif /* SUPPORT_ACC_MTP_RECOVERY */


#define ACCSNS_DELAY(us)	usleep_range(us, us)

static struct i2c_client *client_accsns;
static atomic_t flgEna;
static atomic_t flgSuspend;

static struct regulator* sensor_vdd = 0;


/*--------------------------------------------------------------------------
 * i2c read/write function
 *--------------------------------------------------------------------------*/
static int accsns_i2c_read(u8 *rxData, int length)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= client_accsns->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxData,
		},
		{
			.addr	= client_accsns->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxData,
		},
	};

	do {
		err = i2c_transfer(client_accsns->adapter,
			msgs, ARRAY_SIZE(msgs));
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&client_accsns->adapter->dev,
			"read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int accsns_i2c_write(u8 *txData, int length)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= client_accsns->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txData,
		},
	};

	do {
		err = i2c_transfer(client_accsns->adapter,
			msgs, ARRAY_SIZE(msgs));
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&client_accsns->adapter->dev,
			"write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}


#ifdef SUPPORT_ACC_MTP_RECOVERY
#define ACC_I2C_WRITE_EX_BUF_SIZE 1
int acc_mtp_recovery_i2c_write_ex(uint8_t adr, uint8_t reg, const uint8_t *buf, int len)
{
	struct i2c_msg msg;
	char buffer[ACC_I2C_WRITE_EX_BUF_SIZE + 1];
	int err;
	int i;

	if (len > ACC_I2C_WRITE_EX_BUF_SIZE) {
		return -1;
	}

	buffer[0] = reg;
	for (i = 0; i < len; i++) {
		buffer[i+1] = buf[i];
	}

	msg.addr = adr;
	msg.flags = 0;
	msg.len = len + 1;
	msg.buf = buffer;
	err = i2c_transfer(client_accsns->adapter, &msg, 1);
	if (err != 1) {
		return err;
	}

	return 0;
}

int acc_mtp_recovery_i2c_read_ex(uint8_t adr, uint8_t reg, uint8_t *buf, int len)
{
	struct i2c_msg msg[2];
	int err;

	msg[0].addr = adr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[1].addr = adr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;

	err = i2c_transfer(client_accsns->adapter, msg, 2);
	if (err != 2) {
		return err;
	}

	return 0;
}

static int acc_mtp_recovery_device_check(uint8_t *i2c_addr)
{
	uint8_t buf = 0;
	
	uint8_t i;
	uint8_t addr[] = {0x18,0x10,0x12,0x14,0x16,0x1A,0x1C,0x1E};
	

	int err = 0;

	dev_dbg(&client_accsns->adapter->dev, ACCSNS_LOG_TAG
		"%s\n", __func__);
	
	for(i = 0; i < (sizeof(addr)/sizeof(addr[0])); i++)
	
	{
		
		if (acc_mtp_recovery_i2c_read_ex(addr[i], 0x00, &buf, 1))
		
		{
			err = -1;
		}
		else
		{
			if((buf & 0xFC) == 0xF8)
			{
				
				*i2c_addr = addr[i];
				
				err = 0;
				break;
			}
			else
			{
				err = -2;
			}
		}
	}

	return err;
}

static int acc_mtp_recovery_set_correct_addr(uint8_t *i2c_addr)
{
	uint8_t buf = 0;
	uint8_t addr = *i2c_addr;

	int err = 0;

	dev_dbg(&client_accsns->adapter->dev, ACCSNS_LOG_TAG
		"%s\n", __func__);
	
	buf = 0x0;
	if(acc_mtp_recovery_i2c_write_ex(addr, 0x11, &buf, 1))
	{
		return -1;
	}

	msleep(2);

	
	buf = 0xAA;
	err = acc_mtp_recovery_i2c_write_ex(addr, 0x35, &buf, 1);
	err |= acc_mtp_recovery_i2c_write_ex(addr, 0x35, &buf, 1);
	if(err)
	{
		return -2;
	}

	buf = 0;
	
	if(acc_mtp_recovery_i2c_read_ex(addr, 0x05, &buf, 1))
	{
		
		buf = 0x0A;
		acc_mtp_recovery_i2c_write_ex(addr, 0x35, &buf, 1);
		return -3;
	}

	
	buf = ((buf & 0x1F) | 0x80);
	if(acc_mtp_recovery_i2c_write_ex(addr, 0x05, &buf, 1))
	{
		
		buf = 0x0A;
		acc_mtp_recovery_i2c_write_ex(addr, 0x35, &buf, 1);
		return -4;
	}

	return 0;
}

static void acc_mtp_recovery_restore(uint8_t *buf)
{
	uint8_t reg = 0;
	u8  write_buf[2];

	dev_dbg(&client_accsns->adapter->dev, ACCSNS_LOG_TAG
		"%s\n", __func__);
	
	for(reg = 0x0; reg <= 0x1A; reg++)
	{
		write_buf[0] = reg;
		write_buf[1] = buf[reg];
		accsns_i2c_write(write_buf, 2);
	}

	
	write_buf[0] = 0x35;
	write_buf[1] = 0x0A;
	accsns_i2c_write(write_buf, 2);
}

static uint8_t acc_mtp_recovery_crc8(uint8_t *buf, int size)
{
	uint8_t crcreg = 0xFF;
	uint8_t membyte;
	uint8_t bitno;

	dev_dbg(&client_accsns->adapter->dev, ACCSNS_LOG_TAG
		"%s\n", __func__);
	while(size)
	{
		membyte = *buf;
		for(bitno = 0; bitno < 8; bitno++)
		{
			if((crcreg ^ membyte) & 0x80)
			{
				crcreg = (crcreg << 1) ^ 0x11D;
			}
			else
			{
				crcreg <<= 1;
			}
			membyte <<= 1;
		}
		size--;
		buf++;
	}
	crcreg = ~crcreg;

	return crcreg;
}

static int acc_mtp_recovery_backup(uint8_t *buf)
{
	int ret = 0;
	u8 write_buf[2];

	dev_dbg(&client_accsns->adapter->dev, ACCSNS_LOG_TAG
		"%s\n", __func__);
	
	buf[0] = 0x0;
	accsns_i2c_read(buf, ACCSNS_BACKUP_DATA_NUM);

	
	write_buf[0] = 0x35;
	write_buf[1] = 0x0A;
	accsns_i2c_write(write_buf, 2);

	if(buf[26] != acc_mtp_recovery_crc8(buf,26))
	{
		dev_err(&client_accsns->adapter->dev, ACCSNS_LOG_TAG
			"accelerometer crc check error.\n\n");
		ret = -1;
	}
	else
	{
		ret = 0;
	}

	return ret;
}

static void acc_mtp_recovery_offset_clear(void)
{
	uint8_t reg = 0;
	u8  write_buf[2];

	dev_dbg(&client_accsns->adapter->dev, ACCSNS_LOG_TAG
		"%s\n", __func__);
	for(reg = 0x38; reg <= 0x3A; reg++)
	{
		write_buf[0] = reg;
		write_buf[1] = 0x0;
		accsns_i2c_write(write_buf, 2);
	}
}


int acc_mtp_recovery_main(uint8_t *backup_data)
{
	uint8_t i2c_addr = 0;
	int err = 0;

	dev_dbg(&client_accsns->adapter->dev, ACCSNS_LOG_TAG
		"%s\n", __func__);
	err = acc_mtp_recovery_device_check(&i2c_addr);
	if(err)
	{
		dev_err(&client_accsns->adapter->dev, ACCSNS_LOG_TAG
			"accelerometer sensor not found.\n");
		return NO_ERROR;
	}

	err = acc_mtp_recovery_set_correct_addr(&i2c_addr);
	if(err)
	{
		dev_err(&client_accsns->adapter->dev, ACCSNS_LOG_TAG
			"accelerometer sensor set correct addr failed.\n");
		return NO_ERROR;
	}

	if(backup_data[31] == 1)
	{
		acc_mtp_recovery_restore(backup_data);
		dev_dbg(&client_accsns->adapter->dev, ACCSNS_LOG_TAG
			"accelerometer cfg reg. restore success.\n");
	}
	else
	{
		if(!acc_mtp_recovery_backup(backup_data))
		{
			
			
			
			backup_data[31] = 1;
			dev_dbg(&client_accsns->adapter->dev, ACCSNS_LOG_TAG
				"accelerometer cfg reg. backup success.\n");
		}
		else
		{
			memset((void*)backup_data, 0, 32);
			dev_err(&client_accsns->adapter->dev, ACCSNS_LOG_TAG
				"accelerometer get backup data failed.\n");
		}
	}
	acc_mtp_recovery_offset_clear();

	return NO_ERROR;
}
EXPORT_SYMBOL(acc_mtp_recovery_main);
#endif /* SUPPORT_ACC_MTP_RECOVERY */


/*--------------------------------------------------------------------------
 * accsns function
 *--------------------------------------------------------------------------*/
int accsns_get_acceleration_data(int *xyz)
{
	int err = -1;
	int i;
	u8 sx[ACCSNS_DATA_ACCESS_NUM];

	if (atomic_read(&flgSuspend) == 1)
		return err;

	sx[0] = ACCSNS_XOUT;
	err = accsns_i2c_read(sx, ACCSNS_DATA_ACCESS_NUM);
	if (err < 0)
		return err;
	for (i = 0; i < ACCSNS_3AXIS_NUM; i++) {
		xyz[i] = (sx[2 * i] >> 6) | (sx[2 * i + 1] << 2);
		if (xyz[i] & 0x200)
			xyz[i] = (xyz[i] | 0xFFFFFC00);
	}
	dev_dbg(&client_accsns->adapter->dev,
		ACCSNS_LOG_TAG "x:%d,y:%d,z:%d\n", xyz[0], xyz[1], xyz[2]);

	return err;
}
EXPORT_SYMBOL(accsns_get_acceleration_data);

void accsns_activate(int flgatm, int flg)
{
	u8 buf[2];

	if (flg != 0)
		flg = 1;

	buf[0] = ACCSNS_REG0F;/* PMU_RANGE */
	buf[1] = 0x03;/* RANGE_2G */
	accsns_i2c_write(buf, 2);

	buf[0] = ACCSNS_REG10;/* PMU_BW */
	buf[1] = 0x0d;/* BW_250Hz */
	accsns_i2c_write(buf, 2);

	buf[0] = ACCSNS_REG11;/* PMU_LPW */
	if (flg == 0)
		buf[1] = 0x80;/* Suspend mode */
	else
		buf[1] = 0x00;/* Normal mode */
	accsns_i2c_write(buf, 2);
	ACCSNS_DELAY(2000);
	if (flgatm)
		atomic_set(&flgEna, flg);
}
EXPORT_SYMBOL(accsns_activate);

static int accsns_register_init(void)
{
	int d[ACCSNS_3AXIS_NUM], ret = 0;
	u8  buf[2];

	dev_dbg(&client_accsns->adapter->dev,
		ACCSNS_LOG_TAG "%s\n", __func__);

	buf[0] = ACCSNS_REG14;/* BGW_SOFTRESET */
	buf[1] = 0xB6;
	ret = accsns_i2c_write(buf, 2);
	if (ret < 0)
		return ret;
	ACCSNS_DELAY(4000);

	accsns_activate(0, 1);
	ret = accsns_get_acceleration_data(d);
	accsns_activate(0, 0);
	dev_info(&client_accsns->adapter->dev,
		ACCSNS_LOG_TAG "x:%d y:%d z:%d\n", d[0], d[1], d[2]);
	return ret;
}


/*--------------------------------------------------------------------------
 * suspend/resume function
 *--------------------------------------------------------------------------*/
static int accsns_suspend(struct i2c_client *client, pm_message_t mesg)
{
	dev_dbg(&client->adapter->dev,
		ACCSNS_LOG_TAG "%s\n", __func__);
	atomic_set(&flgSuspend, 1);
	accsns_activate(0, 0);
	return 0;
}

static int accsns_resume(struct i2c_client *client)
{
	dev_dbg(&client->adapter->dev,
		ACCSNS_LOG_TAG "%s\n", __func__);
	atomic_set(&flgSuspend, 0);
	accsns_activate(0, atomic_read(&flgEna));
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void accsns_early_suspend(struct early_suspend *handler)
{
	accsns_suspend(client_accsns, PMSG_SUSPEND);
}

static void accsns_early_resume(struct early_suspend *handler)
{
	accsns_resume(client_accsns);
}

static struct early_suspend accsns_early_suspend_handler = {
	.suspend	= accsns_early_suspend,
	.resume		= accsns_early_resume,
};
#endif


/*--------------------------------------------------------------------------
 * regulator
 *--------------------------------------------------------------------------*/
static int accsns_regulator_enable(struct i2c_client *client)
{
	int err=0;
	uint32_t min_uV=0,max_uV=0,load_uA=0;

	of_property_read_u32(client->dev.of_node, "accel-vdd-min-voltage", &min_uV);
	of_property_read_u32(client->dev.of_node, "accel-vdd-max-voltage", &max_uV);
	of_property_read_u32(client->dev.of_node, "accel-vdd-load-current", &load_uA);
	printk(KERN_NOTICE "[ACC]%s regulator min_uV = %d, max_uV = %d, load_uA = %d\n",
		__func__, min_uV, max_uV, load_uA);

	sensor_vdd = regulator_get(&client->dev, "accel-vdd");
	if( IS_ERR(sensor_vdd) ) {
		printk(KERN_ERR "[ACC]%s regulator_get fail.\n", __func__);
		err = PTR_ERR(sensor_vdd);
		goto exit;
	}

	err = regulator_set_voltage(sensor_vdd, min_uV, max_uV);
	if( err ) {
		printk(KERN_ERR "[ACC]%s regulator_set_voltage fail. err=%d\n", __func__, err);
		goto exit;
	}

	err = regulator_set_optimum_mode(sensor_vdd, load_uA);
	if( err < 0 ) {
		printk(KERN_ERR "[ACC]%s regulator_set_optimum_mode fail. err=%d\n", __func__, err);
		goto exit;
	}

	err = regulator_enable(sensor_vdd);
	if( err ) {
		printk(KERN_ERR "[ACC]%s regulator_enable fail. err=%d\n", __func__, err);
		goto exit;
	}

	usleep_range(3000,3000);
	return 0;
	
exit:
	if( sensor_vdd ) {
		regulator_set_optimum_mode(sensor_vdd, 0);
		regulator_put(sensor_vdd);
		sensor_vdd = 0;
	}
	return err;
}

static void accsns_regulator_disable(void)
{
	if( sensor_vdd ) {
		regulator_disable(sensor_vdd);
		regulator_set_optimum_mode(sensor_vdd, 0);
		regulator_put(sensor_vdd);
		sensor_vdd = 0;
	}
}

/*--------------------------------------------------------------------------
 * i2c device
 *--------------------------------------------------------------------------*/
static int accsns_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	dev_dbg(&client->adapter->dev,
		ACCSNS_LOG_TAG "%s\n", __func__);

	if(accsns_regulator_enable(client)) {
		dev_err(&client->adapter->dev, "not enable regulator\n");
		return -ENODEV;
	}
    
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->adapter->dev, "client not i2c capable\n");
		accsns_regulator_disable();
		return -ENODEV;
	}

	client_accsns = client;

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&accsns_early_suspend_handler);
#endif

	atomic_set(&flgEna, 0);
	atomic_set(&flgSuspend, 0);

	if (accsns_register_init()) {
		dev_err(&client->adapter->dev,
			"failed to initialize sensor\n");
		accsns_regulator_disable();
		return -EIO;
	}

	dev_info(&client->adapter->dev,
		"detected " ACCSNS_DRIVER_NAME "accelerometer\n");

	return 0;
}

static int accsns_remove(struct i2c_client *client)
{
	dev_dbg(&client->adapter->dev,
		ACCSNS_LOG_TAG "%s\n", __func__);
	accsns_activate(0, 0);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&accsns_early_suspend_handler);
#endif
	client_accsns = NULL;
	accsns_regulator_disable();
	return 0;
}


/*--------------------------------------------------------------------------
 * module
 *--------------------------------------------------------------------------*/
static const struct i2c_device_id accsns_id[] = {
	{ ACCSNS_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id accsns_match_table[] = {
	{ .compatible = ACCSNS_DRIVER_NAME,},
	{ },
};

static struct i2c_driver accsns_driver = {
	.probe		= accsns_probe,
	.remove		= accsns_remove,
	.id_table	= accsns_id,
	.driver		= {
	.name		= ACCSNS_DRIVER_NAME,
	.owner		= THIS_MODULE,
	.of_match_table = accsns_match_table,
	},
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= accsns_suspend,
	.resume		= accsns_resume,
#endif
};

static int __init accsns_init(void)
{
	pr_debug(ACCSNS_LOG_TAG "%s\n", __func__);
	return i2c_add_driver(&accsns_driver);
}

static void __exit accsns_exit(void)
{
	pr_debug(ACCSNS_LOG_TAG "%s\n", __func__);
	i2c_del_driver(&accsns_driver);
}

module_init(accsns_init);
module_exit(accsns_exit);

MODULE_DESCRIPTION("Alps Accelerometer Device");
MODULE_AUTHOR("ALPS ELECTRIC CO., LTD.");
MODULE_LICENSE("GPL v2");
