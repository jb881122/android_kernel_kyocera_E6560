/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/mod_devicetable.h>
#include <linux/log2.h>
#include <linux/bitops.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/memory.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>

#define IDTP90XX_FLAG_ADDR16	0x80	/* address pointer is 16 bit */
#define IDTP90XX_FLAG_READONLY	0x40	/* sysfs-entry will be read-only */
#define IDTP90XX_FLAG_IRUGO		0x20	/* sysfs-entry will be world-readable */
#define IDTP90XX_FLAG_TAKE8ADDR	0x10	/* take always 8 addresses (24c00) */

#define IDTP90XX_SIZE_BYTELEN 5
#define IDTP90XX_SIZE_FLAGS 8

#define IDTP90XX_BITMASK(x) (BIT(x) - 1)

/* create non-zero magic value for given eeprom parameters */
#define IDTP90XX_DEVICE_MAGIC(_len, _flags) 		\
	((1 << IDTP90XX_SIZE_FLAGS | (_flags)) 		\
	    << IDTP90XX_SIZE_BYTELEN | ilog2(_len))

#define RXID_ADDR		0xA84C
#define RXID_SIZE	6

#define IND_WAIT_CNT	10

#define IND_DATA_REG	0x8800

#define IND_ADDR_H_REG	0x8801
#define IND_ADDR_L_REG	0x8802

#define IND_CTRL_REG	0x8803
#define IND_CTRL_READ	0x02

#define IND_ST_REG		0x8804
#define IND_ST_BUSY		0x01
#define IND_ST_RESULT	0x02
#define RESULT_WO_ERR	0x02

struct idtp90xx_platform_data {
	u32		byte_len;		/* size (sum of all addr) */
	u16		page_size;		/* for writes */
	u8		flags;
	void		(*setup)(struct memory_accessor *, void *context);
	void		*context;
};

struct idtp90xx_data {
	struct idtp90xx_platform_data chip;

	/*
	 * Lock protects against activities from other Linux tasks,
	 * but not from changes by other I2C masters.
	 */
	struct mutex lock;

	u8 *writebuf;
	unsigned write_max;
	unsigned num_addresses;

	/*
	 * Some chips tie up multiple I2C addresses; dummy devices reserve
	 * them for us, and we'll use them with SMBus calls.
	 */
	struct i2c_client *client[];
};

static unsigned write_timeout = 25;
module_param(write_timeout, uint, 0);
MODULE_PARM_DESC(write_timeout, "Time (in ms) to try writes (default 25)");


static const struct i2c_device_id idtp90xx_ids[] = {
	/* needs 8 addresses as A0-A2 are ignored */
	{ "idtp90xx_kc", IDTP90XX_DEVICE_MAGIC(524288 / 8, IDTP90XX_FLAG_ADDR16) },
	{ /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(i2c, idtp90xx_ids);

static struct idtp90xx_data *the_idtp90xx;
/*-------------------------------------------------------------------------*/

static struct i2c_client *idtp90xx_translate_offset(struct idtp90xx_data *idtp90xx,
		unsigned *offset)
{
	unsigned i;

	if (idtp90xx->chip.flags & IDTP90XX_FLAG_ADDR16) {
		i = *offset >> 16;
		*offset &= 0xffff;
	} else {
		i = *offset >> 8;
		*offset &= 0xff;
	}

	return idtp90xx->client[i];
}

static ssize_t idtp90xx_eeprom_read(struct idtp90xx_data *idtp90xx, char *buf,
		unsigned offset, size_t count)
{
	struct i2c_msg msg[2];
	u8 msgbuf[2];
	struct i2c_client *client;
	unsigned long timeout, read_time;
	int status, i;

	memset(msg, 0, sizeof(msg));


	client = idtp90xx_translate_offset(idtp90xx, &offset);

		i = 0;
		if (idtp90xx->chip.flags & IDTP90XX_FLAG_ADDR16)
			msgbuf[i++] = offset >> 8;
		msgbuf[i++] = offset;

		msg[0].addr = client->addr;
		msg[0].buf = msgbuf;
		msg[0].len = i;

		msg[1].addr = client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].buf = buf;
		msg[1].len = count;

	timeout = jiffies + msecs_to_jiffies(write_timeout);
	do {
		read_time = jiffies;
			status = i2c_transfer(client->adapter, msg, 2);
			if (status == 2)
				status = count;
		dev_dbg(&client->dev, "read %zu@%d --> %d (%ld)\n",
				count, offset, status, jiffies);

		if (status == count)
			return count;

		/* REVISIT: at HZ=100, this is sloooow */
		msleep(1);
	} while (time_before(read_time, timeout));

	return -ETIMEDOUT;
}

static ssize_t idtp90xx_read(struct idtp90xx_data *idtp90xx,
		char *buf, loff_t off, size_t count)
{
	ssize_t retval = 0;

	if (unlikely(!count))
		return count;

	/*
	 * Read data from chip, protecting against concurrent updates
	 * from this host, but not from other I2C masters.
	 */
	mutex_lock(&idtp90xx->lock);

	while (count) {
		ssize_t	status;

		status = idtp90xx_eeprom_read(idtp90xx, buf, off, count);
		if (status <= 0) {
			if (retval == 0)
				retval = status;
			break;
		}
		buf += status;
		off += status;
		count -= status;
		retval += status;
	}

	mutex_unlock(&idtp90xx->lock);

	return retval;
}



static ssize_t idtp90xx_eeprom_write(struct idtp90xx_data *idtp90xx, const char *buf,
		unsigned offset, size_t count)
{
	struct i2c_client *client;
	struct i2c_msg msg;
	ssize_t status;
	unsigned long timeout, write_time;
	unsigned next_page;
	int i;

	/* Get corresponding I2C address and adjust offset */
	client = idtp90xx_translate_offset(idtp90xx, &offset);

	/* write_max is at most a page */
	if (count > idtp90xx->write_max) {
		count = idtp90xx->write_max;
	}

	/* Never roll over backwards, to the start of this page */
	next_page = roundup(offset + 1, idtp90xx->chip.page_size);
	if (offset + count > next_page) {
		count = next_page - offset;
	}

	/* If we'll use I2C calls for I/O, set up the message */
		i = 0;
		msg.addr = client->addr;
		msg.flags = 0;

		/* msg.buf is u8 and casts will mask the values */
		msg.buf = idtp90xx->writebuf;
		if (idtp90xx->chip.flags & IDTP90XX_FLAG_ADDR16)
			msg.buf[i++] = offset >> 8;

		msg.buf[i++] = offset;
		memcpy(&msg.buf[i], buf, count);
		msg.len = i + count;

	/*
	 * Writes fail if the previous one didn't complete yet. We may
	 * loop a few times until this one succeeds, waiting at least
	 * long enough for one entire page write to work.
	 */
	timeout = jiffies + msecs_to_jiffies(write_timeout);
	do {
		write_time = jiffies;
			status = i2c_transfer(client->adapter, &msg, 1);
			if (status == 1)
				status = count;
		dev_dbg(&client->dev, "write %zu@%d --> %zd (%ld)\n",
				count, offset, status, jiffies);

		if (status == count)
			return count;

		/* REVISIT: at HZ=100, this is sloooow */
		msleep(1);
	} while (time_before(write_time, timeout));

	return -ETIMEDOUT;
}

static ssize_t idtp90xx_write(struct idtp90xx_data *idtp90xx, const char *buf, loff_t off,
			  size_t count)
{
	ssize_t retval = 0;

	if (unlikely(!count))
		return count;

	/*
	 * Write data to chip, protecting against concurrent updates
	 * from this host, but not from other I2C masters.
	 */
	mutex_lock(&idtp90xx->lock);

	while (count) {
		ssize_t	status;

		status = idtp90xx_eeprom_write(idtp90xx, buf, off, count);
		if (status <= 0) {
			if (retval == 0)
				retval = status;
			break;
		}
		buf += status;
		off += status;
		count -= status;
		retval += status;
	}

	mutex_unlock(&idtp90xx->lock);

	return retval;
}

char idtp90xx_indirect_access(uint16_t addr)
{
	char write_buf;
	char read_buf;
	int i;

	/* set Indirect Access Address(MSB) */
	write_buf = addr >> 8;
	idtp90xx_write(the_idtp90xx, &write_buf, IND_ADDR_H_REG, 1);

	/* set Indirect Access Address(LSB) */
	write_buf = addr & 0xFF;
	idtp90xx_write(the_idtp90xx, &write_buf, IND_ADDR_L_REG, 1);

	/* set Indirect Access Read */
	write_buf = IND_CTRL_READ;
	idtp90xx_write(the_idtp90xx, &write_buf, IND_CTRL_REG, 1);

	/* check Indirect Access Status(Busy_flag & Result) */
	for (i=0;i<IND_WAIT_CNT;i++) {
		idtp90xx_read(the_idtp90xx, &read_buf, IND_ST_REG, 1);
		if ((read_buf & (IND_ST_BUSY | IND_ST_RESULT)) == RESULT_WO_ERR) {
			break;
		}
	}

	/* get Indirect Access Data */
	if (i < IND_WAIT_CNT) {
		idtp90xx_read(the_idtp90xx, &read_buf, IND_DATA_REG, 1);
		pr_info("ready read_buf:0x%2x count:%d\n", (int)read_buf, i);
	} else {
		pr_err("not ready read_buf:0x%2x\n", (int)read_buf);
	}

	/* clear Indirect Access Status */
	write_buf = 0x00;
	idtp90xx_write(the_idtp90xx, &write_buf, IND_ST_REG, 1);

	return read_buf;
}

void idtp90xx_get_rxid(char *buf)
{
	int i;
	for (i=0;i<RXID_SIZE;i++) {
		*buf = idtp90xx_indirect_access(RXID_ADDR + i);
		pr_info("get rxid[%d]:0x%2x\n", i, *buf);
		buf++;
	}
}
EXPORT_SYMBOL(idtp90xx_get_rxid);

static int idtp90xx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct idtp90xx_platform_data chip;
	bool writable;
	struct idtp90xx_data *idtp90xx;
	int err;
	unsigned i, num_addresses;
	kernel_ulong_t magic;

	if (client->dev.platform_data) {
		chip = *(struct idtp90xx_platform_data *)client->dev.platform_data;
	} else {
		if (!id->driver_data) {
			err = -ENODEV;
			goto err_out;
		}
		magic = id->driver_data;
		chip.byte_len = BIT(magic & IDTP90XX_BITMASK(IDTP90XX_SIZE_BYTELEN));
		magic >>= IDTP90XX_SIZE_BYTELEN;
		chip.flags = magic & IDTP90XX_BITMASK(IDTP90XX_SIZE_FLAGS);
		/*
		 * This is slow, but we can't know all eeproms, so we better
		 * play safe. Specifying custom eeprom-types via platform_data
		 * is recommended anyhow.
		 */
		chip.page_size = 1;

		chip.setup = NULL;
		chip.context = NULL;
	}

	if (!is_power_of_2(chip.byte_len))
		dev_err(&client->dev,
			"byte_len looks suspicious (no power of 2)!\n");
	if (!chip.page_size) {
		dev_err(&client->dev, "page_size must not be 0!\n");
		err = -EINVAL;
		goto err_out;
	}
	if (!is_power_of_2(chip.page_size))
		dev_err(&client->dev,
			"page_size looks suspicious (no power of 2)!\n");

	/* Use I2C operations unless we're stuck with SMBus extensions. */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
			err = -EPFNOSUPPORT;
			goto err_out;
	}

	if (chip.flags & IDTP90XX_FLAG_TAKE8ADDR)
	{
		num_addresses = 8;
	}
	else
	{
		num_addresses =	DIV_ROUND_UP(chip.byte_len,
			(chip.flags & IDTP90XX_FLAG_ADDR16) ? 65536 : 256);
	}

	idtp90xx = kzalloc(sizeof(struct idtp90xx_data) +
		num_addresses * sizeof(struct i2c_client *), GFP_KERNEL);
	if (!idtp90xx) {
		err = -ENOMEM;
		goto err_out;
	}
	the_idtp90xx = idtp90xx;

	mutex_init(&idtp90xx->lock);
	idtp90xx->chip = chip;
	idtp90xx->num_addresses = num_addresses;


	writable = !(chip.flags & IDTP90XX_FLAG_READONLY);
	if (writable) {
		if (i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)) {

			unsigned write_max = chip.page_size;


			idtp90xx->write_max = write_max;

			/* buffer (data + address at the beginning) */
			idtp90xx->writebuf = kmalloc(write_max + 2, GFP_KERNEL);
			if (!idtp90xx->writebuf) {
				err = -ENOMEM;
				goto err_struct;
			}
		} else {
			dev_err(&client->dev,
				"cannot write due to controller restrictions.");
		}
	}

	idtp90xx->client[0] = client;

	/* use dummy devices for multiple-address chips */
	for (i = 1; i < num_addresses; i++) {
		idtp90xx->client[i] = i2c_new_dummy(client->adapter,
					client->addr + i);
		if (!idtp90xx->client[i]) {
			dev_err(&client->dev, "address 0x%02x unavailable\n",
					client->addr + i);
			err = -EADDRINUSE;
			goto err_clients;
		}
	}

	i2c_set_clientdata(client, idtp90xx);

	dev_info(&client->dev, "%s, %s, %u bytes/write\n",
		client->name,
		writable ? "writable" : "read-only", idtp90xx->write_max);

	return 0;

err_clients:
	for (i = 1; i < num_addresses; i++)
		if (idtp90xx->client[i])
			i2c_unregister_device(idtp90xx->client[i]);

	kfree(idtp90xx->writebuf);
err_struct:
	kfree(idtp90xx);
err_out:
	dev_dbg(&client->dev, "probe error %d\n", err);
	return err;
}

static int __devexit idtp90xx_remove(struct i2c_client *client)
{
	struct idtp90xx_data *idtp90xx;
	int i;

	idtp90xx = i2c_get_clientdata(client);

	for (i = 1; i < idtp90xx->num_addresses; i++)
		i2c_unregister_device(idtp90xx->client[i]);

	kfree(idtp90xx->writebuf);
	kfree(idtp90xx);
	return 0;
}

/*-------------------------------------------------------------------------*/

static struct of_device_id idtp90xx_match_table[] = {
	{ .compatible = "at,idtp90xx_kc",},
	{ },
};

static struct i2c_driver idtp90xx_driver = {
	.driver = {
		.name = "idtp90xx_kc",
		.owner = THIS_MODULE,
		.of_match_table = idtp90xx_match_table,
	},
	.probe = idtp90xx_probe,
	.remove = __devexit_p(idtp90xx_remove),
	.id_table = idtp90xx_ids,
};

static int __init idtp90xx_init(void)
{
	return i2c_add_driver(&idtp90xx_driver);
}
module_init(idtp90xx_init);

static void __exit idtp90xx_exit(void)
{
	i2c_del_driver(&idtp90xx_driver);
}
module_exit(idtp90xx_exit);

/* Module information */
MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("Wireless Power Receiver driver");
MODULE_LICENSE("GPL");
