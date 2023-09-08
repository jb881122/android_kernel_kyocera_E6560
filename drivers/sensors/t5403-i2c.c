/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include "t5403.h"

#define T5403_DEBUG        0

#if T5403_DEBUG
#define T5403_DEBUG_LOG( arg... )   printk("T5403:" arg )
#else
#define T5403_DEBUG_LOG( arg... )
#endif

static int t5403_i2c_read_block(void *client, u8 reg, int len, char *buf)
{
	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return i2c_smbus_read_i2c_block_data(client, reg, len, buf);
}

static int t5403_i2c_read_byte(void *client, u8 reg)
{
	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return i2c_smbus_read_byte_data(client, reg);
}

static int t5403_i2c_write_byte(void *client, u8 reg, u8 value)
{
	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return i2c_smbus_write_byte_data(client, reg, value);
}

static const struct t5403_bus_ops t5403_i2c_bus_ops = {
	.read_block	= t5403_i2c_read_block,
	.read_byte	= t5403_i2c_read_byte,
	.write_byte	= t5403_i2c_write_byte
};

static int __devinit t5403_i2c_probe(struct i2c_client *client,
				      const struct i2c_device_id *id)
{
	struct t5403_bus *t5403_i2c;
	int error;

	T5403_DEBUG_LOG("%s(): start\n",__func__);

	t5403_i2c = kzalloc(sizeof(struct t5403_bus), GFP_KERNEL);
	if (!t5403_i2c)
		return -ENOMEM;

	t5403_i2c->bops = &t5403_i2c_bus_ops;
	t5403_i2c->client = client;
	t5403_i2c->id.bustype = BUS_I2C;

	error = t5403_probe(&client->dev, t5403_i2c);
	if (error < 0) {
	        printk("T5403: Failed probe sensor: %d\n", error);
		kfree(t5403_i2c);
		return error;
	}

	i2c_set_clientdata(client, t5403_i2c);
	return 0;
}

static void t5403_i2c_shutdown(struct i2c_client *client)
{
	T5403_DEBUG_LOG("%s(): start\n",__func__);
	t5403_disable(&client->dev);
}

static int __devexit t5403_i2c_remove(struct i2c_client *client)
{
	struct t5403_bus *t5403_i2c = i2c_get_clientdata(client);
	int error;

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	error = t5403_remove(&client->dev);
	if (error < 0)
	        printk("T5403: Failed sysfs_remove_group: %d\n", error);
		return error;

	kfree(t5403_i2c);
	return 0;
}

#if defined(CONFIG_PM)
static int t5403_i2c_suspend(struct device *dev)
{
	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return t5403_disable(dev);
}

static int t5403_i2c_resume(struct device *dev)
{
	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return t5403_enable(dev);
}
#endif

static UNIVERSAL_DEV_PM_OPS(t5403_i2c_pm, 
	t5403_i2c_suspend,
	t5403_i2c_resume,
	NULL);

static const struct i2c_device_id t5403_id[] = {
	{ T5403_DRIVER, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, t5403_id);

static struct of_device_id t5403_match_table[] = {
	{ .compatible = T5403_DRIVER,},
	{ },
};

static struct i2c_driver t5403_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= T5403_DRIVER,
		.pm	= &t5403_i2c_pm,
		.of_match_table = t5403_match_table,
	},
	.id_table	= t5403_id,
	.probe		= t5403_i2c_probe,
	.shutdown	= t5403_i2c_shutdown,
	.remove		= __devexit_p(t5403_i2c_remove)
};

static int __init t5403_i2c_init(void)
{
	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return i2c_add_driver(&t5403_i2c_driver);
}

static void __exit t5403_i2c_exit(void)
{
	T5403_DEBUG_LOG("%s(): start\n",__func__);
	i2c_del_driver(&t5403_i2c_driver);
}

MODULE_DESCRIPTION("T5403 pressure sensor driver");
MODULE_LICENSE("GPL");

module_init(t5403_i2c_init);
module_exit(t5403_i2c_exit);
