/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 */
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "t5403.h"

#include <linux/input.h>

#include <linux/of.h>
#include <linux/regulator/consumer.h>

#define T5403_DEBUG		0

#if T5403_DEBUG
#define T5403_DEBUG_LOG( arg... )   printk("T5403:" arg )
#else
#define T5403_DEBUG_LOG( arg... )
#endif

/* Max/Min barometric pressure values as stated in spec, unit = hPa */
#define T5403_MAX_PRES		1100
#define T5403_MIN_PRES		300

/* Max/Min ambient temperature values as stated in spec, unit = Celsius */
#define T5403_MAX_TEMP		85
#define T5403_MIN_TEMP		-30

/* Chip ID */
#define T5403_CHIP_ID_REG 	0x88
#define T5403_CHIP_ID_MSK	0x7F
#define T5403_CHIP_ID		0x77

/**
 * Calibration coefficients registers
 *
 * The coefficients comes in a consecutive register serie, from C1 to C8, where
 * C1-C4 are unsigned 16-bit integer values and C5-C8 are signed 16-bit values.
 */
#define T5403_C1_LSB_REG	0x8E
#define T5403_C8_MSB_REG	0x9D
#define T5403_CALIB_REGS_SIZE	16

/* Software reset  */
#define T5403_CTRL_RES_REG	0xF0
#define T5403_RESET_CMD		0x73

/**
 * Temperature/Pressure measurement cmd, with the following CTRL_CMD bitfields:
 *
 * b0: sco, set to 1 for conversion start
 * b1-b2: pt, set 00 for pressure meas and 01 for temperature meas
 * b3-b4: mode, set values according to enum t5403_op_mode
 * b5: zero, set to 0 for applying the command correctly
 * b6-b7: don't care
 */
#define T5403_CTRL_CMD_REG	0xF1
#define T5403_CONV_PRES_CMD	0x01
#define T5403_CONV_TEMP_CMD	0x03
#define T5403_MEAS_MODE_POS	3

/* Temperature/Pressure Data registers */
#define T5403_DATA_MSB_REG	0xF6
#define T5403_DATA_LSB_REG	0xF5
#define T5403_DATA_SIZE		2

/* Input poll-intervals (miliseconds) */
#define T5403_DELAY_DEFAULT	100

/* Power-on/Reset start-up time (miliseconds) */
#define T5403_START_UP_TIME	11

/* Idle mode wake-up time (miliseconds) */
#define T5403_WAKE_UP_TIME	2

/* Temperature conversion time (miliseconds) */
#define T5403_TEMP_CONV_TIME	(T5403_WAKE_UP_TIME + 4)


#define ABS_X_DUMMY_VAL		-1


/* Pressure measurement conversion time (miliseconds) including the wakeup */
static const u8 pres_conv_time[T5403_OP_MODE_LAST] = {
	(T5403_WAKE_UP_TIME + 2),
	(T5403_WAKE_UP_TIME + 8),
	(T5403_WAKE_UP_TIME + 16),
	(T5403_WAKE_UP_TIME + 64)
};

enum t5403_conv_type {
	T5403_CONV_PRES,
	T5403_CONV_TEMP
};

struct t5403_calib_coef {
	u16 c1, c2, c3, c4;
	s16 c5, c6, c7, c8;
};

struct t5403_data {
	struct t5403_calib_coef coef;
	u16 raw_pres;
	s16 raw_temp;
};

struct t5403 {
	struct device *dev;
	struct input_dev *input;
	struct delayed_work work;
	struct mutex mutex;
	struct t5403_bus *bus;
	struct t5403_data data;
	enum t5403_op_mode op_mode;
	/*!delay time used by input event */
	u32 delay;
	/*!enable/disable sensor output */
	u32 enable;
	/*!pressure output */
	u32 pressure;
	
	u32 temperature;
	u32 raw_pressure;
	u32 raw_temperature;
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

static struct regulator* sensor_vdd = 0;

static inline int t5403_read_byte(struct t5403_bus *bus, int reg)
{
	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return bus->bops->read_byte(bus->client, reg);
}

static inline int t5403_read_block(struct t5403_bus *bus, int reg,
						int len, char *buf)
{
	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return bus->bops->read_block(bus->client, reg, len, buf);
}

static inline int t5403_write_byte(struct t5403_bus *bus, int reg, int val)
{
	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return bus->bops->write_byte(bus->client, reg, val);
}

static int t5403_start_conv(struct t5403_bus *bus,
					enum t5403_conv_type type,
					enum t5403_op_mode op_mode)
{
	u8 rc;

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	if (type == T5403_CONV_PRES)
		rc = t5403_write_byte(bus, T5403_CTRL_CMD_REG,
					T5403_CONV_PRES_CMD |
					(u8)(op_mode << T5403_MEAS_MODE_POS));
	else
		rc = t5403_write_byte(bus, T5403_CTRL_CMD_REG,
					T5403_CONV_TEMP_CMD);
	return rc;
}

static int t5403_soft_reset(struct t5403_bus *bus)
{
	int error;

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	error = t5403_write_byte(bus, T5403_CTRL_RES_REG, T5403_RESET_CMD);

	if (error < 0)
		return error;

	msleep(T5403_START_UP_TIME);
	return 0;
}

static s32 __devinit t5403_get_calibration_parameters(struct t5403_bus *bus,
					struct t5403_calib_coef *coef)
{
	u16 data[T5403_CALIB_REGS_SIZE >> 1];
	int error;

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	error = t5403_read_block(bus, T5403_C1_LSB_REG, T5403_CALIB_REGS_SIZE,
							(char *)data);
	if (error < T5403_CALIB_REGS_SIZE)
		return -EIO;

	coef->c1 = le16_to_cpu(data[0]);
	coef->c2 = le16_to_cpu(data[1]);
	coef->c3 = le16_to_cpu(data[2]);
	coef->c4 = le16_to_cpu(data[3]);
	coef->c5 = le16_to_cpu(data[4]);
	coef->c6 = le16_to_cpu(data[5]);
	coef->c7 = le16_to_cpu(data[6]);
	coef->c8 = le16_to_cpu(data[7]);

	return 0;
}

/**
 * This function converts the raw temperature to centi-celsius with optimization
 * for integer fixed-point arithmetics with two fractions embedded in the
 * integer (i.e. 27.30 will be reported as 2730 and so on).
 *
 * Formula from application note:
 * Ta = (100 * (((c1 * Tr) / 2^8) + (c2 * 2^6))) / 2^16
 *    = (100 * ((c1 * Tr) / 2^24) + (c2 / 2^10))
 */
static inline int t5403_calculate_temperature(struct t5403_data *data)
{
	s64 temp, val;

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	val = ((s64)(data->coef.c1 * data->raw_temp) * 100);
	temp = (val >> 24);
	val = ((s64)data->coef.c2 * 100);
	temp += (val >> 10);

	return (int)temp;
}

/**
 * This function converts the raw pressure to Pascal, i.e. without fractions.
 *
 * Formula from application note, rev_X:
 * Sensitivity = (c3 + ((c4 * Tr) / 2^17) + ((c5 * Tr^2) / 2^34))
 * Offset = (c6 * 2^14) + ((c7 * Tr) / 2^3) + ((c8 * Tr^2) / 2^19)
 * Pa = (Sensitivity * Pr + Offset) / 2^14
 */
static inline u32 t5403_calculate_pressure(struct t5403_data *data)
{
	s64 s, o, pres, val;

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	s = (s64)data->coef.c3;
	val = (s64)(data->coef.c4 * data->raw_temp);
	s += (val >> 17);
	val = (s64)(data->coef.c5 * data->raw_temp * data->raw_temp);
	s += (val >> 34);

	o = (s64)data->coef.c6 << 14;
	val = (s64)(data->coef.c7 * data->raw_temp);
	o += (val >> 3);
	val = (s64)(data->coef.c8 * data->raw_temp * data->raw_temp);
	o += (val >> 19);

	
	pres = (((s64)(s * data->raw_pres) + o) * 10) >> 14;

	return (u32)pres;
	
}

static void t5403_get_pressure(struct t5403 *t5403, u16 raw_pres, s16 raw_temp,
				u32 *calc_pres)
{
	int temperature;
	
	u32 pressure;
	

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	/* get raw data */
	t5403->data.raw_temp = le16_to_cpu(raw_temp);
	t5403->data.raw_pres = le16_to_cpu(raw_pres);
	
	t5403->raw_temperature = t5403->data.raw_temp;
	t5403->raw_pressure = t5403->data.raw_pres;
	

	/* Calculate pressure and temperature */
	temperature = t5403_calculate_temperature(&t5403->data);
	pressure = t5403_calculate_pressure(&t5403->data);

	/* check temperature */
	if (temperature < T5403_MIN_TEMP * 100 ||
	    temperature > T5403_MAX_TEMP * 100) {
		T5403_DEBUG_LOG("%s: Temperature value is out of range: %d\n", __func__, temperature);
	}

	/* check pressure */
	
	if (pressure < T5403_MIN_PRES * 1000 ||
	    pressure > T5403_MAX_PRES * 1000) {
	
		printk("T5403: Pressure value is out of range: %d Pa\n", pressure);
	}
	*calc_pres = pressure; 
	t5403->pressure = pressure;
	
	t5403->temperature = temperature;
	
}

static void t5403_worker(struct work_struct *work)
{
	struct t5403 *t5403 = container_of(
		(struct delayed_work *)work, struct t5403, work);
	unsigned long delay = msecs_to_jiffies(t5403->delay);
	struct t5403_bus *bus = t5403->bus;
	u32 j1 = jiffies;
	int error;

	u16 raw_pres;
	s16 raw_temp;
	
	u32 *calc_pres;
	

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	/* Start temperature measurement */
	error = t5403_start_conv(bus, T5403_CONV_TEMP, 0);
	if (error < 0) {
		printk("T5403: Temperature command error: %d\n", error);
		goto error_exit;
	}
	msleep(T5403_TEMP_CONV_TIME);
	error = t5403_read_block(bus,T5403_DATA_LSB_REG, T5403_DATA_SIZE,
							(char *)&raw_temp);
	if (error < T5403_DATA_SIZE) {
		printk("T5403: Temperature read error: %d\n", error);
		goto error_exit;
	}

	/* Start pressure measurement */
	error = t5403_start_conv(bus, T5403_CONV_PRES, t5403->op_mode);
	if (error < 0) {
		printk("T5403: Pressure command error: %d\n", error);
		goto error_exit;
	}
	msleep(pres_conv_time[T5403_OP_MODE_U_HIGH] + 0.5);
	error = t5403_read_block(bus, T5403_DATA_LSB_REG, T5403_DATA_SIZE,
							(char *)&raw_pres);
	if (error < T5403_DATA_SIZE) {
		printk("T5403: Pressure read error: %d\n", error);
		goto error_exit;
	}

	/* get pressure */
	t5403_get_pressure(t5403, raw_pres, raw_temp, calc_pres);
	
	input_event(t5403->input, EV_MSC, MSC_RAW, ABS_X_DUMMY_VAL);
	
	input_event(t5403->input, EV_MSC, MSC_RAW, *calc_pres);
	input_sync(t5403->input);

error_exit:
	schedule_delayed_work(&t5403->work, delay-(jiffies-j1));
}

#ifdef CONFIG_PM
int t5403_disable(struct device *dev)
{
	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return 0;
}
EXPORT_SYMBOL(t5403_disable);

int t5403_enable(struct device *dev)
{
	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return 0;
}
EXPORT_SYMBOL(t5403_enable);
#endif

/*
 * @brief delete input device
 *
 * @param data the pointer of bmp client data
 *
 * @return no return value
*/
static void t5403_input_delete(struct t5403 *t5403)
{
	struct input_dev *dev = t5403->input;

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	input_unregister_device(dev);
	input_free_device(dev);
}

/* sysfs callbacks */
/*!
 * @brief get delay value via sysfs node
 *
 * @param dev the pointer of device
 * @param attr the pointer of device attribute file
 * @param buf the pointer of delay buffer
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static ssize_t show_delay(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct t5403 *data = dev_get_drvdata(dev);

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return sprintf(buf, "%d\n", data->delay);
}

/*!
 * @brief set delay value via sysfs node
 *
 * @param dev the pointer of device
 * @param attr the pointer of device attribute file
 * @param buf the pointer of delay buffer
 * @param count buffer size
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static ssize_t store_delay(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct t5403 *data = dev_get_drvdata(dev);
	unsigned long delay;
	int status = kstrtoul(buf, 10, &delay);

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	if (status == 0) {
		data->delay = delay;
		return count;
	}

	return status;
}

/*!
 * @brief get sensor work state via sysfs node
 *
 * @param dev the pointer of device
 * @param attr the pointer of device attribute file
 * @param buf the pointer of enable/disable value buffer
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static ssize_t show_enable(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct t5403 *data = dev_get_drvdata(dev);

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return sprintf(buf, "%d\n", data->enable);
}

/*!
 * @brief enable/disable sensor function via sysfs node
 *
 * @param dev the pointer of device
 * @param attr the pointer of device attribute file
 * @param buf the pointer of enable/disable buffer
 * @param count buffer size
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static ssize_t store_enable(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct t5403 *data = dev_get_drvdata(dev);
	unsigned long enable;
	int status = kstrtoul(buf, 10, &enable);

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	if (status == 0) {
		enable = enable ? 1 : 0;
		if (data->enable != enable) {
			if (enable) {
				#ifdef CONFIG_PM
				t5403_enable(dev);
				#endif
				schedule_delayed_work(&data->work,
					msecs_to_jiffies(data->delay));
			} else{
				cancel_delayed_work_sync(&data->work);
				#ifdef CONFIG_PM
				t5403_disable(dev);
				#endif
			}
			data->enable = enable;
		}
		return count;
	}

	return status;
}

/*!
 * @brief set compersated pressure value via sysfs node
 *
 * @param dev the pointer of device
 * @param attr the pointer of device attribute file
 * @param buf the pointer of pressure buffer
 * @param count buffer size
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static ssize_t show_pressure(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct t5403 *data = dev_get_drvdata(dev);

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return sprintf(buf, "%d\n", data->pressure);
}


static ssize_t show_temperature(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct t5403 *data = dev_get_drvdata(dev);

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return sprintf(buf, "%d\n", data->temperature);
}

static ssize_t show_raw_pressure(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct t5403 *data = dev_get_drvdata(dev);

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return sprintf(buf, "%d\n", data->raw_pressure);
}

static ssize_t show_raw_temperature(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct t5403 *data = dev_get_drvdata(dev);

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	return sprintf(buf, "%d\n", data->raw_temperature);
}


static DEVICE_ATTR(delay, S_IWUSR | S_IRUGO,
			show_delay, store_delay);
static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
			show_enable, store_enable);
static DEVICE_ATTR(pressure, S_IRUGO,
			show_pressure, NULL);

static DEVICE_ATTR(temperature, S_IRUGO,
			show_temperature, NULL);
static DEVICE_ATTR(raw_pressure, S_IRUGO,
			show_raw_pressure, NULL);
static DEVICE_ATTR(raw_temperature, S_IRUGO,
			show_raw_temperature, NULL);


static struct attribute *t5403_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_pressure.attr,

	&dev_attr_temperature.attr,
	&dev_attr_raw_pressure.attr,
	&dev_attr_raw_temperature.attr,

	NULL
};

static const struct attribute_group t5403_attr_group = {
	.attrs = t5403_attributes,
};

__devinit int t5403_probe(struct device *dev, struct t5403_bus *bus)
{
	struct t5403 *t5403;
	struct input_dev *t5403_dev;
	u8 chip_id;
	int error;
	int error_sysfs;
	uint32_t min_uV=0,max_uV=0,load_uA=0;

	T5403_DEBUG_LOG("%s(): start\n",__func__);

	of_property_read_u32(dev->of_node, "pressure-vdd-min-voltage", &min_uV);
	of_property_read_u32(dev->of_node, "pressure-vdd-max-voltage", &max_uV);
	of_property_read_u32(dev->of_node, "pressure-vdd-load-current", &load_uA);
	printk(KERN_NOTICE "[PRESS]%s regulator min_uV = %d, max_uV = %d, load_uA = %d\n",
		__func__, min_uV, max_uV, load_uA);

	sensor_vdd = regulator_get(dev, "pressure-vdd");
	if( IS_ERR(sensor_vdd) ) {
		printk("[PRESS]%s regulator_get fail.\n", __func__);
		error = PTR_ERR(sensor_vdd);
		goto exit;
	}

	error = regulator_set_voltage(sensor_vdd, min_uV, max_uV);
	if( error ) {
		printk("[PRESS]%s regulator_set_voltage fail. err=%d\n", __func__, error);
		goto exit;
	}

	error = regulator_set_optimum_mode(sensor_vdd, load_uA);
	if( error < 0 ) {
		printk("[PRESS]%s regulator_set_optimum_mode fail. err=%d\n", __func__, error);
		goto exit;
	}

	error = regulator_enable(sensor_vdd);
	if( error ) {
		printk("[PRESS]%s regulator_enable fail. err=%d\n", __func__, error);
		goto exit;
	}

	usleep_range(10000,10000);

	
	error = t5403_soft_reset(bus);
	if (error < 0) {
	        printk("T5403: soft reset failed: %d\n", error);
		error = -EIO;
		goto exit_vdd;
	}

	/* Check chip id */
	chip_id = (T5403_CHIP_ID_MSK & t5403_read_byte(bus, T5403_CHIP_ID_REG));
	if (chip_id != T5403_CHIP_ID) {
		printk("T5403: chip id failed: %d\n", chip_id);
		error = -EINVAL;
		goto exit_vdd;
	}

	t5403 = kzalloc(sizeof(struct t5403), GFP_KERNEL);
	if (!t5403){
		error = -ENOMEM;
		goto exit_vdd;
	}

	dev_set_drvdata(dev, t5403);
	t5403->bus = bus;
	t5403->dev = dev;

	/* Calibration */
	error = t5403_get_calibration_parameters(t5403->bus, &t5403->data.coef);
	if (error < 0) {
		printk("T5403: Failed to get calib. params: %d\n", error);
		goto err_free_mem;
	}

	#ifdef CONFIG_HAS_EARLYSUSPEND
	mutex_init(&t5403->mutex);
	#endif

	/* Initialize chip */
	t5403->op_mode = T5403_OP_MODE_U_HIGH;
	t5403->delay  = T5403_DELAY_DEFAULT;
	t5403->enable = 0;

	/* Initialize input device */
	t5403_dev = input_allocate_device();
	if (!t5403_dev)
		return -ENOMEM;
	t5403_dev->name = T5403_DRIVER;
	t5403_dev->id.bustype = BUS_I2C;

	input_set_capability(t5403_dev, EV_MSC, MSC_RAW);
	input_set_drvdata(t5403_dev, t5403);

	error = input_register_device(t5403_dev);
	if (error < 0) {
		input_free_device(t5403_dev);
		goto err_free_mem;
	}
	t5403->input = t5403_dev;

	/* Register sysfs hooks */
	error_sysfs = sysfs_create_group(&t5403->input->dev.kobj, &t5403_attr_group);
	if ( error_sysfs ) {
		printk("T5403: Failed sysfs_create_group: %d\n", error_sysfs);
		goto input_delete;	
	}

	/* workqueue init */
	INIT_DELAYED_WORK(&t5403->work, t5403_worker);

#ifdef CONFIG_HAS_EARLYSUSPEND
	t5403->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	t5403->early_suspend.suspend = t5403_early_suspend;
	t5403->early_suspend.resume = t5403_late_resume;
	register_early_suspend(&t5403->early_suspend);
#endif
	T5403_DEBUG_LOG("Succesfully probe sensor %s\n", T5403_DRIVER);
	return 0;

input_delete:
	t5403_input_delete(t5403);
err_free_mem:
	kfree(t5403);
exit_vdd:
	regulator_disable(sensor_vdd);
exit:
	if( sensor_vdd ) {
		regulator_set_optimum_mode(sensor_vdd, 0);
		regulator_put(sensor_vdd);
		sensor_vdd = 0;
	}

	return error;
}
EXPORT_SYMBOL(t5403_probe);

int __devexit t5403_remove(struct device *dev)
{
	struct t5403 *data = dev_get_drvdata(dev);

	T5403_DEBUG_LOG("%s(): start\n",__func__);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif
	sysfs_remove_group(&data->input->dev.kobj, &t5403_attr_group);
	kfree(data);
	if( sensor_vdd ) {
		regulator_disable(sensor_vdd);
		regulator_set_optimum_mode(sensor_vdd, 0);
		regulator_put(sensor_vdd);
		sensor_vdd = 0;
	}

	return 0;
}
EXPORT_SYMBOL(t5403_remove);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void t5403_early_suspend(struct early_suspend *h)
{
	struct t5403 *data =
		container_of(h, struct t5403, early_suspend);

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	mutex_lock(&data->lock);
	if (data->enable) {
		cancel_delayed_work_sync(&data->work);
		#ifdef CONFIG_PM
		(void) t5403_disable(tdata->dev);
		#endif
	}
	mutex_unlock(&data->lock);
}

static void t5403_late_resume(struct early_suspend *h)
{
	struct t5403 *data =
		container_of(h, struct t5403, early_suspend);

	T5403_DEBUG_LOG("%s(): start\n",__func__);
	mutex_lock(&data->lock);
	if (data->enable) {
		#ifdef CONFIG_PM
		(void) t5403_enable(data->dev);
		#endif
		schedule_delayed_work(&data->work,
			msecs_to_jiffies(data->delay));
	}
	mutex_unlock(&data->lock);
}
#endif

MODULE_DESCRIPTION("T5403 pressure sensor driver");
MODULE_LICENSE("GPL");
