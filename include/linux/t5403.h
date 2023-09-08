/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 */
#ifndef _T5403_H
#define _T5403_H
#include <linux/input.h>

#define T5403_DRIVER	"t5403"

/**
 * enum t5403_op_mode - defines the different operation modes.
 *
 * The T5403 supports different operation modes which provides different
 * accuracy levels (and accompanied conversion time) in terms of RMS noise
 * performance.
 */
enum t5403_op_mode {
	T5403_OP_MODE_LOW,	/* ~8.8Pa RMS Noise, 2ms conv. time */
	T5403_OP_MODE_STANDARD,	/* ~6.4Pa RMS Noise, 8ms conv. time */
	T5403_OP_MODE_HIGH,	/* ~5.0Pa RMS Noise, 16ms conv. time */
	T5403_OP_MODE_U_HIGH,	/* ~4.4Pa RMS Noise, 64ms conv. time */
	T5403_OP_MODE_LAST
};

/**
 * struct t5403_bus_ops - supported bus operations for the t5403 driver
 */
struct t5403_bus_ops {
	int (*read_block)(void *client, u8 reg, int len, char *buf);
	int (*read_byte)(void *client, u8 reg);
	int (*write_byte)(void *client, u8 reg, u8 value);
};

/**
 * struct t5403_platform_data - represents the t5403 driver bus setup
 */
struct t5403_bus {
	const struct t5403_bus_ops *bops;
	void *client;
	struct input_id id;
};

int t5403_probe(struct device *dev, struct t5403_bus *data_bus);
int t5403_remove(struct device *dev);
#if defined(CONFIG_PM)
int t5403_enable(struct device *dev);
int t5403_disable(struct device *dev);
#endif

#endif
