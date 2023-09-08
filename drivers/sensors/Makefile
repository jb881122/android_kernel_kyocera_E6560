# This software is contributed or developed by KYOCERA Corporation.
# (C) 2014 KYOCERA Corporation

ccflags-y := -mfloat-abi=softfp -mfpu=vfp

ccflags-y += -Iinclude/linux

obj-$(CONFIG_INPUT_CLASS_PRESSURE) += pres_kernel_driver.o
obj-$(CONFIG_INPUT_SENSOR_PROXIMITY) += sfh7776.o
obj-$(CONFIG_INPUT_SENSOR_PRESSURE) += t5403-core.o
obj-$(CONFIG_INPUT_SENSOR_PRESSURE) += t5403-i2c.o
obj-$(CONFIG_INPUT_ALPS_INPUT) += alps-input.o
obj-$(CONFIG_INPUT_HSCDTD) += hscdtd007a_i2c.o
obj-$(CONFIG_INPUT_SENSOR_ACCELEROMETER) += bma250e_i2c.o

obj-$(CONFIG_SENSORS)		+= sensors_class.o

