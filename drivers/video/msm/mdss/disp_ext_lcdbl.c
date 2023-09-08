/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 */
/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/msm_mdp.h>
#include "disp_ext.h"
#include "mdss_dsi.h"

#define LCDBL_DEBUG         0

#if LCDBL_DEBUG
#define LCDBL_DEBUG_LOG( msg, ... ) \
	pr_notice("[LCDBL][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define LCDBL_DEBUG_LOG( msg, ... ) \
	pr_debug("[LCDBL][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#endif

#define LCDBL_ERR_LOG( msg, ... ) \
	pr_err("[LCDBL][%s][E](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define LCDBL_WARN_LOG( msg, ... ) \
	pr_warn("[LCDBL][%s][W](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define LCDBL_NOTICE_LOG( msg, ... ) \
	pr_notice("[LCDBL][%s][N](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)


#define LCDBL_LEVEL_MAX 248

static DEFINE_SPINLOCK(lcdbl_s2c_lock);
static void disp_ext_lcdbl_gpio_pulse(int gpio, int pulse)
{
	int i;
	unsigned long flags;

	LCDBL_DEBUG_LOG("[IN] gpio=%d pulse=%d", gpio, pulse);

	spin_lock_irqsave(&lcdbl_s2c_lock, flags);
	for (i = 0; i < pulse; i++) {
		gpio_set_value(gpio ,0);
		gpio_set_value(gpio ,1);
	}
	spin_unlock_irqrestore(&lcdbl_s2c_lock, flags);
	udelay(750);

	LCDBL_DEBUG_LOG("[OUT]");
}

#ifdef CONFIG_DISP_EXT_LCDBL_DEBUG
static int disp_ext_lcdbl_lcdbl_en_gpio_saved = -EINVAL;
static void disp_ext_lcdbl_save_lcdbl_en_gpio(int gpio)
{
	LCDBL_DEBUG_LOG("[IN/OUT]");
	disp_ext_lcdbl_lcdbl_en_gpio_saved = gpio;
}

static int disp_ext_lcdbl_s2c_out_ctrl(const char *val, struct kernel_param *kp)
{
	int ret;
	long s2c = 0;
	int gpio = disp_ext_lcdbl_lcdbl_en_gpio_saved;

	LCDBL_DEBUG_LOG("[IN]");

	ret = kstrtol(val, 0, &s2c);
	if (ret) {
		LCDBL_WARN_LOG("invalid param. ret=%d", ret);
		return ret;
	}

	if (s2c < 1 || 32 < s2c) {
		LCDBL_WARN_LOG("invalid range. s2c=%ld", s2c);
		return -EINVAL;
	}

	if (!gpio_is_valid(gpio)) {
		LCDBL_WARN_LOG("lcdbl en gpio is not specified. gpio=%d", gpio);
		return -EINVAL;
	}

	disp_ext_lcdbl_gpio_pulse(gpio, s2c);

	LCDBL_DEBUG_LOG("[OUT]");

	return ret;
}
module_param_call(lcdbl_s2c_out, disp_ext_lcdbl_s2c_out_ctrl,
					NULL, NULL, 0200);
#else
static inline void disp_ext_lcdbl_save_lcdbl_en_gpio(int gpio) {}
#endif

int disp_ext_lcdbl_get_lcdbl_en_gpio(struct device_node *node)
{
	int gpio;

	LCDBL_DEBUG_LOG("[IN]");

	gpio = of_get_named_gpio(node, "qcom,platform-lcdbl-gpio", 0);
	if (!gpio_is_valid(gpio)) {
		LCDBL_NOTICE_LOG("lcdbl en gpio is not specified");
	} else {
		LCDBL_DEBUG_LOG("lcdbl en gpio[%d] found, but not assigned here.", gpio);
	}

	disp_ext_lcdbl_save_lcdbl_en_gpio(gpio);
	LCDBL_DEBUG_LOG("[OUT]");

	return gpio;
}

int disp_ext_lcdbl_hw_s2c_calc( int and )
{
  int s2c = 1;
  if((0 < and) && (and < 10)){
    s2c = (((-16 * and) + 320) / 10);
  }else if((10 <= and) && (and <= 36)){
    s2c = ((-615 * and) + 23140) / 1000;
  }
  return(s2c);
}

int disp_ext_lcdbl_hw_pwm_calc( int and )
{
  int pwm = 0;
  if((0 < and) && ( and <= 36 )){
    pwm = 26;
  }if((36 < and) && ( and <= 102 )){
    pwm = ((489 * and) + 7000) / 1000;
    pwm++;
  }else if((102 < and) && ( and <= 255 )){
    pwm = ((111 * and) - 5633) / 100;
    pwm++;
  }
  return(pwm);
}

#define DISP_EXT_CMD_ADJ 90
int disp_ext_lcdbl_hw_cmb_calc( int and, int pwm )
{
	int cmb = pwm;
	if(pwm > 32){
		cmb = (pwm * DISP_EXT_CMD_ADJ) / 100;
	}
	return(cmb);
}

static char disp_ext_save_cmb = 0;
static char disp_ext_pre_cmb  = 0;
void disp_ext_lcdbl_cmb_write( struct mdss_dsi_ctrl_pdata *ctrl )
{
	if(disp_ext_pre_cmb != disp_ext_save_cmb){
		disp_ext_panel_cmd_dcs(ctrl, 0x5E, disp_ext_save_cmb);
		disp_ext_pre_cmb = disp_ext_save_cmb;
		LCDBL_DEBUG_LOG("lcdbl cmb set[%d] \n",disp_ext_pre_cmb);
	}
}

u32 disp_ext_lcdbl_gpio_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, u32 level)
{
	int rc ;
	u8 s2c = 0, pwm = 0;
	static u8 pre_s2c = 0;
	static u32 pre_level = 0;

	LCDBL_DEBUG_LOG("[IN]");

#ifdef CONFIG_DISP_EXT_BOARD
	if (disp_ext_board_get_panel_detect() != 1) {
		return(level);
	}
#endif
	if (!ctrl || !gpio_is_valid(ctrl->disp_bl_gpio)) {
		LCDBL_DEBUG_LOG("lcdbl en gpio is not specified");
		return(level);
	}

	if (level == 0) {
		gpio_set_value(ctrl->disp_bl_gpio, 0);
		pre_s2c = 0;
		disp_ext_pre_cmb = disp_ext_save_cmb = 0;
		if (pre_level != 0) {
			gpio_free(ctrl->disp_bl_gpio);
			LCDBL_DEBUG_LOG("lcdbl en gpio[%d], free.", ctrl->disp_bl_gpio);
		}
	} else {
		if (level > LCDBL_LEVEL_MAX) {
			level = LCDBL_LEVEL_MAX;
		}
		if (pre_level == 0) {
			rc = gpio_request(ctrl->disp_bl_gpio, "lcdbl_en");
			if (rc) {
				LCDBL_ERR_LOG("request lcdbl en gpio failed, rc=%d", rc);
				return(level);
			}
			LCDBL_DEBUG_LOG("lcdbl en gpio[%d], assigned.", ctrl->disp_bl_gpio);
		}
		s2c = disp_ext_lcdbl_hw_s2c_calc(level);
		pwm = disp_ext_lcdbl_hw_pwm_calc(level);
		disp_ext_save_cmb = disp_ext_lcdbl_hw_cmb_calc(level, pwm);
		if (s2c != pre_s2c) {
			disp_ext_lcdbl_gpio_pulse(ctrl->disp_bl_gpio, s2c);
			pre_s2c = s2c;
		}
		if (!pre_level) {
			usleep_range(15000, 16000);
			disp_ext_lcdbl_cmb_write(ctrl);
		}
	}
	pre_level = level;

	LCDBL_DEBUG_LOG("[OUT] level:%d s2c:%d pwm:%d", level, s2c, pwm);

	return pwm;
}
