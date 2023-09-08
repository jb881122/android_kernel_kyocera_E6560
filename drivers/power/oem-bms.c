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

#include <linux/module.h>
#include <linux/power_supply.h>
#include <oem-chg_param.h> 
#include <oem-bms.h>
#include <oem-chg_control.h>

#include <oem-hkadc.h>
#include <oem-chg_param.h>
#include <linux/dnand_cdev_driver.h>


#undef BMS_DEBUG
#ifdef BMS_DEBUG
	#define BMS_LOG pr_err
#else
	#define BMS_LOG pr_debug
#endif

#define LOWER_LIMIT_SOC_CHARGING 0
#define UPPER_LIMIT_SOC_CHARGING 99
#define LOWER_LIMIT_SOC_DISCHARGING 0
#define UPPER_LIMIT_SOC_DISCHARGING 100
#define OEM_SOC_FULL 100

#define OEM_SOC_SHUT_DOWN	0
#define OEM_SHUT_DOWN_UV	3200000
#define SHUT_DOWN_MONITOR_TIME_SEC	30



#define REMAINING_IBAT_BUF_SIZE 300
#define REMAINING_CBAT 3000
#define REMAINING_INVALID -1


DEFINE_SPINLOCK(oem_bms_master_lock);
#define MASTERDATA_SPINLOCK_INIT() spin_lock_init(&oem_bms_master_lock);
#define MASTERDATA_LOCK() \
	{ \
	unsigned long oem_bms_irq_flag; \
	spin_lock_irqsave(&oem_bms_master_lock, oem_bms_irq_flag);
#define MASTERDATA_UNLOCK() \
	spin_unlock_irqrestore(&oem_bms_master_lock, oem_bms_irq_flag); \
	}

static struct wake_lock oem_bms_wake_lock;

static atomic_t is_bms_initialized = ATOMIC_INIT(0);

static int remaining_ibat_ary[REMAINING_IBAT_BUF_SIZE];
static int remaining_count = 0;
static int remaining_data_count = 0;
static int oem_chg_remaining_time = REMAINING_INVALID;
static int32_t remaining_buf_total = 0;

static void oem_bms_remaining_time_reset(int remaining_time);


static int current_batt_health_status = OEM_BMS_BATT_HEALTH_NORMAL;
static enum batt_health_check_status batt_health_check_st = BATT_HEALTH_CHECK_NO_CHG;
static enum batt_health_check_status last_batt_health_check_st = BATT_HEALTH_CHECK_NO_CHG;
static bool rbat_avg_valid = false;
static int rbat_avg_mohm = 0;
static int oem_bms_get_rbat_avg(void);



#define ENABLE_CALCULATE_REMAINING_SEC	(1)
#define DEFAULT_IBAT_CALCULATE			(200 * 1000)/* 200mA */
static int oem_bms_debug_enable = 0;
module_param_named(debug_enable, oem_bms_debug_enable, int, S_IRUGO | S_IWUSR);


int g_enable_oem_bms_battery_check = 0;


static int oem_bms_get_batt_property(enum power_supply_property psp)
{
	struct power_supply *psy;
	union power_supply_propval result = {0,};

	psy = power_supply_get_by_name("battery");
	psy->get_property(psy, psp, &result);
	return result.intval;
}


static int oem_bms_get_current_ibat(void)
{
	return oem_bms_get_batt_property(POWER_SUPPLY_PROP_CURRENT_NOW);
}


static int oem_bms_get_current_capacity(void)
{
	return oem_bms_get_batt_property(POWER_SUPPLY_PROP_CAPACITY);
}



static void oem_bms_master_data_write(enum power_supply_property psp, int val)
{
	int* masterdata;

	switch(psp) {

	case POWER_SUPPLY_PROP_OEM_REMAINING:
		masterdata = &oem_chg_remaining_time;
		break;


	case POWER_SUPPLY_PROP_OEM_CRITICAL:
		masterdata = &current_batt_health_status;
		break;

	default:
		masterdata = NULL;
		break;
	}

	if (masterdata != NULL) {
		pr_info("psp:%d val=%d \n", psp, val);
		MASTERDATA_LOCK();
		*masterdata = val;
		MASTERDATA_UNLOCK();
	}
}

static int oem_bms_master_data_read(enum power_supply_property psp, int *intval)
{
	int ret = 0;
	int initialized;
	int* masterdata;

	initialized = atomic_read(&is_bms_initialized);

	if (!initialized) {
		pr_err("called before init \n");
		*intval = 0;
		return -EAGAIN;
	}

	switch(psp) {

	case POWER_SUPPLY_PROP_OEM_REMAINING:
		masterdata = &oem_chg_remaining_time;
		break;


	case POWER_SUPPLY_PROP_OEM_CRITICAL:
		masterdata = &current_batt_health_status;
		break;

	default:
		masterdata = NULL;
		break;
	}

	if (masterdata != NULL) {
		MASTERDATA_LOCK();
		*intval = *masterdata;
		MASTERDATA_UNLOCK();
	} else {
		ret = -EINVAL;
	}
	return ret;
}

int oem_bms_pm_batt_power_get_property(enum power_supply_property psp, int *intval)
{
	int ret = 0;

	switch (psp) {

	case POWER_SUPPLY_PROP_OEM_REMAINING:
		ret = oem_bms_master_data_read(psp, intval);
		break;


	case POWER_SUPPLY_PROP_OEM_CRITICAL:
		ret = oem_bms_master_data_read(psp, intval);
		break;

	default:
		ret = -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL(oem_bms_pm_batt_power_get_property);

void oem_bms_init(void)
{
	rbat_avg_mohm = oem_bms_get_rbat_avg();
	oem_bms_remaining_time_reset(REMAINING_INVALID);
	MASTERDATA_SPINLOCK_INIT();
	atomic_set(&is_bms_initialized, 1);
	wake_lock_init(&oem_bms_wake_lock, WAKE_LOCK_SUSPEND, "oem_bms");
}


static void oem_bms_remaining_time_reset(int remaining_time)
{
	remaining_data_count = 0;
	remaining_count = 0;
	remaining_buf_total = 0;
	memset(remaining_ibat_ary, 0, sizeof(remaining_ibat_ary));
	oem_bms_master_data_write(POWER_SUPPLY_PROP_OEM_REMAINING, remaining_time);
	pr_info("initalize remaining_time \n");
}

int oem_bms_remaining_time_calculation(void)
{
	int batt_status;
	int32_t ibat_average;
	int32_t batt_capacity;
	int32_t remaining_time_sec = REMAINING_INVALID;
	int req_changed_remaining_time = 0;

#if ENABLE_CALCULATE_REMAINING_SEC
	int32_t ibat_total = 0;
	int32_t ibat_curr  = 0;
	int32_t ibat_calc = DEFAULT_IBAT_CALCULATE;
#endif/* ENABLE_CALCULATE_REMAINING_SEC */


	batt_status = oem_chg_get_current_batt_status();

	if (batt_status == POWER_SUPPLY_STATUS_DISCHARGING) {

		if (remaining_data_count < REMAINING_IBAT_BUF_SIZE) {
			remaining_data_count++;
		} else {
			remaining_buf_total -= remaining_ibat_ary[remaining_count];
		}


#if ENABLE_CALCULATE_REMAINING_SEC
		ibat_curr = oem_bms_get_current_ibat();
		remaining_ibat_ary[remaining_count] = ibat_curr;
		remaining_buf_total += ibat_curr;
#else/* ENABLE_CALCULATE_REMAINING_SEC */
		remaining_ibat_ary[remaining_count] = oem_bms_get_current_ibat();
		remaining_buf_total += remaining_ibat_ary[remaining_count];
#endif/* ENABLE_CALCULATE_REMAINING_SEC */


		if (remaining_data_count >= REMAINING_IBAT_BUF_SIZE) {
			ibat_average = (int32_t)(remaining_buf_total / REMAINING_IBAT_BUF_SIZE / 1000);

			if (ibat_average) {
				batt_capacity = (int32_t)oem_bms_get_current_capacity();
				remaining_time_sec = ((3600 * REMAINING_CBAT * batt_capacity) / 100) / ibat_average;
			} else {
				remaining_time_sec = 0;
			}

			oem_bms_master_data_write(POWER_SUPPLY_PROP_OEM_REMAINING, remaining_time_sec);
			req_changed_remaining_time = 1;
		}

#if ENABLE_CALCULATE_REMAINING_SEC
		else if (remaining_data_count > 0) {
			ibat_total = remaining_buf_total + (REMAINING_IBAT_BUF_SIZE - remaining_data_count) * ibat_calc;
			ibat_average = (int32_t)(ibat_total / REMAINING_IBAT_BUF_SIZE / 1000);

			if (ibat_average) {
				batt_capacity = (int32_t)oem_bms_get_current_capacity();
				remaining_time_sec = ((3600 * REMAINING_CBAT * batt_capacity) / 100) / ibat_average;
			} else {
				remaining_time_sec = 0;
			}

			if (oem_bms_debug_enable & 0x01) {
				pr_err("(%d):count[%d/%d] total[%d] ibat[%d:%d] remain[%d]\n",
					__LINE__,
					remaining_data_count, REMAINING_IBAT_BUF_SIZE,
					remaining_buf_total,
					ibat_curr,
					ibat_calc,
					remaining_time_sec);
			}
			oem_bms_master_data_write(POWER_SUPPLY_PROP_OEM_REMAINING, remaining_time_sec);
			req_changed_remaining_time = 1;
		}
#endif/* ENABLE_CALCULATE_REMAINING_SEC */


		remaining_count = (remaining_count + 1) % REMAINING_IBAT_BUF_SIZE;

	} else {
		if (0 != remaining_data_count) {
			oem_bms_remaining_time_reset(remaining_time_sec);
			req_changed_remaining_time = 1;
		}
	}
	return req_changed_remaining_time;
}
EXPORT_SYMBOL(oem_bms_remaining_time_calculation);


static int oem_bms_bound_soc_in_charging(int soc)
{
	soc = max(LOWER_LIMIT_SOC_CHARGING, soc);
	soc = min(UPPER_LIMIT_SOC_CHARGING, soc);
	return soc;
}

static int oem_bms_bound_soc_in_discharging(int soc)
{
	int in_soc = soc;
	soc = max(LOWER_LIMIT_SOC_DISCHARGING, soc);
	soc = min(UPPER_LIMIT_SOC_DISCHARGING, soc);
	if (in_soc != soc)
		pr_err("soc convert %d to %d\n", in_soc, soc);
	return soc;
}


static atomic_t catch_up_full = ATOMIC_INIT(0);
static int oem_bms_get_catch_up_full(void)
{
	return atomic_read(&catch_up_full);
}

void oem_bms_set_catch_up_full(int on_off)
{
	atomic_set(&catch_up_full, on_off);
}
EXPORT_SYMBOL(oem_bms_set_catch_up_full);


static int pre_state = POWER_SUPPLY_STATUS_DISCHARGING;
static int pre_state_result_soc = 0;
static int new_state_in_soc = 0;
static int pre_result_soc = 0;
static int pre_in_soc = 0;
static int pre_calc_soc = 0;
static int base_calc_soc = 0;
static uint16_t pre_ocv_raw = 0;
static bool catch_up_in_soc_monitor = false;
static int check_in_soc = 0;
static int in_soc_match_cnt = 0;
static int pre_elapsed_time = 0;
static int shutdown_base_time = 0;
static bool shutdown_alert = false;
static int filter_base_time = 0;
static bool is_initialized = false;
int oem_bms_correct_soc(int in_soc, int calc_soc, uint16_t ocv_raw)
{
	struct timespec ts;
	static struct timespec last_ts;
	int elapsed_time;
	int filter_check_time = SOC_FILTER_TIME_SEC;
	int result_soc = in_soc;
	int pre_state_result_soc_work = pre_state_result_soc;
	int new_state_in_soc_work = new_state_in_soc;
	int alpha = 0;
	int round;
	int batt_uv = oem_bms_get_batt_property(POWER_SUPPLY_PROP_VOLTAGE_NOW);
	int state = oem_bms_get_batt_property(POWER_SUPPLY_PROP_STATUS);

	if (!is_initialized) {
		pre_state_result_soc = in_soc;
		new_state_in_soc = in_soc;
		pre_result_soc = in_soc;
		pre_in_soc = in_soc;
		pre_calc_soc = calc_soc;
		pre_ocv_raw = ocv_raw;
		check_in_soc = in_soc;
		getnstimeofday(&last_ts);
		shutdown_base_time = last_ts.tv_sec;
		shutdown_alert = false;
		filter_base_time = last_ts.tv_sec;
		is_initialized = true;
		pr_info("initialized in_soc:%d\n", in_soc);
	}

	if (pre_state != state) {
		pr_info("update pre_state_result_soc %d to %d new_state_in_soc %d to %d\n",
			pre_state_result_soc, pre_result_soc, new_state_in_soc, in_soc);

		pre_state_result_soc = pre_result_soc;
		new_state_in_soc = in_soc;

		if ((state == POWER_SUPPLY_STATUS_DISCHARGING) ||
			(state == POWER_SUPPLY_STATUS_CHARGING)) {
			in_soc_match_cnt = 0;
			pre_elapsed_time = 0;
			check_in_soc = in_soc;
			getnstimeofday(&last_ts);
			shutdown_base_time = last_ts.tv_sec;
			shutdown_alert = false;
			filter_base_time = last_ts.tv_sec;
			oem_bms_set_catch_up_full(0);
			catch_up_in_soc_monitor = false;
			wake_lock_timeout(&oem_bms_wake_lock, msecs_to_jiffies((IN_SOC_MONITOR_TIME_SEC + 1) * 1000));
			pr_info("in_soc monitor start wake_lock_timeout:%dsec\n", IN_SOC_MONITOR_TIME_SEC + 1);
		} else {
			wake_unlock(&oem_bms_wake_lock);
		}
	}

	if ((state == POWER_SUPPLY_STATUS_DISCHARGING) ||
		(state == POWER_SUPPLY_STATUS_CHARGING)) {

		if (wake_lock_active(&oem_bms_wake_lock)) {
			getnstimeofday(&ts);
			elapsed_time = ts.tv_sec - last_ts.tv_sec;

			if (elapsed_time > pre_elapsed_time) {
				if(check_in_soc == in_soc) {
					in_soc_match_cnt++;
				} else {
					in_soc_match_cnt = 0;
				}

				if ((in_soc_match_cnt >= IN_SOC_MATCH_CNT) ||
					(elapsed_time >= IN_SOC_MONITOR_TIME_SEC)) {
					pr_info("new_state_in_soc update:%d to %d\n", new_state_in_soc, in_soc);

					new_state_in_soc = in_soc;
					wake_unlock(&oem_bms_wake_lock);
				}
				pr_info("in_soc_monitor\n state old:%d new:%d batt_mv:%d\n result_soc pre_st:%d old:%d\n in_soc new_state:%d old:%d new:%d\n check_in_soc:%d match_cnt:%d elapsed_time:%d wake_lock:%d\n",
					pre_state, state, batt_uv/1000,
					pre_state_result_soc, pre_result_soc,
					new_state_in_soc, pre_in_soc, in_soc,
					check_in_soc, in_soc_match_cnt, elapsed_time,
					wake_lock_active(&oem_bms_wake_lock));

				pre_elapsed_time = elapsed_time;
				check_in_soc = in_soc;
			}
		} else {
			if (state == POWER_SUPPLY_STATUS_DISCHARGING) {
				if (catch_up_in_soc_monitor) {
					if (calc_soc <= in_soc) {
						pr_info("catch up in_soc\n pre_state_result_soc %d to %d\n new_state_in_soc %d to %d\n",
							pre_state_result_soc, pre_result_soc, new_state_in_soc, in_soc);

						pre_state_result_soc = pre_result_soc;
						new_state_in_soc = in_soc;
						catch_up_in_soc_monitor = false;
					} else if (calc_soc > base_calc_soc) {
						/* calc_soc larger than base_calc_soc */
					} else if ((calc_soc - base_calc_soc) <= CALC_SOC_DOWN_DIFF) {
						pr_info("calc_soc change\n pre_state_result_soc %d to %d\n new_state_in_soc %d to %d\n",
							pre_state_result_soc, pre_result_soc, new_state_in_soc, calc_soc);
						pr_info("base_calc_soc %d to %d\n", base_calc_soc, calc_soc);

						pre_state_result_soc = pre_result_soc;
						new_state_in_soc = calc_soc;
						base_calc_soc = calc_soc;
					} else {
						base_calc_soc = calc_soc;
					}
				} else if (calc_soc > in_soc) {
					pr_info("calc_soc larger than in_soc\n pre_state_result_soc %d to %d\n new_state_in_soc %d to %d\n",
						pre_state_result_soc, pre_result_soc, new_state_in_soc, calc_soc);
					pr_info("base_calc_soc %d to %d\n", base_calc_soc, calc_soc);

					pre_state_result_soc = pre_result_soc;
					new_state_in_soc = calc_soc;
					base_calc_soc = calc_soc;
					catch_up_in_soc_monitor = true;

					pr_info("ocv_raw old:%d new:%d calc_soc old:%d new:%d in_soc:%d\n",
						pre_ocv_raw, ocv_raw, pre_calc_soc, calc_soc, in_soc);
				}
			}

			if ((!catch_up_in_soc_monitor) &&
				((pre_in_soc - in_soc) >= IN_SOC_CHECK_DIFF)) {
				pr_info("diff is larger than %d update\n pre_state_result_soc %d to %d\n new_state_in_soc %d to %d pre_in_soc:%d\n",
					IN_SOC_CHECK_DIFF, pre_state_result_soc, pre_result_soc, new_state_in_soc, in_soc, pre_in_soc);

				pre_state_result_soc = pre_result_soc;
				new_state_in_soc = in_soc;
			}
		}
	}

	switch (state) {

	case POWER_SUPPLY_STATUS_FULL:
		result_soc = OEM_SOC_FULL;
		break;

	case POWER_SUPPLY_STATUS_CHARGING:
		if (pre_state_result_soc > CORRECT_SOC_MAX_CHG) {
			pre_state_result_soc_work = CORRECT_SOC_MAX_CHG;
		} else {
			pre_state_result_soc_work = pre_state_result_soc;
		}

		if (new_state_in_soc > CORRECT_SOC_MAX_CHG) {
			new_state_in_soc_work = CORRECT_SOC_MAX_CHG;
		} else {
			new_state_in_soc_work = new_state_in_soc;
		}

		alpha = abs(pre_state_result_soc_work - new_state_in_soc_work);

		if (wake_lock_active(&oem_bms_wake_lock)) {
			result_soc = pre_state_result_soc;
		} else if (alpha > CORRECT_DIFF_MAX) {
			result_soc = in_soc;
			pr_err("alpha is invalid %d\n", alpha);
		} else if (new_state_in_soc_work > in_soc) {
			round = new_state_in_soc_work / 2;
			result_soc = (pre_state_result_soc_work * in_soc + round) / new_state_in_soc_work;
		} else {
			round = (SOC_MAX - new_state_in_soc_work) / 2;
			result_soc = SOC_MAX - ((SOC_MAX - pre_state_result_soc_work) * (SOC_MAX - in_soc) + round) / (SOC_MAX - new_state_in_soc_work);
		}

		if (oem_bms_get_catch_up_full()) {
			getnstimeofday(&ts);
			elapsed_time = ts.tv_sec - filter_base_time;

			if (elapsed_time >= filter_check_time) {
				pr_info("filter_soc:%d result_soc:%d pre_result_soc:%d elapsed_time:%d check_time:%d\n",
					pre_result_soc + 1, result_soc, pre_result_soc, elapsed_time, filter_check_time);
				result_soc = pre_result_soc + 1;
				filter_base_time = ts.tv_sec;

				pr_info("catch_up_full update\n pre_state_result_soc %d to %d\n new_state_in_soc %d to %d\n",
					pre_state_result_soc, result_soc, new_state_in_soc, in_soc);
				oem_bms_set_catch_up_full(0);
				pre_state_result_soc = result_soc;
				new_state_in_soc = in_soc;
			} else {
				result_soc = pre_result_soc;
			}
		} else if (result_soc > pre_result_soc) {
			getnstimeofday(&ts);
			elapsed_time = ts.tv_sec - filter_base_time;

			if (pre_result_soc >= CV_CHG_CHECK_SOC) {
				filter_check_time = CV_SOC_FILTER_TIME_SEC;
			}

			if (elapsed_time >= filter_check_time) {
				pr_info("filter_soc:%d result_soc:%d pre_result_soc:%d elapsed_time:%d check_time:%d\n",
					pre_result_soc + 1, result_soc, pre_result_soc, elapsed_time, filter_check_time);
				result_soc = pre_result_soc + 1;
				filter_base_time = ts.tv_sec;
			} else {
				result_soc = pre_result_soc;
			}
		}

		result_soc = oem_bms_bound_soc_in_charging(result_soc);
		break;

	case POWER_SUPPLY_STATUS_DISCHARGING:
		if (pre_state_result_soc < CORRECT_SOC_MAX_DISCHG) {
			pre_state_result_soc_work = CORRECT_SOC_MAX_DISCHG;
		} else {
			pre_state_result_soc_work = pre_state_result_soc;
		}

		if (new_state_in_soc < CORRECT_SOC_MAX_DISCHG) {
			new_state_in_soc_work = CORRECT_SOC_MAX_DISCHG;
		} else {
			new_state_in_soc_work = new_state_in_soc;
		}

		alpha = abs(pre_state_result_soc_work - new_state_in_soc_work);

		if (wake_lock_active(&oem_bms_wake_lock)) {
			result_soc = pre_state_result_soc;
		} else if (alpha > CORRECT_DIFF_MAX) {
			result_soc = in_soc;
			pr_err("alpha is invalid %d\n", alpha);
		} else if (catch_up_in_soc_monitor) {
			if (calc_soc > base_calc_soc) {
				result_soc = pre_result_soc;
			} else {
				round = new_state_in_soc_work / 2;
				result_soc = (pre_state_result_soc_work * calc_soc + round) / new_state_in_soc_work;
			}
		} else {
			round = new_state_in_soc_work / 2;
			result_soc = (pre_state_result_soc_work * in_soc + round) / new_state_in_soc_work;
		}

		result_soc = oem_bms_bound_soc_in_discharging(result_soc);

		if (batt_uv <= OEM_SHUT_DOWN_UV) {
			getnstimeofday(&ts);

			if (!shutdown_alert) {
				shutdown_base_time = ts.tv_sec;
				shutdown_alert = true;
			}
			elapsed_time = ts.tv_sec - shutdown_base_time;
			pr_info("low battery result_soc:%d in_soc:%d batt_mv:%d elapsed_time:%d\n",
				result_soc, in_soc, batt_uv/1000, elapsed_time);

			if (elapsed_time >= SHUT_DOWN_MONITOR_TIME_SEC) {
				result_soc = OEM_SOC_SHUT_DOWN;
				pr_err("to shut down, because battery is low voltage\n");
			}
		} else {
			shutdown_alert = false;
		}
		break;

	default:
		pr_err("unknown POWER_SUPPLY_STATUS:%d\n", state);
		break;
	}

	if ((pre_result_soc != result_soc)	||
		(pre_in_soc != in_soc)	||
		(pre_calc_soc != calc_soc)	||
		(pre_ocv_raw != ocv_raw)	||
		(pre_state != state)) {
		pr_info("result_soc:%d A:%d B:%d X:%d alpha:%d\n",
			result_soc, pre_state_result_soc_work, new_state_in_soc_work, in_soc, alpha);
		pr_info("change\n state old:%d new:%d batt_mv:%d\n result_soc pre_st:%d old:%d new:%d\n in_soc new_state:%d old:%d new:%d\n",
			pre_state, state, batt_uv/1000,
			pre_state_result_soc, pre_result_soc, result_soc,
			new_state_in_soc, pre_in_soc, in_soc);
		pr_info("ocv_raw old:%d new:%d calc_soc old:%d new:%d in_soc:%d catch_up_in_soc_monitor:%d\n",
			pre_ocv_raw, ocv_raw, pre_calc_soc, calc_soc, in_soc, catch_up_in_soc_monitor);
	}
	pre_result_soc = result_soc;
	pre_in_soc = in_soc;
	pre_calc_soc = calc_soc;
	pre_ocv_raw = ocv_raw;
	pre_state = state;

	return result_soc;
}
EXPORT_SYMBOL(oem_bms_correct_soc);



static void oem_bms_reset_rbat_data(void)
{
	int i;

	for (i = 0; i < BATT_HEALTH_RBAT_SAMPLES; i++) {
		oem_param_charger.rbat_data[i] = 0;
	}
	kdnand_id_write(CHG_DNAND_CMD,
		CHG_DNAND_RBAT_DATA,
		(uint8_t*)&oem_param_charger.rbat_data[0],
		(sizeof(oem_param_charger.rbat_data[0]) * BATT_HEALTH_RBAT_SAMPLES));

	oem_param_charger.critical_batt = OEM_BMS_BATT_HEALTH_NORMAL;
	kdnand_id_write(CHG_DNAND_CMD,
		CHG_DNAND_CRITICAL_BATT,
		(uint8_t*)&oem_param_charger.critical_batt, 1);

	oem_param_charger.rbat_data_idx = 0;
	kdnand_id_write(CHG_DNAND_CMD,
		CHG_DNAND_RBAT_IDX,
		(uint8_t*)&oem_param_charger.rbat_data_idx, 1);

	pr_info("called\n");
}

static void oem_bms_set_rbat_data(int rbat_mohm)
{
	int i;

	oem_param_charger.rbat_data[oem_param_charger.rbat_data_idx] = rbat_mohm;
	kdnand_id_write(CHG_DNAND_CMD,
		CHG_DNAND_RBAT_DATA + oem_param_charger.rbat_data_idx * 2,
		(uint8_t*)&oem_param_charger.rbat_data[oem_param_charger.rbat_data_idx], 2);

	oem_param_charger.rbat_data_idx++;
	if (oem_param_charger.rbat_data_idx == BATT_HEALTH_RBAT_SAMPLES) {
		oem_param_charger.rbat_data_idx = 0;
	}

	kdnand_id_write(CHG_DNAND_CMD,
		CHG_DNAND_RBAT_IDX,
		&oem_param_charger.rbat_data_idx, 1);

	for (i = 0; i < BATT_HEALTH_RBAT_SAMPLES; i++) {
		pr_info("rbat_data [%d]:%dmohm\n", i, oem_param_charger.rbat_data[i]);
	}
	pr_info("next idx:%d\n", oem_param_charger.rbat_data_idx);
}

static void oem_bms_set_critical_batt(uint8_t critical_batt)
{
	oem_param_charger.critical_batt = critical_batt;
	kdnand_id_write(CHG_DNAND_CMD,
		CHG_DNAND_CRITICAL_BATT,
		(uint8_t*)&oem_param_charger.critical_batt, 1);

	oem_bms_master_data_write(POWER_SUPPLY_PROP_OEM_CRITICAL,
		oem_param_charger.critical_batt);

	pr_info(":%d\n", oem_param_charger.critical_batt);
}

static int oem_bms_get_rbat_avg(void)
{
	int i;
	int total_mohm = 0;
	int avg_mohm = 0;

	rbat_avg_valid = true;

	for (i = 0; i < BATT_HEALTH_RBAT_SAMPLES; i++) {
		if ((oem_param_charger.rbat_data[i] != 0x0000) &&
			(oem_param_charger.rbat_data[i] != 0xFFFF)) {
			total_mohm += oem_param_charger.rbat_data[i];
		} else {
			rbat_avg_valid = false;
			break;
		}
	}

	if (i) {
		avg_mohm = total_mohm / i;
	}
	pr_info("rbat_avg mohm:%d samples:%d valid:%d\n",
		avg_mohm, i, rbat_avg_valid);

	return avg_mohm;
}

static int oem_bms_battery_health_check(void)
{
	static int vbat1_uv = 0;
	static int vbat2_uv = 0;
	static int ibat1_ua = 0;
	static int rbat_mohm = 0;
	static int check_count = 0;
	int req_ps_changed = false;
	int chg_connect = oem_chg_get_current_chg_connect();
	int batt_volt = oem_chg_get_current_batt_voltage();
	int batt_temp = oem_chg_get_current_batt_temp();


	if ((batt_temp >= BATT_TEMP_HOT_CHG_OFF) ||
		(batt_temp <= BATT_TEMP_COLD_CHG_OFF)) {
		batt_health_check_st = BATT_HEALTH_CHECK_END;
	}

	if (chg_connect == POWER_SUPPLY_OEM_CONNECT_NONE) {
		/* charger is not connected */
		if (batt_health_check_st == BATT_HEALTH_CHECK_GET) {
			oem_chg_set_charging();
		}
		batt_health_check_st = BATT_HEALTH_CHECK_NO_CHG;
	}

	switch (batt_health_check_st) {
		case BATT_HEALTH_CHECK_NO_CHG:
			if (chg_connect != POWER_SUPPLY_OEM_CONNECT_NONE) {
				if (batt_volt <= BATT_HEALTH_CHECK_VOLT_UV) {
					batt_health_check_st = BATT_HEALTH_CHECK_WAIT;
				} else {
					batt_health_check_st = BATT_HEALTH_CHECK_END;
				}
			}
			break;

		case BATT_HEALTH_CHECK_WAIT:
			if (batt_temp <= BATT_HEALTH_TEMP_COLD) {
				/* battery temperature is cold */
				batt_health_check_st = BATT_HEALTH_CHECK_END;
			} else if (batt_volt >= BATT_HEALTH_CALC_VOLT_UV) {
				vbat1_uv = oem_hkadc_get_vbat_latest();
				ibat1_ua = oem_bms_get_batt_property(POWER_SUPPLY_PROP_CURRENT_NOW);
				batt_health_check_st = BATT_HEALTH_CHECK_GET;
				oem_chg_set_charger_power_supply();
				check_count = 0;
			}
			break;

		case BATT_HEALTH_CHECK_GET:
			check_count++;
			if (check_count >= BAT_FET_OFF_WAIT_CNT) {
				vbat2_uv = oem_hkadc_get_vbat_latest();
				oem_chg_set_charging();
				batt_health_check_st = BATT_HEALTH_CHECK_CALC;
			}
			break;

		case BATT_HEALTH_CHECK_CALC:
			rbat_mohm = abs(((vbat1_uv - vbat2_uv) * 1000) / ibat1_ua);

			if (abs(rbat_avg_mohm - rbat_mohm) >= BATT_HEALTH_RBAT_CHECK_MOHM) {
				oem_bms_reset_rbat_data();
			}
			oem_bms_set_rbat_data(rbat_mohm);
			rbat_avg_mohm = oem_bms_get_rbat_avg();

			if (rbat_avg_valid &&
				(rbat_avg_mohm >= BATT_HEALTH_RBAT_ERROR_MOHM)) {
				oem_bms_set_critical_batt(OEM_BMS_BATT_HEALTH_CRITICAL);
			} else {
				oem_bms_set_critical_batt(OEM_BMS_BATT_HEALTH_NORMAL);
			}
			batt_health_check_st = BATT_HEALTH_CHECK_END;
			req_ps_changed = true;

			pr_info("rbat_mohm:%d vbat1_uv:%d vbat2_uv:%d ibat1_ua:%d\n",
				rbat_mohm, vbat1_uv, vbat2_uv, ibat1_ua);
			break;

		case BATT_HEALTH_CHECK_END:
			/* no processing */
			break;

		default:
			break;
	}
	if (last_batt_health_check_st != batt_health_check_st) {
		pr_info("st:%d connect:%d volt:%dmV temp:%dC\n",
			batt_health_check_st, chg_connect, batt_volt/1000, batt_temp/10);
	}
	last_batt_health_check_st = batt_health_check_st;
	return req_ps_changed;
}

int oem_bms_get_batt_health_check_st(void)
{
	return batt_health_check_st;
}
EXPORT_SYMBOL(oem_bms_get_batt_health_check_st);

int oem_bms_control(void)
{
	int req_ps_changed_batt = false;

	req_ps_changed_batt |= oem_bms_remaining_time_calculation();
	
	if (g_enable_oem_bms_battery_check) {
		req_ps_changed_batt |= oem_bms_battery_health_check();
	}
	
	return req_ps_changed_batt;
}
EXPORT_SYMBOL(oem_bms_control);


#define DETERIORATION_THRESH_GOOD_TO_NORMAL(x)	((x * 80)/ 100)
#define DETERIORATION_THRESH_NORMAL_TO_GOOD(x)	((x * 90)/ 100)
#define DETERIORATION_THRESH_NORMAL_TO_DEAD(x)	((x * 60)/ 100)
#define DETERIORATION_THRESH_DEAD_TO_NORMAL(x)	((x * 70)/ 100)

int oem_bms_get_deteriorationstatus(int *batt_deterioration_status, int new_fcc_mah, int default_fcc_mah)
{
	BMS_LOG("batt_deterioration_status[%d], new_fcc_mah[%d], default_fcc_mah[%d]\n",
					*batt_deterioration_status, new_fcc_mah, default_fcc_mah);
	switch( *batt_deterioration_status )
	{
		case BATT_DETERIORATION_GOOD:
			if (new_fcc_mah >= DETERIORATION_THRESH_GOOD_TO_NORMAL(default_fcc_mah)) {
				*batt_deterioration_status = BATT_DETERIORATION_GOOD;
			} else {
				*batt_deterioration_status = BATT_DETERIORATION_NORMAL;
			}
			break;
		case BATT_DETERIORATION_NORMAL:
			if (new_fcc_mah >= DETERIORATION_THRESH_NORMAL_TO_GOOD(default_fcc_mah)) {
				*batt_deterioration_status = BATT_DETERIORATION_GOOD;
			} else if (new_fcc_mah >= DETERIORATION_THRESH_NORMAL_TO_DEAD(default_fcc_mah)) {
				*batt_deterioration_status = BATT_DETERIORATION_NORMAL;
			} else {
				*batt_deterioration_status = BATT_DETERIORATION_DEAD;
				oem_bms_set_critical_batt(OEM_BMS_BATT_HEALTH_CRITICAL);
			}
			break;
		case BATT_DETERIORATION_DEAD:
			if (new_fcc_mah >= DETERIORATION_THRESH_DEAD_TO_NORMAL(default_fcc_mah)) {
				*batt_deterioration_status = BATT_DETERIORATION_NORMAL;
			} else {
				*batt_deterioration_status = BATT_DETERIORATION_DEAD;
			}
			break;
		default:
			break;
	}
	oem_param_deterioration_status_backup(*batt_deterioration_status);

	return *batt_deterioration_status;
}

