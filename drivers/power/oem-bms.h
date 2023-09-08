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

#ifndef __OEM_BMS_H
#define __OEM_BMS_H


#include <linux/power_supply.h>/* for enum power_supply_property */
enum batt_deterioration_type {
	BATT_DETERIORATION_GOOD,
	BATT_DETERIORATION_NORMAL,
	BATT_DETERIORATION_DEAD,
};


#define SOC_MAX		100
#define CORRECT_SOC_MAX_CHG			99
#define CORRECT_SOC_MAX_DISCHG		1

#define CORRECT_DIFF_MAX			100


#define IN_SOC_MONITOR_TIME_SEC		60
#define IN_SOC_MATCH_CNT			5
#define IN_SOC_CHECK_DIFF			2

#define SOC_FILTER_TIME_SEC			60
#define FULL_CHG_CHECK_SOC			99
#define CV_SOC_FILTER_TIME_SEC		(7 * 60)
#define CV_CHG_CHECK_SOC			96
#define CALC_SOC_DOWN_DIFF			(-2)

#define CHG_DNAND_CMD					18
#define CHG_DNAND_CRITICAL_BATT			8
#define CHG_DNAND_RBAT_IDX				9
#define CHG_DNAND_RBAT_DATA				10
#define BATT_HEALTH_RBAT_SAMPLES		10
#define BATT_HEALTH_CHECK_VOLT_UV		4000000
#define BATT_HEALTH_CALC_VOLT_UV		4100000
#define BATT_HEALTH_RBAT_ERROR_MOHM		1000
#define BATT_HEALTH_RBAT_CHECK_MOHM		600
#define BATT_HEALTH_TEMP_COLD			100
#define BAT_FET_OFF_WAIT_CNT			2


enum oem_bms_batt_health_status {
	OEM_BMS_BATT_HEALTH_NORMAL,
	OEM_BMS_BATT_HEALTH_CRITICAL
};

enum batt_health_check_status {
	BATT_HEALTH_CHECK_NO_CHG,
	BATT_HEALTH_CHECK_WAIT,
	BATT_HEALTH_CHECK_GET,
	BATT_HEALTH_CHECK_CALC,
	BATT_HEALTH_CHECK_END
};


/**
 * oem_bms_correct_soc - readjustment oem soc
 */
extern int oem_bms_correct_soc(int in_soc, int calc_soc, uint16_t ocv_raw);
int oem_bms_get_deteriorationstatus(int *batt_deterioration_status, int new_fcc_mah, int default_fcc_mah);

extern int oem_bms_control(void);
extern int oem_bms_get_batt_health_check_st(void);


extern void oem_bms_init(void);
extern int oem_bms_pm_batt_power_get_property(enum power_supply_property psp, int *intval);
extern int oem_bms_remaining_time_calculation(void);
extern void oem_bms_set_catch_up_full(int on_off);
#endif
