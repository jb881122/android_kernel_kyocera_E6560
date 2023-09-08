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

#include <linux/debugfs.h>
#include <linux/slab.h>
#include <mach/msm_smsm.h>
#include <oem-chg_param.h>
#include <mach/oem_fact.h>

#include <linux/dnand_cdev_driver.h>
#include <oem-bms.h>



int oem_option_item1_bit0 = 0;
int oem_option_item1_bit1 = 0;
int oem_option_item1_bit2 = 0;
int oem_option_item1_bit3 = 0;
int oem_option_item1_bit4 = 0;

static void oem_get_fact_option(void)
{
	oem_option_item1_bit0 = oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 0);
	oem_option_item1_bit1 = oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 1);
	oem_option_item1_bit2 = oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 2);
	oem_option_item1_bit3 = oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 3);
	oem_option_item1_bit4 = oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 4);

	pr_info("factory option item1 bit 0:%d 1:%d 2:%d 3:%d 4:%d\n",
		oem_option_item1_bit0, oem_option_item1_bit1, oem_option_item1_bit2,
		oem_option_item1_bit3, oem_option_item1_bit4);
}

oem_chg_param_hkadc_type oem_param_hkadc = {
	.cal_vbat1 = 0x0010C8E0,
	.cal_vbat2 = 0x00155CC0
};

oem_chg_param_charger_type oem_param_charger = {

	.critical_batt	= 0x00,		/* offset 8 */
	.rbat_data_idx	= 0x00,		/* offset 9 */
	.rbat_data[0]	= 0x0000,	/* offset 10,11 */
	.rbat_data[1]	= 0x0000,	/* offset 12,13 */
	.rbat_data[2]	= 0x0000,	/* offset 14,15 */
	.rbat_data[3]	= 0x0000,	/* offset 16,17 */
	.rbat_data[4]	= 0x0000,	/* offset 18,19 */
	.rbat_data[5]	= 0x0000,	/* offset 20,21 */
	.rbat_data[6]	= 0x0000,	/* offset 22,23 */
	.rbat_data[7]	= 0x0000,	/* offset 24,25 */
	.rbat_data[8]	= 0x0000,	/* offset 26,27 */
	.rbat_data[9]	= 0x0000,	/* offset 28,29 */

	.reserve2_4 = 0xFFFF,

	.pma_rxid[0] = 0x00,		/* offset 32 */
	.pma_rxid[1] = 0x00,		/* offset 33 */
	.pma_rxid[2] = 0x00,		/* offset 34 */
	.pma_rxid[3] = 0x00,		/* offset 35 */
	.pma_rxid[4] = 0x00,		/* offset 36 */
	.pma_rxid[5] = 0x00,		/* offset 37 */

	.reserve2_8 = 0xFFFF,
	.reserve3_1 = 0xFFFF,
	.reserve3_2 = 0xFFFF,
	.reserve3_3 = 0xFFFF,
	.reserve3_4 = 0xFFFF,
	.reserve3_5 = 0xFFFF,
	.reserve3_6 = 0xFFFF,
	.reserve3_7 = 0xFFFF,
	.reserve3_8 = 0xFFFF,
	.reserve4_1 = 0xFFFF,
	.reserve4_2 = 0xFFFF,
	.reserve4_3 = 0xFFFF,
	.reserve4_4 = 0xFFFF,
	.reserve4_5 = 0xFFFF,
	.reserve4_6 = 0xFFFF,
	.reserve4_7 = 0xFFFF,
	.reserve4_8 = 0xFFFF,
	.reserve5_1 = 0xFFFF,
	.reserve5_2 = 0xFFFF,
	.reserve5_3 = 0xFFFF,
	.reserve5_4 = 0xFFFF,
	.reserve5_5 = 0xFFFF,
	.reserve5_6 = 0xFFFF,
	.reserve5_7 = 0xFFFF,
	.reserve5_8 = 0xFFFF,
	.reserve6_1 = 0xFFFF,
	.reserve6_2 = 0xFFFF,
	.reserve6_3 = 0xFFFF,
	.reserve6_4 = 0xFFFF,
	.reserve6_5 = 0xFFFF,
	.reserve6_6 = 0xFFFF,
	.reserve6_7 = 0xFFFF,
	.reserve6_8 = 0xFFFF,
	.reserve7_1 = 0xFFFF,
	.reserve7_2 = 0xFFFF,
	.reserve7_3 = 0xFFFF,
	.reserve7_4 = 0xFFFF,
	.reserve7_5 = 0xFFFF,
	.reserve7_6 = 0xFFFF,
	.reserve7_7 = 0xFFFF,
	.reserve7_8 = 0xFFFF,
	.reserve8_1 = 0xFFFF,
	.reserve8_2 = 0xFFFF,
	.reserve8_3 = 0xFFFF,
	.reserve8_4 = 0xFFFF,
	.reserve8_5 = 0xFFFF,
	.reserve8_6 = 0xFFFF,
	.reserve8_7 = 0xFFFF,
	.reserve8_8 = 0xFFFF,

	/* the bottom is used by oem_param_cycles for 32 bytes from here. */

};

oem_chg_param_share_type oem_param_share = {
	.factory_mode_1 = 0x00,
	.test_mode_chg_disable = 0x00,
	.cool_down_mode_disable = 0x00,	
	.reserve1_4 = 0xFF,
	.reserve1_5 = 0xFF,
	.reserve1_6 = 0xFF,
	.reserve1_7 = 0xFF,
	.reserve1_8 = 0xFF
};


oem_chg_param_cycles_type oem_param_cycles = {
	.last_chargecycles = 0x0000,
	.last_charge_increase = 0x00,
	.reserved = 0xFF,
	.batt_deteriorationstatus = 0x00000000,
	.fcc = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
	.chgcyl = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
	.reserved2 ={0xFF, 0xFF, 0xFF, 0xFF,
				 0xFF, 0xFF, 0xFF, 0xFF,
				 0xFF, 0xFF, 0xFF, 0xFF,
				 0xFF, 0xFF}
};
typedef struct {
	oem_chg_param_cycles_type		oem_chg_param_cycles;
} oem_chg_param_c_type;


typedef struct {
	oem_chg_param_hkadc_type		oem_chg_param_hkadc;
	oem_chg_param_charger_type		oem_chg_param_charger;
	oem_chg_param_share_type		oem_chg_param_share;
} oem_chg_param_type;

uint8_t oem_cmp_zero_flag = 0;
static void oem_param_hkadc_init(oem_chg_param_type *ptr)
{
	if (ptr == NULL){
		pr_err("chg param hkadc read error.\n");
		return;
	}

	SET_CHG_PARAM2(0x00000000, 0xFFFFFFFF, hkadc, cal_vbat1);
	SET_CHG_PARAM2(0x00000000, 0xFFFFFFFF, hkadc, cal_vbat2);
}

static void oem_param_charger_init(oem_chg_param_type *ptr)
{

	if (ptr == NULL){
		pr_err("chg param charger read error.\n");
		return;
	}

	SET_CHG_PARAM(0xFF, charger, critical_batt);
	SET_CHG_PARAM(0xFF, charger, rbat_data_idx);
	SET_CHG_PARAM(0xFFFF, charger, rbat_data[0]);
	SET_CHG_PARAM(0xFFFF, charger, rbat_data[1]);
	SET_CHG_PARAM(0xFFFF, charger, rbat_data[2]);
	SET_CHG_PARAM(0xFFFF, charger, rbat_data[3]);
	SET_CHG_PARAM(0xFFFF, charger, rbat_data[4]);
	SET_CHG_PARAM(0xFFFF, charger, rbat_data[5]);
	SET_CHG_PARAM(0xFFFF, charger, rbat_data[6]);
	SET_CHG_PARAM(0xFFFF, charger, rbat_data[7]);
	SET_CHG_PARAM(0xFFFF, charger, rbat_data[8]);
	SET_CHG_PARAM(0xFFFF, charger, rbat_data[9]);


	SET_CHG_PARAM2(0x00, 0xFF, charger, pma_rxid[0]);
	SET_CHG_PARAM2(0x00, 0xFF, charger, pma_rxid[1]);
	SET_CHG_PARAM2(0x00, 0xFF, charger, pma_rxid[2]);
	SET_CHG_PARAM2(0x00, 0xFF, charger, pma_rxid[3]);
	SET_CHG_PARAM2(0x00, 0xFF, charger, pma_rxid[4]);
	SET_CHG_PARAM2(0x00, 0xFF, charger, pma_rxid[5]);
	pr_info("pma_rxid:0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x\n",
		oem_param_charger.pma_rxid[0],
		oem_param_charger.pma_rxid[1],
		oem_param_charger.pma_rxid[2],
		oem_param_charger.pma_rxid[3],
		oem_param_charger.pma_rxid[4],
		oem_param_charger.pma_rxid[5]);

}

static void oem_param_share_init(oem_chg_param_type *ptr)
{
	if (ptr == NULL){
		pr_err("chg param share read error.\n");
		return;
	}

	memcpy(&oem_param_share, &ptr->oem_chg_param_share,
				sizeof(oem_chg_param_share_type));
}


static int oem_param_cycles_offset(void)
{
	int offset = CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE - sizeof(oem_chg_param_cycles_type);

	return offset;
}

void oem_param_cycles_dump(const char* msg)
{
	const oem_chg_param_cycles_type* ptr = &oem_param_cycles;
	int size = sizeof(*ptr);
	int offset = oem_param_cycles_offset();

	printk("%s(%d):>%s size=%d offset=%d\n", __func__, __LINE__, msg, size, offset);

	printk("last_chargecycles=%04x last_charge_increase=%02x batt_deteriorationstatus=%08x\n",
			ptr->last_chargecycles,
			ptr->last_charge_increase,
			ptr->batt_deteriorationstatus);
	printk("fcc=[%02x,%02x,%02x,%02x,%02x] cycle=[%02x,%02x,%02x,%02x,%02x]\n",
			ptr->fcc[0], ptr->fcc[1], ptr->fcc[2], ptr->fcc[3], ptr->fcc[4],
			ptr->chgcyl[0], ptr->chgcyl[1], ptr->chgcyl[2], ptr->chgcyl[3], ptr->chgcyl[4]);
}

static void oem_param_cycles_init(oem_chg_param_type *ptr)
{
	int offset = oem_param_cycles_offset();
	uint8_t* cycles_ptr = 0;

	if (ptr == NULL){
		pr_err("chg param cycles read error.\n");
		return;
	}

	cycles_ptr = (uint8_t*)ptr + offset;

	memcpy(&oem_param_cycles, cycles_ptr,
				sizeof(oem_chg_param_cycles_type));

	printk("%s(%d):cycles_ptr=%08x offset=%d\n",
		 __func__, __LINE__,
		(int)cycles_ptr, offset);

	printk("last_chargecycles=%04x last_charge_increase=%02x batt_deteriorationstatus=%08x\n",
		(int)oem_param_cycles.last_chargecycles,
		(int)oem_param_cycles.last_charge_increase,
		(int)oem_param_cycles.batt_deteriorationstatus);

	if (oem_param_cycles.last_chargecycles == 0xFFFF) {
		oem_param_cycles.last_chargecycles = 0x0000;
	}

	if (oem_param_cycles.last_charge_increase == 0xFF) {
		oem_param_cycles.last_charge_increase = 0x00;
	}

	if (oem_param_cycles.batt_deteriorationstatus == 0xFFFFFFFF) {
		oem_param_cycles.batt_deteriorationstatus = 0x00000000;
	}

	oem_param_cycles_dump(__func__);
}

void oem_param_cycles_backup(uint16_t charge_cycles, u8 charge_increase)
{
	int offset = oem_param_cycles_offset();
	int ret;

	oem_param_cycles.last_chargecycles = charge_cycles;
	oem_param_cycles.last_charge_increase = charge_increase;

	pr_err("last_chargecycles[%d], last_charge_increase[%d]\n",
				oem_param_cycles.last_chargecycles,
				oem_param_cycles.last_charge_increase);

	oem_param_cycles_dump(__func__);
	ret = kdnand_id_write(CHG_DNAND_CMD, offset, (uint8_t *)&oem_param_cycles, sizeof(oem_param_cycles));

	if(ret)
		pr_err("kdnand_id_write error. ret = %d\n", ret);

	return;
}

void oem_param_fcc_backup(u8 fcc, u8 chgcyl, u8 pos)
{
	int offset = oem_param_cycles_offset();
	int ret;

	oem_param_cycles.fcc[pos] = fcc;
	oem_param_cycles.chgcyl[pos] = chgcyl;

	pr_debug("fcc[%d], chgcyl[%d], pos[%d]\n",
				oem_param_cycles.fcc[pos],
				oem_param_cycles.chgcyl[pos],
				pos);

	oem_param_cycles_dump(__func__);
	ret = kdnand_id_write(CHG_DNAND_CMD, offset, (uint8_t *)&oem_param_cycles, sizeof(oem_param_cycles));

	if(ret)
		pr_err("kdnand_id_write error. ret = %d\n", ret);

	return;
}

void oem_param_deterioration_status_backup(int batt_deterioration_status)
{
	int offset = oem_param_cycles_offset();
	int ret;
	int current_batt_deteriorationstatus = oem_param_cycles.batt_deteriorationstatus;

	if(oem_param_cycles.batt_deteriorationstatus != batt_deterioration_status) {
		oem_param_cycles.batt_deteriorationstatus = batt_deterioration_status;

		pr_err("batt_deteriorationstatus[%d]\n",
					oem_param_cycles.batt_deteriorationstatus);

		oem_param_cycles_dump(__func__);
		ret = kdnand_id_write(CHG_DNAND_CMD, offset, (uint8_t *)&oem_param_cycles, sizeof(oem_param_cycles));

		if(ret) {
			pr_err("kdnand_id_write error. ret = %d\n", ret);
			oem_param_cycles.batt_deteriorationstatus = current_batt_deteriorationstatus;
		}
	}
	return;
}


static int init_flag = 0;
void oem_chg_param_init(void)
{
	uint32_t *smem_ptr = NULL;
	uint32_t *cmp_ptr  = NULL;

	if (init_flag) {
		return;
	}

	smem_ptr = (uint32_t *)kc_smem_alloc(SMEM_CHG_PARAM, CHG_PARAM_SIZE);
	if (smem_ptr == NULL) {
		pr_err("chg param read error.\n");
		init_flag = 1;
		return;
	}

	cmp_ptr = kmalloc((CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE), GFP_KERNEL);
	if (cmp_ptr == NULL) {
		pr_err("kmalloc error.\n");
		init_flag = 1;
		return;
	}

	memset(cmp_ptr, 0x00, (CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE));
	if (0 == memcmp(smem_ptr, cmp_ptr, (CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE))) {
		pr_err("smem data all '0'\n");
		memset(smem_ptr, 0xFF, (CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE));
		oem_cmp_zero_flag = 1;
	}

	oem_param_hkadc_init((oem_chg_param_type*)smem_ptr);
	oem_param_charger_init((oem_chg_param_type*)smem_ptr);
	oem_param_share_init((oem_chg_param_type*)smem_ptr);
	oem_param_cycles_init((oem_chg_param_type*)smem_ptr);

	kfree(cmp_ptr);
	init_flag = 1;

	oem_get_fact_option();

}
