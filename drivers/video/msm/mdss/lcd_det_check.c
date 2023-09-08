/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 */

#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>

#include "disp_ext.h"
#include "mdss_dsi.h"
#include "lcd_det_check.h"

extern u32 mdss_dsi_panel_cmd_read(struct mdss_dsi_ctrl_pdata *ctrl, char cmd0,
		char cmd1, void (*fxn)(int), char *rbuf, int len);

static struct fb_info *disp_ext_file_fb_info(struct file *file)
{
	struct inode *inode = file->f_path.dentry->d_inode;
	int fbidx = iminor(inode);
	struct fb_info *info = registered_fb[fbidx];

	if (info != file->private_data) {
		info = NULL;
	}

	return info;
}

static struct fb_info *disp_ext_fb0_info_get(void)
{
	char *filename = "/dev/graphics/fb0";
	struct fb_info *info;
	struct file *file;

	file = filp_open(filename, O_RDWR | O_APPEND, 0);
	info = disp_ext_file_fb_info(file);
	return info;
}

static char disp_ext_kernel_reg_read(struct msm_fb_data_type *mfd, char cmd)
{
	char rbuf[4] = {0};
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	mdss_dsi_panel_cmd_read(ctrl_pdata, cmd, 0, NULL, rbuf, 0);

	return rbuf[0];
}

int lcd_det_check(void)
{
	struct fb_info *info;
	struct msm_fb_data_type *mfd;
	unsigned char rdata_0a, rdata_0c;
	int ret = 1;

	info = disp_ext_fb0_info_get();
	if (info == NULL) {
		pr_err("[LCD] %s: cannot get fb0\n", __func__);
		return -1;
	}

	mfd = (struct msm_fb_data_type *)info->par;
	if (mfd == NULL) {
		pr_err("[LCD] %s: cannot get mfd\n", __func__);
		return -1;
	}

	if (!mfd->panel_power_on) {
		pr_err("[LCD] %s: not power on\n", __func__);
		return -1;
	}

	rdata_0a = disp_ext_kernel_reg_read(mfd, 0x0a);

	rdata_0c = disp_ext_kernel_reg_read(mfd, 0x0c);

	pr_info("[LCD] %s: read power mode   : 0x%02x\n", __func__, rdata_0a);
	pr_info("[LCD] %s: read pixel format : 0x%02x\n", __func__, rdata_0c);

	rdata_0a = rdata_0a & 0x14;

	rdata_0c = rdata_0c & 0x70;

	if ((rdata_0a != 0x14) || (rdata_0c != 0x70)) {
		ret = 0;
		pr_info("[LCD] %s: panel do not detect\n", __func__);
	} else {
		pr_info("[LCD] %s: panel detect\n", __func__);
	}

	return ret;
}
