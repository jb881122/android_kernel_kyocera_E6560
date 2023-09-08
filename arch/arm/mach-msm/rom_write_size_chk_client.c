/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>


struct rom_write_chk_device_t {
	struct miscdevice misc;
};


struct rom_write_chk_ioctl_info {
    unsigned int   check_sector_count;
    unsigned int   write_sector_num;
};


#define IOC_MAGIC 0xF9
#define ROM_WRITE_CHK_WAIT_FOR_REQ        _IOR(IOC_MAGIC, 1, struct rom_write_chk_ioctl_info)
#define ROM_WRITE_CHK_END                 _IOR(IOC_MAGIC, 2, struct rom_write_chk_ioctl_info)

#define SAVE_SECTOR_UNIT       0x800  
#define CHK_ENABLE  0
#define CHK_DISABLE 1

wait_queue_head_t event_q;

atomic_t  rom_write_unit_count;
unsigned char rom_write_chk_enable = CHK_ENABLE;
unsigned int  rom_write_sector     = 0;


static long rom_write_chk_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg)
{
	struct rom_write_chk_ioctl_info info;
    unsigned long result = -EFAULT;
	int ret;
	
	switch(cmd)
	{
		case ROM_WRITE_CHK_WAIT_FOR_REQ:
			result = copy_from_user(&info, (struct rom_write_chk_ioctl_info __user *)arg, sizeof(info));
			ret = wait_event_interruptible(event_q, atomic_read(&rom_write_unit_count) >= info.check_sector_count);
			if(ret < 0) {
				return ret;
			}
			info.write_sector_num = rom_write_sector;
			result = copy_to_user((struct rom_write_chk_ioctl_info __user *)arg, &info, sizeof(info));
			break;

		case ROM_WRITE_CHK_END:
			rom_write_chk_enable = CHK_DISABLE;
		
		default:
			break;
	}
	return 0;
}


void rom_write_chk_count(struct mmc_host *host, struct mmc_request *mrq)
{
	if(rom_write_chk_enable) {
		return;
	}

	if((host->index) || ((mrq->cmd->opcode != MMC_WRITE_BLOCK) && (mrq->cmd->opcode != MMC_WRITE_MULTIPLE_BLOCK))) {
		return;
	}

	rom_write_sector += mrq->data->blocks;

	if(rom_write_sector > (atomic_read(&rom_write_unit_count) + 1) * SAVE_SECTOR_UNIT) {
		atomic_inc(&rom_write_unit_count);
		wake_up(&event_q);
	}

	return;
}

static int rom_write_chk_open(struct inode *ip, struct file *fp)
{
	return nonseekable_open(ip, fp);
}

static int rom_write_chk_release(struct inode *ip, struct file *fp)
{
	return 0;
}

static const struct file_operations rom_write_chk_fops = {
	.owner = THIS_MODULE,
	.open = rom_write_chk_open,
	.unlocked_ioctl = rom_write_chk_ioctl,
	.release = rom_write_chk_release,
};

static struct rom_write_chk_device_t rom_write_chk_device = {
	.misc = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "rom_write_chk",
		.fops = &rom_write_chk_fops,
	}
};

static void __exit rom_write_chk_exit(void)
{
	misc_deregister(&rom_write_chk_device.misc);
}

static int __init rom_write_chk_init(void)
{
	int ret;

	init_waitqueue_head(&event_q);

	ret = misc_register(&rom_write_chk_device.misc);
	return ret;
}

module_init(rom_write_chk_init);
module_exit(rom_write_chk_exit);

MODULE_DESCRIPTION("rom_write_chk Driver");
MODULE_LICENSE("GPL v2");

