/***********************************************************
** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** File: - wtcustom.c
** Description: source  wtcustom
**
** Version: 1.0
** Date : 2018/05/10
** Author: Ming.He@ODM_WT.BSP.Kernel.Driver
**
** ------------------------------- Revision History: -------------------------------
**  	<author>		    <data> 	         <version >	       <desc>
**       Ming.He@ODM_WT     2018/05/11       1.0               source  wtcustom for factory mode and debug usage
**
****************************************************************/
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <asm/uaccess.h>

#define WTCUSTOM_PATH "wtCustom"

struct wtcustom{
unsigned int ps_cal;
unsigned int others[8];
unsigned int ps_offset;
unsigned int ps_delta;
unsigned int gain_als;
};

struct wtcustom g_wtcustom_items = {
0,
0,
0,
0,
0,
};

static ssize_t wtcustom_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    char wtcustom_temp[256];

	printk("wtcustom_read enter\n");

    if(*ppos)
        return 0;
    *ppos += count;

printk("wtcustom_read name:%s,count=%d\n",file->f_path.dentry->d_iname,count);

	wtcustom_temp[0] = g_wtcustom_items.ps_cal&0xff;
	wtcustom_temp[1] = (g_wtcustom_items.ps_cal&0xff00)>>8;
	wtcustom_temp[2] = (g_wtcustom_items.ps_cal&0xff0000)>>16;
	wtcustom_temp[3] = (g_wtcustom_items.ps_cal&0xff000000)>>24;
	wtcustom_temp[36] = g_wtcustom_items.ps_offset&0xff;
	wtcustom_temp[37] = (g_wtcustom_items.ps_offset&0xff00)>>8;
	wtcustom_temp[38] = (g_wtcustom_items.ps_offset&0xff0000)>>16;
	wtcustom_temp[39] = (g_wtcustom_items.ps_offset&0xff000000)>>24;
	wtcustom_temp[40] = g_wtcustom_items.ps_delta&0xff;
	wtcustom_temp[41] = (g_wtcustom_items.ps_delta&0xff00)>>8;
	wtcustom_temp[42] = (g_wtcustom_items.ps_delta&0xff0000)>>16;
	wtcustom_temp[43] = (g_wtcustom_items.ps_delta&0xff000000)>>24;
	wtcustom_temp[44] = g_wtcustom_items.gain_als&0xff;
	wtcustom_temp[45] = (g_wtcustom_items.gain_als&0xff00)>>8;
	wtcustom_temp[46] = (g_wtcustom_items.gain_als&0xff0000)>>16;
	wtcustom_temp[47] = (g_wtcustom_items.gain_als&0xff000000)>>24;

printk("wtcustom_read %d,%d,%d,%d,\n",g_wtcustom_items.ps_cal,g_wtcustom_items.ps_offset,g_wtcustom_items.ps_delta,g_wtcustom_items.gain_als);

    if (copy_to_user(buf, wtcustom_temp, 256)) {
        printk("%s: copy to user error.", __func__);
        return -1;
    }
	printk("wtcustom_read end\n");

	return 256;
}

static ssize_t wtcustom_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    char wtcustom_temp[256] = {0};

	printk("wtcustom_write enter\n");

    if(*ppos)
        return 0;
    *ppos += count;

	if (copy_from_user(wtcustom_temp, buf, count))
		return -EFAULT;

printk("wtcustom_write name:%s,buf=%s,count=%d\n",file->f_path.dentry->d_iname,buf,count);

	g_wtcustom_items.ps_cal = (wtcustom_temp[0]|(wtcustom_temp[1]<< 8)|(wtcustom_temp[2]<< 16)|(wtcustom_temp[3]<< 24));
	g_wtcustom_items.ps_offset = (wtcustom_temp[36]|(wtcustom_temp[37]<< 8)|(wtcustom_temp[38]<< 16)|(wtcustom_temp[39]<< 24));
	g_wtcustom_items.ps_delta = (wtcustom_temp[40]|(wtcustom_temp[41]<< 8)|(wtcustom_temp[42]<< 16)|(wtcustom_temp[43]<< 24));
	g_wtcustom_items.gain_als = (wtcustom_temp[44]|(wtcustom_temp[45]<< 8)|(wtcustom_temp[46]<< 16)|(wtcustom_temp[47]<< 24));
printk("wtcustom_read %d,%d,%d,%d,\n",g_wtcustom_items.ps_cal,g_wtcustom_items.ps_offset,g_wtcustom_items.ps_delta,g_wtcustom_items.gain_als);

	return 256;
}

void wtcustom_offset_set(unsigned int offset)
{
	g_wtcustom_items.ps_offset = offset;
printk("wtcustom_offset_set %d\n",g_wtcustom_items.ps_offset);
}
EXPORT_SYMBOL_GPL(wtcustom_offset_set);

void wtcustom_psCal_set(unsigned int psCal)
{
	g_wtcustom_items.ps_cal = psCal;
printk("wtcustom_psCal_set %d\n",g_wtcustom_items.ps_cal);
}
EXPORT_SYMBOL_GPL(wtcustom_psCal_set);

unsigned int wtcustom_psCal_get(void)
{
	return g_wtcustom_items.ps_cal;
printk("wtcustom_psCal_get %d\n",g_wtcustom_items.ps_cal);
}
EXPORT_SYMBOL_GPL(wtcustom_psCal_get);

void wtcustom_delta_set(unsigned int psDelta)
{
	g_wtcustom_items.ps_delta = psDelta;
printk("wtcustom_delta_set %d\n",g_wtcustom_items.ps_delta);
}
EXPORT_SYMBOL_GPL(wtcustom_delta_set);

unsigned int wtcustom_delta_get(void)
{
	return g_wtcustom_items.ps_delta;
printk("wtcustom_delta_get %d\n",g_wtcustom_items.ps_delta);
}
EXPORT_SYMBOL_GPL(wtcustom_delta_get);


static const struct file_operations wtcustom_fops =
{
    .write = wtcustom_write,
    .read  = wtcustom_read,
    .owner = THIS_MODULE,
};

static int __init proc_wtcustom_init(void)
{
    struct proc_dir_entry *wtcustom_dir;
	struct proc_dir_entry *sensor;

    wtcustom_dir = proc_mkdir(WTCUSTOM_PATH, NULL);
    if (!wtcustom_dir)
    {
        pr_err("[proc_wtcustom_init][ERR]: create %s dir fail\n", WTCUSTOM_PATH);
        return -1;
    }

	sensor = proc_create("Sensor", 0666, wtcustom_dir, &wtcustom_fops);
	if (sensor == NULL)
		pr_err("[proc_wtcustom_init][ERR]: create wtcustom fail\n");

    return 0;
}
fs_initcall(proc_wtcustom_init);
