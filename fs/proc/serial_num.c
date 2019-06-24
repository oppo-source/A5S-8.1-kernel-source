/***********************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** File: - serial_num.c
** Description: Source file for serial num check.
**           To create proc node and set the value
** Version: 1.0
** Date : 2018/08/17
** Author: Haibin1.zhang@ODM_WT.BSP.Kernel.Security
**
** ------------------------------- Revision History: -------------------------------
**              <author>                <data>     <version >          <desc>
**  Haibin1.zhang       2018/08/17     1.0     build this module
**  Haibin1.zhang       2018/10/15     1.1     modify cpuid api

****************************************************************/

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/string.h>

#define HRID_SIZE 4

extern u32 get_devinfo_with_index(u32 index);

static void serialno_get(unsigned char * rid)
{
    u32 i, j;

    for (i = 0; i < HRID_SIZE; i++) {
        u32 reg_val = 0;
        reg_val = get_devinfo_with_index(12 + i);
        for (j = 0; j < 4; j++) {
            *(rid + i * 4 + j) = (reg_val & (0xff << (8 * j))) >> (8 * j);
        }
    }
}

static int serialno_proc_show(struct seq_file *m, void *v)
{
	int i;
	unsigned char hrid[16] = {0};

	serialno_get(hrid);
	for (i = 0; i < 16; i++) {
		seq_printf(m, "%02X", hrid[i]);
	}

	return 0;
}

static int serialno_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, serialno_proc_show, NULL);
}

static const struct file_operations serialno_proc_fops = {
	.open		= serialno_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_serialno_init(void)
{
	proc_create("serial_num", 0, NULL, &serialno_proc_fops);
	return 0;
}
fs_initcall(proc_serialno_init);
