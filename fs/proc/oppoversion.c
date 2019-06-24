/***********************************************************
** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** File: - oppoversion.c
** Description: source  oppoversion
**
** Version: 1.0
** Date : 2018/05/10
** Author: Ming.He@ODM_WT.BSP.Kernel.Driver
**
** ------------------------------- Revision History: -------------------------------
**       <author>           <data>           <version >        <desc>
**       Ming.He@ODM_WT     2018/05/11       1.0               source  oppoversion for os usage
**
****************************************************************/
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <asm/uaccess.h>

#define ITEM_LENGTH 50
#define OPPOVERSION_PATH "oppoVersion"

struct oppoversion{
char name[ITEM_LENGTH];
char version [ITEM_LENGTH];
};

struct oppoversion g_oppoversion_items[] = {
    {"modemType", "Null"},
    {"ocp", "Null"},
    {"operatorName", "Null"},
    {"bootMode", "Null"},
    {"pcbVersion", "Null"},
    {"prjVersion", "Null"},
    {"secureStage", "Null"},
    {"secureType", "Null"},
    {"serialID", "Null"},
    {"ramSize", "Null"},
};

int oppoversion_search_item(char *item_name)
{
    int i = 0;

    for(i = 0; i < ARRAY_SIZE(g_oppoversion_items); i++)
    {
        if(strcmp(item_name, g_oppoversion_items[i].name) == 0)
        {
            return i;
        }
    }

    return -1;
}

static ssize_t oppoversion_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int item_index = -1;
    char oppoversion_temp[ITEM_LENGTH];

    if(*ppos)
        return 0;
    *ppos += count;

    item_index = oppoversion_search_item(file->f_path.dentry->d_iname);

    if(item_index < 0)
    {
        pr_err("[oppoversion][ERR]: oppoversion_proc_read %s fail\n", file->f_path.dentry->d_iname);
        return 0;
    }

    strncpy(oppoversion_temp, g_oppoversion_items[item_index].version, ITEM_LENGTH);

    if (copy_to_user(buf, oppoversion_temp, strlen(oppoversion_temp) + 1)) {
        pr_err("%s: copy to user error.", __func__);
        return -1;
    }

    return strlen(oppoversion_temp);
}

void oppoversion_info_set(char *name, char *version)
{
    int item_index = -1;

    item_index = oppoversion_search_item(name);

    if(item_index < 0)
    {
        pr_err("[oppoversion][ERR]: oppoversion_info_set %s fail\n", name);
        return;
    }

    memset(g_oppoversion_items[item_index].version, 0, ITEM_LENGTH);
    strncpy(g_oppoversion_items[item_index].version, version, ITEM_LENGTH);

    return;
}
EXPORT_SYMBOL_GPL(oppoversion_info_set);

typedef enum  {
	OPPO_UNKOWN = 0,
	OPPO_18361 = 18361,
}OPPO_PROJECT;

#define PRJ_NAME_LEN 5

unsigned int str2u(char *str)
{
	int len = 0;
	int i = 0;
	unsigned int val = 0;

	len = strlen(str);

	for(i = 0; i < len; i++)
	{
		val = val*10 + (str[i] - '0');
	}

	return val;
}

//ODM_WT.BSP.Kernel.Driver.1372106, 2018/06/17, Add interface for driver level usage
unsigned int get_project(void)
{
	int item_index = -1;

	item_index = oppoversion_search_item("prjVersion");

	if(item_index < 0){
        pr_err("[oppoversion][ERR]: get_project fail\n");
        return 0;
    }

	if(strlen(g_oppoversion_items[item_index].version) != PRJ_NAME_LEN){
		pr_err("[oppoversion][ERR]: get_project str len error");
		return 0;
	}

	return str2u(g_oppoversion_items[item_index].version);
}
EXPORT_SYMBOL_GPL(get_project);

unsigned int is_project(OPPO_PROJECT project )
{
	int item_index = -1;

	item_index = oppoversion_search_item("prjVersion");

	if(item_index < 0){
        pr_err("[oppoversion][ERR]: get_project fail\n");
        return 0;
    }

	if(strlen(g_oppoversion_items[item_index].version) != PRJ_NAME_LEN){
		pr_err("[oppoversion][ERR]: get_project str len error");
		return 0;
	}

	if(project == str2u(g_oppoversion_items[item_index].version)){
		return 1;
	}
	else{
		return 0;
	}
}
EXPORT_SYMBOL_GPL(is_project);

unsigned char get_PCB_Version(void)
{
	int item_index = -1;

	item_index = oppoversion_search_item("pcbVersion");

	if(item_index < 0){
        pr_err("[oppoversion][ERR]: get_PCB_Version fail\n");
        return 0;
    }

	return g_oppoversion_items[item_index].version[0];
}
EXPORT_SYMBOL_GPL(get_PCB_Version);

unsigned char get_Modem_Version(void)
{
	int item_index = -1;

	item_index = oppoversion_search_item("modemType");

	if(item_index < 0){
        pr_err("[oppoversion][ERR]: get_Modem_Version fail\n");
        return 0;
    }

	return g_oppoversion_items[item_index].version[0];
}
EXPORT_SYMBOL_GPL(get_Modem_Version);

unsigned char get_Operator_Version(void)
{
	int item_index = -1;

	item_index = oppoversion_search_item("operatorName");

	if(item_index < 0){
        pr_err("[oppoversion][ERR]: get_Operator_Version fail\n");
        return 0;
    }

	return g_oppoversion_items[item_index].version[0];
}
EXPORT_SYMBOL_GPL(get_Operator_Version);

unsigned char get_bootMode(void)
{
	int item_index = -1;

	item_index = oppoversion_search_item("bootMode");

	if(item_index < 0){
        pr_err("[oppoversion][ERR]: get_bootMode fail\n");
        return 0;
    }

	return g_oppoversion_items[item_index].version[0];
}
EXPORT_SYMBOL_GPL(get_bootMode);

static const struct file_operations oppoversion_fops =
{
    .write = NULL,
    .read  = oppoversion_proc_read,
    .owner = THIS_MODULE,
};

static int __init proc_oppoversion_init(void)
{
    int i;
    struct proc_dir_entry *oppoversion_dir;
    struct proc_dir_entry *oppoversion_item;

    oppoversion_dir = proc_mkdir(OPPOVERSION_PATH, NULL);
    if (!oppoversion_dir)
    {
        pr_err("[proc_oppoversion_init][ERR]: create %s dir fail\n", OPPOVERSION_PATH);
        return -1;
    }

    for(i = 0; i < ARRAY_SIZE(g_oppoversion_items); i++)
    {
        oppoversion_item = proc_create(g_oppoversion_items[i].name, 0444, oppoversion_dir, &oppoversion_fops);
        if (oppoversion_item == NULL)
            pr_err("[proc_oppoversion_init][ERR]: create %s fail\n", g_oppoversion_items[i].name);
    }
    return 0;
}
fs_initcall(proc_oppoversion_init);
