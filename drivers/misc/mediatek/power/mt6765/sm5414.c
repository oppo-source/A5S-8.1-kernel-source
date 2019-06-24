/*******************************************************************************
 *  Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
 *  ODM_WT_EDIT
 *  FILE: - sm5414.c
 *  Description : Add charger driver file
 *  Version: 1.0
 *  Date : 2018/6/20
 *  Author: Maosheng.Zhang@ODM_WT.BSP.Charger.Basic
 *
 *  -------------------------Revision History:----------------------------------
 *   <author>	 <data> 	<version >			<desc>
 *  Maosheng.Zhang	2018/08/06	1.0				Add charger driver file
 ******************************************************************************/ 
#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <mt-plat/mtk_boot_common.h>
#include "sm5414.h"
 
extern int topoff_flag;
static int dump_flag;

#if defined(CONFIG_MTK_GAUGE_VERSION) && (CONFIG_MTK_GAUGE_VERSION == 30)
#include <mt-plat/charger_class.h>
#include "mtk_charger_intf.h"
#endif /* #if (CONFIG_MTK_GAUGE_VERSION == 30) */
 
#ifdef CONFIG_TINNO_JEITA_CURRENT
#ifdef CONFIG_TINNO_CUSTOM_CHARGER_PARA
#include "cust_charging.h"
#else
#include "cust_charger_init.h"
#endif
#endif

#if 0
#define GPIO_SM5414_CHGEN_PIN(x)        (x | 0x80000000)
#define GPIO_SM5414_nSHDN_PIN(x)        (x | 0x80000000)
#define GPIO_SM5414_EINT_PIN(x)         (x | 0x80000000)
#endif

#ifndef I2C_MASK_FLAG
#define I2C_MASK_FLAG	0x00FF
#define I2C_ENEXT_FLAG	0x0200
#endif
 /**********************************************************
  *
  *   [I2C Slave Setting]
  *
  *********************************************************/
#define sm5414_SLAVE_ADDR_WRITE   0x92
#define sm5414_SLAVE_ADDR_READ    0x93
 
#define STATUS_OK	0
#define STATUS_FAIL	1
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))
bool oppo_full_logo;

 /* ============================================================ // */
 /* Global variable */
 /* ============================================================ // */
 // BATREG
 const unsigned int VBAT_CV_VTH[] = {
	 4100000, 4125000, 4150000, 4175000,
	 4200000, 4225000, 4250000, 4275000,
	 4300000, 4325000, 4350000, 4375000,
	 4400000, 4425000, 4450000, 4475000,
 };
 
 // Topoff Current
 const unsigned int CS_ITERM[] = {
	 100000,  150000,  200000,	250000,
	 300000,  350000,  400000,	450000,
	 500000,  550000,  600000,	650000,
	 650000,  650000,  650000,  650000,
 };
 
 // Fastchg
 const unsigned int CS_VTH[] = {
    100000,   150000,   200000,  250000,
    300000,   350000,   400000,  450000,
    500000,   550000,   600000,  650000,
    700000,   750000,   800000,  850000,
    900000,   950000,   1000000, 1050000,
    1100000,  1150000,  1200000, 1250000,
    1300000,  1350000,  1400000, 1450000,
    1500000,  1550000,  1600000, 1650000,
    1700000,  1750000,  1800000, 1850000,
    1900000,  1950000,  2000000, 2050000,
    2100000,  2150000,  2200000, 2250000,
    2300000,  2350000,  2400000, 2450000,
    2500000,
 };

 // Input current limit
 const unsigned int INPUT_CS_VTH[] = {
	 100000,   150000,	 200000,  250000,
	 300000,   350000,	 400000,  450000,
	 500000,   550000,	 600000,  650000,
	 700000,   750000,	 800000,  850000,
	 900000,   950000,	 1000000, 1050000,
	 1100000,  1150000,  1200000, 1250000,
	 1300000,  1350000,  1400000, 1450000,
	 1500000,  1550000,  1600000, 1650000,
	 1700000,  1750000,  1800000, 1850000,
	 1900000,  1950000,  2000000, 2050000,
 };
 
 static struct i2c_client *new_client;
 static const struct i2c_device_id sm5414_i2c_id[] = { {"subpmic_pmu", 0}, {} };
 
 static bool chargin_hw_init_done = 0;
 static int sm5414_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);
 
#ifdef CONFIG_OF
 static const struct of_device_id sm5414_of_match[] = {
	 {.compatible = "mediatek,subpmic_pmu",},
	 {},
 };
 
 MODULE_DEVICE_TABLE(of, sm5414_of_match);
#endif
 
 static struct i2c_driver sm5414_driver = {
	 .driver = {
		 .name = "subpmic_pmu",
#ifdef CONFIG_OF
		 .of_match_table = sm5414_of_match,
#endif
	 },
	 .probe = sm5414_driver_probe,
	 .id_table = sm5414_i2c_id,
 };
 
 /**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
 unsigned char sm5414_reg[SM5414_REG_NUM] = { 0 };
 
 static DEFINE_MUTEX(sm5414_i2c_access);
 
 int g_sm5414_hw_exist = 0;
 
 /**********************************************************
  *
  *   [I2C Function For Read/Write sm5414]
  *
  *********************************************************/
#if 0
 int sm5414_read_byte(unsigned char cmd, unsigned char *returnData)
 {
	 int ret = 0;
	 struct i2c_msg msgs[] =
	 {
		 {
			 .addr = ((new_client->addr&I2C_MASK_FLAG)|I2C_ENEXT_FLAG),
			 .flags = 0,
			 .len = 1,
			 .buf = &cmd,
		 },
		 {
			 .addr = ((new_client->addr&I2C_MASK_FLAG)|I2C_ENEXT_FLAG),
			 .flags = I2C_M_RD,
			 .len = 1,
			 .buf = returnData,
		 },
	 };
	 mutex_lock(&sm5414_i2c_access);
	 ret = i2c_transfer(new_client->adapter, msgs, 2);
	 if(ret < 0){
		 //chr_err("%s: read 0x%x register failed\n",__func__,cmd);
		printk("%s: read 0x%x register failed\n",__func__,cmd);
	 }
	 //pr_debug("%s:register = 0x%x , value = 0x%x\n",__func__,cmd,*returnData);
	printk("%s:register = 0x%x , value = 0x%x\n",__func__,cmd,*returnData);
	 mutex_unlock(&sm5414_i2c_access);
	 return 1;
 }
 
 int sm5414_write_byte(unsigned char cmd, unsigned char writeData)
 {
	 char write_data[2] = { 0 };
	 int ret = 0;
	 struct i2c_msg msgs[] =
	 {
		 {
			 .addr = ((new_client->addr&I2C_MASK_FLAG)|I2C_ENEXT_FLAG),
			 .flags = 0,
			 .len = 2,
			 .buf = write_data,
		 },
	 };
 
	 write_data[0] = cmd;
	 write_data[1] = writeData;
	 //pr_debug("%s cmd = 0x%x writeData = 0x%x\n",__func__,cmd,writeData);
 	printk("%s cmd = 0x%x writeData = 0x%x\n",__func__,cmd,writeData);
	 mutex_lock(&sm5414_i2c_access);
	 ret = i2c_transfer(new_client->adapter, msgs, 1);
	 if (ret < 0){
		 //chr_err("%s: write 0x%x to 0x%x register failed\n",__func__,writeData,cmd);
		printk("%s: write 0x%x to 0x%x register failed\n",__func__,writeData,cmd);
		 mutex_unlock(&sm5414_i2c_access);
		 return ret;
	 }
	 mutex_unlock(&sm5414_i2c_access);
	 return 1;
 }
#else
 int sm5414_read_byte(unsigned char cmd, unsigned char *returnData)
 {
	struct i2c_client *i2c = (struct i2c_client *)new_client;
	return i2c_smbus_read_i2c_block_data(i2c, cmd, 1, returnData);
 }

 int sm5414_write_byte(unsigned char cmd, unsigned char writeData)
 {
	struct i2c_client *i2c = (struct i2c_client *)new_client;

	i2c_smbus_write_i2c_block_data(i2c, cmd, 1, &writeData);

	return sm5414_read_byte(cmd,&writeData);

 }


#endif
 /**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
 unsigned int sm5414_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
		 unsigned char SHIFT)
 {
	 unsigned char sm5414_reg = 0;
	 int ret = 0;

	 ret = sm5414_read_byte(RegNum, &sm5414_reg);
	 printk("[sm5414_read_interface] Reg[%x]=0x%x\n", RegNum, sm5414_reg);
	 sm5414_reg &= (MASK << SHIFT);
	 *val = (sm5414_reg >> SHIFT);
	 printk("[sm5414_read_interface] val=0x%x\n", *val);

	 return ret;
 }

 unsigned int sm5414_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
		 unsigned char SHIFT)
 {
	 unsigned char sm5414_reg = 0;
	 int ret = 0;

	 ret = sm5414_read_byte(RegNum, &sm5414_reg);

	 sm5414_reg &= ~(MASK << SHIFT);
	 sm5414_reg |= (val << SHIFT);

	 ret = sm5414_write_byte(RegNum, sm5414_reg);
	 /* Check */
	 sm5414_read_byte(RegNum, &sm5414_reg);

	 return ret;
 }

//write one register directly
unsigned int sm5414_reg_config_interface (unsigned char RegNum, unsigned char val)
{
    return sm5414_write_byte(RegNum, val);
}

unsigned int sm5414_get_topoff_status(unsigned char *val)
{
    return sm5414_read_interface((unsigned char)(SM5414_STATUS),val,
                                (unsigned char)(SM5414_STATUS_TOPOFF_MASK),
                                (unsigned char)(SM5414_STATUS_TOPOFF_SHIFT));
}

void sm5414_set_enboost(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CTRL),(unsigned char)(val),
                            (unsigned char)(SM5414_CTRL_ENBOOST_MASK),
                            (unsigned char)(SM5414_CTRL_ENBOOST_SHIFT));
}

void sm5414_set_chgen(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CTRL),(unsigned char)(val),
                            (unsigned char)(SM5414_CTRL_CHGEN_MASK),
                            (unsigned char)(SM5414_CTRL_CHGEN_SHIFT));
}

void sm5414_set_suspen(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CTRL),(unsigned char)(val),
                            (unsigned char)(SM5414_CTRL_SUSPEN_MASK),
                            (unsigned char)(SM5414_CTRL_SUSPEN_SHIFT));
}

void sm5414_set_reset(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CTRL),(unsigned char)(val),
                            (unsigned char)(SM5414_CTRL_RESET_MASK),
                            (unsigned char)(SM5414_CTRL_RESET_SHIFT));
}

void sm5414_set_encomparator(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CTRL),(unsigned char)(val),
                            (unsigned char)(SM5414_CTRL_ENCOMPARATOR_MASK),
                            (unsigned char)(SM5414_CTRL_ENCOMPARATOR_SHIFT));
}

//vbusctrl
void sm5414_set_vbuslimit(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_VBUSCTRL),(unsigned char)(val),
                            (unsigned char)(SM5414_VBUSCTRL_VBUSLIMIT_MASK),
                            (unsigned char)(SM5414_VBUSCTRL_VBUSLIMIT_SHIFT));
}

//chgctrl1
void sm5414_set_prechg(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL1),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL1_PRECHG_MASK),
                            (unsigned char)(SM5414_CHGCTRL1_PRECHG_SHIFT));
}

void sm5414_set_aiclen(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL1),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL1_AICLEN_MASK),
                            (unsigned char)(SM5414_CHGCTRL1_AICLEN_SHIFT));
}

void sm5414_set_autostop(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL1),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL1_AUTOSTOP_MASK),
                            (unsigned char)(SM5414_CHGCTRL1_AUTOSTOP_SHIFT));
}
void sm5414_set_aiclth(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL1),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL1_AICLTH_MASK),
                            (unsigned char)(SM5414_CHGCTRL1_AICLTH_SHIFT));
}

//chgctrl2
void sm5414_set_fastchg(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL2),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL2_FASTCHG_MASK),
                            (unsigned char)(SM5414_CHGCTRL2_FASTCHG_SHIFT));
}

//chgctrl3
void sm5414_set_weakbat(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL3),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL3_WEAKBAT_MASK),
                            (unsigned char)(SM5414_CHGCTRL3_WEAKBAT_SHIFT));
}
void sm5414_set_batreg(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL3),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL3_BATREG_MASK),
                            (unsigned char)(SM5414_CHGCTRL3_BATREG_SHIFT));
}

//chgctrl4
void sm5414_set_dislimit(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL4),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL4_DISLIMIT_MASK),
                            (unsigned char)(SM5414_CHGCTRL4_DISLIMIT_SHIFT));
}

void sm5414_set_topoff(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL4),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL4_TOPOFF_MASK),
                            (unsigned char)(SM5414_CHGCTRL4_TOPOFF_SHIFT));
}

//chgctrl5
void sm5414_set_topofftimer(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL5),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL5_TOPOFFTIMER_MASK),
                            (unsigned char)(SM5414_CHGCTRL5_TOPOFFTIMER_SHIFT));
}

void sm5414_set_fasttimer(unsigned int val)
{

    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL5),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL5_FASTTIMER_MASK),
                            (unsigned char)(SM5414_CHGCTRL5_FASTTIMER_SHIFT));
}

void sm5414_set_votg(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL5),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL5_VOTG_MASK),
                            (unsigned char)(SM5414_CHGCTRL5_VOTG_SHIFT));
}

#if 0
int sm5414_chg_enable_gpio(unsigned int enable)
{
    struct sm5414_info *info = i2c_get_clientdata(new_client);

    if (info == NULL) {
        return -EINVAL;
    }
    if (enable) {
        mt_set_gpio_out(GPIO_SM5414_CHGEN_PIN(info->nCHGEN),GPIO_OUT_ZERO);
    } else {
        mt_set_gpio_out(GPIO_SM5414_CHGEN_PIN(info->nCHGEN),GPIO_OUT_ONE);
    }
    return 0;
}
#endif

int sm5414_otg_enable(unsigned int enable)
{
    struct sm5414_info *info = i2c_get_clientdata(new_client);

    if (info == NULL) {
        return -EINVAL;
    }

    if (enable) {
        //Before turning on OTG, system must turn off charing function.
        sm5414_set_chgen(CHARGE_DIS);
        sm5414_set_enboost(ENBOOST_EN);
        info->otg_cnt = 0;
    } else {
        sm5414_set_suspen(SUSPEND_EN);
        mdelay(1);
        sm5414_set_enboost(ENBOOST_DIS);
        mdelay(1);
        sm5414_set_suspen(SUSPEND_DIS);
    }

    return 0;
}

int is_sm5414_exist(void)
{
    printk("g_sm5414_hw_exist = %d\n", g_sm5414_hw_exist);

    return g_sm5414_hw_exist;
}

void sm5414_dump_register(void)
{
    int i = 0;
	// printk("dump regs\n");
    for (i = SM5414_INTMASK1; i < SM5414_REG_NUM; i++) {
        sm5414_read_byte(i, &sm5414_reg[i]);
        printk("%s : [0x%x]=0x%x\n", __func__, i, sm5414_reg[i]);
    }
    printk("\n");
}

unsigned int sm5414_hw_component_detect(void)
{
    unsigned char val = 0;
    int ret = 0;

    ret = sm5414_read_interface(0x0E, &val, 0xFF, 0x0);
    if (ret == 0) {
        return ret;
    }

    g_sm5414_hw_exist = 1;

    printk("exist = %d, Reg[0x0E] = 0x%x\n", g_sm5414_hw_exist, val);
    return g_sm5414_hw_exist;
}

void sm5414_reg_init(void)
{
    //INT MASK 1/2/3
    sm5414_write_byte(SM5414_INTMASK1, 0xFF);
    sm5414_write_byte(SM5414_INTMASK2, 0xEF); //OTGFAILM
    sm5414_write_byte(SM5414_INTMASK3, 0xFF);

    sm5414_set_encomparator(ENCOMPARATOR_EN);
    sm5414_set_topoff(TOPOFF_200mA);
    sm5414_set_batreg(BATREG_4_3_5_0_V);

    sm5414_set_autostop(AUTOSTOP_DIS);
}

static irqreturn_t sm5414_irq_handler(struct sm5414_info *info)
{
     u8 int_value[3] = {0};

     sm5414_read_byte(SM5414_INT1, &int_value[SM5414_INT1]);
     sm5414_read_byte(SM5414_INT2, &int_value[SM5414_INT2]);
     sm5414_read_byte(SM5414_INT3, &int_value[SM5414_INT3]);

     printk("INT1 : 0x%x, INT2 : 0x%x, INT3 : 0x%x\n",
            int_value[SM5414_INT1],int_value[SM5414_INT2],int_value[SM5414_INT3]);

     if (int_value[SM5414_INT2] & SM5414_INT2_OTGFAIL) {
         printk("OTG FAIL is occurred!!\n");
         //When OTG boost Fail is occurred, OTG boost is retried one more time.
         sm5414_otg_enable(ENBOOST_DIS);
         if (info->otg_cnt < 1) {
            sm5414_otg_enable(ENBOOST_EN);
            msleep(80);
            info->otg_cnt++;
         }
     }
     return IRQ_HANDLED;
}

static int sm5414_irq_init(struct sm5414_info *info)
{
     int ret = 0;

     printk("Start\n");

     if (info->dev->of_node) {
		ret = gpio_request_one(info->nINT, GPIOF_IN, "sm5414_irq_gpio");
		if (ret < 0) {
			pr_debug("%s: gpio request fail\n", __func__);
			return ret;
		}
		info->irq = gpio_to_irq(info->nINT);
		if (info->irq < 0) {
			pr_debug("%s: irq mapping fail\n", __func__);
			return -1;
		}
		printk("info->irq = %d\n", info->irq);

		ret = request_threaded_irq(info->irq, NULL, (irq_handler_t)sm5414_irq_handler,
		            IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND | IRQF_ONESHOT, "sm5414-irq", info);
		if (ret < 0) {
		    pr_debug("request_irq IRQ LINE NOT AVAILABLE!.\n");
		}
     } else {
         printk("request_irq can not find  eint device node!.\n");
         ret = -1;
     }

     printk("Done\n");

     return ret;
}

static void sm5414_gpio_init(struct sm5414_info *info)
{
#if 0
    mt_set_gpio_mode(GPIO_SM5414_CHGEN_PIN(info->nCHGEN),GPIO_MODE_GPIO);
    mt_set_gpio_out(GPIO_SM5414_CHGEN_PIN(info->nCHGEN),GPIO_OUT_ZERO);
    mdelay(10);		//delay is neccesary

    mt_set_gpio_mode(GPIO_SM5414_EINT_PIN(info->nINT), GPIO_MODE_GPIO);
    mt_set_gpio_pull_enable(GPIO_SM5414_EINT_PIN(info->nINT), GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_SM5414_EINT_PIN(info->nINT), GPIO_PULL_UP);
    mdelay(10);		//delay is neccesary

    mt_set_gpio_mode(GPIO_SM5414_nSHDN_PIN(info->nSHDN),GPIO_MODE_GPIO);
    mt_set_gpio_out(GPIO_SM5414_nSHDN_PIN(info->nSHDN),GPIO_OUT_ONE);
    mdelay(10);		//delay is neccesary
#endif
}

static int sm5414_parse_dt(struct device *dev, struct sm5414_info *info)
{
     struct device_node *np = dev->of_node;
     int rc;

     rc = of_property_read_u32(np, "nCHGEN" ,&info->nCHGEN);
     if (rc) {
         pr_debug("nCHGEN not defined.\n");
         return rc;
     }

     rc = of_property_read_u32(np, "nINT" ,&info->nINT);
     if (rc) {
         pr_debug("nINT not defined.\n");
         return rc;
     }

     rc = of_property_read_u32(np, "nSHDN" ,&info->nSHDN);
     if (rc) {
         pr_debug("nSHDN not defined.\n");
         return rc;
     }

     return 0;
} 

 unsigned int charging_value_to_parameter(const unsigned int *parameter, const unsigned int array_size, const unsigned int val)
 {
	 if (val < array_size) {
		 return parameter[val];
	 } else {
		 printk("Can't find the parameter \r\n");
		 return parameter[0];
	 }
 }
 
 static u32 charging_parameter_to_value(const u32 *parameter, const u32 array_size, const u32 val)
 {
	 u32 i;
 
	 for (i = 0; i < array_size; i++)
		 if (val == *(parameter + i))
			 return i;
 
	 chr_info("NO register value match \r\n");
 
	 return 0;
 }
 
 static u32 bmt_find_closest_level(const u32 *pList, u32 number, u32 level)
 {
	 u32 i;
	 u32 max_value_in_last_element;
 
	 if (pList[0] < pList[1])
		 max_value_in_last_element = true;
	 else
		 max_value_in_last_element = false;
 
	 if (max_value_in_last_element == true) {
		 if(level > pList[number-1])
			 level = pList[number-1];
 
		 if(level < pList[0])
		 {
			 level = pList[0];
		 }
 
		 for (i = (number - 1); i != 0; i--) /* max value in the last element */
		 {
			 if (pList[i] <= level)
				 return pList[i];
		 }
 
		 chr_info("Can't find closest level, small value first \r\n");
		 return pList[0];
		 /* return CHARGE_CURRENT_0_00_MA; */
	 } else {
		 if(level > pList[0])
			 level = pList[0];
 
		 if(level < pList[number-1])
		 {
			 level = pList[number-1];
		 }
 
		 for (i = 0; i < number; i++)	 /* max value in the first element */
		 {
			 if (pList[i] <= level)
				 return pList[i];
		 }
 
		 chr_info("Can't find closest level, large value first \r\n");
		 return pList[number - 1];
		 /* return CHARGE_CURRENT_0_00_MA; */
	 }
 }
 
#if defined(CONFIG_MTK_GAUGE_VERSION) && (CONFIG_MTK_GAUGE_VERSION == 30)
 static unsigned int charging_hw_init(void)
 {
	unsigned int status = STATUS_OK;
    unsigned int array_size;
	//unsigned int i;
#if 0
	mt_set_gpio_mode(GPIO_SM5414_SHDN_PIN,GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_SM5414_SHDN_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SM5414_SHDN_PIN,GPIO_OUT_ONE);
#endif

    //INT MASK 1/2/3
    sm5414_write_byte(SM5414_INTMASK1, 0xFF);
    sm5414_write_byte(SM5414_INTMASK2, 0xEF); //OTGFAILM
    sm5414_write_byte(SM5414_INTMASK3, 0xFF);

	sm5414_set_suspen(SUSPEND_DIS);
    sm5414_set_encomparator(ENCOMPARATOR_EN);

	array_size = sizeof(CS_ITERM)/sizeof(CS_ITERM[0]);
	
/*
	for(i=array_size-1; i>0; i--)
	{
		if(CS_ITERM[i] <= TERMINATION_CURRENT){
			sm5414_set_topoff(i);
			break;
		}
	}
*/
	sm5414_set_topoff(TOPOFF_200mA);
	
	sm5414_set_batreg(BATREG_4_3_5_0_V); //VREG 4.350V

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	 if (wireless_charger_gpio_number != 0) {
		 mt_set_gpio_mode(wireless_charger_gpio_number, 0);  /* 0:GPIO mode */
		 mt_set_gpio_dir(wireless_charger_gpio_number, 0);	 /* 0: input, 1: output */
	 }
#endif
 
#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
	 mt_set_gpio_mode(vin_sel_gpio_number, 0);	 /* 0:GPIO mode */
	 mt_set_gpio_dir(vin_sel_gpio_number, 0);	 /* 0: input, 1: output */
#endif

	sm5414_set_autostop(AUTOSTOP_DIS);

	return status;
 }
 
 static int sm5414_charger_plug_in(struct charger_device *chg_dev)
 {
	 //do nothing
	 return 0;
 }
 
 static int sm5414_charger_plug_out(struct charger_device *chg_dev)
 {
	 //do nothing
	 return 0;
 }
 
 static int sm5414_charging_enable(struct charger_device *chg_dev, bool en)
 {
	 unsigned int status = STATUS_OK;

	 if (en == true) {
		 sm5414_set_chgen(CHARGE_EN);
	 } 
	 else{
#if defined(CONFIG_USB_MTK_HDRC_HCD)
		 if (mt_usb_is_device())
#endif
		 {
			sm5414_set_chgen(CHARGE_DIS);
		 }
#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		 sm5414_set_chgen(CHARGE_DIS);
#endif
	 }
 
	 return status;
 }
 
 static int sm5414_charger_is_enabled(struct charger_device *chg_dev, bool *en)
 {
	 return 0;
 }
 
 static int sm5414_charger_get_ichg(struct charger_device *chg_dev, u32 *uA)
 {
	 unsigned int status = STATUS_OK;
	 unsigned int array_size;
	 unsigned char reg_value;
	 
	 //Get current level
	 array_size = GETARRAYNUM(CS_VTH);
	 sm5414_read_interface(SM5414_CHGCTRL2, &reg_value, SM5414_CHGCTRL2_FASTCHG_MASK, SM5414_CHGCTRL2_FASTCHG_SHIFT);//FASTCHG
	 *(unsigned int *)uA = charging_value_to_parameter(CS_VTH,array_size,reg_value);
	 
	 return status;
 }
 
 static int sm5414_charger_set_ichg(struct charger_device *chg_dev, u32 uA)
 {
	 unsigned int status = STATUS_OK;
	 unsigned int set_chr_current;
	 unsigned int array_size=0;
	 unsigned int register_value;
	 unsigned int current_value = uA;
	 //printk("sm5414 set charging current = %d\n",uA);
	 array_size = GETARRAYNUM(CS_VTH);
	 set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value);
 
	 register_value = charging_parameter_to_value(CS_VTH, array_size, set_chr_current);
 
	 sm5414_set_fastchg(register_value);//FASTCHG
 
	 return status;
 }
 
 static int sm5414_charger_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
 {
	 *uA = 100000;
	 return 0;
 }
 
 static int sm5414_charger_set_cv(struct charger_device *chg_dev, u32 uV)
 {
	 unsigned int status = STATUS_OK;
	 unsigned int array_size;
	 unsigned int set_cv_voltage;
	 unsigned short register_value;
	 unsigned int cv_value = uV;
 
	 array_size = GETARRAYNUM(VBAT_CV_VTH);
	 set_cv_voltage = bmt_find_closest_level(VBAT_CV_VTH, array_size, cv_value);
	 register_value = charging_parameter_to_value(VBAT_CV_VTH, array_size, set_cv_voltage);
	 sm5414_set_batreg(register_value);

	 return status;
 }
 
 static int sm5414_charger_get_cv(struct charger_device *chg_dev, u32 *uV)
 {
	 //do nothing
	 return 0;
 }
 
 static int sm5414_charger_get_aicr(struct charger_device *chg_dev, u32 *uA)
 {
	 //do nothing
	 return 0;
 }
 
 static int sm5414_charger_set_aicr(struct charger_device *chg_dev, u32 uA)
 {
	 unsigned int status = STATUS_OK;
	 unsigned int current_value = uA;
	 unsigned int set_chr_current;
	 unsigned int array_size;
	 unsigned int register_value;
	 //printk("sm5414 current_value = %d\n",current_value);
	 array_size = GETARRAYNUM(INPUT_CS_VTH);
	 set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size, current_value);
	 register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size ,set_chr_current);
	 
	 sm5414_set_vbuslimit(register_value);//VBUSLIMIT
	 
	 return status;
 }
 
 static int sm5414_charger_get_min_aicr(struct charger_device *chg_dev, u32 *uA)
 {
	 //do nothing
	 *uA = 100000;
	 return 0;
 }
 
 static int sm5414_charger_set_mivr(struct charger_device *chg_dev, u32 uV)
 {
	 //do nothing
	 return 0;
 }
 
 static int sm5414_charger_is_timer_enabled(struct charger_device *chg_dev,
		 bool *en)
 {
	 return 0;
 }
 
 static int sm5414_charger_enable_timer(struct charger_device *chg_dev, bool en)
 {
	 //do nothing
	 return 0;
 }
 
 static int sm5414_charger_enable_te(struct charger_device *chg_dev, bool en)
 {
	 //do nothing
	 return 0;
 }
 
 static int sm5414_charger_enable_otg(struct charger_device *chg_dev, bool en)
 {
	 if (en)
	 {
		 sm5414_otg_enable(1);
	 }else{
		 sm5414_otg_enable(0);
	 }
 
	 return 0;
 }
 
 static int sm5414_charger_is_charging_done(struct charger_device *chg_dev,
		 bool *done)
 {
	 int status = STATUS_OK;
	 /*
	 unsigned char val;
	 unsigned int ret;
	 
	 ret = sm5414_get_topoff_status(&val);
	 if (ret == 0) {
		 return ret;
	 }
	 
	 //Fullcharged status
	 if (val == 0x1) {
		 *done = 1;
	 } else {
		 *done = 0;
	 }
	 */
	 if(topoff_flag == 200) {
		*done = 1;
		oppo_full_logo = true;
	 } else if(topoff_flag == 300) {
		*done = 0;
	 }
	 return status;

 }
 
 static int sm5414_charger_dump_registers(struct charger_device *chg_dev)
 {
	dump_flag++;
	if(dump_flag >= 30) {
	sm5414_dump_register();
		dump_flag = 0;
		}
	return 0;
 }
 
 static int sm5414_charger_do_event(struct charger_device *chg_dev, u32 event,
		 u32 args)
 {
	 struct sm5414_info *info = charger_get_data(chg_dev);
 
	 switch (event) {
		 case EVENT_EOC:
			 chr_info("do eoc event\n");
			 charger_dev_notify(info->chg_dev, CHARGER_DEV_NOTIFY_EOC);
			 break;
		 case EVENT_RECHARGE:
			 chr_info("do recharge event\n");
			 charger_dev_notify(info->chg_dev, CHARGER_DEV_NOTIFY_RECHG);
			 break;
		 default:
			 break;
	 }
	 return 0;
 }
 
 
 static const struct charger_ops sm5414_chg_ops = {
	 /* cable plug in/out */
	 .plug_in = sm5414_charger_plug_in,
	 .plug_out = sm5414_charger_plug_out,
	 /* enable */
	 .enable = sm5414_charging_enable,
	 .is_enabled = sm5414_charger_is_enabled,
	 /* charging current */
	 .get_charging_current = sm5414_charger_get_ichg,
	 .set_charging_current = sm5414_charger_set_ichg,
	 .get_min_charging_current = sm5414_charger_get_min_ichg,
	 /* charging voltage */
	 .set_constant_voltage = sm5414_charger_set_cv,
	 .get_constant_voltage = sm5414_charger_get_cv,
	 /* charging input current */
	 .get_input_current = sm5414_charger_get_aicr,
	 .set_input_current = sm5414_charger_set_aicr,
	 .get_min_input_current = sm5414_charger_get_min_aicr,
	 /* charging mivr */
	 .set_mivr = sm5414_charger_set_mivr,
	 /* safety timer */
	 .is_safety_timer_enabled = sm5414_charger_is_timer_enabled,
	 .enable_safety_timer = sm5414_charger_enable_timer,
	 /* charing termination */
	 .enable_termination = sm5414_charger_enable_te,
	 /* OTG */
	 .enable_otg = sm5414_charger_enable_otg,
	 /* misc */
	 .is_charging_done = sm5414_charger_is_charging_done,
	 .dump_registers = sm5414_charger_dump_registers,
	 /* event */
	 .event = sm5414_charger_do_event,
 };
 
 static const struct charger_properties sm5414_chg_props = {
	 .alias_name = "sm5414",
 };
#endif /* #if (CONFIG_MTK_GAUGE_VERSION == 30) */
 
 static int sm5414_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {
	 struct sm5414_info *info = NULL;
	 bool use_dt = client->dev.of_node;	 
	 int ret = 0;
	 int err = 0;
	 
	 pr_err("[sm5414_driver_probe]\n");
 
	 new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);
	 if (!new_client) {
		 err = -ENOMEM;
		 goto exit;
	 }
	 memset(new_client, 0, sizeof(struct i2c_client));
	 info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	 if (!info)
	 {
		 return -ENOMEM;
	 }
	 /* platform data */
	goto sm5414_test;
	 if (use_dt) {
        ret = sm5414_parse_dt(&client->dev, info);
        if (ret) {
            pr_debug("cannot read from dt.\n");
            return -ENOMEM;
        }		
	 } else {
		 pr_debug("no dt specify\n");
		 return -EINVAL;
	 }
	sm5414_test:
	 client->addr = 0x49;
	 new_client = client;
	 info->dev = &client->dev;
	 info->i2c = client;
	 info->chip_rev = 0;
	 info->chg_name = "primary_chg";
	 i2c_set_clientdata(client, info);
	 if(!sm5414_hw_component_detect()){
		 pr_err("sm5414 is not exist\n");
		 err = -1;
		 goto exit;
	 }
	 /* initialize device */
	 sm5414_gpio_init(info);
	 charging_hw_init();
	 /* --------------------- */
 
#if defined(CONFIG_MTK_GAUGE_VERSION) && (CONFIG_MTK_GAUGE_VERSION == 30)
	 /* charger class register */
	 info->chg_dev = charger_device_register(info->chg_name, info->dev, info,
			 &sm5414_chg_ops,
			 &sm5414_chg_props);
	 if (IS_ERR(info->chg_dev)) {
		 pr_debug("charger device register fail\n");
		 return PTR_ERR(info->chg_dev);
	 }
#endif /* #if (CONFIG_MTK_GAUGE_VERSION == 30) */
	goto sm5414_test2;
	 ret = sm5414_irq_init(info);
	 if (ret < 0) {
		 pr_err("Error : can't initialize SM5414 irq.\n");
		 return -EINVAL;
	 }
	sm5414_test2:
	 sm5414_dump_register();
	 chargin_hw_init_done = 1;

	 pr_err("[sm5414_driver_probe] Done\n");
 
	 return 0;
 
 exit:
	 return err;
 
 }
 
 /**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
 unsigned char g_reg_value_sm5414 = 0;
 static ssize_t show_sm5414_access(struct device *dev, struct device_attribute *attr, char *buf)
 {
	 int i=0;
	 int offset = 0;
 
	 sm5414_dump_register();
	 for(i = 0; i<12; i++)
	 {
		 offset += sprintf(buf+offset, "reg[0x%x] = 0x%x\n", i, sm5414_reg[i]);
	 }
 
	 return offset;
 }
 
 static ssize_t store_sm5414_access(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t size)
 {
	 int ret = 0;
	 char *pvalue = NULL, *addr, *val;
	 unsigned int reg_value = 0;
	 unsigned int reg_address = 0;
 
	 pr_debug("[store_sm5414_access]\n");
 
	 if (buf != NULL && size != 0) {
		 pr_debug("[store_sm5414_access] buf is %s and size is %zu\n", buf, size);
		 /*reg_address = kstrtoul(buf, 16, &pvalue);*/
 
		 pvalue = (char *)buf;
		 if (size > 3) {
			 addr = strsep(&pvalue, " ");
			 ret = kstrtou32(addr, 16, (unsigned int *)&reg_address);
		 } else
			 ret = kstrtou32(pvalue, 16, (unsigned int *)&reg_address);
 
		 if (size > 3) {
			 val = strsep(&pvalue, " ");
			 ret = kstrtou32(val, 16, (unsigned int *)&reg_value);
 
			 pr_debug("[store_sm5414_access] write sm5414 reg 0x%x with value 0x%x !\n",reg_address, reg_value);
			 ret = sm5414_config_interface(reg_address, reg_value, 0xFF, 0x0);
		 } else {
			 ret = sm5414_read_interface(reg_address, &g_reg_value_sm5414, 0xFF, 0x0);
			 pr_debug("[store_sm5414_access] read sm5414 reg 0x%x with value 0x%x !\n",reg_address, g_reg_value_sm5414);
			 pr_debug("[store_sm5414_access] Please use \"cat sm5414_access\" to get value\r\n");
		 }
	 }
	 return size;
 }
 
 static DEVICE_ATTR(sm5414_access, 0664, show_sm5414_access, store_sm5414_access);	 /* 664 */
 
 static int sm5414_user_space_probe(struct platform_device *dev)
 {
	 int ret_device_file = 0;
 
	 pr_debug("******** sm5414_user_space_probe!! ********\n");
 
	 ret_device_file = device_create_file(&(dev->dev), &dev_attr_sm5414_access);
 
	 return 0;
 }
 
 struct platform_device sm5414_user_space_device = {
	 .name = "sm5414-user",
	 .id = -1,
 };
 
 static struct platform_driver sm5414_user_space_driver = {
	 .probe = sm5414_user_space_probe,
	 .driver = {
		 .name = "sm5414-user",
	 },
 };
 
 static int __init sm5414_subsys_init(void)
 {
	 int ret = 0;
 
	 if (i2c_add_driver(&sm5414_driver) != 0)
		 pr_debug("[sm5414_init] failed to register sm5414 i2c driver.\n");
	 else
		 pr_debug("[sm5414_init] Success to register sm5414 i2c driver.\n");
 
	 /* sm5414 user space access interface */
	 ret = platform_device_register(&sm5414_user_space_device);
	 if (ret) {
		 pr_debug("****[sm5414_init] Unable to device register(%d)\n", ret);
		 return ret;
	 }
	 ret = platform_driver_register(&sm5414_user_space_driver);
	 if (ret) {
		 pr_debug("****[sm5414_init] Unable to register driver (%d)\n", ret);
		 return ret;
	 }
 
	 return 0;
 }
 
 static void __exit sm5414_exit(void)
 {
	 i2c_del_driver(&sm5414_driver);
 }
 
 /* module_init(sm5414_init); */
 /* module_exit(sm5414_exit); */
 subsys_initcall(sm5414_subsys_init);
 
 MODULE_LICENSE("GPL");
 MODULE_DESCRIPTION("I2C sm5414 Driver");
 
