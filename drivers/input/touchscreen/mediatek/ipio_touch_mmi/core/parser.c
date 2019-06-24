/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - parser.c
** Description : This program is for ili9881 driver parser.c
** Version: 1.0
** Date : 2018/5/17
** Author: Zhonghua.Hu@ODM_WT.BSP.TP-FP.TP
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/

#include "../common.h"
#include "../platform.h"
#include "parser.h"
#include "config.h"
#include <linux/firmware.h>


#define PARSER_MAX_CFG_BUF          (512 * 3)
#define PARSER_MAX_KEY_NUM	        (600 * 3)
#define PARSER_MAX_KEY_NAME_LEN	    100
#define PARSER_MAX_KEY_VALUE_LEN	2000

#define INI_ERR_OUT_OF_LINE     -1

struct ini_file_data {
	char pSectionName[PARSER_MAX_KEY_NAME_LEN];
	char pKeyName[PARSER_MAX_KEY_NAME_LEN];
	char pKeyValue[PARSER_MAX_KEY_VALUE_LEN];
	int iSectionNameLen;
	int iKeyNameLen;
	int iKeyValueLen;
} ilitek_ini_file_data[PARSER_MAX_KEY_NUM];

int g_ini_items = 0;

static int isspace_t(int x)
{
    if(x==' '||x=='\t'||x=='\n'||x=='\f'||x=='\b'||x=='\r')
        return 1;
    else
        return 0;
}

static char *ini_str_trim_r(char *buf)
{
	int len, i;
	char tmp[512] = { 0 };

	len = strlen(buf);

	for (i = 0; i < len; i++) {
		if (buf[i] != ' ')
			break;
	}

	if (i < len)
		strncpy(tmp, (buf + i), (len - i));

	strncpy(buf, tmp, len);
	return buf;
}

/* Count the number of each line and assign the content to tmp buffer */
static int get_ini_phy_line(char *data, char *buffer, int maxlen,int loadsize,int filesize)
{
	int i = 0;
	int j = 0;
	int iRetNum = -1;
	char ch1 = '\0';
	int allsize =0;
	allsize =loadsize;
	for (i = 0, j = 0; i < maxlen; j++) {
		ch1 = data[j];
		allsize++;
		iRetNum = j + 1;
		//printk("===hfs i is %d,%d,%d,%c\n",i,iRetNum,ch1,ch1);
		if (ch1 == '\n' || ch1 == '\r') {	/* line end */
			ch1 = data[j + 1];
			if (ch1 == '\n' || ch1 == '\r') {
				iRetNum++;
			}

			break;
		} else if (allsize > filesize) {
			//iRetNum = -2;
			break;	/* file end */
		}

		buffer[i++] = ch1;
	}
	//printk("===hfs iRetNum is %d \n",iRetNum);
	buffer[i] = '\0';
	return iRetNum;
}

static int get_ini_phy_data(char *data, int fsize)
{
	int i, n = 0, res = 0 , banchmark_flag = 0, empty_section, nodetype_flag = 0;
	int offset = 0, isEqualSign = 0;
	char *ini_buf = NULL, *tmpSectionName = NULL, *temp;
	char M_CFG_SSL = '[';
	char M_CFG_SSR = ']';
/* char M_CFG_NIS = ':'; */
	char M_CFG_NTS = '#';
	char M_CFG_EQS = '=';

	if (data == NULL) {
		ipio_err("INI data is NULL\n");
		res = -EINVAL;
		goto out;
	}

	ini_buf = kzalloc((PARSER_MAX_CFG_BUF + 1) * sizeof(char), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ini_buf)) {
		ipio_err("Failed to allocate ini_buf memory, %ld\n", PTR_ERR(ini_buf));
		res = -ENOMEM;
		goto out;
	}

	tmpSectionName = kzalloc((PARSER_MAX_CFG_BUF + 1) * sizeof(char), GFP_KERNEL);
	if (ERR_ALLOC_MEM(tmpSectionName)) {
		ipio_err("Failed to allocate tmpSectionName memory, %ld\n", PTR_ERR(tmpSectionName));
		res = -ENOMEM;
		goto out;
	}

	temp = strnstr(data, TYPE_MARK, fsize);
	if(temp != NULL) {
		//ipio_debug(DEBUG_PARSER, "Find Type mark, locat = %d",strlen(temp-data));
		ipio_debug(DEBUG_PARSER, "Find Type mark\n");
		if(core_config->core_type == CORE_TYPE_B)
			offset = temp-data;
		else
			fsize = temp-data;
	}

	while (true) {
		banchmark_flag = 0;
		empty_section = 0;
		nodetype_flag = 0;
		if (g_ini_items > PARSER_MAX_KEY_NUM) {
			ipio_err("MAX_KEY_NUM: Out of length\n");
			goto out;
		}

		if(offset >= fsize)
			goto out;/*over size*/

		//printk("===@hfs before n is %d,offset is %d",n,offset);

		if (ERR_ALLOC_MEM(ini_buf)) {
			ipio_err("Failed to allocate ini_buf memory, %ld\n", PTR_ERR(ini_buf));
			ini_buf = kzalloc((PARSER_MAX_CFG_BUF + 1) * sizeof(char), GFP_KERNEL);
		}

		n = get_ini_phy_line(data + offset, ini_buf, PARSER_MAX_CFG_BUF,offset,fsize);

		if (n < 0) {
			ipio_err("End of Line\n");
			goto out;
		}

		offset += n;

		//printk("===hfs offset is %d,n is %d",offset,n);
		n = strlen(ini_str_trim_r(ini_buf));

		if (n == 0 || ini_buf[0] == M_CFG_NTS)
			continue;

		/* Get section names */
		if (n > 2 && ((ini_buf[0] == M_CFG_SSL && ini_buf[n - 1] != M_CFG_SSR))) {
			ipio_err("Bad Section: %s\n", ini_buf);
			res = -EINVAL;
			goto out;
		} else {
			if (ini_buf[0] == M_CFG_SSL) {
				ilitek_ini_file_data[g_ini_items].iSectionNameLen = n - 2;
				if (ilitek_ini_file_data[g_ini_items].iSectionNameLen > PARSER_MAX_KEY_NAME_LEN) {
					ipio_err("MAX_KEY_NAME_LEN: Out Of Length\n");
					res = INI_ERR_OUT_OF_LINE;
					goto out;
				}

				ini_buf[n - 1] = 0x00;
				strcpy((char *)tmpSectionName, ini_buf + 1);
				banchmark_flag = 0;
				nodetype_flag = 0;
				ipio_debug(DEBUG_PARSER, "Section Name: %s, Len: %d\n", tmpSectionName, n - 2);
				continue;
			}
		}

		/* copy section's name without square brackets to its real buffer */
		strcpy(ilitek_ini_file_data[g_ini_items].pSectionName, tmpSectionName);
		ilitek_ini_file_data[g_ini_items].iSectionNameLen = strlen(tmpSectionName);

		isEqualSign = 0;
		for (i = 0; i < n; i++) {
			if (ini_buf[i] == M_CFG_EQS) {
				isEqualSign = i;
				break;
			}
			if(ini_buf[i] == M_CFG_SSL || ini_buf[i] == M_CFG_SSR){
				empty_section = 1;
				break;
			}
		}

		if (isEqualSign == 0)
		{
			if(empty_section)
				continue;

			if (strstr(ilitek_ini_file_data[g_ini_items].pSectionName,"Benchmark_Data") > 0){
				banchmark_flag = 1;
				isEqualSign =-1;
			}
			else if (strstr(ilitek_ini_file_data[g_ini_items].pSectionName,"Node Type") > 0){
				nodetype_flag = 1;
				isEqualSign =-1;
			}
			else{
				continue;
			}
		}
		if(banchmark_flag){
		/* Get Key names */
			ilitek_ini_file_data[g_ini_items].iKeyNameLen = strlen(BENCHMARK_KEY_NAME);
			strcpy(ilitek_ini_file_data[g_ini_items].pKeyName, BENCHMARK_KEY_NAME);
			ilitek_ini_file_data[g_ini_items].iKeyValueLen = n;
		}
		else if(nodetype_flag){
		/* Get Key names */
			ilitek_ini_file_data[g_ini_items].iKeyNameLen = strlen(NODE_TYPE_KEY_NAME);
			strcpy(ilitek_ini_file_data[g_ini_items].pKeyName, NODE_TYPE_KEY_NAME);
			ilitek_ini_file_data[g_ini_items].iKeyValueLen = n;
		}
		else{
		/* Get Key names */
			ilitek_ini_file_data[g_ini_items].iKeyNameLen = isEqualSign;
			if (ilitek_ini_file_data[g_ini_items].iKeyNameLen > PARSER_MAX_KEY_NAME_LEN) {
				/* ret = CFG_ERR_OUT_OF_LEN; */
				ipio_err("MAX_KEY_NAME_LEN: Out Of Length\n");
				res = INI_ERR_OUT_OF_LINE;
				goto out;
			}

			memcpy(ilitek_ini_file_data[g_ini_items].pKeyName,
			ini_buf, ilitek_ini_file_data[g_ini_items].iKeyNameLen);
			ilitek_ini_file_data[g_ini_items].iKeyValueLen = n - isEqualSign - 1;
		}

		/* Get a value assigned to a key */

		if (ilitek_ini_file_data[g_ini_items].iKeyValueLen > PARSER_MAX_KEY_VALUE_LEN) {
			ipio_err("MAX_KEY_VALUE_LEN: Out Of Length\n");
			res = INI_ERR_OUT_OF_LINE;
			goto out;
		}

		memcpy(ilitek_ini_file_data[g_ini_items].pKeyValue,
		       ini_buf + isEqualSign + 1, ilitek_ini_file_data[g_ini_items].iKeyValueLen);

		ipio_debug(DEBUG_PARSER, "%s = %s\n", ilitek_ini_file_data[g_ini_items].pKeyName,
		    ilitek_ini_file_data[g_ini_items].pKeyValue);

		g_ini_items++;
	}

out:
	ipio_kfree((void **)&ini_buf);
	ipio_kfree((void **)&tmpSectionName);
	return res;
}

static void init_ilitek_ini_data(void)
{
	int i;

	g_ini_items = 0;

	/* Initialise ini strcture */
	for (i = 0; i < PARSER_MAX_KEY_NUM; i++) {
		memset(ilitek_ini_file_data[i].pSectionName, 0, PARSER_MAX_KEY_NAME_LEN);
		memset(ilitek_ini_file_data[i].pKeyName, 0, PARSER_MAX_KEY_NAME_LEN);
		memset(ilitek_ini_file_data[i].pKeyValue, 0, PARSER_MAX_KEY_VALUE_LEN);
		ilitek_ini_file_data[i].iSectionNameLen = 0;
		ilitek_ini_file_data[i].iKeyNameLen = 0;
		ilitek_ini_file_data[i].iKeyValueLen = 0;
	}
}

/* get_ini_key_value - get ini's key and value based on its section from its array
 *
 * A function is digging into the key and value by its section from the ini array.
 * The comparsion is not only a string's name, but its length.
 */
static int get_ini_key_value(char *section, char *key, char *value)
{
	int i = 0;
	int ret = -2;
	int len = 0;

	len = strlen(key);

	for (i = 0; i < g_ini_items; i++) {
		if (strcmp(section, ilitek_ini_file_data[i].pSectionName) != 0)
			continue;

		if (strcmp(key, ilitek_ini_file_data[i].pKeyName) == 0) {
			memcpy(value, ilitek_ini_file_data[i].pKeyValue, ilitek_ini_file_data[i].iKeyValueLen);
			ipio_debug(DEBUG_PARSER, " value:%s , pKeyValue: %s\n", value, ilitek_ini_file_data[i].pKeyValue);
			ret = 0;
			break;
		}
	}
	return ret;
}



void core_parser_nodetype(int32_t* type_ptr, char *desp)
{

	int i = 0, j = 0, index1 =0, temp, count = 0;
	char str[512] = { 0 }, record = ',';

	for (i = 0; i < g_ini_items; i++) {

		if ((strstr(ilitek_ini_file_data[i].pSectionName, desp) <= 0) ||
			strcmp(ilitek_ini_file_data[i].pKeyName, NODE_TYPE_KEY_NAME) != 0) {
				continue;
			}

		record = ',';
		for(j=0, index1 = 0; j <= ilitek_ini_file_data[i].iKeyValueLen; j++){

			if(ilitek_ini_file_data[i].pKeyValue[j] == ';' || j == ilitek_ini_file_data[i].iKeyValueLen){

				if(record != '.')
				{
					memset(str,0 ,sizeof(str));
					memcpy(str,&ilitek_ini_file_data[i].pKeyValue[index1], (j -index1));
					temp=katoi(str);
					if(count>=576)
					{
						ipio_info("core_parser_nodetype error count =%d\n", count);
					}
					type_ptr[count] = temp;
					printk("%04d,",temp);
					count++;
				}
				record = ilitek_ini_file_data[i].pKeyValue[j];
				index1 = j+1;
			}
		}
		printk("\n");

	}
}

void core_parser_benchmark(int32_t* max_ptr, int32_t* min_ptr, int8_t type, char *desp)
{
	int i = 0, j = 0, index1 =0, temp, count = 0;
	char str[512] = { 0 }, record = ',';
	int32_t data[4];

	for (i = 0; i < g_ini_items; i++) {
		if ((strstr(ilitek_ini_file_data[i].pSectionName, desp) <= 0) ||
			strcmp(ilitek_ini_file_data[i].pKeyName, BENCHMARK_KEY_NAME) != 0) {
				continue;
		}

		record = ',';
		for(j = 0, index1 = 0; j <= ilitek_ini_file_data[i].iKeyValueLen; j++) {
			if(ilitek_ini_file_data[i].pKeyValue[j] == ',' || ilitek_ini_file_data[i].pKeyValue[j] == ';' ||
				ilitek_ini_file_data[i].pKeyValue[j] == '.'|| j == ilitek_ini_file_data[i].iKeyValueLen) {

				if(record != '.') {
					memset(str, 0, sizeof(str));
					memcpy(str, &ilitek_ini_file_data[i].pKeyValue[index1], (j - index1));

					temp = katoi(str);
					//ipio_info("core_parser_benchmark, count = %d\n", count);
					data[(count % 4)] = temp;
					if ((count % 4) == 3) {
						if (data[0] == 1) {
							if (type == VALUE) {
								max_ptr[count/4] = data[1] + data[2];
								min_ptr[count/4] = data[1] + data[3];
							} else {
								max_ptr[count/4] = data[1] + (data[1] * data[2]) / 100;
								min_ptr[count/4] = data[1] - (data[1] * data[3]) / 100;
							}
						} else {
							max_ptr[count/4] = INT_MAX;
							min_ptr[count/4] = INT_MIN;
						}
					}
					count++;
				}
				record = ilitek_ini_file_data[i].pKeyValue[j];
				index1 = j + 1;
				}
		}
	}
}
EXPORT_SYMBOL(core_parser_benchmark);

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/11/15,Update new INI for TP test
int core_parser_get_u8_array(char *key, uint8_t *buf, size_t len)
{
	char *s = key;
	char *pToken;
	int res, conut = 0;
    long s_to_long = 0;

	if(isspace_t((int)(unsigned char)*s) == 0)
	{
		while((pToken = strsep(&s, ",")) != NULL){
			res = kstrtol(pToken, 0, &s_to_long);
			if(res == 0)
				buf[conut] = s_to_long;
			else
				ipio_info("convert string too long, res = %d\n", res);
			conut++;
			if(conut >= len)
				break;
		}
	}

	return conut;
}
EXPORT_SYMBOL(core_parser_get_u8_array);
#endif

int core_parser_get_int_data(char *section, char *keyname, char *rv)
{
	int len = 0;
	char value[512] = { 0 };

	if (rv == NULL || section == NULL || keyname == NULL) {
		ipio_err("Parameters are invalid\n");
		return -EINVAL;
	}

	/* return a white-space string if get nothing */
	if (get_ini_key_value(section, keyname, value) < 0) {
		sprintf(rv, "%s", value);
		return 0;
	}

	len = sprintf(rv, "%s", value);
	return len;
}
EXPORT_SYMBOL(core_parser_get_int_data);

int core_parser_path(char *path)
{
	int res = 0;
#if 	0
	int fsize = 0;
	char *tmp = NULL;
	struct file *f = NULL;
	struct inode *inode;
	mm_segment_t old_fs;
	loff_t pos = 0;
#endif
    const struct firmware *fw_ini = NULL;
	ipio_debug(DEBUG_PARSER, "path = %s\n", path);

    res = request_firmware(&fw_ini, path, &(ipd->spi->dev));
    if (res != 0) {
        ipio_err("%s : request mp file failed! ret = %d\n", __func__, res);
		goto out;
    }

	ipio_debug(DEBUG_PARSER, "fw_ini->size = %d\n", (int)fw_ini->size);
	if (fw_ini->size <= 0) {
		ipio_err("The size of file is invaild\n");
		res = -EINVAL;
		goto out;
	}
#if 0
	ipio_info("path = %s\n", path);
	f = filp_open(path, O_RDONLY, 0);
	if (ERR_ALLOC_MEM(f)) {
		ipio_err("Failed to open the file at %ld.\n", PTR_ERR(f));
		res = -ENOENT;
		return res;
	}

#if KERNEL_VERSION(3, 18, 0) >= LINUX_VERSION_CODE
	inode = f->f_dentry->d_inode;
#else
	inode = f->f_path.dentry->d_inode;
#endif

	fsize = inode->i_size;
	ipio_info("fsize = %d\n", fsize);
	if (fsize <= 0) {
		ipio_err("The size of file is invaild\n");
		res = -EINVAL;
		goto out;
	}

	tmp = kmalloc(fsize+1, GFP_KERNEL);
	if (ERR_ALLOC_MEM(tmp)) {
		ipio_err("Failed to allocate tmp memory, %ld\n", PTR_ERR(tmp));
		res = -ENOMEM;
		goto out;
	}

	/* ready to map user's memory to obtain data by reading files */
	old_fs = get_fs();
	set_fs(get_ds());
	vfs_read(f, tmp, fsize, &pos);
	set_fs(old_fs);
	tmp[fsize] = 0x00;
#endif
	init_ilitek_ini_data();

	res = get_ini_phy_data((uint8_t *)fw_ini->data,(int)fw_ini->size);
	if (res < 0) {
		ipio_err("Failed to get physical ini data, res = %d\n", res);
		goto out;
	}

	ipio_debug(DEBUG_PARSER, "Parsing INI file doen\n");

out:
#if 0
	ipio_kfree((void **)&tmp);
	filp_close(f, NULL);
#endif
    if(fw_ini != NULL) {
        release_firmware(fw_ini);
    }
	return res;
}
EXPORT_SYMBOL(core_parser_path);
