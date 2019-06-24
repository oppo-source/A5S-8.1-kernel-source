/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - parser.h
** Description : This program is for ili9881 driver parser.h
** Version: 1.0
** Date : 2018/5/17
** Author: Zhonghua.Hu@ODM_WT.BSP.TP-FP.TP
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/

#ifndef __PARSER_H
#define __PARSER_H
#include "config.h"

#define BENCHMARK_KEY_NAME "Benchmark_Data"
#define NODE_TYPE_KEY_NAME "node_type_Data"
#define TYPE_MARK "[Driver Type Mark]"
#define VALUE 0

extern void core_parser_nodetype(int32_t* type_ptr, char *desp);
extern void core_parser_benchmark(int32_t* max_ptr, int32_t* min_ptr, int8_t type, char *desp);
extern int core_parser_get_u8_array(char *key, uint8_t *buf, size_t len);
extern int core_parser_get_int_data(char *section, char *keyname, char *rv);
extern int core_parser_path(char *path);

#endif
