// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 */

#ifndef __BM_ADSP_ULOG_H__
#define __BM_ADSP_ULOG_H__

/* Generic definitions */
enum bm_ulog_level_type {
	BM_LOG_LEVEL_NONE = 0,
	BM_LOG_LEVEL_ERR = 1,
	BM_LOG_LEVEL_WARN = 2,
	BM_LOG_LEVEL_INFO = 3,
	BM_LOG_LEVEL_DEBUG = 4,
	BM_LOG_LEVEL_ALL_LOGS
};

enum bm_ulog_category_bitmap {
	BM_WPP_SELF_HOST		= 1ULL << 0,
	BM_WPP_DEVICE			= 1ULL << 1,
	BM_WPP_IOIMPL			= 1ULL << 2,
	BM_PMIC_WPP_DEVICE		= 1ULL << 3,
	BM_WPP_NONE			= 1ULL << 4,
	BM_WPP_SMCHGFGGGE		= 1ULL << 5,
	BM_WPP_PLATFORM			= 1ULL << 6,
	BM_WPP_IRQ			= 1ULL << 7,
	BM_WPP_NOTIFY			= 1ULL << 8,
	BM_WPP_STATEMACHINE		= 1ULL << 9,
	BM_WPP_STATEMACHINELEVEL	= 1ULL << 10,
	BM_WPP_BATTERRHANDLE		= 1ULL << 11,
	BM_WPP_FGGGE			= 1ULL << 12,
	BM_WPP_PARALLELCHARGING		= 1ULL << 13,
	BM_WPP_SMCHG			= 1ULL << 14,
	BM_WPP_SWJEITA			= 1ULL << 15,
	BM_WPP_QBG			= 1ULL << 16,
	BM_QBG				= 1ULL << 17,
	BM_ADC				= 1ULL << 18,
	BM_CAD				= 1ULL << 19,
	BM_PMICPLATFORM			= 1ULL << 20,
	BM_PMICHAL			= 1ULL << 21,
	BM_AGGREGATOR			= 1ULL << 22,
	BM_SUMMARY			= 1ULL << 23,
	BM_USBC_COMMON			= 1ULL << 24,
	BM_USBC_DPM			= 1ULL << 25,
	BM_USBC_LPM			= 1ULL << 26,
	BM_USBC_PRL			= 1ULL << 27,
	BM_USBC_PE			= 1ULL << 28,
	BM_USBC_TCPC			= 1ULL << 29,
	BM_USBC_TCPCPLATFORM		= 1ULL << 30,
	BM_USBC_SUMMARY			= 1ULL << 31,
	BM_INIT				= 1ULL << 32,
	BM_ULOG				= 1ULL << 33,
	BM_ICMLOG			= 1ULL << 34,
	BM_SSDEVLOG			= 1ULL << 35,
	BM_USBC_ULOG			= 1ULL << 36,
	BM_LOGCATEGORYMAX		= 1ULL << 63,
};

#define BM_USBC_ALL (                                                   \
            BM_USBC_COMMON | BM_USBC_DPM | BM_USBC_LPM | BM_USBC_PRL | BM_USBC_PE |    \
            BM_USBC_TCPC | BM_USBC_TCPCPLATFORM | BM_USBC_SUMMARY | BM_USBC_ULOG)

#define BM_BM_ALL (                                                       \
            BM_WPP_SELF_HOST | BM_WPP_DEVICE | BM_WPP_IOIMPL |    \
            BM_PMIC_WPP_DEVICE | BM_WPP_NONE | BM_WPP_SMCHGFGGGE |        \
            BM_WPP_PLATFORM | BM_WPP_IRQ | BM_WPP_NOTIFY |       \
            BM_WPP_STATEMACHINE | BM_WPP_STATEMACHINELEVEL |          \
            BM_WPP_BATTERRHANDLE | BM_WPP_FGGGE |                     \
            BM_WPP_PARALLELCHARGING | BM_WPP_SMCHG |                  \
            BM_WPP_SWJEITA | BM_WPP_QBG | BM_QBG |              \
            BM_ADC | BM_CAD | BM_PMICPLATFORM |               \
            BM_PMICHAL | BM_SUMMARY | BM_INIT |               \
            BM_ULOG | BM_ICMLOG | BM_SSDEVLOG)

#define BM_ALL  (BM_USBC_ALL | BM_BM_ALL)

/* exported APIs */
int bm_ulog_get_log(char *buf, u32 size);
int bm_ulog_get_mask_log(enum bm_ulog_category_bitmap categories,
		    enum bm_ulog_level_type level,
		    char *buf, u32 size);

int bm_ulog_print_log(u32 size);
int bm_ulog_print_mask_log(enum bm_ulog_category_bitmap categories,
		    enum bm_ulog_level_type level, u32 size);

#endif /* __BM_ADSP_ULOG_H__ */
