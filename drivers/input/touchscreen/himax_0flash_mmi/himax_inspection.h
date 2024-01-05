/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for inspection functions
 *
 *  Copyright (C) 2019 Himax Corporation.
 *
 *  This software is licensed under the terms of the GNU General Public
 *  License version 2,  as published by the Free Software Foundation,  and
 *  may be copied,  distributed,  and modified under those terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include "himax_platform.h"
#include "himax_common.h"
#include "himax_ic_core.h"

/*#define HX_GAP_TEST*/
/*#define HX_INSPECT_LPWUG_TEST*/
/*#define HX_ACT_IDLE_TEST*/

#define HX_RSLT_OUT_PATH "/sdcard/"
#define HX_RSLT_OUT_FILE "hx_test_result.txt"
#define PI(x...) pr_info(x)
#define PD(x...) pr_debug(x)

#ifdef HX_ESD_RECOVERY
	extern u8 HX_ESD_RESET_ACTIVATE;
#endif
#ifdef HX_112F_SET
#define BS_RAWDATA	8
#define BS_NOISE	8
#else
#define BS_RAWDATA	10
#define BS_NOISE	10
#endif
#define BS_OPENSHORT	0
#define	BS_LPWUG	1
#define	BS_LPWUG_dile	1
#define	BS_ACT_IDLE	1

/* skip notch & dummy */
#define SKIP_NOTCH_START 5
#define SKIP_NOTCH_END 10
/* TX+SKIP_NOTCH_START */
#define SKIP_DUMMY_START 23
/* TX+SKIP_NOTCH_END*/
#define SKIP_DUMMY_END 28

#ifdef HX_112F_SET
#define	NOISEFRAME	60
#else
#define	NOISEFRAME	(BS_NOISE+1)
#endif

#define NORMAL_IDLE_RAWDATA_NOISEFRAME 10
#define LPWUG_RAWDATAFRAME 1
#define LPWUG_NOISEFRAME 1
#define LPWUG_IDLE_RAWDATAFRAME 1
#define LPWUG_IDLE_NOISEFRAME 1

#define OTHERSFRAME	2

#define	UNIFMAX	500


/*Himax MP Password*/
#define	PWD_OPEN_START	0x77
#define	PWD_OPEN_END	0x88
#define	PWD_SHORT_START	0x11
#define	PWD_SHORT_END	0x33
#define	PWD_RAWDATA_START	0x00
#define	PWD_RAWDATA_END	0x99
#define	PWD_NOISE_START	0x00
#define	PWD_NOISE_END	0x99
#define	PWD_SORTING_START	0xAA
#define	PWD_SORTING_END	0xCC


#define PWD_ACT_IDLE_START		0x22
#define PWD_ACT_IDLE_END		0x44



#define PWD_LPWUG_START	0x55
#define PWD_LPWUG_END	0x66

#define PWD_LPWUG_IDLE_START	0x50
#define PWD_LPWUG_IDLE_END	0x60


/*Himax DataType*/
#define DATA_SORTING	0x0A
#define DATA_OPEN	0x0B
#define DATA_MICRO_OPEN	0x0C
#define DATA_SHORT	0x0A
#define DATA_RAWDATA	0x0A
#define DATA_NOISE	0x0F
#define DATA_BACK_NORMAL	0x00
#define DATA_LPWUG_RAWDATA	0x0C
#define DATA_LPWUG_NOISE	0x0F
#define DATA_ACT_IDLE_RAWDATA	0x0A
#define DATA_ACT_IDLE_NOISE	0x0F
#define DATA_LPWUG_IDLE_RAWDATA	0x0A
#define DATA_LPWUG_IDLE_NOISE	0x0F

/*Himax Data Ready Password*/
#define	Data_PWD0	0xA5
#define	Data_PWD1	0x5A

/* ASCII format */
#define ASCII_LF	(0x0A)
#define ASCII_CR	(0x0D)
#define ASCII_COMMA	(0x2C)
#define ASCII_ZERO	(0x30)
#define CHAR_EL	'\0'
#define CHAR_NL	'\n'
#define ACSII_SPACE	(0x20)
/* INSOECTION Setting */
/*extern int HX_CRITERIA_ITEM;*/
/*extern int HX_CRITERIA_SIZE;*/
/*extern char *g_file_path;*/
/*extern char *g_rslt_data;*/
void himax_inspection_init(void);
extern int *g_inspt_crtra_flag;

enum THP_INSPECTION_ENUM {
	HIMAX_INSPECTION_OPEN,
	HIMAX_INSPECTION_MICRO_OPEN,
	HIMAX_INSPECTION_SHORT,
	HIMAX_INSPECTION_RAWDATA,
	HIMAX_INSPECTION_BPN_RAWDATA,
	HIMAX_INSPECTION_WEIGHT_NOISE,
	HIMAX_INSPECTION_ABS_NOISE,
	HIMAX_INSPECTION_SORTING,
	HIMAX_INSPECTION_BACK_NORMAL,

	HIMAX_INSPECTION_GAPTEST_RAW,

	HIMAX_INSPECTION_ACT_IDLE_RAWDATA,
	HIMAX_INSPECTION_ACT_IDLE_NOISE,

	HIMAX_INSPECTION_LPWUG_RAWDATA,
	HIMAX_INSPECTION_LPWUG_NOISE,
	HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA,
	HIMAX_INSPECTION_LPWUG_IDLE_NOISE,

};


enum HX_CRITERIA_ENUM {
	IDX_RAWMIN		= 0,
	IDX_RAWMAX,
	IDX_BPN_RAWMIN,
	IDX_BPN_RAWMAX,
	IDX_SHORTMIN,
	IDX_SHORTMAX,
	IDX_OPENMIN,
	IDX_OPENMAX,
	IDX_M_OPENMIN,
	IDX_M_OPENMAX,
	IDX_WT_NOISEMIN,
	IDX_WT_NOISEMAX,
	IDX_ABS_NOISEMIN,
	IDX_ABS_NOISEMAX,
	IDX_SORTMIN,
	IDX_SORTMAX,

	IDX_GAP_HOR_RAWMAX,
	IDX_GAP_HOR_RAWMIN,
	IDX_GAP_VER_RAWMAX,
	IDX_GAP_VER_RAWMIN,

	IDX_ACT_IDLE_NOISE_MIN,
	IDX_ACT_IDLE_NOISE_MAX,
	IDX_ACT_IDLE_RAWDATA_MIN,
	IDX_ACT_IDLE_RAWDATA_MAX,

	IDX_LPWUG_NOISE_MIN,
	IDX_LPWUG_NOISE_MAX,
	IDX_LPWUG_RAWDATA_MIN,
	IDX_LPWUG_RAWDATA_MAX,

	IDX_LPWUG_IDLE_NOISE_MIN,
	IDX_LPWUG_IDLE_NOISE_MAX,
	IDX_LPWUG_IDLE_RAWDATA_MIN,
	IDX_LPWUG_IDLE_RAWDATA_MAX,

};

/* Error code of Inspection */
enum HX_INSPECT_ERR_ENUM {
	HX_INSPECT_OK =                0x00000000, /* OK */
	HX_INSPECT_EFILE =             0x00000001, /*Criteria file error*/
	HX_INSPECT_ERAW =              0x00000002, /* Raw data error */
	HX_INSPECT_WT_ENOISE =         0x00000004, /* Noise error */
	HX_INSPECT_ABS_ENOISE =        0x00000008, /* Noise error */
	HX_INSPECT_EOPEN =             0x00000010, /* Sensor open error */
	HX_INSPECT_EMOPEN =            0x00000020, /* Sensor micro open error */
	HX_INSPECT_ESHORT =            0x00000040, /* Sensor short error */

	HX_INSPECT_EGAP_RAW =          0x00000080, /* Raw Data GAP  */

	HX_INSPECT_EACT_IDLE_RAW =     0x00000100, /* ACT_IDLE RAW ERROR */
	HX_INSPECT_EACT_IDLE_NOISE =   0x00000200, /* ACT_IDLE NOISE ERROR */

	HX_INSPECT_ELPWUG_RAW =        0x00000400, /* LPWUG RAW ERROR */
	HX_INSPECT_ELPWUG_NOISE =      0x00000800, /* LPWUG NOISE ERROR */
	HX_INSPECT_ELPWUG_IDLE_RAW =   0x00001000, /* LPWUG IDLE RAW ERROR */
	HX_INSPECT_ELPWUG_IDLE_NOISE = 0x00002000, /* LPWUG IDLE NOISE ERROR */

	HX_INSPECT_EGETRAW =           0x00004000, /* Get raw data errors */
	HX_INSPECT_MEMALLCTFAIL =      0x00008000, /* Get raw data errors */
	/*HX_INSPECT_ESPI = , */        /*SPI communication error,no use*/
};

extern void himax_inspect_data_clear(void);
extern char *hx_self_test_file_name;
