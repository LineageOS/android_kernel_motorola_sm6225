#ifndef __LINUX_POWER_RT9426A_BATTERY_H
#define __LINUX_POWER_RT9426A_BATTERY_H

#define RT9426A_DRIVER_VER		0x0001

#define RT9426A_Unseal_Key		0x12345678
#define RT9426A_DEVICE_ID		0x426A
#define RT9426A_EXTREG_SIZE		224

#define RT9426A_REG_CNTL		0x00
#define RT9426A_REG_RSVD		0x02
#define RT9426A_REG_CURR		0x04
#define RT9426A_REG_TEMP		0x06
#define RT9426A_REG_VBAT		0x08
#define RT9426A_REG_FLAG1		0x0A
#define RT9426A_REG_FLAG2		0x0C
#define RT9426A_REG_DEVICE_ID		0x0E
#define RT9426A_REG_RM			0x10
#define RT9426A_REG_FCC			0x12
#define RT9426A_REG_AI			0x14
#define RT9426A_REG_BCCOMP		0x1A
#define RT9426A_REG_DUMMY		0x1E
#define RT9426A_REG_INTT		0x28
#define RT9426A_REG_CYC			0x2A
#define RT9426A_REG_SOC			0x2C
#define RT9426A_REG_SOH			0x2E
#define RT9426A_REG_FLAG3		0x30
#define RT9426A_REG_IRQ			0x36
#define RT9426A_REG_ADV			0x3A
#define RT9426A_REG_DC			0x3C
#define RT9426A_REG_BDCNTL		0x3E
#define RT9426A_REG_SWINDOW1		0x40
#define RT9426A_REG_SWINDOW2		0x42
#define RT9426A_REG_SWINDOW3		0x44
#define RT9426A_REG_SWINDOW4		0x46
#define RT9426A_REG_SWINDOW5		0x48
#define RT9426A_REG_SWINDOW6		0x4A
#define RT9426A_REG_SWINDOW7		0x4C
#define RT9426A_REG_SWINDOW8		0x4E
#define RT9426A_REG_SWINDOW9		0x50
#define RT9426A_REG_OCV			0x62
#define RT9426A_REG_AV			0x64
#define RT9426A_REG_AT			0x66
#define RT9426A_REG_TOTAL_CHKSUM	0x68
#define RT9426A_REG_RSVD2		0x7C

#define RT9426A_BATPRES_MASK		0x0040
#define RT9426A_RI_MASK			0x0100
#define RT9426A_BATEXIST_FLAG_MASK	0x8000
#define RT9426A_USR_TBL_USED_MASK	0x0800
#define RT9426A_CSCOMP1_OCV_MASK	0x0300
#define RT9426A_UNSEAL_MASK		0x0003
#define RT9426A_UNSEAL_STATUS		0x0001
#define RT9426A_GAUGE_BUSY_MASK		0x0008

#define RT9426A_SMOOTH_POLL		20
#define RT9426A_NORMAL_POLL		30
#define RT9426A_SOCALRT_MASK		0x20
#define RT9426A_SOCL_SHFT		0
#define RT9426A_SOCL_MASK		0x1F
#define RT9426A_SOCL_MAX		32
#define RT9426A_SOCL_MIN		1

#define RT9426A_RDY_MASK		0x0080

#define RT9426A_UNSEAL_PASS		0
#define RT9426A_UNSEAL_FAIL		1
#define RT9426A_PAGE_0			0
#define RT9426A_PAGE_1			1
#define RT9426A_PAGE_2			2
#define RT9426A_PAGE_3			3
#define RT9426A_PAGE_4			4
#define RT9426A_PAGE_5			5
#define RT9426A_PAGE_6			6
#define RT9426A_PAGE_7			7
#define RT9426A_PAGE_8			8
#define RT9426A_PAGE_9			9
#define RT9426A_PAGE_10			10
#define RT9426A_PAGE_11			11
#define RT9426A_PAGE_12			12
#define RT9426A_PAGE_13			13
#define RT9426A_PAGE_14			14
#define RT9426A_PAGE_15			15

#define RT9426A_TOTAL_CHKSUM_CMD	0x9A12
#define RT9426A_WPAGE_CMD		0x6550
#define RT9426A_SEAL_CMD		0x0020

/* for calibration */
#define RT9426A_CALI_ENTR_CMD         0x0081
#define RT9426A_CALI_EXIT_CMD         0x0080
#define RT9426A_CURR_CONVERT_CMD      0x0009
#define RT9426A_VOLT_CONVERT_CMD      0x008C
#define RT9426A_CALI_MODE_MASK        0x1000
#define RT9426A_SYS_TICK_ON_CMD       0xBBA1
#define RT9426A_SYS_TICK_OFF_CMD      0xBBA0

#define RT9426A_CALI_MODE_PASS        0
#define RT9426A_CALI_MODE_FAIL        1

/* for Enter/Exit Shutdown */
#define RT9426A_SHDN_MASK        0x4000
#define RT9426A_SHDN_ENTR_CMD    0x64AA
#define RT9426A_SHDN_EXIT_CMD    0x6400

#define TA_IS_CONNECTED		1
#define TA_IS_DISCONNECTED	0
#define RT9426A_FD_TBL_IDX	4
#define RT9426A_FD_DATA_IDX	10
#define RT9426A_FD_BASE		2500

#define RT9426A_NEW_RS_UNIT		50  /* unit:0.01mR ; 50 x 0.01 = 0.5mR */

/* for Handling of Cycle Cnt & BCCOMP */
#define RT9426A_SET_CYCCNT_KEY	0xCC01
/* for checking result of writing ocv */
#define RT9426A_WRITE_OCV_PASS        0
#define RT9426A_WRITE_OCV_FAIL        (-1)
#define RT9426A_IDX_OF_OCV_CKSUM      76

/* for force temp set */
#define RT9426A_OPCFG1_TEMP_SRC_MASK      0xC000
#define RT9426A_OPCFG1_TEMP_SRC_HOST      0x4000

struct data_point {
	union {
		int x;
		int voltage;
		int soc;
	};
	union {
		int y;
		int temperature;
	};
	union {
		int z;
		int curr;
	};
	union {
		int w;
		int offset;
	};
};

struct soc_offset_table {
	int soc_voltnr;
	int tempnr;
	struct data_point *soc_offset_data;
};

/* temperature source table */
enum {
	RT9426A_TEMP_FROM_AP,
	RT9426A_TEMP_FROM_IC,
};

struct fg_ocv_table {
	int data[8];
};

struct fg_extreg_table {
	u8 data[16];
};

struct rt9426a_platform_data {
	u32 dtsi_version[2];
	u32 para_version;
	int soc_offset_size[2];
	struct soc_offset_table soc_offset;
	int offset_interpolation_order[2];
	struct fg_extreg_table extreg_table[14];
	int battery_type;
	char *bat_name;
	int boot_gpio;
	int chg_sts_gpio;
	int chg_inh_gpio;
	int chg_done_gpio;
	u32 temp_source;
	u32 volt_source;
	u32 curr_source;

	/* for current scaling */
	u32 rs_ic_setting;  // in unit of 0.01 Ohm
	u32 rs_schematic;   // in unit of 0.01 Ohm
	/* add for smooth soc */
	int smooth_soc_en;
	/* add for aging cv */
	u32 fcc[5];			/* for aging_sts=0~4 */
	u32 fc_vth[5];		/* for aging_sts=0~4 */
	unsigned int ocv_table[5][10][8];	/* for aging_sts=0~4 */
};

enum {
	RT9426A_INIT_SEAL_ERR = -6,
	RT9426A_INIT_DEV_ID_ERR = -5,
	RT9426A_INIT_CKSUM_ERR = -4,
	RT9426A_INIT_UNSEAL_ERR = -3,
	RT9426A_INIT_BYPASS = -2,
	RT9426A_INIT_FAIL = -1,
	RT9426A_INIT_PASS = 0,
};

#endif /* __LINUX_POWER_RT9426A_BATTERY_H */
